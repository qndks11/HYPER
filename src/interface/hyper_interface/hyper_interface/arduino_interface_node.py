"""ROS 2 <-> Arduino serial bridge.

Subscribes to the same /velocity [m/s] and /steering_angle [rad] topics that
hyper_control's vehicle_controller (sim) and hyper_planner's controller (real)
already publish to, and forwards them to an Arduino over USB serial as a small
fixed-size binary packet. The Arduino (see ../arduino/hyper_motor_interface)
unpacks the packet and drives the throttle and steering motor drivers off the
raw command. Both drive (wheel encoder) and steering (position sensor) run
closed loop on the Arduino side; it reports the actual velocity and steering
angle back over the same serial link. This node republishes those to ROS 2 on
/velocity_actual and /steering_angle_actual (verify with `ros2 topic echo` --
no Serial Monitor needed, it cannot be open at the same time as this node
anyway), and combines them into a bicycle-model /odom (nav_msgs/Odometry) for
hyper_localization's EKF (see dual_ekf_navsat.yaml's odom0: odom, which only
fuses the twist -- linear/angular velocity -- fields, not pose). This node
still dead-reckons x/y/yaw into /odom's pose anyway (see _publish_odom()) --
not for the EKF (its pose covariance is kept large so the EKF ignores it,
avoiding double-integrating the same velocity both here and inside the EKF),
but so /odom alone is directly useful for a quick sanity check of measured
distance against actual distance traveled, without needing the full EKF
stack (hyper_localization/odometry.launch.py) running.

Wire format (11 bytes, little-endian), matching hyper_motor_interface.ino:
    byte 0    : 0xAA              (start-of-frame 1)
    byte 1    : 0x55              (start-of-frame 2)
    byte 2-5  : float32           velocity      [m/s]
    byte 6-9  : float32           steering_angle[rad]
    byte 10   : uint8             checksum = XOR of bytes 2-9

Telemetry (Arduino -> this node), ASCII lines, matching send_telemetry() in
hyper_motor_interface.ino:
    STEER,<target_rad>,<current_rad>\\n
    VEL,<target_mps>,<current_mps>\\n
"""
import math
import struct

import rclpy
import serial
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float64

_SOF = bytes([0xAA, 0x55])


def _make_packet(velocity: float, steering_angle: float) -> bytes:
    payload = struct.pack('<ff', velocity, steering_angle)
    checksum = 0
    for byte in payload:
        checksum ^= byte
    return _SOF + payload + bytes([checksum])


class ArduinoInterfaceNode(Node):

    def __init__(self):
        super().__init__('arduino_interface_node')

        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('send_rate', 50.0)
        self.declare_parameter('command_timeout', 0.3)
        self.declare_parameter('max_velocity', 10.0)
        self.declare_parameter('max_steering_angle', 0.5235988)
        # Front-to-rear axle distance [m], for the bicycle-model yaw rate used
        # in /odom (angular.z = velocity * tan(steering_angle) / wheel_base).
        # Measured by hand on the vehicle. Re-measure and update (here and in
        # parameters.yaml) if the chassis changes.
        self.declare_parameter('wheel_base', 0.72)

        self._command_timeout = self.get_parameter('command_timeout').value
        self._max_velocity = self.get_parameter('max_velocity').value
        self._max_steering_angle = self.get_parameter('max_steering_angle').value
        self._wheel_base = self.get_parameter('wheel_base').value

        self._velocity = 0.0
        self._steering_angle = 0.0
        self._last_velocity_time = self.get_clock().now()
        self._last_steering_time = self.get_clock().now()

        # Latest actuals reported back by the Arduino, combined into /odom
        # once both have arrived at least once.
        self._steering_angle_actual = 0.0
        self._velocity_actual = 0.0

        # Dead-reckoned pose for /odom (see _publish_odom()) -- reset to the
        # origin at node startup, so distance/position readings are relative
        # to wherever the vehicle was when this node was launched, not any
        # global frame.
        self._odom_x = 0.0
        self._odom_y = 0.0
        self._odom_yaw = 0.0
        self._last_odom_time = None

        serial_port = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
        try:
            self._serial = serial.Serial(serial_port, baud_rate, timeout=0.0)
        except serial.SerialException as exc:
            self.get_logger().fatal(
                f"Could not open serial port '{serial_port}' @ {baud_rate} baud: {exc}")
            raise

        self.get_logger().info(f"Connected to Arduino on '{serial_port}' @ {baud_rate} baud")

        self._rx_buffer = ''

        self.create_subscription(Float64, '/velocity', self._velocity_callback, 10)
        self.create_subscription(Float64, '/steering_angle', self._steering_callback, 10)
        self._steering_actual_pub = self.create_publisher(Float64, '/steering_angle_actual', 10)
        self._velocity_actual_pub = self.create_publisher(Float64, '/velocity_actual', 10)
        self._odom_pub = self.create_publisher(Odometry, '/odom', 10)

        send_period = 1.0 / self.get_parameter('send_rate').value
        self._timer = self.create_timer(send_period, self._timer_callback)

    def _velocity_callback(self, msg: Float64):
        self._velocity = max(-self._max_velocity, min(self._max_velocity, msg.data))
        self._last_velocity_time = self.get_clock().now()

    def _steering_callback(self, msg: Float64):
        self._steering_angle = max(
            -self._max_steering_angle, min(self._max_steering_angle, msg.data))
        self._last_steering_time = self.get_clock().now()

    def _timer_callback(self):
        now = self.get_clock().now()
        timeout_ns = self._command_timeout * 1e9

        # Fail-safe: stop sending a stale command if the publisher went quiet
        # (node died, Ctrl-C'd, etc.) rather than let the Arduino keep driving
        # on the last value it heard.
        velocity = self._velocity if (now - self._last_velocity_time).nanoseconds \
            < timeout_ns else 0.0
        steering_angle = self._steering_angle if (now - self._last_steering_time).nanoseconds \
            < timeout_ns else 0.0

        try:
            self._serial.write(_make_packet(velocity, steering_angle))
        except serial.SerialException as exc:
            self.get_logger().error(f'Serial write failed: {exc}', throttle_duration_sec=1.0)

        # Drain and line-buffer whatever text the Arduino sent (boot banner,
        # STEER,<target>,<current> telemetry) -- buffered rather than treating
        # each read() as one line, since a line can arrive split across reads.
        try:
            waiting = self._serial.in_waiting
            if waiting:
                self._rx_buffer += self._serial.read(waiting).decode('ascii', errors='replace')
                while '\n' in self._rx_buffer:
                    line, self._rx_buffer = self._rx_buffer.split('\n', 1)
                    self._handle_arduino_line(line.strip())
        except serial.SerialException:
            pass

    def _handle_arduino_line(self, line: str):
        if not line:
            return

        if line.startswith('STEER,'):
            fields = line.split(',')
            if len(fields) == 3:
                try:
                    self._steering_angle_actual = float(fields[2])
                except ValueError:
                    self.get_logger().debug(f'Arduino: malformed telemetry line: {line}')
                    return
                self._steering_actual_pub.publish(Float64(data=self._steering_angle_actual))
            return

        if line.startswith('VEL,'):
            fields = line.split(',')
            if len(fields) == 3:
                try:
                    self._velocity_actual = float(fields[2])
                except ValueError:
                    self.get_logger().debug(f'Arduino: malformed telemetry line: {line}')
                    return
                self._velocity_actual_pub.publish(Float64(data=self._velocity_actual))
                self._publish_odom()
            return

        self.get_logger().debug(f'Arduino: {line}')

    def _publish_odom(self):
        # Bicycle-model yaw rate from the two actuals above.
        angular_z = self._velocity_actual * math.tan(self._steering_angle_actual) \
            / self._wheel_base

        now = self.get_clock().now()

        # Dead-reckon x/y/yaw by integrating velocity/yaw rate over the time
        # since the last VEL telemetry line (see _handle_arduino_line()) --
        # simple Euler integration, fine at this ~10Hz telemetry rate for a
        # sanity-check odometry. Skip the very first call (no prior time to
        # integrate from yet).
        if self._last_odom_time is not None:
            dt = (now - self._last_odom_time).nanoseconds / 1e9
            self._odom_x += self._velocity_actual * math.cos(self._odom_yaw) * dt
            self._odom_y += self._velocity_actual * math.sin(self._odom_yaw) * dt
            self._odom_yaw += angular_z * dt
            self._odom_yaw = math.atan2(math.sin(self._odom_yaw), math.cos(self._odom_yaw))
        self._last_odom_time = now

        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'body_link'

        # Pose IS dead-reckoned above (x/y/yaw) so /odom alone shows a usable
        # distance/position -- but its covariance is still kept large so the
        # EKF (see the module docstring) ignores it and does its own
        # integration from this same message's twist instead, rather than
        # double-integrating the same velocity in two places.
        msg.pose.pose.position.x = self._odom_x
        msg.pose.pose.position.y = self._odom_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation = Quaternion(
            x=0.0, y=0.0, z=math.sin(self._odom_yaw / 2.0), w=math.cos(self._odom_yaw / 2.0))
        msg.pose.covariance = [0.0] * 36
        for i in range(6):
            msg.pose.covariance[i * 6 + i] = 1e6

        msg.twist.twist.linear.x = self._velocity_actual
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = angular_z

        # Diagonal only. vy/vz are pinned near zero (rigid Ackermann vehicle,
        # no lateral/vertical slip assumed) rather than left unconstrained.
        # vroll/vpitch are not physically meaningful here and marked
        # unreliable; tune vx/vyaw's variance against how noisy /odom
        # actually looks once this is running.
        msg.twist.covariance = [0.0] * 36
        msg.twist.covariance[0 * 6 + 0] = 0.02   # vx
        msg.twist.covariance[1 * 6 + 1] = 0.001  # vy
        msg.twist.covariance[2 * 6 + 2] = 0.001  # vz
        msg.twist.covariance[3 * 6 + 3] = 1e6    # vroll
        msg.twist.covariance[4 * 6 + 4] = 1e6    # vpitch
        msg.twist.covariance[5 * 6 + 5] = 0.05   # vyaw

        self._odom_pub.publish(msg)

    def destroy_node(self):
        if hasattr(self, '_serial') and self._serial.is_open:
            try:
                self._serial.write(_make_packet(0.0, 0.0))  # leave the motors stopped
            except serial.SerialException:
                pass
            self._serial.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ArduinoInterfaceNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
