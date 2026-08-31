#!/usr/bin/env python3
"""Serial driver for the E2BOX EBIMU-9DOFV5 AHRS module.

This sensor has no ROS driver upstream -- it speaks its own plain-text
UART protocol (see EBIMU-9DOFV5_rev3.pdf, sections 5/6) directly over
VCC/GND/TX/RX, usually through a USB-UART bridge (CP210x/FTDI/CH340/...).

On connect the node pushes the module into one fixed, known output
configuration instead of trusting whatever was left over from a previous
session or the factory defaults:

    soc1   ASCII output mode
    sof2   attitude as quaternion (not Euler -- avoids gimbal-lock/wrap
           issues and is what sensor_msgs/Imu wants anyway)
    sog1   gyroscope (angular velocity) streaming on
    soa1   accelerometer streaming on, gravity included (matches
           sensor_msgs/Imu convention and REP 145)
    som0 / sod0 / sot0 / sots0
           magnetometer, distance, temperature, timestamp streaming off

That fixes the wire format to exactly 10 comma-separated fields per line:
quaternion(4) + gyro-xyz(3) + accel-xyz(3). If you need magnetometer or
temperature too, extend _SETUP_COMMANDS and _handle_line() together --
the module always appends new fields in the sof/sog/soa/som/sod/sot/sots
command order (spec section 5-1).

Quaternion field order on the wire is [z][y][x][w] (spec 5-1/6-1-4), not
ROS's [x][y][z][w] -- reordered below. No other axis remapping is applied:
this publishes in the sensor's own body frame as-is. Whether that lines up
with the vehicle's ENU/base_link convention (particularly yaw sign and
sign of angular_velocity.z) has to be checked on the actual vehicle -- see
imu_enu_relay.py in hyper_localization, which does exactly that correction
for the other IMU on this platform (WitMotion WT901BLE) and is the
expected pattern to reuse here if this module replaces or joins it.
"""
import math
import time

import rclpy
from rclpy.node import Node
import serial
from sensor_msgs.msg import Imu

_SETUP_COMMANDS = ('soc1', 'sof2', 'sog1', 'soa1', 'som0', 'sod0', 'sot0', 'sots0')

_DEG2RAD = math.pi / 180.0
_G_TO_MS2 = 9.80665


class EbimuNode(Node):

    def __init__(self):
        super().__init__('ebimu_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic', 'imu')
        self.declare_parameter('output_rate_ms', 10)
        self.declare_parameter('reconnect_wait_seconds', 2.0)
        self.declare_parameter(
            'orientation_covariance',
            [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.02])
        self.declare_parameter(
            'angular_velocity_covariance',
            [0.0025, 0.0, 0.0, 0.0, 0.0025, 0.0, 0.0, 0.0, 0.0025])
        self.declare_parameter(
            'linear_acceleration_covariance',
            [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01])

        self._port = self.get_parameter('port').value
        self._baudrate = self.get_parameter('baudrate').value
        self._frame_id = self.get_parameter('frame_id').value
        self._output_rate_ms = int(self.get_parameter('output_rate_ms').value)
        self._reconnect_wait = self.get_parameter('reconnect_wait_seconds').value
        self._orientation_cov = list(self.get_parameter('orientation_covariance').value)
        self._angular_velocity_cov = list(
            self.get_parameter('angular_velocity_covariance').value)
        self._linear_acceleration_cov = list(
            self.get_parameter('linear_acceleration_covariance').value)

        topic = self.get_parameter('topic').value
        self._pub = self.create_publisher(Imu, topic, 10)

        self._ser = None
        self._rx_buf = b''
        self._connect()

        # Poll at ~2x the sensor's own output rate so the OS-level serial
        # buffer gets drained promptly instead of several lines piling up
        # into one read() call.
        period = max(0.001, (self._output_rate_ms / 1000.0) / 2.0)
        self._timer = self.create_timer(period, self._poll)

    def _connect(self):
        while rclpy.ok():
            try:
                self._ser = serial.Serial(self._port, self._baudrate, timeout=0.05)
                self._configure_sensor()
                self.get_logger().info(
                    f'EBIMU connected on {self._port} @ {self._baudrate}bps')
                return
            except serial.SerialException as exc:
                self.get_logger().warn(
                    f'Could not open {self._port}: {exc}; retrying in '
                    f'{self._reconnect_wait}s')
                time.sleep(self._reconnect_wait)

    def _configure_sensor(self):
        # '<' switches the module out of streaming into command mode (spec
        # 3-2), so the setup commands below don't race against data lines.
        self._ser.reset_input_buffer()
        commands = (f'sor{self._output_rate_ms}',) + _SETUP_COMMANDS
        for cmd in commands:
            self._ser.write(f'<{cmd}>'.encode('ascii'))
            self._ser.flush()
            if not self._await_ack():
                self.get_logger().warn(f'EBIMU did not ack "<{cmd}>"')

    def _await_ack(self, timeout_s=2.0):
        # Spec 3-2 shows the response landing within "about 500ms" of the
        # closing '>', but that's the sensor's own budget -- it says
        # nothing about how promptly *this process* gets scheduled to read
        # it back. Measured on hardware, a tight standalone script sees
        # '<ok>' well under 500ms every time, but inside the ROS node
        # (rclpy/DDS background threads competing for the GIL) the same
        # loop occasionally missed a 0.5s window even though the command
        # had, in fact, landed (confirmed via "<cfg>"). 2s buys enough
        # margin that a timeout here means the command really didn't land,
        # not that we were just scheduled late.
        buf = b''
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            chunk = self._ser.read(64)
            if chunk:
                buf += chunk
                if b'<ok>' in buf:
                    return True
                if b'<er>' in buf:
                    return False
        return False

    def _poll(self):
        if self._ser is None:
            return
        try:
            chunk = self._ser.read(4096)
        except serial.SerialException as exc:
            self.get_logger().error(f'EBIMU serial read failed: {exc}; reconnecting')
            self._ser.close()
            self._ser = None
            self._connect()
            return

        if not chunk:
            return
        self._rx_buf += chunk
        *lines, self._rx_buf = self._rx_buf.split(b'\r\n')
        for line in lines:
            self._handle_line(line)

    def _handle_line(self, line: bytes):
        if not line.startswith(b'*'):
            return  # command echo/ack or a partial line -- not sensor data
        try:
            qz, qy, qx, qw, gx, gy, gz, ax, ay, az = (
                float(v) for v in line[1:].split(b','))
        except ValueError:
            self.get_logger().debug(f'Unparseable EBIMU line: {line!r}')
            return

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id

        norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
        if norm > 0.0:
            msg.orientation.x = qx / norm
            msg.orientation.y = qy / norm
            msg.orientation.z = qz / norm
            msg.orientation.w = qw / norm
        msg.orientation_covariance = self._orientation_cov

        msg.angular_velocity.x = gx * _DEG2RAD
        msg.angular_velocity.y = gy * _DEG2RAD
        msg.angular_velocity.z = gz * _DEG2RAD
        msg.angular_velocity_covariance = self._angular_velocity_cov

        msg.linear_acceleration.x = ax * _G_TO_MS2
        msg.linear_acceleration.y = ay * _G_TO_MS2
        msg.linear_acceleration.z = az * _G_TO_MS2
        msg.linear_acceleration_covariance = self._linear_acceleration_cov

        self._pub.publish(msg)

    def destroy_node(self):
        # Deliberately does NOT send "<stop>" here. The module's stopped
        # state outlasts this process and a later reconnect's "<start>" --
        # confirmed on hardware, only a full "<reset>" (power-cycle
        # equivalent) brought streaming back. Sending stop on every node
        # shutdown would silently brick the next run's data until someone
        # notices and resets it by hand, so we just close the port instead
        # and leave the module streaming.
        if self._ser is not None:
            self._ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = EbimuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
