#!/usr/bin/env python3
"""Correct the WitMotion IMU orientation into the ROS ENU convention.

The WT901BLE driver copies the sensor's compass-style angles straight into the
orientation quaternion: yaw grows clockwise (looking down) and is referenced to
the sensor's own zero, not REP-103 ENU (0 = East, growing counter-clockwise).
That gives a heading in RViz that is both mirrored (turning left turns the arrow
right) and rotated by a constant.

The correction applied here is:

    out_yaw = yaw_sign * in_yaw + yaw_offset_rad

Note that no EKF fuses this yaw as an absolute heading any more: the WT901BLE
runs in 6-axis mode, so its yaw drifts ~0.5 deg/min from wherever it was powered
on (see config/datums.yaml). Absolute heading now comes from gps_heading.py
instead, and imu0_config index 5 is false in both filters. The yaw fixed up here
is only for display and sanity checks.

What still matters a great deal is flip_angular_velocity_z. With the
magnetometer gone, yaw is *propagated* purely by integrating the z gyro, so a
flipped sign turns every left turn into an estimated right turn. Verify on the
car: rotate it counter-clockwise and check that
`ros2 topic echo /imu --field angular_velocity` reports positive z. If it is
negative, flip imu_flip_gyro_z in datums.yaml.

roll, pitch, accel and the covariances are passed through untouched.
"""

import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


def _quat_to_euler(x, y, z, w):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def _euler_to_quat(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ImuEnuRelay(Node):
    def __init__(self):
        super().__init__('imu_enu_relay')
        self.yaw_sign = self.declare_parameter('yaw_sign', -1.0).value
        self.yaw_offset_rad = self.declare_parameter('yaw_offset_rad', 0.0).value
        self.flip_gyro_z = self.declare_parameter('flip_angular_velocity_z', False).value
        self.pub = self.create_publisher(Imu, 'imu', 10)
        self.sub = self.create_subscription(Imu, 'imu/raw', self._on_imu, 10)

    def _on_imu(self, msg):
        o = msg.orientation
        roll, pitch, yaw = _quat_to_euler(o.x, o.y, o.z, o.w)
        yaw = math.atan2(
            math.sin(self.yaw_sign * yaw + self.yaw_offset_rad),
            math.cos(self.yaw_sign * yaw + self.yaw_offset_rad),
        )
        o.x, o.y, o.z, o.w = _euler_to_quat(roll, pitch, yaw)
        if self.flip_gyro_z:
            msg.angular_velocity.z = -msg.angular_velocity.z
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuEnuRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
