#!/usr/bin/env python3
"""Correct the WitMotion IMU orientation into the ROS ENU convention.

The WT901BLE driver copies the sensor's compass-style angles straight into the
orientation quaternion: yaw grows clockwise (looking down) and is referenced to
the sensor's own zero, not REP-103 ENU (0 = East, growing counter-clockwise).
That gives a heading in RViz that is both mirrored (turning left turns the arrow
right) and rotated by a constant.

navsat_transform's yaw_offset/declination cannot fix this -- they only rotate the
GPS odometry, while ekf_global fuses the IMU's absolute yaw directly. So the fix
belongs upstream of the EKF, here:

    out_yaw = yaw_sign * in_yaw + yaw_offset_rad

Calibrate on the car: point the vehicle due North, echo /imu, and adjust
yaw_offset_rad until the reported yaw is +pi/2 (1.5708). Check the sign by
rotating the car counter-clockwise -- yaw must increase. roll, pitch, accel and
the covariances are passed through untouched.
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
        self.flip_gyro_z = self.declare_parameter('flip_angular_velocity_z', True).value
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
