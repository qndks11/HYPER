#!/usr/bin/env python3

import asyncio
import math
import struct
import threading

import rclpy
from bleak import BleakClient, BleakScanner
from rclpy.node import Node
from sensor_msgs.msg import Imu

# WT901BLE "0x61" combined-output frame: 0x55 0x61, then 9 little-endian
# int16 raw values (accel xyz, gyro xyz, angle xyz), 20 bytes total.
FRAME_LENGTH = 20
FRAME_HEADER = 0x55
FRAME_FLAG_ACC_GYRO_ANGLE = 0x61

# Scale factors from the WitMotion register protocol: raw int16 spans the
# device's configured full-scale range (+/-16g, +/-2000 deg/s, +/-180 deg).
ACC_SCALE = 16.0 * 9.80665 / 32768.0
GYRO_SCALE = 2000.0 * math.pi / 180.0 / 32768.0
ANGLE_SCALE = 180.0 * math.pi / 180.0 / 32768.0


def euler_to_quaternion(roll, pitch, yaw):
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class WitmotionBleImuNode(Node):

    def __init__(self):
        super().__init__('witmotion_ble_node')

        self.declare_parameter('device_name', 'WT901BLE68')
        self.declare_parameter('device_address', '')
        self.declare_parameter(
            'service_uuid', '0000ffe5-0000-1000-8000-00805f9a34fb')
        self.declare_parameter(
            'notify_uuid', '0000ffe4-0000-1000-8000-00805f9a34fb')
        self.declare_parameter('frame_id', 'imu_link')
        self.declare_parameter('topic', 'imu/data')
        self.declare_parameter('scan_timeout', 10.0)
        self.declare_parameter('reconnect_wait_seconds', 5.0)

        self.device_name = self.get_parameter('device_name').value
        self.device_address = self.get_parameter('device_address').value
        self.notify_uuid = self.get_parameter('notify_uuid').value
        self.frame_id = self.get_parameter('frame_id').value
        self.scan_timeout = float(self.get_parameter('scan_timeout').value)
        self.reconnect_wait_seconds = float(
            self.get_parameter('reconnect_wait_seconds').value)

        topic = self.get_parameter('topic').value
        self._publisher = self.create_publisher(Imu, topic, 10)

        self._rx_buffer = bytearray()
        self._running = True
        # bleak is asyncio-only; run its event loop on a background thread
        # so rclpy.spin() on the main thread stays free to serve callbacks.
        self._loop_thread = threading.Thread(
            target=lambda: asyncio.run(self._ble_main()), daemon=True)
        self._loop_thread.start()

    async def _discover_address(self):
        devices = await BleakScanner.discover(timeout=self.scan_timeout)
        for device in devices:
            if device.name and self.device_name in device.name:
                return device.address
        return None

    async def _ble_main(self):
        while self._running:
            try:
                address = self.device_address or await self._discover_address()
                if not address:
                    self.get_logger().warn(
                        f'No BLE device found matching "{self.device_name}"; '
                        'retrying...'
                    )
                    await asyncio.sleep(self.reconnect_wait_seconds)
                    continue

                async with BleakClient(address) as client:
                    await client.start_notify(self.notify_uuid, self._on_notify)
                    self.get_logger().info(f'Connected to IMU at {address}')
                    while self._running and client.is_connected:
                        await asyncio.sleep(1.0)
                    self.get_logger().warn('IMU disconnected')
            except Exception as exc:
                self.get_logger().warn(f'BLE error: {exc}')

            if self._running:
                await asyncio.sleep(self.reconnect_wait_seconds)

    def _on_notify(self, _sender, data: bytearray):
        self._rx_buffer += data
        while len(self._rx_buffer) >= FRAME_LENGTH:
            if self._rx_buffer[0] != FRAME_HEADER:
                del self._rx_buffer[0]
                continue
            if self._rx_buffer[1] != FRAME_FLAG_ACC_GYRO_ANGLE:
                del self._rx_buffer[0]
                continue
            frame = bytes(self._rx_buffer[:FRAME_LENGTH])
            del self._rx_buffer[:FRAME_LENGTH]
            self._publish_frame(frame)

    def _publish_frame(self, frame: bytes):
        raw = struct.unpack('<9h', frame[2:20])
        acc = [v * ACC_SCALE for v in raw[0:3]]
        gyro = [v * GYRO_SCALE for v in raw[3:6]]
        roll, pitch, yaw = (v * ANGLE_SCALE for v in raw[6:9])
        qx, qy, qz, qw = euler_to_quaternion(roll, pitch, yaw)

        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw
        msg.angular_velocity.x = gyro[0]
        msg.angular_velocity.y = gyro[1]
        msg.angular_velocity.z = gyro[2]
        msg.linear_acceleration.x = acc[0]
        msg.linear_acceleration.y = acc[1]
        msg.linear_acceleration.z = acc[2]
        self._publisher.publish(msg)

    def destroy_node(self):
        self._running = False
        self._loop_thread.join(timeout=2.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = WitmotionBleImuNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
