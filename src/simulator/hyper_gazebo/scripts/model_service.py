#!/usr/bin/env python3
import math
import subprocess

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class ModelService(Node):
    def __init__(self):
        super().__init__('model_service')
        self.declare_parameter('world_name', 'course_world')
        self.declare_parameter('model_name', 'accel_pedestrian')
        self.declare_parameter('model_uri', 'model://person_standing')
        self.declare_parameter('static', True)
        self.declare_parameter('pose', [-37.77, 3.72, 0.0, 0.0, 0.0, 1.5708])

        self.spawn_srv = self.create_service(Trigger, '~/spawn', self.handle_spawn)
        self.remove_srv = self.create_service(Trigger, '~/remove', self.handle_remove)

    def handle_spawn(self, request, response):
        world_name = self.get_parameter('world_name').value
        model_name = self.get_parameter('model_name').value
        model_uri = self.get_parameter('model_uri').value
        static = self.get_parameter('static').value
        x, y, z, roll, pitch, yaw = self.get_parameter('pose').value
        qx, qy, qz, qw = _rpy_to_quaternion(roll, pitch, yaw)
        service = f'/world/{world_name}/create'

        request_str = (
            f'sdf_filename: "{model_uri}" '
            f'name: "{model_name}" '
            f'allow_renaming: false '
            f'is_static: {"true" if static else "false"} '
            f'pose: {{ '
            f'position: {{ x: {x} y: {y} z: {z} }} '
            f'orientation: {{ x: {qx} y: {qy} z: {qz} w: {qw} }} '
            f'}}'
        )

        try:
            result = subprocess.run(
                [
                    'ign', 'service', '-s', service,
                    '--reqtype', 'ignition.msgs.EntityFactory',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '2000',
                    '-r', request_str,
                ],
                capture_output=True, text=True, timeout=5,
            )
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            response.success = False
            response.message = f'failed to call {service}: {exc}'
            return response

        response.success = (result.returncode == 0)
        response.message = result.stdout.strip() or result.stderr.strip()
        return response

    def handle_remove(self, request, response):
        world_name = self.get_parameter('world_name').value
        model_name = self.get_parameter('model_name').value
        service = f'/world/{world_name}/remove'

        try:
            result = subprocess.run(
                [
                    'ign', 'service', '-s', service,
                    '--reqtype', 'ignition.msgs.Entity',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '2000',
                    '-r', f'name: "{model_name}" type: MODEL',
                ],
                capture_output=True, text=True, timeout=5,
            )
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            response.success = False
            response.message = f'failed to call {service}: {exc}'
            return response

        response.success = (result.returncode == 0)
        response.message = result.stdout.strip() or result.stderr.strip()
        return response


def _rpy_to_quaternion(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


def main():
    rclpy.init()
    node = ModelService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
