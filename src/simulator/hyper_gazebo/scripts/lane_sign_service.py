#!/usr/bin/env python3
"""두 개의 차로 제어 표지판 위치를 맞바꿉니다 -- 시뮬레이션 테스트용.

lane_sign_arrow_1(초록 아래 화살표 = 차로 개방)과 lane_sign_x_1(빨간 X = 차로
폐쇄)이 어느 차로 위에 걸리느냐에 따라 인지/판단이 달라져야 합니다. 코스를 다시
띄우지 않고 그 상황을 만들려고, 이 서비스는 두 모델의 현재 pose를 gz에서 읽어
서로 바꿔 set_pose 합니다.

model_service.py / teleport_service.py와 같은 방식입니다 -- 요청 필드가 없는
std_srvs/srv/Trigger 하나이고, 대상은 파라미터로 받습니다. 매번 부를 때마다
두 표지판이 자리를 맞바꾸므로, 두 번 부르면 원래대로 돌아옵니다.

현재 pose는 `ign model -m <name> -p`로 읽습니다(월드 파일의 값이 아니라 지금
시뮬레이터에 있는 값). 그래서 표지판을 GUI에서 끌어다 옮겨 놓았더라도 그
위치를 기준으로 맞바꿉니다.
"""
import math
import re
import subprocess

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class LaneSignService(Node):

    def __init__(self):
        super().__init__('lane_sign_service')
        self.declare_parameter('world_name', 'course_world')
        self.declare_parameter('model_a', 'lane_sign_arrow_1')
        self.declare_parameter('model_b', 'lane_sign_x_1')

        self.swap_srv = self.create_service(Trigger, '~/swap', self.handle_swap)

    # ------------------------------------------------------------------ 호출

    def handle_swap(self, request, response):
        world_name = self.get_parameter('world_name').value
        model_a = self.get_parameter('model_a').value
        model_b = self.get_parameter('model_b').value

        try:
            pose_a = self._current_pose(world_name, model_a)
            pose_b = self._current_pose(world_name, model_b)
        except Exception as exc:  # noqa: BLE001 -- 설정 실수를 응답으로 돌려줍니다
            response.success = False
            response.message = str(exc)
            return response

        # 위치(x, y, z)만 맞바꾸고 방향은 각자 그대로 둡니다 -- 표지판이 바라보는
        # 쪽은 어느 차로를 제어하느냐와 무관하게 유지되어야 하기 때문입니다.
        # 하나라도 실패하면 표지판이 겹칠 수 있으니 둘 다 보고합니다.
        ok_a, msg_a = self._set_pose(world_name, model_a, pose_b[:3] + pose_a[3:])
        ok_b, msg_b = self._set_pose(world_name, model_b, pose_a[:3] + pose_b[3:])

        response.success = ok_a and ok_b
        if response.success:
            response.message = (
                f'{model_a} <-> {model_b} : '
                f'({pose_a[0]:.2f}, {pose_a[1]:.2f}) <-> ({pose_b[0]:.2f}, {pose_b[1]:.2f})')
            self.get_logger().info(response.message)
        else:
            response.message = f'{model_a}: {msg_a} | {model_b}: {msg_b}'
            self.get_logger().error(response.message)
        return response

    # ------------------------------------------------------------------ gz

    def _current_pose(self, world_name, model_name):
        """`ign model -m <name> -p` 출력에서 (x, y, z, roll, pitch, yaw)."""
        try:
            result = subprocess.run(
                ['ign', 'model', '-m', model_name, '-p'],
                capture_output=True, text=True, timeout=5,
            )
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            raise RuntimeError(f'ign model 호출 실패: {exc}')

        numbers = re.findall(r'-?\d+\.\d+', result.stdout)
        # 첫 6개가 [x y z][roll pitch yaw]. 모델이 없으면 Pose 블록이 안 나옵니다.
        if len(numbers) < 6:
            raise RuntimeError(
                f"'{model_name}'의 pose를 못 읽었습니다 -- 월드에 있는 이름이 맞나요? "
                f'(출력: {result.stdout.strip() or result.stderr.strip()})')
        return tuple(float(n) for n in numbers[:6])

    def _set_pose(self, world_name, model_name, pose):
        x, y, z, roll, pitch, yaw = pose
        qx, qy, qz, qw = _rpy_to_quaternion(roll, pitch, yaw)
        service = f'/world/{world_name}/set_pose'
        request_str = (
            f'name: "{model_name}" '
            f'position: {{ x: {x} y: {y} z: {z} }} '
            f'orientation: {{ x: {qx} y: {qy} z: {qz} w: {qw} }}'
        )
        try:
            result = subprocess.run(
                [
                    'ign', 'service', '-s', service,
                    '--reqtype', 'ignition.msgs.Pose',
                    '--reptype', 'ignition.msgs.Boolean',
                    '--timeout', '2000',
                    '-r', request_str,
                ],
                capture_output=True, text=True, timeout=5,
            )
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            return False, f'failed to call {service}: {exc}'

        output = (result.stdout or '').strip()
        error = (result.stderr or '').strip()
        # `ign service`는 타임아웃해도 종료 코드 0을 냅니다 -- 응답 본문까지 봐야 합니다.
        ok = result.returncode == 0 and 'true' in output.lower()
        return ok, (output or error or 'set_pose 실패')


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
    node = LaneSignService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
