#!/usr/bin/env python3
"""차량을 mission.yaml의 라벨 위치로 순간이동시킵니다 -- 시뮬레이션 테스트용.

미션 후반의 스텝(평행 주차, 완주) 하나를 고치고 확인하려고 코스를 처음부터
주행할 이유가 없습니다. 이 서비스는 gz의 /world/<world>/set_pose를 불러 차량
모델을 라벨 위치로 옮깁니다.

model_service.py와 같은 방식입니다 -- 목적지는 요청 필드가 아니라 `label`
파라미터로 받고, 실행은 인자 없는 std_srvs/srv/Trigger입니다. hyper_rqt 패널의
콤보 박스가 이 파라미터를 set_parameters로 바꾼 다음 Trigger를 부릅니다.

좌표계: 라벨은 map 프레임인데 set_pose는 gz world 프레임입니다. 이 둘은 sim에서
일치합니다 -- vehicle.launch.py의 스폰 기본값(41.0866, -45.6842, yaw 1.64)이
코스 CSV의 웨이포인트 0번과 같은 점이기 때문입니다. 다른 데서 녹화한 코스를
쓰거나 스폰 위치를 옮기면 이 전제가 깨집니다.

EKF: 여기서는 아무것도 리셋하지 않습니다. ekf_local(엔코더+IMU)은 애초에 순간이동을
못 보므로 odom 프레임은 그대로 이어지고, ekf_global이 점프한 /gps/fix를 물고
map->odom을 몇 초에 걸쳐 끌어옵니다. 그동안은 차량의 map 상 위치가 실제와 다르므로
바로 미션을 시작하지 말고 /odometry/filtered_map이 자리를 잡을 때까지 기다리세요.
안 잡히면 ekf_local/set_pose와 ekf_global/set_pose(robot_localization/srv/SetPose)를
같이 불러 주는 수밖에 없습니다.

속도는 건드리지 않습니다. gz에는 set_pose만 있고 속도를 0으로 만드는 서비스가
없어서, 달리는 중에 부르면 옮겨진 자리에서 그대로 굴러갑니다. 세워 놓고 부르세요.
"""
import csv
import math
import os
import subprocess

import rclpy
import yaml
from rclpy.node import Node
from std_srvs.srv import Trigger

DEFAULT_MISSION_YAML = os.path.join(
    os.path.expanduser('~'), 'HYPER', 'src', 'planning', 'hyper_planner',
    'config', 'mission.yaml')
DEFAULT_WAYPOINT_CSV = os.path.join(
    os.path.expanduser('~'), 'HYPER', 'src', 'planning', 'hyper_waypoint',
    'waypoints', 'sim.csv')


class TeleportService(Node):

    def __init__(self):
        super().__init__('teleport_service')
        self.declare_parameter('world_name', 'course_world')
        self.declare_parameter('model_name', 'ackermann_steering_vehicle')
        self.declare_parameter('mission_yaml', DEFAULT_MISSION_YAML)
        self.declare_parameter('waypoint_csv', DEFAULT_WAYPOINT_CSV)
        self.declare_parameter('label', '')
        # 스폰 높이와 같게 둡니다. 바닥에 박아 넣으면 물리 엔진이 튕겨 냅니다.
        self.declare_parameter('z', 0.36)
        # 라벨보다 이만큼 코스를 따라 이동한 지점에 놓습니다. 음수 = 라벨 앞쪽
        # (아직 도달하기 전). 스텝 하나를 처음부터 보려면 -10 쯤이 편합니다.
        self.declare_parameter('offset_m', 0.0)

        self.teleport_srv = self.create_service(Trigger, '~/teleport', self.handle_teleport)

        try:
            labels = sorted(self._load_labels())
        except Exception as exc:  # noqa: BLE001 -- 노드 시작 시 경고로만
            self.get_logger().warn(f'mission.yaml을 아직 못 읽었습니다: {exc}')
        else:
            self.get_logger().info(f"라벨 {len(labels)}개: {', '.join(labels)}")

    # ------------------------------------------------------------------ 데이터

    def _load_labels(self):
        path = self.get_parameter('mission_yaml').value
        with open(path) as handle:
            mission = yaml.safe_load(handle) or {}
        labels = mission.get('labels') or {}
        if not labels:
            raise ValueError(f'{path}에 labels가 없습니다')
        return labels

    def _load_waypoints(self):
        path = self.get_parameter('waypoint_csv').value
        points = []
        with open(path) as handle:
            for row in csv.DictReader(handle):
                try:
                    points.append((float(row['x']), float(row['y']), float(row['yaw'])))
                except (KeyError, TypeError, ValueError):
                    continue
        if not points:
            raise ValueError(f'{path}에서 x/y/yaw를 읽지 못했습니다')
        return points

    def _pose_for_label(self, label):
        """라벨 이름 -> (x, y, yaw).

        라벨에는 x/y만 있고 헤딩이 없습니다(코스를 다시 녹화해도 살아남게 하려고
        좌표만 저장합니다 -- mission.yaml 주석 참고). 그래서 헤딩은 CSV에서 가장
        가까운 웨이포인트의 yaw, 즉 녹화 당시 실제 차체 방향을 씁니다.
        """
        labels = self._load_labels()
        if label not in labels:
            raise ValueError(
                f"라벨 '{label}'이 없습니다. 가능한 값: {', '.join(sorted(labels))}")
        target = labels[label]
        tx, ty = float(target['x']), float(target['y'])

        points = self._load_waypoints()
        nearest = min(
            range(len(points)),
            key=lambda i: (points[i][0] - tx) ** 2 + (points[i][1] - ty) ** 2)

        offset = float(self.get_parameter('offset_m').value)
        index = self._walk(points, nearest, offset) if offset else nearest
        x, y, yaw = points[index]
        if not offset:
            # 오프셋이 없으면 위치는 라벨 좌표를 그대로 쓰고 헤딩만 빌려 옵니다
            # (라벨이 웨이포인트 사이에 있을 수 있으므로).
            x, y = tx, ty
        return x, y, yaw, index, nearest

    @staticmethod
    def _walk(points, start, offset):
        """start에서 코스를 따라 offset 미터만큼 걸어간 인덱스."""
        step = 1 if offset > 0 else -1
        remaining = abs(offset)
        index = start
        while 0 <= index + step < len(points):
            nxt = index + step
            remaining -= math.hypot(
                points[nxt][0] - points[index][0], points[nxt][1] - points[index][1])
            index = nxt
            if remaining <= 0.0:
                break
        return index

    # ------------------------------------------------------------------ 호출

    def handle_teleport(self, request, response):
        label = self.get_parameter('label').value
        if not label:
            response.success = False
            response.message = (
                "label 파라미터가 비어 있습니다. "
                "ros2 param set /teleport_service label <라벨> 로 정하세요")
            return response

        try:
            x, y, yaw, index, nearest = self._pose_for_label(label)
        except Exception as exc:  # noqa: BLE001 -- 설정 실수를 응답으로 돌려줍니다
            response.success = False
            response.message = str(exc)
            return response

        z = float(self.get_parameter('z').value)
        qx, qy, qz, qw = _yaw_to_quaternion(yaw)
        world_name = self.get_parameter('world_name').value
        model_name = self.get_parameter('model_name').value
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
            response.success = False
            response.message = f'failed to call {service}: {exc}'
            return response

        output = (result.stdout or '').strip()
        error = (result.stderr or '').strip()
        # `ign service`는 자기 호출이 타임아웃해도 종료 코드 0을 냅니다. 응답에
        # "data: true"가 들어 있는지까지 봐야 실제로 성공한 것입니다.
        ok = result.returncode == 0 and 'true' in output.lower()

        where = f'{label}'
        if index != nearest:
            where += f' (offset {self.get_parameter("offset_m").value:+.1f} m, wp #{index})'
        response.success = ok
        response.message = (
            f'{where} -> x={x:.2f} y={y:.2f} yaw={math.degrees(yaw):.1f}deg'
            if ok else (output or error or 'set_pose 실패'))
        if ok:
            self.get_logger().info(response.message)
        else:
            self.get_logger().error(f'{service}: {response.message}')
        return response


def _yaw_to_quaternion(yaw):
    return 0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5)


def main():
    rclpy.init()
    node = TeleportService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
