#!/usr/bin/env python3
# =====================================================================
# /scan 전방 섹터 필터 -- 실차 라이다가 차 뒤쪽 로컬 코스트맵을 지우는 것을 막는다
#
# 왜 필요한가
# ------------
# 시뮬레이션의 라이다는 전방 180도만 봅니다(vehicle.xacro의 <horizontal> min/max_angle
# -pi/2..+pi/2). 그건 성능 타협이 아니라 "뒤쪽 코스트맵을 유지한다"는 설계입니다 --
# 뒤를 보는 빔이 아예 없으면 nav2 obstacle_layer가 차 뒤를 raytrace로 지울 방법이
# 없으므로, 전진하며 마킹한 셀이 rolling window(30x30 m)에서 밀려날 때까지 남습니다.
#
# 그런데 실차의 RPLidar는 360도를 다 훑어서 /scan에 그대로 싣습니다. 그래서 차가
# 지나온 콘/장애물이 뒤로 빠지는 즉시 뒤쪽 빔에 지워집니다. 두 가지 경로로 지워집니다:
#
#   1. 뒤가 비어 있으면 rplidar_ros가 그 빔을 inf로 냅니다. nav2_controller.yaml의
#      scan 소스는 inf_is_valid: true라 inf를 range_max - eps로 바꿔 정상 빔처럼
#      레이캐스팅하므로, raytrace_max_range(20 m)까지 전부 FREE가 됩니다.
#   2. obstacle_max_range(9 m)보다 멀리 뒤로 빠진 장애물은 마킹 대상이 아닌데
#      raytrace_max_range는 20 m라, 다시 마킹되지 못한 채 지워지기만 합니다.
#
# 이 노드는 그 차이를 없앱니다. scan_raw(360도)를 받아 [lower_angle, upper_angle]
# 밖의 빔을 NaN으로 지우고 scan으로 다시 냅니다.
#
# 왜 배열을 자르지 않고 NaN으로 덮는가
# --------------------------------------
# 1. 각도 규약에 안 휘둘립니다. 드라이버가 angle_min을 -pi로 내든 0으로 내든,
#    angle_increment의 부호가 어느 쪽이든, 각 빔의 각도를 실제로 계산해서 판정하므로
#    섹터가 인덱스 공간에서 두 토막으로 갈라져도 맞게 동작합니다(자르기는 한 덩어리를
#    전제합니다).
# 2. angle_min/angle_max/angle_increment가 그대로라 RViz나 다른 구독자가 보는
#    스캔의 기하가 변하지 않습니다.
#
# NaN이 맞는 값인 이유: laser_geometry가 NaN 빔을 포인트클라우드에서 통째로 버리므로
# 마킹도 레이캐스팅도 일어나지 않습니다 -- 정확히 "그 방향은 보지 않았다"는 뜻입니다.
# 0.0이나 inf로 덮으면 안 됩니다. nav2 obstacle_layer의 laserScanValidInfCallback은
# `!isfinite(r) && r > 0`인 빔만 range_max - eps로 바꾸는데, NaN은 `NaN > 0`이 false라
# 걸리지 않고 그대로 버려집니다(inf는 걸려서 레이캐스팅됩니다 -- 우리가 막으려는 것).
#
# 되돌리려면
# -----------
# rplidar.launch.py의 use_front_filter:=false면 드라이버가 /scan을 직접 내고 이
# 노드는 뜨지 않습니다(360도 원본을 보고 싶을 때).
# =====================================================================
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

NAN = float('nan')


def normalize(angle):
    """각도를 (-pi, pi]로 감습니다."""
    return math.atan2(math.sin(angle), math.cos(angle))


class ScanFrontFilter(Node):

    def __init__(self):
        super().__init__('scan_front_filter')

        # 기본값은 vehicle.xacro의 시뮬레이션 라이다 FOV와 같은 전방 180도입니다.
        # 둘은 반드시 같이 움직여야 합니다 -- 다르면 실차와 시뮬레이션의 코스트맵이
        # 다르게 차고 지워집니다.
        self.declare_parameter('lower_angle', -math.pi / 2.0)
        self.declare_parameter('upper_angle', math.pi / 2.0)

        self._lower = normalize(self.get_parameter('lower_angle').value)
        self._upper = normalize(self.get_parameter('upper_angle').value)

        # (angle_min, angle_increment, 빔 개수) -> 남길 인덱스 집합. 스캔마다 각도를
        # 다시 계산하지 않기 위한 캐시입니다. RPLidar는 angle_compensate: true라도
        # 회전 속도에 따라 빔 개수가 스캔마다 달라질 수 있어 키에 개수를 넣습니다.
        self._mask_key = None
        self._keep = ()

        self._pub = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self._sub = self.create_subscription(
            LaserScan, 'scan_raw', self._on_scan, qos_profile_sensor_data)

        self.get_logger().info(
            f'front sector [{math.degrees(self._lower):.1f}, '
            f'{math.degrees(self._upper):.1f}] deg: scan_raw -> scan')

    def _keep_mask(self, scan):
        """이 스캔 기하에서 살려 둘 빔의 bool 리스트."""
        key = (scan.angle_min, scan.angle_increment, len(scan.ranges))
        if key != self._mask_key:
            lower, upper = self._lower, self._upper
            # lower > upper면 섹터가 +-pi를 가로지르는 것으로 해석합니다
            # (예: 뒤쪽만 남기려면 lower=pi/2, upper=-pi/2).
            wraps = lower > upper
            keep = []
            for i in range(len(scan.ranges)):
                a = normalize(scan.angle_min + i * scan.angle_increment)
                inside = (a >= lower or a <= upper) if wraps else (lower <= a <= upper)
                keep.append(inside)
            self._mask_key = key
            self._keep = tuple(keep)
        return self._keep

    def _on_scan(self, scan):
        keep = self._keep_mask(scan)
        scan.ranges = [r if k else NAN for r, k in zip(scan.ranges, keep)]
        self._pub.publish(scan)


def main():
    rclpy.init()
    node = ScanFrontFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
