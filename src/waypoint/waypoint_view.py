#!/usr/bin/env python3
"""
course_viz — course.yaml 을 RViz2 에 표시하는 노드

waypoint_recorder 가 만든 course.yaml 을 읽어서:
  · 경로 선         : 흰색 LINE_STRIP  + nav_msgs/Path
  · 웨이포인트 점   : 작은 회색 구(SPHERE)
  · 진행방향(yaw)   : 점마다 화살표(arrow_stride 마다)
  · 이벤트 강조     : stop_line=빨강, fork=노랑, parking=파랑 큰 구 + 텍스트
  · 분기(branch)    : 분기별 색 다른 LINE_STRIP
를 MarkerArray 로 발행합니다. (전부 map 프레임)

RViz2 에서 보는 법
  1) rviz2 실행
  2) Fixed Frame = map
  3) Add → By topic → /course_markers (MarkerArray)
     (원하면 /course_path 도 Path 로 추가)

실행
  python3 course_viz.py --ros-args -p course_yaml:=course.yaml

의존성: rclpy, visualization_msgs, nav_msgs, geometry_msgs, std_msgs, pyyaml
"""

import os
import math
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA


# 이벤트별 색상(r, g, b, a)
EVENT_COLORS = {
    'stop_line': (1.0, 0.1, 0.1, 1.0),   # 빨강
    'fork':      (1.0, 0.9, 0.1, 1.0),   # 노랑
    'parking':   (0.2, 0.4, 1.0, 1.0),   # 파랑
}
# 분기 궤적용 색 팔레트(순환)
BRANCH_PALETTE = [
    (0.2, 0.8, 0.4, 1.0),  # 초록
    (0.9, 0.5, 0.1, 1.0),  # 주황
    (0.7, 0.3, 0.9, 1.0),  # 보라
    (0.1, 0.8, 0.9, 1.0),  # 청록
]


def rgba(c):
    m = ColorRGBA()
    m.r, m.g, m.b, m.a = (float(c[0]), float(c[1]), float(c[2]), float(c[3]))
    return m


def yaw_to_quat(yaw):
    q = Quaternion()
    q.z = math.sin(yaw / 2.0)
    q.w = math.cos(yaw / 2.0)
    return q


class CourseViz(Node):
    def __init__(self):
        super().__init__('course_viz')

        path = self.declare_parameter('course_yaml', 'course.yaml').value
        path = os.path.abspath(os.path.expanduser(path))
        self.arrow_stride = self.declare_parameter('arrow_stride', 3).value  # yaw 화살표 간격
        self.show_segment = self.declare_parameter('show_segment_labels', False).value

        if not os.path.exists(path):
            self.get_logger().error(f"course.yaml 을 찾을 수 없습니다: '{path}'")
            raise FileNotFoundError(path)

        with open(path) as f:
            self.data = yaml.safe_load(f)

        self.frame = self.data.get('frame_id', 'map')
        self.wps = self.data.get('waypoints', [])
        self.branches = self.data.get('branches', {}) or {}
        self.get_logger().info(
            f"로드: '{path}'  (frame={self.frame}, waypoints={len(self.wps)}, "
            f"branches={len(self.branches)})")

        # RViz가 늦게 붙어도 받도록 latched(transient_local) QoS
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.marker_pub = self.create_publisher(MarkerArray, '/course_markers', qos)
        self.path_pub = self.create_publisher(Path, '/course_path', qos)

        self.markers = self.build_markers()
        self.path_msg = self.build_path()

        self.publish_all()
        # latched지만, 토픽 재연결 대비해 2초마다 한 번 더 쏨
        self.create_timer(2.0, self.publish_all)

    # ------------------------------------------------------------------
    def publish_all(self):
        stamp = self.get_clock().now().to_msg()
        for m in self.markers.markers:
            m.header.stamp = stamp
        self.path_msg.header.stamp = stamp
        self.marker_pub.publish(self.markers)
        self.path_pub.publish(self.path_msg)

    # ------------------------------------------------------------------
    def _new_marker(self, mid, mtype, scale, color, ns='course'):
        m = Marker()
        m.header.frame_id = self.frame
        m.ns = ns
        m.id = mid
        m.type = mtype
        m.action = Marker.ADD
        m.scale.x, m.scale.y, m.scale.z = (float(scale[0]), float(scale[1]), float(scale[2]))
        m.color = rgba(color)
        m.pose.orientation.w = 1.0
        return m

    def build_markers(self):
        arr = MarkerArray()
        mid = 0

        # 0) 이전 마커 싹 지우기(파일 바뀌었을 때 잔상 방지)
        clear = Marker()
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        if not self.wps:
            return arr

        # 1) 경로 선 (흰색 LINE_STRIP)
        line = self._new_marker(mid, Marker.LINE_STRIP, (0.05, 0, 0), (1, 1, 1, 0.9))
        for w in self.wps:
            line.points.append(Point(x=float(w['x']), y=float(w['y']), z=0.0))
        arr.markers.append(line); mid += 1

        # 2) 웨이포인트 점 (회색 구) — SPHERE_LIST 한 개로
        pts = self._new_marker(mid, Marker.SPHERE_LIST, (0.10, 0.10, 0.10), (0.6, 0.6, 0.6, 0.9))
        for w in self.wps:
            pts.points.append(Point(x=float(w['x']), y=float(w['y']), z=0.0))
        arr.markers.append(pts); mid += 1

        # 3) yaw 화살표 (arrow_stride 마다)
        for k, w in enumerate(self.wps):
            if k % max(1, self.arrow_stride) != 0:
                continue
            a = self._new_marker(mid, Marker.ARROW, (0.30, 0.05, 0.05), (0.3, 0.7, 1.0, 0.8), ns='yaw')
            a.pose.position.x = float(w['x'])
            a.pose.position.y = float(w['y'])
            a.pose.orientation = yaw_to_quat(float(w.get('yaw', 0.0)))
            arr.markers.append(a); mid += 1

        # 4) 이벤트 강조 + 텍스트
        for w in self.wps:
            ev = w.get('event', 'none')
            if ev in EVENT_COLORS:
                c = EVENT_COLORS[ev]
                big = self._new_marker(mid, Marker.SPHERE, (0.30, 0.30, 0.30), c, ns='event')
                big.pose.position.x = float(w['x'])
                big.pose.position.y = float(w['y'])
                arr.markers.append(big); mid += 1

                txt = self._new_marker(mid, Marker.TEXT_VIEW_FACING, (0, 0, 0.25), (1, 1, 1, 1), ns='event_text')
                txt.pose.position.x = float(w['x'])
                txt.pose.position.y = float(w['y'])
                txt.pose.position.z = 0.35
                txt.text = ev
                arr.markers.append(txt); mid += 1

            # (옵션) 구간 라벨
            if self.show_segment and w.get('id', 0) % 20 == 0:
                seg = self._new_marker(mid, Marker.TEXT_VIEW_FACING, (0, 0, 0.20), (0.8, 0.8, 0.8, 0.8), ns='segment')
                seg.pose.position.x = float(w['x'])
                seg.pose.position.y = float(w['y'])
                seg.pose.position.z = 0.6
                seg.text = str(w.get('segment', ''))
                arr.markers.append(seg); mid += 1

        # 5) 분기 궤적 (분기별 색 다른 선)
        for bi, (key, bpts) in enumerate(self.branches.items()):
            if not bpts:
                continue
            color = BRANCH_PALETTE[bi % len(BRANCH_PALETTE)]
            bl = self._new_marker(mid, Marker.LINE_STRIP, (0.07, 0, 0), color, ns='branch')
            for p in bpts:
                bl.points.append(Point(x=float(p['x']), y=float(p['y']), z=0.05))
            arr.markers.append(bl); mid += 1

            # 분기 이름 텍스트(시작점 위)
            p0 = bpts[0]
            bt = self._new_marker(mid, Marker.TEXT_VIEW_FACING, (0, 0, 0.22), color, ns='branch_text')
            bt.pose.position.x = float(p0['x'])
            bt.pose.position.y = float(p0['y'])
            bt.pose.position.z = 0.5
            bt.text = key
            arr.markers.append(bt); mid += 1

        return arr

    def build_path(self):
        msg = Path()
        msg.header.frame_id = self.frame
        for w in self.wps:
            ps = PoseStamped()
            ps.header.frame_id = self.frame
            ps.pose.position.x = float(w['x'])
            ps.pose.position.y = float(w['y'])
            ps.pose.orientation = yaw_to_quat(float(w.get('yaw', 0.0)))
            msg.poses.append(ps)
        return msg


def main():
    rclpy.init()
    try:
        node = CourseViz()
        rclpy.spin(node)
    except (KeyboardInterrupt, FileNotFoundError):
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()