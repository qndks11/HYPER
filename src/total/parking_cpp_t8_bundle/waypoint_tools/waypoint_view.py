#!/usr/bin/env python3
"""
waypoint_view — course.yaml 을 RViz2 에 표시하는 노드

waypoint_recorder 가 만든 course.yaml (events + paths 스키마) 을 읽어서:
  · 경로 선       : paths 항목별로 색이 다른 LINE_STRIP
  · 웨이포인트 점 : 작은 회색 구(SPHERE_LIST)
  · 진행방향(yaw) : 점마다 화살표(arrow_stride 마다)
  · 경로 이름     : 각 경로 시작점 위에 TEXT_VIEW_FACING
  · 이벤트 목록   : events 는 GPS(lat/lon) 기준이라 로컬 좌표가 없어
                    로그로만 요약 출력합니다.
를 MarkerArray 로 발행합니다.

주의: course.yaml 의 각 path 는 녹화 시작 시점 차량 pose 를 원점으로 하는
event_local 좌표계이며, path 끼리 공통 좌표계를 공유하지 않습니다.
여러 경로를 한 번에 표시하면 원점 부근에서 서로 겹쳐 보이는 게 정상입니다.
경로 하나만 보고 싶으면 path_name 파라미터를 지정하세요(이 경우에만
/course_path 로 nav_msgs/Path 도 함께 발행됩니다).

RViz2 에서 보는 법
  1) rviz2 실행
  2) Fixed Frame = event_local (course.yaml 의 frame_id, 기본값)
  3) Add → By topic → /course_markers (MarkerArray)
     (path_name 지정 시 /course_path 도 Path 로 추가 가능)

실행
  python3 waypoint_view.py --ros-args -p course_yaml:=course.yaml
  # 특정 경로만: -p path_name:=intersection_A_straight

의존성: rclpy, visualization_msgs, nav_msgs, geometry_msgs, std_msgs, pyyaml
"""

import os
import math
import yaml

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy

from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import ColorRGBA


# 경로별 색 팔레트(순환)
PATH_PALETTE = [
    (1.0, 1.0, 1.0, 0.9),  # 흰색
    (0.2, 0.8, 0.4, 1.0),  # 초록
    (0.9, 0.5, 0.1, 1.0),  # 주황
    (0.7, 0.3, 0.9, 1.0),  # 보라
    (0.1, 0.8, 0.9, 1.0),  # 청록
    (1.0, 0.9, 0.1, 1.0),  # 노랑
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
        super().__init__('waypoint_view')

        course_yaml = self.declare_parameter('course_yaml', 'course.yaml').value
        course_yaml = os.path.abspath(os.path.expanduser(course_yaml))
        self.arrow_stride = int(self.declare_parameter('arrow_stride', 3).value)
        self.path_name = self.declare_parameter('path_name', '').value

        if not os.path.exists(course_yaml):
            self.get_logger().error(f"course.yaml 을 찾을 수 없습니다: '{course_yaml}'")
            raise FileNotFoundError(course_yaml)

        with open(course_yaml) as f:
            self.data = yaml.safe_load(f) or {}

        self.frame = self.data.get('frame_id', 'map')
        self.events = self.data.get('events', {}) or {}
        all_paths = self.data.get('paths', {}) or {}

        if self.path_name:
            if self.path_name not in all_paths:
                self.get_logger().error(
                    f"path_name='{self.path_name}' 이 course.yaml 에 없습니다. "
                    f"사용 가능: {sorted(all_paths.keys())}")
                raise FileNotFoundError(self.path_name)
            self.paths = {self.path_name: all_paths[self.path_name]}
        else:
            self.paths = all_paths

        self.get_logger().info(
            f"로드: '{course_yaml}'  (frame={self.frame}, "
            f"events={len(self.events)}, paths={len(self.paths)}/{len(all_paths)})")
        for event_id, ev in self.events.items():
            self.get_logger().info(
                f"  event '{event_id}': type={ev.get('event_type')}, "
                f"paths={ev.get('paths', {})}")

        # RViz가 늦게 붙어도 받도록 latched(transient_local) QoS
        qos = QoSProfile(depth=1)
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self.marker_pub = self.create_publisher(MarkerArray, '/course_markers', qos)
        # 경로가 하나로 좁혀졌을 때만 nav_msgs/Path 발행(여러 경로는 좌표계가 달라 이어붙일 수 없음)
        self.path_pub = self.create_publisher(Path, '/course_path', qos) if len(self.paths) == 1 else None

        self.markers = self.build_markers()
        self.path_msg = self.build_path() if self.path_pub else None

        self.publish_all()
        # latched지만, 토픽 재연결 대비해 2초마다 한 번 더 쏨
        self.create_timer(2.0, self.publish_all)

    # ------------------------------------------------------------------
    def publish_all(self):
        stamp = self.get_clock().now().to_msg()
        for m in self.markers.markers:
            m.header.stamp = stamp
        self.marker_pub.publish(self.markers)
        if self.path_pub is not None:
            self.path_msg.header.stamp = stamp
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
        clear.header.frame_id = self.frame
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        for pi, (key, points) in enumerate(self.paths.items()):
            if not points:
                continue
            color = PATH_PALETTE[pi % len(PATH_PALETTE)]

            # 1) 경로 선
            line = self._new_marker(mid, Marker.LINE_STRIP, (0.05, 0, 0), color, ns=f'path/{key}')
            for p in points:
                line.points.append(Point(x=float(p['x']), y=float(p['y']), z=0.0))
            arr.markers.append(line); mid += 1

            # 2) 웨이포인트 점 (회색 구) — SPHERE_LIST 한 개로
            pts = self._new_marker(mid, Marker.SPHERE_LIST, (0.08, 0.08, 0.08), (0.6, 0.6, 0.6, 0.9), ns=f'points/{key}')
            for p in points:
                pts.points.append(Point(x=float(p['x']), y=float(p['y']), z=0.0))
            arr.markers.append(pts); mid += 1

            # 3) yaw 화살표 (arrow_stride 마다)
            for k, p in enumerate(points):
                if k % max(1, self.arrow_stride) != 0:
                    continue
                a = self._new_marker(mid, Marker.ARROW, (0.25, 0.04, 0.04), (0.3, 0.7, 1.0, 0.8), ns=f'yaw/{key}')
                a.pose.position.x = float(p['x'])
                a.pose.position.y = float(p['y'])
                a.pose.orientation = yaw_to_quat(float(p.get('yaw', 0.0)))
                arr.markers.append(a); mid += 1

            # 4) 경로 이름 텍스트(시작점 위)
            p0 = points[0]
            txt = self._new_marker(mid, Marker.TEXT_VIEW_FACING, (0, 0, 0.25), color, ns='path_text')
            txt.pose.position.x = float(p0['x'])
            txt.pose.position.y = float(p0['y'])
            txt.pose.position.z = 0.4
            txt.text = key
            arr.markers.append(txt); mid += 1

        return arr

    def build_path(self):
        _, points = next(iter(self.paths.items()))
        msg = Path()
        msg.header.frame_id = self.frame
        for p in points:
            ps = PoseStamped()
            ps.header.frame_id = self.frame
            ps.pose.position.x = float(p['x'])
            ps.pose.position.y = float(p['y'])
            ps.pose.orientation = yaw_to_quat(float(p.get('yaw', 0.0)))
            msg.poses.append(ps)
        return msg


def main():
    rclpy.init()
    node = None
    try:
        node = CourseViz()
        rclpy.spin(node)
    except (KeyboardInterrupt, FileNotFoundError, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
