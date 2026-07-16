#!/usr/bin/env python3
"""
behavior_supervisor_registered_event.py

등록된 GPS 이벤트 안에서만 정지선·신호등을 사용한다.

상태:
  LANE_FOLLOW
    - 등록 이벤트가 없으면 정지선 검출을 완전히 무시
    - 등록 GPS 이벤트 반경에 들어오면 APPROACH

  APPROACH
    - 분홍색 차선 추종을 유지하면서 감속
    - 해당 이벤트에서만 정지선 거리와 신호 사용
    - 정지선 기준 거리 도착:
        초록/허용 신호 -> TURN_BRIDGE
        빨강/미확인    -> STOP_AT_LIGHT

  TURN_BRIDGE
    - 현재 차량 자세에 저장된 상대경로를 배치해 /bridge_path 발행
    - 경로 끝 도착 후 LANE_FOLLOW

필수 토픽:
  /gps/fix                         sensor_msgs/NavSatFix
  /odometry/filtered_map           nav_msgs/Odometry
  /stopline/detection             std_msgs/Float64MultiArray
                                  data[0] = distance [m]
                                  data[1] = detected (1.0/0.0)
  /perception/sign                 std_msgs/String
  /mission/turn                    std_msgs/String
"""

import math
import os
import time
from pathlib import Path as FilePath

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path as RosPath
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64MultiArray, String


LANE_FOLLOW = 'LANE_FOLLOW'
APPROACH = 'APPROACH'
STOP_AT_LIGHT = 'STOP_AT_LIGHT'
TURN_BRIDGE = 'TURN_BRIDGE'


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z),
    )


def gps_valid(msg):
    return (
        msg.status.status >= 0
        and math.isfinite(msg.latitude)
        and math.isfinite(msg.longitude)
        and abs(msg.latitude) <= 90.0
        and abs(msg.longitude) <= 180.0
    )


def haversine_m(lat1, lon1, lat2, lon2):
    earth_radius = 6371000.0

    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)

    a = (
        math.sin(delta_phi / 2.0) ** 2
        + math.cos(phi1)
        * math.cos(phi2)
        * math.sin(delta_lambda / 2.0) ** 2
    )

    return 2.0 * earth_radius * math.atan2(
        math.sqrt(a),
        math.sqrt(max(0.0, 1.0 - a)),
    )


class BehaviorSupervisorRegisteredEvent(Node):
    def __init__(self):
        super().__init__('behavior_supervisor')

        yaml_file = str(
            self.declare_parameter(
                'event_path_yaml',
                'course.yaml',
            ).value
        )

        # 상대 경로이면 현재 터미널 위치가 아니라
        # behavior.py가 있는 폴더를 기준으로 course.yaml을 찾는다.
        yaml_candidate = FilePath(
            os.path.expanduser(yaml_file)
        )

        if yaml_candidate.is_absolute():
            self.yaml_path = yaml_candidate.resolve()
        else:
            script_directory = FilePath(__file__).resolve().parent
            self.yaml_path = (
                script_directory / yaml_candidate
            ).resolve()

        self.tick_hz = float(
            self.declare_parameter('tick_hz', 10.0).value
        )
        self.gps_timeout = float(
            self.declare_parameter('gps_timeout', 1.5).value
        )
        self.odom_timeout = float(
            self.declare_parameter('odom_timeout', 0.5).value
        )
        self.perception_timeout = float(
            self.declare_parameter(
                'perception_timeout',
                0.5,
            ).value
        )
        self.sign_timeout = float(
            self.declare_parameter('sign_timeout', 1.5).value
        )
        self.stop_distance_m = float(
            self.declare_parameter(
                'stop_line_trigger_distance_m',
                0.50,
            ).value
        )
        self.bridge_exit_tolerance_m = float(
            self.declare_parameter(
                'bridge_exit_tolerance_m',
                2.0,
            ).value
        )
        self.min_bridge_time_s = float(
            self.declare_parameter(
                'min_bridge_time_s',
                1.0,
            ).value
        )
        self.event_rearm_margin_m = float(
            self.declare_parameter(
                'event_rearm_margin_m',
                5.0,
            ).value
        )
        self.default_mission = str(
            self.declare_parameter(
                'default_mission',
                'straight',
            ).value
        ).lower()
        self.left_on_green_allowed = bool(
            self.declare_parameter(
                'left_on_green_allowed',
                False,
            ).value
        )
        self.right_on_green_allowed = bool(
            self.declare_parameter(
                'right_on_green_allowed',
                True,
            ).value
        )
        # 신호등이 이벤트 경로 방향을 직접 결정한다.
        # green       -> straight
        # left_arrow  -> left
        # right_arrow -> right
        self.arrow_signal_selects_path = bool(
            self.declare_parameter(
                'arrow_signal_selects_path',
                True,
            ).value
        )

        self.events = {}
        self.paths = {}
        self.load_yaml()

        self.state = LANE_FOLLOW
        self.active_event_id = None
        self.active_path_key = None
        self.transformed_path = None
        self.bridge_end = None
        self.bridge_started_monotonic = 0.0
        self.completed_events = set()

        self.current_gps = None
        self.current_pose = None
        self.t_gps = -1e9
        self.t_odom = -1e9

        self.stop_line_detected = False
        self.stop_line_distance = math.inf
        self.t_stopline = -1e9

        self.sign = 'none'
        self.t_sign = -1e9
        self.mission = self.default_mission

        self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.on_gps,
            10,
        )
        self.create_subscription(
            Odometry,
            '/odometry/filtered_map',
            self.on_odom,
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            '/stopline/detection',
            self.on_stopline,
            10,
        )
        self.create_subscription(
            String,
            '/perception/sign',
            self.on_sign,
            10,
        )
        self.create_subscription(
            String,
            '/mission/turn',
            self.on_mission,
            10,
        )
        self.create_subscription(
            String,
            '/event/cmd',
            self.on_event_command,
            10,
        )

        self.mode_pub = self.create_publisher(
            String,
            '/driving_mode',
            10,
        )
        self.bridge_pub = self.create_publisher(
            RosPath,
            '/bridge_path',
            10,
        )
        self.active_event_pub = self.create_publisher(
            String,
            '/active_event',
            10,
        )

        self.create_timer(
            1.0 / self.tick_hz,
            self.tick,
        )

        self.get_logger().info(
            '등록 GPS 이벤트 전용 판단부 시작\n'
            f'  yaml={self.yaml_path}\n'
            f'  events={list(self.events.keys())}\n'
            '  등록되지 않은 위치의 정지선은 무시\n'
            '  signal path rule: green=straight, '
            'left_arrow=left, right_arrow=right'
        )

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def load_yaml(self):
        try:
            with open(self.yaml_path, 'r', encoding='utf-8') as file:
                data = yaml.safe_load(file) or {}

            self.events = data.get('events') or {}
            self.paths = data.get('paths') or {}

            if not isinstance(self.events, dict):
                raise ValueError('events가 dictionary가 아닙니다.')

            if not isinstance(self.paths, dict):
                raise ValueError('paths가 dictionary가 아닙니다.')

        except Exception as exc:
            self.events = {}
            self.paths = {}
            self.get_logger().error(
                f'이벤트 YAML 로드 실패: {exc}'
            )

    def on_gps(self, msg):
        if not gps_valid(msg):
            return

        self.current_gps = (
            float(msg.latitude),
            float(msg.longitude),
        )
        self.t_gps = self.now_s()

    def on_odom(self, msg):
        self.current_pose = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
            yaw_from_quaternion(msg.pose.pose.orientation),
        )
        self.t_odom = self.now_s()

    def on_stopline(self, msg):
        # lane_detection.cpp publishes:
        #   data[0] = stop-line distance [m]
        #   data[1] = detection valid (1.0 / 0.0)
        if len(msg.data) < 2:
            return

        distance = float(msg.data[0])
        detected = float(msg.data[1]) >= 0.5

        self.stop_line_detected = detected

        if detected and math.isfinite(distance) and distance >= 0.0:
            self.stop_line_distance = distance
        else:
            self.stop_line_distance = math.inf

        self.t_stopline = self.now_s()

    def on_sign(self, msg):
        value = str(msg.data).strip().lower()

        if value not in {
            'none',
            'red',
            'green',
            'left_arrow',
            'right_arrow',
        }:
            value = 'none'

        self.sign = value
        self.t_sign = self.now_s()

    def on_mission(self, msg):
        value = str(msg.data).strip().lower()

        if value:
            self.mission = value
            self.get_logger().info(
                f'다음 교차로 방향: {self.mission}'
            )

    def on_event_command(self, msg):
        command = str(msg.data).strip()

        if command == 'reset':
            self.reset_to_lane_follow('수동 reset')
            return

        if command == 'reload':
            self.load_yaml()
            self.get_logger().info(
                f'YAML 다시 로드: {list(self.events.keys())}'
            )
            return

        parts = [part.strip() for part in command.split(':')]

        # GPS 없이 정지선·경로 시험용:
        # force_event:intersection_A
        if len(parts) == 2 and parts[0] == 'force_event':
            event_id = parts[1]

            if event_id not in self.events:
                self.get_logger().warn(
                    f"없는 이벤트: '{event_id}'"
                )
                return

            self.active_event_id = event_id
            self.set_state(APPROACH, '수동 이벤트 활성화')

    def gps_fresh(self, now):
        return (
            self.current_gps is not None
            and now - self.t_gps <= self.gps_timeout
        )

    def odom_fresh(self, now):
        return (
            self.current_pose is not None
            and now - self.t_odom <= self.odom_timeout
        )

    def stop_line_fresh(self, now):
        return (
            self.stop_line_detected
            and now - self.t_stopline <= self.perception_timeout
            and math.isfinite(self.stop_line_distance)
        )

    def current_sign(self, now):
        if now - self.t_sign <= self.sign_timeout:
            return self.sign

        return 'none'

    def set_state(self, new_state, reason=''):
        if new_state == self.state:
            return

        old_state = self.state
        self.state = new_state

        self.get_logger().info(
            f'STATE: {old_state} -> {new_state}'
            + (f' | {reason}' if reason else '')
        )

    def distance_to_event(self, event_id):
        if self.current_gps is None:
            return math.inf

        event = self.events.get(event_id) or {}

        try:
            latitude = float(event['latitude'])
            longitude = float(event['longitude'])
        except (KeyError, TypeError, ValueError):
            return math.inf

        return haversine_m(
            self.current_gps[0],
            self.current_gps[1],
            latitude,
            longitude,
        )

    def rearm_completed_events(self):
        for event_id in list(self.completed_events):
            event = self.events.get(event_id) or {}
            radius = float(event.get('approach_radius_m', 8.0))
            distance = self.distance_to_event(event_id)

            if distance > radius + self.event_rearm_margin_m:
                self.completed_events.remove(event_id)

    def find_registered_event(self):
        best_id = None
        best_distance = math.inf

        for event_id, event in self.events.items():
            if event_id in self.completed_events:
                continue

            radius = float(event.get('approach_radius_m', 8.0))
            distance = self.distance_to_event(event_id)

            if distance <= radius and distance < best_distance:
                best_id = event_id
                best_distance = distance

        return best_id, best_distance

    def effective_direction(self, now):
        """
        신호등 상태가 실제 이벤트 경로 방향을 직접 결정한다.

        green       -> straight
        left_arrow  -> left
        right_arrow -> right

        signal_required=False인 이벤트처럼 신호를 사용하지 않는 경우에만
        /mission/turn 값을 보조적으로 사용한다.
        """
        sign = self.current_sign(now)

        if sign == 'green':
            return 'straight'

        if sign == 'left_arrow':
            return 'left'

        if sign == 'right_arrow':
            return 'right'

        return self.mission

    def selected_path_key(self, now):
        if self.active_event_id is None:
            return None

        event = self.events.get(self.active_event_id) or {}
        direction_paths = event.get('paths') or {}
        direction = self.effective_direction(now)

        return direction_paths.get(direction)

    def signal_allows_entry(self, now):
        if self.active_event_id is None:
            return False

        event = self.events.get(self.active_event_id) or {}
        signal_required = bool(
            event.get('signal_required', True)
        )

        if not signal_required:
            return True

        sign = self.current_sign(now)
        direction = self.effective_direction(now)

        if sign == 'red' or sign == 'none':
            return False

        if direction == 'straight':
            return sign == 'green'

        if direction == 'left':
            return (
                sign == 'left_arrow'
                or (
                    self.left_on_green_allowed
                    and sign == 'green'
                )
            )

        if direction == 'right':
            return (
                sign == 'right_arrow'
                or (
                    self.right_on_green_allowed
                    and sign == 'green'
                )
            )

        return False

    def create_transformed_path(self, path_key):
        if self.current_pose is None:
            return None

        local_points = self.paths.get(path_key) or []

        if len(local_points) < 2:
            return None

        x0, y0, yaw0 = self.current_pose
        cos_yaw = math.cos(yaw0)
        sin_yaw = math.sin(yaw0)

        path = RosPath()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()

        for point in local_points:
            local_x = float(point['x'])
            local_y = float(point['y'])
            local_yaw = float(point.get('yaw', 0.0))

            map_x = x0 + cos_yaw * local_x - sin_yaw * local_y
            map_y = y0 + sin_yaw * local_x + cos_yaw * local_y
            map_yaw = normalize_angle(yaw0 + local_yaw)

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = map_x
            pose.pose.position.y = map_y
            pose.pose.orientation.z = math.sin(map_yaw / 2.0)
            pose.pose.orientation.w = math.cos(map_yaw / 2.0)
            path.poses.append(pose)

        return path

    def enter_bridge(self):
        now = self.now_s()

        if not self.odom_fresh(now):
            self.set_state(
                STOP_AT_LIGHT,
                'odometry가 신선하지 않음',
            )
            return

        direction = self.effective_direction(now)
        path_key = self.selected_path_key(now)

        if path_key is None:
            self.set_state(
                STOP_AT_LIGHT,
                f"'{self.active_event_id}'에 "
                f"'{direction}' 경로 없음",
            )
            return

        transformed = self.create_transformed_path(path_key)

        if transformed is None:
            self.set_state(
                STOP_AT_LIGHT,
                f"경로 변환 실패: '{path_key}'",
            )
            return

        self.active_path_key = path_key
        self.transformed_path = transformed

        end_position = transformed.poses[-1].pose.position
        self.bridge_end = (
            float(end_position.x),
            float(end_position.y),
        )
        self.bridge_started_monotonic = time.monotonic()

        self.bridge_pub.publish(transformed)
        self.set_state(
            TURN_BRIDGE,
            f'event={self.active_event_id}, '
            f'direction={direction}, path={path_key}',
        )

    def distance_to_bridge_end(self):
        if self.current_pose is None or self.bridge_end is None:
            return math.inf

        return math.hypot(
            self.current_pose[0] - self.bridge_end[0],
            self.current_pose[1] - self.bridge_end[1],
        )

    def finish_bridge(self):
        completed_event = self.active_event_id

        if completed_event is not None:
            self.completed_events.add(completed_event)

        self.reset_to_lane_follow(
            f'이벤트 완료: {completed_event}'
        )

    def reset_to_lane_follow(self, reason=''):
        self.active_event_id = None
        self.active_path_key = None
        self.transformed_path = None
        self.bridge_end = None
        self.set_state(LANE_FOLLOW, reason)

    def publish_bridge_path(self):
        if self.transformed_path is None:
            return

        stamp = self.get_clock().now().to_msg()
        self.transformed_path.header.stamp = stamp

        for pose in self.transformed_path.poses:
            pose.header.stamp = stamp

        self.bridge_pub.publish(self.transformed_path)

    def tick(self):
        now = self.now_s()

        if self.gps_fresh(now):
            self.rearm_completed_events()

        if self.state == LANE_FOLLOW:
            # 이 상태에서는 카메라 정지선 결과를 사용하지 않는다.
            if self.gps_fresh(now):
                event_id, distance = self.find_registered_event()

                if event_id is not None:
                    self.active_event_id = event_id
                    self.set_state(
                        APPROACH,
                        f'등록 이벤트 접근: {event_id}, '
                        f'distance={distance:.2f}m',
                    )

        elif self.state == APPROACH:
            # active_event가 존재하는 경우에만 정지선과 신호를 사용한다.
            if self.active_event_id is None:
                self.reset_to_lane_follow(
                    '활성 이벤트 없음'
                )

            elif (
                self.stop_line_fresh(now)
                and self.stop_line_distance <= self.stop_distance_m
            ):
                if self.signal_allows_entry(now):
                    # 초록/허용 신호: 멈추지 않고 상대경로로 전환
                    self.enter_bridge()
                else:
                    # 빨강/미확인: 정지선에서 정지
                    self.set_state(
                        STOP_AT_LIGHT,
                        f'sign={self.current_sign(now)}',
                    )

        elif self.state == STOP_AT_LIGHT:
            # 정지했던 현재 자세를 상대경로의 새로운 원점으로 사용한다.
            if self.signal_allows_entry(now):
                self.enter_bridge()

        elif self.state == TURN_BRIDGE:
            self.publish_bridge_path()

            elapsed = (
                time.monotonic()
                - self.bridge_started_monotonic
            )

            if (
                elapsed >= self.min_bridge_time_s
                and self.distance_to_bridge_end()
                <= self.bridge_exit_tolerance_m
            ):
                self.finish_bridge()

        self.mode_pub.publish(String(data=self.state))
        self.active_event_pub.publish(
            String(data=self.active_event_id or '')
        )


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorSupervisorRegisteredEvent()

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