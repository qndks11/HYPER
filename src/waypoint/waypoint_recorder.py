#!/usr/bin/env python3
"""
event_path_recorder_gps.py

전체 코스가 아니라 등록된 이벤트 구간만 course.yaml에 저장한다.

저장 구조:
- 이벤트 기준 GPS: 어느 정지선/교차로인지 식별
- 이벤트 상대 경로: 정지선 출발 위치를 (0, 0, 0)으로 저장

명령:
  # 교차로 상대 경로
  record_start:<event_id>:<direction>
  record_end
  mark_event:<event_id>

  # 경사로 정지선 이벤트
  mark_stopline:<event_id>
  set_stop_duration:<event_id>:<seconds>

  # 가속 장애물 구간
  mark_accel_start:<event_id>
  mark_accel_end:<event_id>
  set_accel_radius:<event_id>:<start_m>:<end_m>
  set_accel_speed:<event_id>:<mps>
  set_obstacle_distance:<event_id>:<stop_m>:<clear_m>
  set_obstacle_width:<event_id>:<half_width_m>
  set_obstacle_clear_hold:<event_id>:<seconds>

  # 공통
  set_radius:<event_id>:<meters>
  delete_path:<event_id>:<direction>
  delete_event:<event_id>
  list
  status
  save

  # 주차 (T존/평행주차)
  mark_parking:<event_id>:<t_zone|parallel>
  record_start:<event_id>:entry       # 차선 없는 구간을 건너 주차 시작점까지 (전진만)
  record_end
  record_start:<event_id>:spot        # 주차 시작점 -> 실제 주차 위치 (전후진 가능)
  set_gear:<forward|reverse>          # spot 기록 중 기어 전환 시점마다 호출
  record_end
  set_hold_duration:<event_id>:<seconds>

예:
  record_start:intersection_A:straight
  mark_stopline:slope_A
  set_radius:slope_A:2.5
  set_stop_duration:slope_A:5.0
  mark_parking:parking_t1:t_zone
  record_start:parking_t1:entry
  record_end
  record_start:parking_t1:spot
  set_gear:reverse
  record_end
  save
"""

import math
import os
from pathlib import Path

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String


DIRECTIONS = {'straight', 'left', 'right'}
PARKING_STYLES = {'t_zone': 't_zone_parking', 'parallel': 'parallel_parking'}
PARKING_KEYS = {'entry', 'spot'}
GEARS = {'forward': 'forward', 'reverse': 'reverse', 'back': 'reverse'}


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


def remove_duplicates(points, epsilon=1e-6):
    """
    Python list와 NumPy 배열을 모두 안전하게 처리한다.

    기존의 `if not points:`는 NumPy 배열에서 사용할 수 없으므로
    배열로 변환한 뒤 size를 검사한다.
    """
    points_array = np.asarray(points, dtype=float)

    if points_array.size == 0:
        return np.zeros((0, 3), dtype=float)

    if points_array.ndim == 1:
        if points_array.shape[0] != 3:
            raise ValueError(
                '경로 점은 [x, y, yaw] 3개 값이어야 합니다.'
            )
        points_array = points_array.reshape(1, 3)

    if points_array.ndim != 2 or points_array.shape[1] < 3:
        raise ValueError(
            '경로 데이터는 N x 3 이상의 배열이어야 합니다.'
        )

    output = [points_array[0, :3].copy()]

    for point in points_array[1:]:
        point = point[:3]

        if np.linalg.norm(point[:2] - output[-1][:2]) > epsilon:
            output.append(point.copy())

    return np.asarray(output, dtype=float)


def smooth_xy(points, window):
    points = np.asarray(points, dtype=float)

    if len(points) < 3 or window <= 1:
        return points.copy()

    window = min(int(window), len(points))

    if window % 2 == 0:
        window -= 1

    if window <= 1:
        return points.copy()

    kernel = np.ones(window, dtype=float) / float(window)
    output = points.copy()
    output[:, 0] = np.convolve(points[:, 0], kernel, mode='same')
    output[:, 1] = np.convolve(points[:, 1], kernel, mode='same')

    # 이벤트 시작점과 끝점은 유지한다.
    output[0, :2] = points[0, :2]
    output[-1, :2] = points[-1, :2]
    return output


def resample_xy(points, spacing):
    points = remove_duplicates(points)

    if len(points) < 2 or spacing <= 0.0:
        return points

    segment_lengths = np.linalg.norm(
        np.diff(points[:, :2], axis=0),
        axis=1,
    )
    cumulative = np.concatenate(([0.0], np.cumsum(segment_lengths)))
    total = float(cumulative[-1])

    if total <= spacing:
        return points

    samples = np.arange(0.0, total, spacing)

    if len(samples) == 0 or samples[-1] < total:
        samples = np.append(samples, total)

    output = np.zeros((len(samples), 3), dtype=float)
    output[:, 0] = np.interp(samples, cumulative, points[:, 0])
    output[:, 1] = np.interp(samples, cumulative, points[:, 1])
    return output


def calculate_yaws(points):
    points = np.asarray(points, dtype=float)
    yaws = np.zeros(len(points), dtype=float)

    if len(points) < 2:
        return yaws

    for index in range(len(points)):
        if index == 0:
            delta = points[1, :2] - points[0, :2]
        elif index == len(points) - 1:
            delta = points[-1, :2] - points[-2, :2]
        else:
            delta = points[index + 1, :2] - points[index - 1, :2]

        if np.linalg.norm(delta) < 1e-9:
            yaws[index] = yaws[index - 1] if index > 0 else 0.0
        else:
            yaws[index] = math.atan2(delta[1], delta[0])

    return yaws


def process_path(raw_points, smooth_window, spacing, fix_origin=True):
    points = remove_duplicates(raw_points)

    if len(points) == 0:
        return []

    points = smooth_xy(points, smooth_window)
    points = resample_xy(points, spacing)
    points[:, 2] = calculate_yaws(points)

    if fix_origin:
        # 기록 시작 자세를 항상 로컬 원점으로 고정한다. 주차 spot_path의 두 번째 이후 기어
        # 구간에는 적용하지 않는다 -- 그 구간의 첫 점은 진짜 로컬 원점이 아니라 이전 구간이
        # 끝난 실제 위치이므로, 여기서 [0,0,0]으로 밀어버리면 구간 사이가 끊어진다.
        points[0, :] = [0.0, 0.0, 0.0]

    return [
        {
            'x': round(float(point[0]), 4),
            'y': round(float(point[1]), 4),
            'yaw': round(float(point[2]), 5),
        }
        for point in points
    ]


class EventPathRecorderGps(Node):
    def __init__(self):
        super().__init__('event_path_recorder')

        output = str(
            self.declare_parameter(
                'output',
                'course.yaml',
            ).value
        )

        # 상대 경로이면 현재 터미널 위치가 아니라
        # 이 recorder 파일이 있는 폴더를 기준으로 저장한다.
        output_candidate = Path(
            os.path.expanduser(output)
        )

        if output_candidate.is_absolute():
            self.output_path = output_candidate.resolve()
        else:
            script_directory = Path(__file__).resolve().parent
            self.output_path = (
                script_directory / output_candidate
            ).resolve()
        self.default_radius = float(
            self.declare_parameter(
                'default_approach_radius_m',
                2.5,
            ).value
        )
        self.min_dist = float(
            self.declare_parameter('min_dist', 0.15).value
        )
        self.resample_ds = float(
            self.declare_parameter('resample_ds', 0.15).value
        )
        self.smooth_window = int(
            self.declare_parameter('smooth_window', 5).value
        )
        self.sensor_timeout = float(
            self.declare_parameter('sensor_timeout', 1.0).value
        )

        self.events = {}
        self.paths = {}
        self.load_existing()

        self.current_odom = None
        self.current_gps = None
        self.t_odom = -1e9
        self.t_gps = -1e9

        self.recording_event = None
        self.recording_direction = None
        self.recording_path_key = None
        self.origin_pose = None
        self.raw_points = []
        self.last_local_xy = None

        # 주차 entry/spot 기록 상태. entry는 위 raw_points와 같은 단일 구간 방식이지만
        # event_type을 덮어쓰지 않기 위해 별도 경로로 처리한다. spot은 기어 전환마다
        # 새 구간을 시작하는 다중 구간 방식이라 상태를 통째로 분리했다.
        self.parking_recording_event = None
        self.parking_recording_key = None  # 'entry' or 'spot'
        self.parking_origin_pose = None
        self.parking_last_local_xy = None
        self.parking_raw_points = []
        self.parking_segments = []  # [{'gear': 'forward'|'reverse', 'points': [[x,y,yaw], ...]}]
        self.parking_current_gear = 'forward'

        self.create_subscription(
            Odometry,
            '/odometry/filtered_map',
            self.on_odom,
            10,
        )
        self.create_subscription(
            NavSatFix,
            '/gps/fix',
            self.on_gps,
            10,
        )
        self.create_subscription(
            String,
            '/waypoint/cmd',
            self.on_command,
            10,
        )

        self.get_logger().info(
            '교차로/경사로 정지선 이벤트 Recorder 시작\n'
            f'  output={self.output_path}\n'
            f'  events={list(self.events.keys())}'
        )

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def load_existing(self):
        if not self.output_path.exists():
            return

        try:
            with open(self.output_path, 'r', encoding='utf-8') as file:
                data = yaml.safe_load(file) or {}

            self.events = data.get('events') or {}
            self.paths = data.get('paths') or {}

            if not isinstance(self.events, dict):
                self.events = {}

            if not isinstance(self.paths, dict):
                self.paths = {}

        except Exception as exc:
            self.get_logger().warn(
                f'기존 YAML 로드 실패: {exc}'
            )
            self.events = {}
            self.paths = {}

    def odom_fresh(self):
        return (
            self.current_odom is not None
            and self.now_s() - self.t_odom <= self.sensor_timeout
        )

    def gps_fresh(self):
        return (
            self.current_gps is not None
            and self.now_s() - self.t_gps <= self.sensor_timeout
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
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        yaw = yaw_from_quaternion(msg.pose.pose.orientation)

        self.current_odom = (x, y, yaw)
        self.t_odom = self.now_s()

        if self.recording_path_key is not None:
            local = self.map_to_local(x, y, yaw)

            if self.last_local_xy is None:
                self.raw_points.append(local)
                self.last_local_xy = (local[0], local[1])
            else:
                distance = math.hypot(
                    local[0] - self.last_local_xy[0],
                    local[1] - self.last_local_xy[1],
                )

                if distance >= self.min_dist:
                    self.raw_points.append(local)
                    self.last_local_xy = (local[0], local[1])

        if self.parking_recording_key is not None:
            local = self.pose_to_local(self.parking_origin_pose, x, y, yaw)

            if self.parking_last_local_xy is None:
                self._append_parking_point(local)
                self.parking_last_local_xy = (local[0], local[1])
            else:
                distance = math.hypot(
                    local[0] - self.parking_last_local_xy[0],
                    local[1] - self.parking_last_local_xy[1],
                )

                if distance >= self.min_dist:
                    self._append_parking_point(local)
                    self.parking_last_local_xy = (local[0], local[1])

    def _append_parking_point(self, local):
        if self.parking_recording_key == 'entry':
            self.parking_raw_points.append(local)
        else:
            self.parking_segments[-1]['points'].append(local)

    @staticmethod
    def pose_to_local(origin_pose, x, y, yaw):
        x0, y0, yaw0 = origin_pose
        dx = x - x0
        dy = y - y0

        cos_yaw = math.cos(yaw0)
        sin_yaw = math.sin(yaw0)

        return [
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
            normalize_angle(yaw - yaw0),
        ]

    def map_to_local(self, x, y, yaw):
        return self.pose_to_local(self.origin_pose, x, y, yaw)

    def on_command(self, msg):
        command = str(msg.data).strip()

        if command == 'record_end':
            if self.parking_recording_key is not None:
                self.end_recording_parking()
            else:
                self.end_recording()
            return

        if command == 'save':
            self.save()
            return

        if command == 'list':
            self.print_list()
            return

        if command == 'status':
            self.print_status()
            return

        parts = [part.strip() for part in command.split(':')]

        if not parts:
            return

        if parts[0] == 'record_start' and len(parts) == 3 and parts[2] in PARKING_KEYS:
            self.start_recording_parking(parts[1], parts[2])
        elif parts[0] == 'record_start' and len(parts) == 3:
            self.start_recording(parts[1], parts[2])
        elif parts[0] == 'mark_parking' and len(parts) == 3:
            self.mark_parking(parts[1], parts[2])
        elif parts[0] == 'set_gear' and len(parts) == 2:
            self.set_gear(parts[1])
        elif parts[0] == 'set_hold_duration' and len(parts) == 3:
            self.set_hold_duration(parts[1], parts[2])
        elif parts[0] == 'mark_event' and len(parts) == 2:
            self.mark_event(parts[1])
        elif parts[0] == 'set_radius' and len(parts) == 3:
            self.set_radius(parts[1], parts[2])
        elif parts[0] == 'delete_path' and len(parts) == 3:
            self.delete_path(parts[1], parts[2])
        elif parts[0] == 'delete_event' and len(parts) == 2:
            self.delete_event(parts[1])
        elif parts[0] in {'mark_stopline', 'mark_stop'} and len(parts) == 2:
            self.mark_stopline(parts[1])
        elif parts[0] == 'set_stop_duration' and len(parts) == 3:
            self.set_stop_duration(parts[1], parts[2])
        elif parts[0] in {'mark_accel_start', 'mark_accel_zone'} and len(parts) == 2:
            self.mark_accel_start(parts[1])
        elif parts[0] == 'mark_accel_end' and len(parts) == 2:
            self.mark_accel_end(parts[1])
        elif parts[0] == 'set_accel_radius' and len(parts) == 4:
            self.set_accel_radius(parts[1], parts[2], parts[3])
        elif parts[0] == 'set_accel_speed' and len(parts) == 3:
            self.set_accel_value(parts[1], 'target_speed_mps', parts[2], '목표 속도')
        elif parts[0] == 'set_obstacle_width' and len(parts) == 3:
            self.set_accel_value(parts[1], 'obstacle_half_width_m', parts[2], '감지 반폭')
        elif parts[0] == 'set_obstacle_clear_hold' and len(parts) == 3:
            self.set_accel_value(parts[1], 'obstacle_clear_hold_s', parts[2], '해제 유지 시간')
        elif parts[0] == 'set_obstacle_distance' and len(parts) == 4:
            self.set_obstacle_distance(parts[1], parts[2], parts[3])
        else:
            self.get_logger().warn(
                f"명령 형식 오류: '{command}'"
            )

    def ensure_event(self, event_id):
        if event_id in self.events:
            return True

        if not self.gps_fresh():
            self.get_logger().warn(
                '신선한 /gps/fix가 없어 새 이벤트를 만들 수 없습니다.'
            )
            return False

        latitude, longitude = self.current_gps

        self.events[event_id] = {
            'event_type': 'intersection',
            'latitude': latitude,
            'longitude': longitude,
            'approach_radius_m': self.default_radius,
            'signal_required': True,
            'paths': {},
        }

        self.get_logger().info(
            f"새 GPS 이벤트 등록: '{event_id}' "
            f'({latitude:.8f}, {longitude:.8f})'
        )
        return True

    def mark_event(self, event_id):
        if not self.gps_fresh():
            self.get_logger().warn(
                '신선한 /gps/fix가 없습니다.'
            )
            return

        latitude, longitude = self.current_gps
        event = self.events.setdefault(
            event_id,
            {
                'event_type': 'intersection',
                'approach_radius_m': self.default_radius,
                'signal_required': True,
                'paths': {},
            },
        )
        event['latitude'] = latitude
        event['longitude'] = longitude
        event.setdefault('event_type', 'intersection')
        event.setdefault('approach_radius_m', self.default_radius)
        event.setdefault('signal_required', True)
        event.setdefault('paths', {})

        self.get_logger().info(
            f"이벤트 기준 GPS 갱신: '{event_id}' "
            f'({latitude:.8f}, {longitude:.8f})'
        )

    def set_radius(self, event_id, radius_text):
        try:
            radius = float(radius_text)
        except ValueError:
            self.get_logger().warn('반경은 숫자여야 합니다.')
            return

        if radius <= 0.0:
            self.get_logger().warn('반경은 0보다 커야 합니다.')
            return

        if not self.ensure_event(event_id):
            return

        self.events[event_id]['approach_radius_m'] = radius
        self.get_logger().info(
            f"'{event_id}' 접근 반경={radius:.2f}m"
        )

    def mark_stopline(self, event_id):
        if not self.gps_fresh():
            self.get_logger().warn('신선한 /gps/fix가 없습니다.')
            return

        latitude, longitude = self.current_gps
        previous = self.events.get(event_id) or {}
        radius = float(previous.get('approach_radius_m', self.default_radius))
        duration = float(previous.get('stop_duration_s', 5.0))

        # 같은 ID에 연결된 교차로 경로가 있으면 경로 데이터도 정리한다.
        for path_key in (previous.get('paths') or {}).values():
            self.paths.pop(path_key, None)

        self.events[event_id] = {
            'event_type': 'hill_stop',
            'latitude': float(latitude),
            'longitude': float(longitude),
            'approach_radius_m': radius,
            'stop_duration_s': duration,
            'signal_required': False,
            'paths': {},
        }

        self.get_logger().info(
            f"경사로 정지선 이벤트 등록: '{event_id}' "
            f"({latitude:.8f}, {longitude:.8f}), "
            f"stop={duration:.1f}s"
        )


    def mark_accel_start(self, event_id):
        if not self.gps_fresh():
            self.get_logger().warn('신선한 /gps/fix가 없습니다.')
            return

        latitude, longitude = self.current_gps
        previous = self.events.get(event_id) or {}
        for path_key in (previous.get('paths') or {}).values():
            self.paths.pop(path_key, None)

        previous_end = previous.get('end')
        self.events[event_id] = {
            'event_type': 'accel_obstacle',
            'start': {
                'latitude': float(latitude),
                'longitude': float(longitude),
            },
            'end': previous_end,
            'start_radius_m': float(previous.get(
                'start_radius_m',
                previous.get('approach_radius_m', self.default_radius),
            )),
            'end_radius_m': float(previous.get('end_radius_m', self.default_radius)),
            'target_speed_mps': float(previous.get('target_speed_mps', 4.0)),
            'obstacle_stop_distance_m': float(previous.get('obstacle_stop_distance_m', 2.0)),
            'obstacle_clear_distance_m': float(previous.get('obstacle_clear_distance_m', 2.5)),
            'obstacle_half_width_m': float(previous.get('obstacle_half_width_m', 0.55)),
            'obstacle_clear_hold_s': float(previous.get('obstacle_clear_hold_s', 1.0)),
            'signal_required': False,
            'paths': {},
        }
        if self.events[event_id]['end'] is None:
            self.events[event_id].pop('end')

        self.get_logger().info(
            f"가속 장애물 구간 시작점 등록: '{event_id}' "
            f"({latitude:.8f}, {longitude:.8f})"
        )

    def mark_accel_end(self, event_id):
        if not self.gps_fresh():
            self.get_logger().warn('신선한 /gps/fix가 없습니다.')
            return

        event = self.events.get(event_id)
        if not event or event.get('event_type') != 'accel_obstacle':
            self.get_logger().warn(
                f"'{event_id}' 시작점이 없습니다. mark_accel_start를 먼저 실행하세요."
            )
            return

        latitude, longitude = self.current_gps
        event['end'] = {
            'latitude': float(latitude),
            'longitude': float(longitude),
        }
        event.setdefault('end_radius_m', self.default_radius)
        self.get_logger().info(
            f"가속 장애물 구간 끝점 등록: '{event_id}' "
            f"({latitude:.8f}, {longitude:.8f})"
        )

    def set_accel_radius(self, event_id, start_text, end_text):
        try:
            start_radius = float(start_text)
            end_radius = float(end_text)
        except ValueError:
            self.get_logger().warn('시작/끝 반경은 숫자여야 합니다.')
            return

        if start_radius <= 0.0 or end_radius <= 0.0:
            self.get_logger().warn('시작/끝 반경은 0보다 커야 합니다.')
            return

        event = self.events.get(event_id)
        if not event or event.get('event_type') != 'accel_obstacle':
            self.get_logger().warn(
                f"'{event_id}'는 가속 장애물 구간이 아닙니다. mark_accel_start를 먼저 실행하세요."
            )
            return

        event['start_radius_m'] = start_radius
        event['end_radius_m'] = end_radius
        self.get_logger().info(
            f"'{event_id}' 시작 반경={start_radius:.2f}m, 끝 반경={end_radius:.2f}m"
        )

    def set_accel_value(self, event_id, key, value_text, label):
        try:
            value = float(value_text)
        except ValueError:
            self.get_logger().warn(f'{label}은 숫자여야 합니다.')
            return
        if value <= 0.0:
            self.get_logger().warn(f'{label}은 0보다 커야 합니다.')
            return
        event = self.events.get(event_id)
        if not event or event.get('event_type') != 'accel_obstacle':
            self.get_logger().warn(
                f"'{event_id}'는 가속 장애물 구간이 아닙니다. mark_accel_start를 먼저 실행하세요."
            )
            return
        event[key] = value
        self.get_logger().info(f"'{event_id}' {label}={value}")

    def set_obstacle_distance(self, event_id, stop_text, clear_text):
        try:
            stop = float(stop_text)
            clear = float(clear_text)
        except ValueError:
            self.get_logger().warn('정지/해제 거리는 숫자여야 합니다.')
            return
        if stop <= 0.0 or clear <= stop:
            self.get_logger().warn('정지 거리는 0보다 커야 하고 해제 거리는 정지 거리보다 커야 합니다.')
            return
        event = self.events.get(event_id)
        if not event or event.get('event_type') != 'accel_obstacle':
            self.get_logger().warn(
                f"'{event_id}'는 가속 장애물 구간이 아닙니다. mark_accel_start를 먼저 실행하세요."
            )
            return
        event['obstacle_stop_distance_m'] = stop
        event['obstacle_clear_distance_m'] = clear
        self.get_logger().info(
            f"'{event_id}' obstacle stop={stop:.2f}m, clear={clear:.2f}m"
        )

    def set_stop_duration(self, event_id, duration_text):
        try:
            duration = float(duration_text)
        except ValueError:
            self.get_logger().warn('정지 시간은 숫자여야 합니다.')
            return

        if duration <= 0.0:
            self.get_logger().warn('정지 시간은 0보다 커야 합니다.')
            return

        event = self.events.get(event_id)
        if not event:
            self.get_logger().warn(
                f"없는 이벤트: '{event_id}'. mark_stopline을 먼저 실행하세요."
            )
            return

        if event.get('event_type') != 'hill_stop':
            self.get_logger().warn(f"'{event_id}'는 경사로 정지 이벤트가 아닙니다.")
            return

        event['stop_duration_s'] = duration
        self.get_logger().info(
            f"'{event_id}' 정지 시간={duration:.2f}s"
        )

    def mark_parking(self, event_id, style_text):
        style = style_text.lower()
        if style not in PARKING_STYLES:
            self.get_logger().warn(
                f"주차 스타일은 {sorted(PARKING_STYLES)} 중 하나여야 합니다."
            )
            return

        if not self.gps_fresh():
            self.get_logger().warn('신선한 /gps/fix가 없습니다.')
            return

        latitude, longitude = self.current_gps
        previous = self.events.get(event_id) or {}

        event = {
            'event_type': PARKING_STYLES[style],
            'latitude': float(latitude),
            'longitude': float(longitude),
            'approach_radius_m': float(previous.get('approach_radius_m', self.default_radius)),
            'signal_required': False,
            'hold_duration_s': float(previous.get('hold_duration_s', 0.0)),
            'paths': {},
        }
        if previous.get('entry_path'):
            event['entry_path'] = previous['entry_path']
        if previous.get('spot_path'):
            event['spot_path'] = previous['spot_path']

        self.events[event_id] = event
        self.get_logger().info(
            f"주차 이벤트 등록: '{event_id}' style={style} "
            f'({latitude:.8f}, {longitude:.8f})'
        )

    def set_hold_duration(self, event_id, duration_text):
        try:
            duration = float(duration_text)
        except ValueError:
            self.get_logger().warn('정차 시간은 숫자여야 합니다.')
            return

        if duration < 0.0:
            self.get_logger().warn('정차 시간은 0 이상이어야 합니다.')
            return

        event = self.events.get(event_id)
        if not event or event.get('event_type') not in PARKING_STYLES.values():
            self.get_logger().warn(
                f"'{event_id}'는 주차 이벤트가 아닙니다. mark_parking을 먼저 실행하세요."
            )
            return

        event['hold_duration_s'] = duration
        self.get_logger().info(f"'{event_id}' 정차 시간={duration:.2f}s")

    def start_recording(self, event_id, direction):
        direction = direction.lower()

        if direction not in DIRECTIONS:
            self.get_logger().warn(
                f'방향은 {sorted(DIRECTIONS)} 중 하나여야 합니다.'
            )
            return

        if self.recording_path_key is not None:
            self.get_logger().warn(
                f"이미 '{self.recording_path_key}' 기록 중입니다."
            )
            return


        if not self.odom_fresh():
            self.get_logger().warn(
                '신선한 /odometry/filtered_map이 없습니다.'
            )
            return

        if not self.ensure_event(event_id):
            return

        path_key = f'{event_id}_{direction}'

        self.events[event_id]['event_type'] = 'intersection'
        self.events[event_id]['signal_required'] = True

        self.recording_event = event_id
        self.recording_direction = direction
        self.recording_path_key = path_key
        self.origin_pose = self.current_odom
        self.raw_points = [[0.0, 0.0, 0.0]]
        self.last_local_xy = (0.0, 0.0)

        self.get_logger().info(
            f"기록 시작: event='{event_id}', "
            f"direction='{direction}', path='{path_key}'\n"
            '이 위치가 실행 시 정지선 출발 기준점이 됩니다.'
        )

    def end_recording(self):
        if self.recording_path_key is None:
            self.get_logger().warn('기록 중인 경로가 없습니다.')
            return

        if self.odom_fresh():
            x, y, yaw = self.current_odom
            final_point = self.map_to_local(x, y, yaw)

            if self.last_local_xy is None or math.hypot(
                final_point[0] - self.last_local_xy[0],
                final_point[1] - self.last_local_xy[1],
            ) > 1e-3:
                self.raw_points.append(final_point)

        processed = process_path(
            self.raw_points,
            self.smooth_window,
            self.resample_ds,
        )

        event_id = self.recording_event
        direction = self.recording_direction
        path_key = self.recording_path_key

        if len(processed) < 2:
            self.get_logger().warn(
                f"'{path_key}' 점이 너무 적어 저장하지 않았습니다."
            )
        else:
            self.paths[path_key] = processed
            self.events[event_id].setdefault('paths', {})
            self.events[event_id]['paths'][direction] = path_key

            self.get_logger().info(
                f"기록 완료: '{path_key}', points={len(processed)}"
            )

        self.recording_event = None
        self.recording_direction = None
        self.recording_path_key = None
        self.origin_pose = None
        self.raw_points = []
        self.last_local_xy = None

    def start_recording_parking(self, event_id, key):
        if event_id not in self.events:
            self.get_logger().warn(
                f"'{event_id}'가 없습니다. mark_parking을 먼저 실행하세요."
            )
            return

        if self.events[event_id].get('event_type') not in PARKING_STYLES.values():
            self.get_logger().warn(
                f"'{event_id}'는 주차 이벤트가 아닙니다. mark_parking을 먼저 실행하세요."
            )
            return

        if self.recording_path_key is not None or self.parking_recording_key is not None:
            self.get_logger().warn('이미 다른 경로를 기록 중입니다.')
            return

        if not self.odom_fresh():
            self.get_logger().warn('신선한 /odometry/filtered_map이 없습니다.')
            return

        self.parking_recording_event = event_id
        self.parking_recording_key = key
        self.parking_origin_pose = self.current_odom
        self.parking_last_local_xy = (0.0, 0.0)

        if key == 'entry':
            self.parking_raw_points = [[0.0, 0.0, 0.0]]
            self.get_logger().info(
                f"주차 진입경로 기록 시작: event='{event_id}'\n"
                '이 위치가 실행 시 트리거 기준점이 됩니다 (차선 없는 구간만 기록, 전진 전용).'
            )
        else:
            # T존은 대회 규정상 기동 구간에서 전진을 안 쓰므로 처음부터 reverse로 시작한다
            # (매번 set_gear:reverse를 안 눌러도 되고, forward/reverse 시작점이 같은 위치에
            # 찍혀서 기어 판정이 애매해지는 것도 방지). 평행주차는 기존대로 forward로 시작.
            default_gear = 'reverse' if self.events[event_id]['event_type'] == 't_zone_parking' else 'forward'
            self.parking_current_gear = default_gear
            self.parking_segments = [{'gear': default_gear, 'points': [[0.0, 0.0, 0.0]]}]
            self.get_logger().info(
                f"주차 기동경로 기록 시작: event='{event_id}' (기본 기어={default_gear})\n"
                'set_gear:forward/reverse로 전후진 구간을 나누세요.'
            )

    def set_gear(self, gear_text):
        gear = GEARS.get(gear_text.lower())
        if gear is None:
            self.get_logger().warn('기어는 forward 또는 reverse 여야 합니다.')
            return

        if self.parking_recording_key != 'spot':
            self.get_logger().warn('spot_path 기록 중에만 set_gear를 사용할 수 있습니다.')
            return

        if gear == self.parking_current_gear:
            return

        if not self.odom_fresh():
            self.get_logger().warn('신선한 /odometry/filtered_map이 없어 구간을 나눌 수 없습니다.')
            return

        x, y, yaw = self.current_odom
        seed = self.pose_to_local(self.parking_origin_pose, x, y, yaw)
        self.parking_current_gear = gear
        self.parking_segments.append({'gear': gear, 'points': [seed]})
        self.parking_last_local_xy = (seed[0], seed[1])
        self.get_logger().info(f'기어 전환: {gear} (구간 {len(self.parking_segments)})')

    def end_recording_parking(self):
        if self.parking_recording_key is None:
            self.get_logger().warn('기록 중인 주차 경로가 없습니다.')
            return

        event_id = self.parking_recording_event
        key = self.parking_recording_key

        if self.odom_fresh():
            x, y, yaw = self.current_odom
            final_point = self.pose_to_local(self.parking_origin_pose, x, y, yaw)
            if key == 'entry':
                if math.hypot(
                    final_point[0] - self.parking_last_local_xy[0],
                    final_point[1] - self.parking_last_local_xy[1],
                ) > 1e-3:
                    self.parking_raw_points.append(final_point)
            else:
                last_points = self.parking_segments[-1]['points']
                if math.hypot(
                    final_point[0] - last_points[-1][0],
                    final_point[1] - last_points[-1][1],
                ) > 1e-3:
                    last_points.append(final_point)

        path_key = f'{event_id}_{key}'

        if key == 'entry':
            processed = process_path(self.parking_raw_points, self.smooth_window, self.resample_ds)
            if len(processed) < 2:
                self.get_logger().warn(f"'{path_key}' 점이 너무 적어 저장하지 않았습니다.")
            else:
                self.paths[path_key] = processed
                self.events[event_id]['entry_path'] = path_key
                self.get_logger().info(f"기록 완료: '{path_key}', points={len(processed)}")
        else:
            combined = []
            for index, segment in enumerate(self.parking_segments):
                processed = process_path(
                    segment['points'], self.smooth_window, self.resample_ds, fix_origin=(index == 0))
                for point in processed:
                    point['gear'] = segment['gear']
                    combined.append(point)
            if len(combined) < 2:
                self.get_logger().warn(f"'{path_key}' 점이 너무 적어 저장하지 않았습니다.")
            else:
                self.paths[path_key] = combined
                self.events[event_id]['spot_path'] = path_key
                self.get_logger().info(
                    f"기록 완료: '{path_key}', points={len(combined)}, "
                    f"segments={len(self.parking_segments)}"
                )

        self.parking_recording_event = None
        self.parking_recording_key = None
        self.parking_origin_pose = None
        self.parking_raw_points = []
        self.parking_segments = []
        self.parking_last_local_xy = None

    def delete_path(self, event_id, direction):
        event = self.events.get(event_id)

        if not event:
            self.get_logger().warn(f"없는 이벤트: '{event_id}'")
            return

        path_key = (event.get('paths') or {}).pop(direction, None)

        if path_key:
            self.paths.pop(path_key, None)
            self.get_logger().info(
                f"경로 삭제: '{path_key}'"
            )
        else:
            self.get_logger().warn(
                f"'{event_id}'에 '{direction}' 경로가 없습니다."
            )

    def delete_event(self, event_id):
        event = self.events.pop(event_id, None)

        if event is None:
            self.get_logger().warn(f"없는 이벤트: '{event_id}'")
            return

        for path_key in (event.get('paths') or {}).values():
            self.paths.pop(path_key, None)
        if event.get('entry_path'):
            self.paths.pop(event['entry_path'], None)
        if event.get('spot_path'):
            self.paths.pop(event['spot_path'], None)

        self.get_logger().info(
            f"이벤트와 연결 경로 삭제: '{event_id}'"
        )

    def print_list(self):
        if not self.events:
            self.get_logger().info('저장된 이벤트가 없습니다.')
            return

        lines = ['저장 이벤트 목록']

        for event_id, event in self.events.items():
            event_type = event.get('event_type', 'intersection')
            if event_type == 'hill_stop':
                lines.append(
                    f"  {event_id}: type=hill_stop, "
                    f"radius={event.get('approach_radius_m', self.default_radius)}m, "
                    f"stop={event.get('stop_duration_s', 5.0)}s"
                )
            elif event_type == 'accel_obstacle':
                lines.append(
                    f"  {event_id}: type=accel_obstacle, "
                    f"start={event.get('start')}, end={event.get('end')}, "
                    f"start_radius={event.get('start_radius_m', self.default_radius)}m, "
                    f"end_radius={event.get('end_radius_m', self.default_radius)}m, "
                    f"speed={event.get('target_speed_mps', 4.0)}m/s, "
                    f"stop={event.get('obstacle_stop_distance_m', 2.0)}m"
                )
            elif event_type in PARKING_STYLES.values():
                lines.append(
                    f"  {event_id}: type={event_type}, "
                    f"radius={event.get('approach_radius_m', self.default_radius)}m, "
                    f"entry_path={event.get('entry_path')}, "
                    f"spot_path={event.get('spot_path')}, "
                    f"hold={event.get('hold_duration_s', 0.0)}s"
                )
            else:
                lines.append(
                    f"  {event_id}: type=intersection, "
                    f"radius={event.get('approach_radius_m', self.default_radius)}m, "
                    f"paths={event.get('paths', {})}"
                )

        self.get_logger().info('\n'.join(lines))

    def print_status(self):
        self.get_logger().info(
            'Recorder 상태\n'
            f'  gps={self.current_gps}\n'
            f'  odom={self.current_odom}\n'
            f'  recording={self.recording_path_key}\n'
            f'  raw_points={len(self.raw_points)}\n'
            f'  events={list(self.events.keys())}\n'
            f'  output={self.output_path}'
        )

    def save(self):
        if self.recording_path_key is not None:
            self.get_logger().warn(
                'record_end 후 저장하세요.'
            )
            return


        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        temporary = Path(str(self.output_path) + '.tmp')

        data = {
            'frame_id': 'event_local',
            'coordinate_convention': {
                'x': 'forward',
                'y': 'left',
                'yaw': 'relative_radians',
            },
            'events': self.events,
            'paths': self.paths,
        }

        with open(temporary, 'w', encoding='utf-8') as file:
            yaml.safe_dump(
                data,
                file,
                allow_unicode=True,
                sort_keys=False,
            )

        os.replace(temporary, self.output_path)

        self.get_logger().info(
            f'YAML 저장 완료: {self.output_path}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = EventPathRecorderGps()
    keyboard_shutdown = False

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        keyboard_shutdown = True
    except Exception as exc:
        # 콜백 오류가 발생했을 때 finally에서 end_recording()을
        # 다시 호출해 같은 오류를 두 번 출력하지 않게 한다.
        node.get_logger().error(
            f'Recorder 실행 중 오류: {type(exc).__name__}: {exc}'
        )
    finally:
        try:
            # Ctrl+C로 정상 종료한 경우에만 진행 중인 경로를 자동 종료한다.
            if keyboard_shutdown and node.recording_path_key is not None:
                node.end_recording()


            node.save()
        except Exception as exc:
            node.get_logger().error(
                f'종료 처리 또는 YAML 저장 실패: '
                f'{type(exc).__name__}: {exc}'
            )
        finally:
            node.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()


if __name__ == '__main__':
    main()