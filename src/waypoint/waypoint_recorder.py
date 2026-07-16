#!/usr/bin/env python3
"""
event_path_recorder_gps.py

전체 코스가 아니라 등록된 이벤트 구간만 course.yaml에 저장한다.

저장 구조:
- 이벤트 기준 GPS: 어느 정지선/교차로인지 식별
- 이벤트 상대 경로: 정지선 출발 위치를 (0, 0, 0)으로 저장

명령:
  record_start:<event_id>:<direction>
  record_end
  mark_event:<event_id>
  set_radius:<event_id>:<meters>
  delete_path:<event_id>:<direction>
  delete_event:<event_id>
  list
  status
  save

예:
  record_start:intersection_A:straight
  record_start:intersection_A:left
  record_start:intersection_A:right
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


def process_path(raw_points, smooth_window, spacing):
    points = remove_duplicates(raw_points)

    if len(points) == 0:
        return []

    points = smooth_xy(points, smooth_window)
    points = resample_xy(points, spacing)
    points[:, 2] = calculate_yaws(points)

    # 기록 시작 자세를 항상 로컬 원점으로 고정한다.
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
            'GPS 이벤트 상대경로 Recorder 시작\n'
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

        if self.recording_path_key is None:
            return

        local = self.map_to_local(x, y, yaw)

        if self.last_local_xy is None:
            self.raw_points.append(local)
            self.last_local_xy = (local[0], local[1])
            return

        distance = math.hypot(
            local[0] - self.last_local_xy[0],
            local[1] - self.last_local_xy[1],
        )

        if distance >= self.min_dist:
            self.raw_points.append(local)
            self.last_local_xy = (local[0], local[1])

    def map_to_local(self, x, y, yaw):
        x0, y0, yaw0 = self.origin_pose
        dx = x - x0
        dy = y - y0

        cos_yaw = math.cos(yaw0)
        sin_yaw = math.sin(yaw0)

        return [
            cos_yaw * dx + sin_yaw * dy,
            -sin_yaw * dx + cos_yaw * dy,
            normalize_angle(yaw - yaw0),
        ]

    def on_command(self, msg):
        command = str(msg.data).strip()

        if command == 'record_end':
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

        if parts[0] == 'record_start' and len(parts) == 3:
            self.start_recording(parts[1], parts[2])
        elif parts[0] == 'mark_event' and len(parts) == 2:
            self.mark_event(parts[1])
        elif parts[0] == 'set_radius' and len(parts) == 3:
            self.set_radius(parts[1], parts[2])
        elif parts[0] == 'delete_path' and len(parts) == 3:
            self.delete_path(parts[1], parts[2])
        elif parts[0] == 'delete_event' and len(parts) == 2:
            self.delete_event(parts[1])
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
                'approach_radius_m': self.default_radius,
                'signal_required': True,
                'paths': {},
            },
        )
        event['latitude'] = latitude
        event['longitude'] = longitude
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

        self.get_logger().info(
            f"이벤트와 연결 경로 삭제: '{event_id}'"
        )

    def print_list(self):
        if not self.events:
            self.get_logger().info('저장된 이벤트가 없습니다.')
            return

        lines = ['저장 이벤트 목록']

        for event_id, event in self.events.items():
            lines.append(
                f"  {event_id}: "
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