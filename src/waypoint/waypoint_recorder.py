#!/usr/bin/env python3
"""
waypoint_recorder — 연습 주행 때만 실행하는 웨이포인트 "기록" 노드

문서 4.2장의 6단계 절차를 구현합니다.
  1) 위치 스트림 수집   : EKF의 /odometry/filtered_map (x, y) [map 프레임, 미터]
                         (GPS까지 융합된 절대 좌표. odom 전용 EKF는 세션마다
                          원점/방향이 달라져서 다른 실행에서 기록한 course.yaml과
                          안 맞으므로 반드시 map 프레임을 써야 한다.)
  2) 거리 기반 다운샘플 : 직전 저장점에서 min_dist 이상 이동했을 때만 추가
  3) 진행 방향(yaw) 계산: 앞뒤 점 차분 + 이동평균 스무딩 (저장 시 일괄 계산)
  4) 후처리            : (x,y) 스무딩 → arc-length 등간격 재샘플링
  5) 의미 태깅         : 운전 중 토픽으로 segment/event/branch 마킹
  6) 출력             : course.yaml 저장 (11.1 waypoint_manager가 그대로 로드)

주행(녹화)을 멈추는 방법
  - Ctrl+C 로 종료하면 자동으로 후처리 후 course.yaml 을 저장합니다.
  - 또는 /waypoint/cmd 로 "save" 를 보내면 즉시 저장합니다.

마킹 인터페이스 (std_msgs/String 을 /waypoint/cmd 로 발행)
  ros2 topic pub --once /waypoint/cmd std_msgs/String "data: 'seg:straight_1'"
  ros2 topic pub --once /waypoint/cmd std_msgs/String "data: 'event:stop_line'"
  ros2 topic pub --once /waypoint/cmd std_msgs/String "data: 'event:fork'"
  ros2 topic pub --once /waypoint/cmd std_msgs/String "data: 'branch_start:intersection_A_left'"
  ros2 topic pub --once /waypoint/cmd std_msgs/String "data: 'branch_end'"
  ros2 topic pub --once /waypoint/cmd std_msgs/String "data: 'save'"

  - seg:<name>           : 지금 위치부터 구간 이름을 <name> 으로 전환
  - event:<tag>          : 지금 위치(현재 가장 가까운 점)에 이벤트 태그
                           (none / stop_line / fork / parking)
  - branch_start:<key>   : 교차로 분기 궤적 기록 시작 (메인 경로 일시정지)
  - branch_end           : 분기 궤적 기록 종료 (메인 경로로 복귀)
  - save                 : 즉시 후처리 후 저장

대안: 연습 주행을 rosbag2 로 녹화한 뒤, 이 노드 대신 오프라인 스크립트로
      /odometry/filtered_map 를 추출해 동일 포맷으로 생성해도 됩니다.

의존성: rclpy, nav_msgs, std_msgs, numpy
"""

import math
import numpy as np

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import String


# ----------------------------------------------------------------------------
# 각도 유틸: 래핑(±pi 경계) 안전한 이동평균 — sin/cos를 따로 평균
# ----------------------------------------------------------------------------
def smooth_angles(angles, win):
    if win <= 1 or len(angles) < 3:
        return np.asarray(angles, dtype=float)
    s = np.sin(angles)
    c = np.cos(angles)
    k = np.ones(win) / win
    s = np.convolve(s, k, mode='same')
    c = np.convolve(c, k, mode='same')
    return np.arctan2(s, c)


# ----------------------------------------------------------------------------
# 진행 방향(yaw) 계산: yaw[i] = atan2(y[i+1]-y[i-1], x[i+1]-x[i-1])
#   - 양 끝점은 전진/후진 차분으로 처리
# ----------------------------------------------------------------------------
def compute_yaws(pts):
    n = len(pts)
    yaws = np.zeros(n)
    if n < 2:
        return yaws
    for i in range(n):
        if i == 0:
            dx, dy = pts[1] - pts[0]
        elif i == n - 1:
            dx, dy = pts[-1] - pts[-2]
        else:
            dx, dy = pts[i + 1] - pts[i - 1]
        yaws[i] = math.atan2(dy, dx)
    return yaws


def moving_average_xy(pts, win):
    if win <= 1 or len(pts) < 3:
        return pts
    k = np.ones(win) / win
    x = np.convolve(pts[:, 0], k, mode='same')
    y = np.convolve(pts[:, 1], k, mode='same')
    out = np.stack([x, y], axis=1)
    # 끝점 2개는 평균 왜곡이 크므로 원본 유지(경로 시작/끝 보존)
    out[0] = pts[0]
    out[-1] = pts[-1]
    return out


def arclength_resample(pts, ds):
    """arc-length 기준 등간격 재샘플링 → lookahead 계산이 안정적이 됨(4.2-4)."""
    if len(pts) < 2:
        return pts
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    total = s[-1]
    if total < ds:
        return pts
    new_s = np.arange(0.0, total, ds)
    new_s = np.append(new_s, total)  # 마지막 점 포함
    x = np.interp(new_s, s, pts[:, 0])
    y = np.interp(new_s, s, pts[:, 1])
    return np.stack([x, y], axis=1)


class WaypointRecorder(Node):
    def __init__(self):
        super().__init__('waypoint_recorder')

        # --- 파라미터 ---
        self.out_path     = self.declare_parameter('output', 'course.yaml').value
        self.min_dist     = self.declare_parameter('min_dist', 0.2).value      # 4.2-2 다운샘플 간격
        self.resample_ds  = self.declare_parameter('resample_ds', 0.2).value   # 4.2-4 재샘플 간격
        self.smooth_win   = self.declare_parameter('smooth_window', 5).value   # (x,y) 이동평균 창
        self.yaw_smooth   = self.declare_parameter('yaw_smooth_window', 5).value
        self.do_resample  = self.declare_parameter('resample', True).value
        self.first_seg    = self.declare_parameter('first_segment', 'straight_1').value

        # --- 기록 버퍼 ---
        self.raw = []                 # 메인 경로 점 [[x,y], ...]
        self.last_saved = None        # 마지막으로 저장한 점(다운샘플 기준)
        self.cur_xy = None            # 실시간 현재 위치(마킹 기준)

        # 마킹 보관 (저장 시 최종 인덱스로 스냅)
        self.event_marks = []         # [(x, y, tag), ...]
        self.seg_anchors = [(0.0, 0.0, self.first_seg)]  # [(x, y, name), ...]; 시작 구간

        # 분기(branch) 기록 상태
        self.branch_key = None        # 현재 기록 중인 분기 키 (None이면 메인)
        self.branches = {}            # {key: [[x,y], ...]}
        self.branch_last = None

        self.saved = False            # 중복 저장 방지

        # --- 토픽 ---
        # GPS까지 융합된 절대(map) 프레임 사용. odom 전용 EKF(/odometry/filtered_odom)는
        # 세션(노드 재시작)마다 원점/방향이 달라져서, 다른 실행에서 기록한 course.yaml과
        # 좌표가 어긋난다(회전까지 어긋나면 나중에 주행 시 "엉뚱한 방향으로 가는" 증상 발생).
        self.create_subscription(Odometry, '/odometry/filtered_map', self.on_odom, 10)
        self.create_subscription(String, '/waypoint/cmd', self.on_cmd, 10)

        self.get_logger().info(
            f"기록 시작: min_dist={self.min_dist}m, 저장경로='{self.out_path}'. "
            f"종료(Ctrl+C) 또는 /waypoint/cmd 'save' 로 저장합니다.")

    # ------------------------------------------------------------------
    # 1) 위치 스트림 수집 + 2) 거리 기반 다운샘플링
    # ------------------------------------------------------------------
    def on_odom(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.cur_xy = (x, y)

        if self.branch_key is not None:
            # 분기 궤적 기록 중 → 별도 버퍼로
            if self._moved_enough(self.branch_last, x, y):
                self.branches[self.branch_key].append([x, y])
                self.branch_last = (x, y)
            return

        # 메인 경로
        if self._moved_enough(self.last_saved, x, y):
            self.raw.append([x, y])
            self.last_saved = (x, y)
            if len(self.raw) % 50 == 0:
                self.get_logger().info(f"수집 점 수: {len(self.raw)}")

    def _moved_enough(self, last, x, y):
        if last is None:
            return True
        dx, dy = x - last[0], y - last[1]
        return (dx * dx + dy * dy) >= (self.min_dist * self.min_dist)

    # ------------------------------------------------------------------
    # 5) 의미 태깅 / 제어 명령
    # ------------------------------------------------------------------
    def on_cmd(self, msg):
        cmd = msg.data.strip()

        if cmd == 'save':
            self.save()
            return

        if cmd == 'branch_end':
            if self.branch_key is not None:
                n = len(self.branches[self.branch_key])
                self.get_logger().info(f"분기 '{self.branch_key}' 기록 종료 ({n}점)")
                self.branch_key = None
                self.branch_last = None
            return

        if ':' not in cmd:
            self.get_logger().warn(f"알 수 없는 명령: '{cmd}'")
            return

        key, val = cmd.split(':', 1)
        val = val.strip()

        if key == 'event':
            if self.cur_xy is None:
                self.get_logger().warn("아직 위치 수신 전 — event 무시")
                return
            self.event_marks.append((self.cur_xy[0], self.cur_xy[1], val))
            self.get_logger().info(f"이벤트 태깅: {val} @ ({self.cur_xy[0]:.2f}, {self.cur_xy[1]:.2f})")

        elif key == 'seg':
            if self.cur_xy is None:
                return
            self.seg_anchors.append((self.cur_xy[0], self.cur_xy[1], val))
            self.get_logger().info(f"구간 전환: {val}")

        elif key == 'branch_start':
            self.branch_key = val
            self.branches.setdefault(val, [])
            self.branch_last = None
            self.get_logger().info(f"분기 '{val}' 기록 시작 — 메인 경로 일시정지")

        else:
            self.get_logger().warn(f"알 수 없는 명령: '{cmd}'")

    # ------------------------------------------------------------------
    # 3)+4) 후처리  6) course.yaml 출력
    # ------------------------------------------------------------------
    def save(self):
        if self.saved:
            return
        if len(self.raw) < 3:
            self.get_logger().warn(f"수집 점이 너무 적습니다({len(self.raw)}) — 저장 생략")
            return

        pts = np.array(self.raw, dtype=float)

        # 4) 후처리: (x,y) 스무딩 → arc-length 재샘플링
        pts = moving_average_xy(pts, self.smooth_win)
        if self.do_resample:
            pts = arclength_resample(pts, self.resample_ds)

        # 3) yaw 계산 + 스무딩
        yaws = compute_yaws(pts)
        yaws = smooth_angles(yaws, self.yaw_smooth)

        # 5) 의미 태깅을 "최종 점"에 스냅
        segments = self._assign_segments(pts)
        events = self._assign_events(pts)

        # 6) YAML 작성 (11.1 waypoint_manager 로드 포맷과 동일)
        self._write_yaml(pts, yaws, segments, events)
        self.saved = True
        self.get_logger().info(
            f"저장 완료 → '{self.out_path}'  (waypoints={len(pts)}, branches={len(self.branches)})")

    def _assign_segments(self, pts):
        """seg_anchors 를 가장 가까운 최종 인덱스에 스냅하고, 그 인덱스부터 구간명 적용."""
        names = [self.first_seg] * len(pts)
        # (인덱스, 이름)으로 변환 후 인덱스 순 정렬
        anchored = []
        for ax, ay, name in self.seg_anchors:
            idx = int(np.argmin(np.sum((pts - np.array([ax, ay])) ** 2, axis=1)))
            anchored.append((idx, name))
        anchored.sort(key=lambda t: t[0])
        for idx, name in anchored:
            for j in range(idx, len(pts)):
                names[j] = name
        return names

    def _assign_events(self, pts):
        events = ['none'] * len(pts)
        for ex, ey, tag in self.event_marks:
            idx = int(np.argmin(np.sum((pts - np.array([ex, ey])) ** 2, axis=1)))
            events[idx] = tag
        return events

    def _write_yaml(self, pts, yaws, segments, events):
        lines = []
        lines.append("frame_id: map")
        lines.append(f"resolution_m: {self.resample_ds:.2f}")
        lines.append("waypoints:")
        for i in range(len(pts)):
            x, y = pts[i]
            lines.append(
                f"  - {{id: {i}, x: {x:.2f}, y: {y:.2f}, yaw: {yaws[i]:.3f}, "
                f"segment: {segments[i]}, event: {events[i]}}}")

        if self.branches:
            lines.append("branches:")
            for key, bpts in self.branches.items():
                lines.append(f"  {key}:")
                if len(bpts) == 0:
                    continue
                bp = np.array(bpts, dtype=float)
                byaws = smooth_angles(compute_yaws(bp), self.yaw_smooth)
                for k in range(len(bp)):
                    bx, by = bp[k]
                    lines.append(f"    - {{x: {bx:.2f}, y: {by:.2f}, yaw: {byaws[k]:.3f}}}")

        with open(self.out_path, 'w') as f:
            f.write("\n".join(lines) + "\n")


def main():
    rclpy.init()
    node = WaypointRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 종료 시 자동 저장 (Ctrl+C 포함)
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()