#!/usr/bin/env python3
"""
behavior_supervisor.py — 판단(감독) 노드  [방식 B: 통합]

아키텍처 3.6 / 5장 시나리오 / 6장 상태머신의 "두뇌".
controller.py(제어 노드, 모드 mux)와 짝을 이룬다.

  ┌─ 위치 매칭 (11.1)  : course.yaml 로드 → EKF 위치를 단조 제약으로 매칭,
  │                     앞쪽 preview 안의 event/mission 선행 감지
  ├─ 상태 머신 (11.3)  : 6.2 표대로 가/서/돌기 + 장애물 회피(AVOID) 판단
  └─ 브리지 발행 (11.1): 교차로 분기 궤적을 nav_msgs/Path 로 직접 발행

→ 출력이 controller 가 바로 먹는 형태라 글루 노드가 필요 없다.

  발행: /driving_mode   (String) : "LANE_FOLLOW"|"STOP_AT_LIGHT"|"TURN_BRIDGE"|"AVOID"|"WAYPOINT_FOLLOW"
        /bridge_path    (Path)   : 교차로 분기 궤적 (map 프레임)
        /waypoint_path  (Path)   : 차선 유실 시 fail-safe 용 코스 좌표 (map 프레임, WAYPOINT_FOLLOW 중 발행)
  구독: /odometry/filtered_map (Odometry)    — 코스 매칭용 위치 (절대 map 프레임 필수:
                                                odom 전용 필터는 세션마다 원점/방향이
                                                달라져 course.yaml과 어긋난다)
        /perception/sign  (String)           — red/green/left_arrow/right_arrow/none
        /obstacles        (MarkerArray)      — 회피 판단용 (base_link: x앞 y좌 가정)
        /lane/center      (Float64MultiArray)— data=[offset_m, steering_angle_deg, curvature_px, valid]
                                                (valid 는 index 3)

  ※ 차선 유실 fail-safe (신규): LANE_FOLLOW 중 /lane/center 의 valid 가 꺼지거나
     타임아웃되면, course.yaml 이 로드돼 있고 위치(odom)가 신선한 경우에 한해
     WAYPOINT_FOLLOW 로 전환한다. 이 상태에서는 현재 위치 앞쪽 코스 좌표를
     /waypoint_path 로 계속 발행해 controller 가 Pure Pursuit 로 완주하도록 한다.
     차선이 일정 시간(lane_regain_s) 이상 안정적으로 다시 잡히면 LANE_FOLLOW 로 복귀.
     (코스가 없으면 대안이 없으므로 기존처럼 controller 의 감속·정지 fail-safe 로 남는다.)

※ 문서 11.1+11.3 은 둘 다 /driving_mode 를 발행해 충돌한다. 이 노드는 그걸
  통합해 모드 발행 주체를 하나로 둔다. 방식 A(분리)로 쪼개는 법은 파일 끝 주석 참고.
"""

import math
import time
import yaml
import numpy as np
from scipy.spatial import cKDTree

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray

# 내부 상태(표시는 6.2와 동일). APPROACH 는 controller 에겐 LANE_FOLLOW 로 매핑.
# WAYPOINT_FOLLOW: 차선 유실 fail-safe (신규) — controller 에도 같은 이름으로 매핑.
LANE_FOLLOW, APPROACH, STOP, TURN_BRIDGE, AVOID, WAYPOINT_FOLLOW = \
    'LANE_FOLLOW', 'APPROACH', 'STOP', 'TURN_BRIDGE', 'AVOID', 'WAYPOINT_FOLLOW'


class BehaviorSupervisor(Node):
    def __init__(self):
        super().__init__('behavior_supervisor')

        # --- 파라미터 ---
        course   = self.declare_parameter('course_yaml', 'course.yaml').value
        self.win        = self.declare_parameter('search_window', 40).value      # 단조 탐색 창
        self.preview_m  = self.declare_parameter('preview_m', 4.0).value         # 이벤트 선행 감지(m)
        self.min_bridge = self.declare_parameter('min_bridge_s', 1.5).value      # 브리지 최소 유지(s)
        self.tick_hz    = self.declare_parameter('tick_hz', 10.0).value          # 상태머신 주기

        # 장애물 회피 판정
        self.obs_trigger  = self.declare_parameter('obstacle_trigger_m', 3.0).value  # 이 거리 안이면 회피
        self.lane_half    = self.declare_parameter('lane_halfwidth_m', 0.5).value     # 내 차로 폭/2
        self.obs_hold     = self.declare_parameter('obstacle_hold_s', 1.0).value      # 회피 히스테리시스

        self.timeout = self.declare_parameter('input_timeout', 0.4).value        # 입력 신선도(s)

        # 차선 유실 fail-safe(WAYPOINT_FOLLOW) 파라미터
        self.wp_horizon   = self.declare_parameter('waypoint_horizon_m', 6.0).value   # 앞쪽 발행 길이(m)
        self.lane_regain_s = self.declare_parameter('lane_regain_s', 0.5).value       # 차선 복귀 판정 유지시간(s)

        # --- 코스 로드 (없으면 반응형으로만 동작: LANE_FOLLOW + AVOID) ---
        self.pts = np.zeros((0, 2))
        self.events = []
        self.segments = []
        self.branch_tag = []
        self.branches = {}
        self.cumdist = np.zeros(0)
        self.tree = None
        self.last_idx = 0
        self.cur_idx = 0
        try:
            with open(course) as f:
                data = yaml.safe_load(f)
            wps = data['waypoints']
            self.pts = np.array([[w['x'], w['y']] for w in wps])
            self.events     = [w.get('event', 'none')     for w in wps]
            self.segments   = [w.get('segment', '')       for w in wps]
            self.branch_tag = [w.get('branch', 'straight') for w in wps]
            self.branches   = data.get('branches', {})
            self.tree = cKDTree(self.pts)
            seg = np.linalg.norm(np.diff(self.pts, axis=0), axis=1)
            self.cumdist = np.concatenate([[0.0], np.cumsum(seg)])
            self.get_logger().info(f'course 로드: {len(self.pts)} pts')
        except Exception as e:
            self.get_logger().warn(f'course.yaml 로드 실패 → 반응형 모드: {e}')

        # --- 런타임 상태 ---
        self.state = LANE_FOLLOW
        self.sign = 'none'
        self.lane_valid = True
        self.lane_valid_since = self.now_s()   # lane_valid 가 연속으로 True 였던 시작 시각
        self.mission = 'none'        # straight / left / right  (코스 fork 의 branch 에서 옴)
        self.branch_key = ''         # 브리지 궤적 키 (예: intersection_A_left)
        self.bridge_t0 = 0.0
        self.obstacle_ahead = False
        self.obstacle_side = 'left'
        self.t_odom = self.t_sign = self.t_obs_seen = self.t_lane = -1e9

        # --- I/O ---
        self.create_subscription(Odometry,          '/odometry/filtered_map', self.on_odom, 10)
        self.create_subscription(String,            '/perception/sign',   self.on_sign, 10)
        self.create_subscription(MarkerArray,       '/obstacles',         self.on_obs,  10)
        self.create_subscription(Float64MultiArray, '/lane/center',       self.on_lane, 10)
        self.mode_pub = self.create_publisher(String, '/driving_mode', 10)
        self.path_pub = self.create_publisher(Path,   '/bridge_path',  10)
        self.wp_path_pub = self.create_publisher(Path, '/waypoint_path', 10)  # 차선 유실 fail-safe 용

        self.create_timer(1.0 / self.tick_hz, self.tick)
        self.get_logger().info('behavior_supervisor(통합) 시작')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ----------------------- 콜백 -----------------------
    def on_sign(self, m):
        self.sign = m.data
        self.t_sign = self.now_s()

    def on_lane(self, m):
        # data = [offset_m, steering_angle_deg, curvature_px, valid]  (lane_detection.cpp 규약)
        d = list(m.data) + [0.0, 0.0, 0.0, 0.0]
        valid = d[3] > 0.5   # d[2]는 curvature_px, valid는 d[3]
        now = self.now_s()
        if valid and not self.lane_valid:
            self.lane_valid_since = now        # False -> True 로 전이한 시각 기록
        if not valid:
            self.lane_valid_since = None
        self.lane_valid = valid
        self.t_lane = now                      # 신선도 판정용

    def on_odom(self, m):
        # 코스가 있을 때만 단조 매칭(11.1) 수행
        if len(self.pts) == 0:
            self.t_odom = self.now_s()
            return
        xy = np.array([m.pose.pose.position.x, m.pose.pose.position.y])
        self.cur_idx = self.match_index(xy)
        self.t_odom = self.now_s()

    def on_obs(self, m):
        """가장 가까운 '내 차로 앞쪽' 장애물을 찾는다.
        markers.pose 는 x=전방 y=좌측(base_link 정렬) 가정 — 아니면 tf2 변환 필요."""
        nearest = None
        for mk in m.markers:
            if getattr(mk, 'action', 0) != 0:    # ADD(0) 만
                continue
            px, py = mk.pose.position.x, mk.pose.position.y
            if px <= 0.0 or px > self.obs_trigger:        # 뒤/먼 것 제외
                continue
            if abs(py) > self.lane_half:                  # 내 차로 밖 제외
                continue
            if nearest is None or px < nearest[0]:
                nearest = (px, py)
        if nearest is not None:
            self.obstacle_ahead = True
            self.obstacle_side = 'left' if nearest[1] > 0 else 'right'
            self.t_obs_seen = self.now_s()
        else:
            self.obstacle_ahead = False

    # ----------------------- 코스 매칭(11.1) -----------------------
    def match_index(self, xy):
        lo = self.last_idx
        hi = min(self.last_idx + self.win, len(self.pts))
        seg = self.pts[lo:hi]
        i = lo + int(np.argmin(np.sum((seg - xy) ** 2, axis=1)))
        self.last_idx = i                      # 진척도 갱신(뒤로 안 감)
        return i

    def upcoming(self, i):
        """앞쪽 preview_m 안의 첫 이벤트 → (event, mission, branch_key)."""
        if len(self.pts) == 0:
            return 'none', 'none', ''
        d0 = self.cumdist[i]
        for j in range(i, len(self.pts)):
            if self.cumdist[j] - d0 > self.preview_m:
                break
            ev = self.events[j]
            if ev != 'none':
                mission = self.branch_tag[j]            # left/straight/right
                key = f'{self.segments[j]}_{mission}'   # 예: intersection_A_left
                return ev, mission, key
        return 'none', 'none', ''

    # ----------------------- 브리지 발행(11.1) -----------------------
    def publish_bridge(self, key):
        pts = self.branches.get(key, [])
        if not pts:
            self.get_logger().warn(f'브리지 키 없음: {key}')
            return
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for p in pts:
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(p['x'])
            ps.pose.position.y = float(p['y'])
            yaw = float(p.get('yaw', 0.0))
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        self.path_pub.publish(path)

    def enter_bridge(self):
        self.publish_bridge(self.branch_key)   # controller 가 받아 Pure Pursuit
        self.bridge_t0 = time.time()
        self.state = TURN_BRIDGE

    # ----------------------- 차선 유실 fail-safe(WAYPOINT_FOLLOW) -----------------------
    def publish_waypoint_path(self, idx):
        """현재 위치(idx) 부터 waypoint_horizon_m 만큼의 코스 좌표를
        controller 의 Pure Pursuit 가 먹을 수 있게 /waypoint_path 로 발행."""
        if self.tree is None or len(self.pts) == 0:
            return
        d0 = self.cumdist[idx]
        end = idx
        for j in range(idx, len(self.pts)):
            end = j
            if self.cumdist[j] - d0 > self.wp_horizon:
                break
        pts = self.pts[idx:end + 1]
        if len(pts) < 2:                        # 코스 끝부분이라 점이 모자라면 마지막 두 점 사용
            pts = self.pts[max(0, len(self.pts) - 2):]
            if len(pts) < 2:
                return

        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        for k in range(len(pts)):
            x, y = pts[k]
            if k < len(pts) - 1:
                nx, ny = pts[k + 1]
                yaw = math.atan2(ny - y, nx - x)
            else:
                yaw = 0.0
            ps = PoseStamped()
            ps.header.frame_id = 'map'
            ps.pose.position.x = float(x)
            ps.pose.position.y = float(y)
            ps.pose.orientation.z = math.sin(yaw / 2.0)
            ps.pose.orientation.w = math.cos(yaw / 2.0)
            path.poses.append(ps)
        self.wp_path_pub.publish(path)

    # ----------------------- 상태 머신(6.2 + AVOID) -----------------------
    def tick(self):
        now = self.now_s()
        odom_ok = (now - self.t_odom) < self.timeout
        sign = self.sign if (now - self.t_sign) < self.timeout else 'none'
        obs = self.obstacle_ahead or ((now - self.t_obs_seen) < self.obs_hold)

        # 코스가 있고 위치가 신선하면 선행 이벤트 갱신
        ev, mission, key = ('none', 'none', '')
        if odom_ok:
            ev, mission, key = self.upcoming(self.cur_idx)

        # 차선 신선도까지 반영한 "지금 차선을 믿을 수 있는가" 판정.
        # (기존엔 self.lane_valid 만 봐서, /lane/center 가 아예 끊겨도
        #  마지막 값이 True 였으면 영영 유실을 못 알아챘음)
        lane_fresh = (now - self.t_lane) < self.timeout
        lane_ok = self.lane_valid and lane_fresh

        s = self.state
        is_turn = lambda mis: mis in ('left', 'right')

        if s == LANE_FOLLOW:
            if obs:
                self.state = AVOID
            elif ev in ('stop_line', 'fork'):       # ② 교차로 접근
                self.mission, self.branch_key = mission, key
                self.state = APPROACH
            elif not lane_ok:
                # 차선 유실: 코스가 있고 위치가 신선하면 waypoint fail-safe 로 전환.
                # 코스가 없으면 대안이 없어 LANE_FOLLOW 에 남고, controller 가
                # 알아서 감속·정지한다(기존 동작 유지).
                if self.tree is not None and odom_ok:
                    self.state = WAYPOINT_FOLLOW

        elif s == WAYPOINT_FOLLOW:                    # 차선 유실 fail-safe 로 완주 중
            if obs:
                self.state = AVOID
            elif ev in ('stop_line', 'fork'):         # 코스 추종 중에도 교차로는 그대로 처리
                self.mission, self.branch_key = mission, key
                self.state = APPROACH
            elif not odom_ok:
                # 위치까지 잃으면 코스 추종도 불가 → controller 의 정지 fail-safe 에 맡김
                pass
            elif lane_ok and self.lane_valid_since is not None \
                    and (now - self.lane_valid_since) > self.lane_regain_s:
                # 차선이 잠깐 튀는 게 아니라 lane_regain_s 이상 안정적으로 재확보됨
                self.state = LANE_FOLLOW

        elif s == APPROACH:                          # 차선 추종 유지 + 신호 감시
            if obs:
                self.state = AVOID
            elif is_turn(self.mission):
                if sign == f'{self.mission}_arrow':  # ④ 좌/우회전 시작
                    self.enter_bridge()
                elif sign in ('red', 'green'):       # 화살표 전이면 정지선 대기
                    self.state = STOP
            else:                                    # 직진 미션
                if sign == 'green':
                    self.state = LANE_FOLLOW          # 그냥 통과
                elif sign == 'red':
                    self.state = STOP

        elif s == STOP:                              # ③ 정지선 대기
            if is_turn(self.mission) and sign == f'{self.mission}_arrow':
                self.enter_bridge()
            elif self.mission == 'straight' and sign == 'green':
                self.state = LANE_FOLLOW

        elif s == TURN_BRIDGE:                        # ⑤ 새 차선 잡히면 복귀
            if self.lane_valid and (time.time() - self.bridge_t0) > self.min_bridge:
                self.state = LANE_FOLLOW

        elif s == AVOID:                              # 장애물 사라지면 복귀
            if not obs:
                self.state = LANE_FOLLOW

        # WAYPOINT_FOLLOW 유지 중이면 매 tick 현재 위치 앞쪽 코스를 계속 발행
        # (controller 의 Pure Pursuit 가 항상 신선한 목표점을 갖도록)
        if self.state == WAYPOINT_FOLLOW and odom_ok:
            self.publish_waypoint_path(self.cur_idx)

        # 내부 상태 → controller 의 모드로 매핑해 발행
        mode = {
            LANE_FOLLOW:     'LANE_FOLLOW',
            APPROACH:        'LANE_FOLLOW',   # 접근 중에도 핸들은 차선
            STOP:            'STOP_AT_LIGHT',
            TURN_BRIDGE:     'TURN_BRIDGE',
            AVOID:           'AVOID',
            WAYPOINT_FOLLOW: 'WAYPOINT_FOLLOW',
        }[self.state]
        self.mode_pub.publish(String(data=mode))


def main():
    rclpy.init()
    node = BehaviorSupervisor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

# ─────────────────────────────────────────────────────────────────────
# 방식 A(분리)로 쪼개고 싶으면:
#   1) 이 파일의 [위치 매칭 + upcoming + publish_bridge] 부분을
#      waypoint_manager 로 빼고, /course/upcoming("event|mission|key") 와
#      /bridge_path 를 발행하게 한다.
#   2) 남은 [상태 머신 tick] 은 그 토픽을 구독해 /driving_mode 만 발행.
#   동작은 동일, 노드만 2개로 나뉜다(디버깅·교체 ↑, 문서 6.3 권장).
# ─────────────────────────────────────────────────────────────────────