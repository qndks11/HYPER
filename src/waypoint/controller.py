#!/usr/bin/env python3
"""
controller.py — 제어 노드 (모드 mux)

아키텍처 3.7 / 5장 시나리오 / 6장 상태머신의 "손발".
판단(감독) 노드가 정해준 /driving_mode 에 따라 추종 기준(reference)만 바꿔서
조향·속도 명령(/cmd, AckermannDriveStamped)을 생성한다.

  LANE_FOLLOW     : /lane/center(offset, heading_err) → Stanley 횡제어
  TURN_BRIDGE     : /bridge_path → Pure Pursuit (문서 11.2 로직 내장)
  STOP_AT_LIGHT   : 차선 유지한 채 속도 0 으로 감속·정지
  AVOID           : 차선추종 + 횡방향 바이어스 (스텁 — /obstacles 기하로 보강할 자리)
  WAYPOINT_FOLLOW : 차선 유실 시 감독 노드가 보내는 /waypoint_path(코스 좌표) 로
                    Pure Pursuit 추종 (fail-safe: 정지 대신 완주 우선)

설계 포인트
  1) 입력 콜백은 "최신값 캐싱"만 하고, 실제 제어는 고정 주기 타이머에서 1번 계산.
     → 센서(카메라/EKF) 주기가 흔들려도 /cmd 주기는 일정 (8장: 제어 20~50Hz).
  2) 조향 rate-limit + 속도 accel/decel-limit → 서보/구동에 부드러운 명령.
  3) 입력 워치독: odom/차선이 끊기면 보수적으로 감속·정지(fail-safe).

메시지 규약(문서 11.x 의 std_msgs 버전에 맞춤)
  /driving_mode  : std_msgs/String   ("LANE_FOLLOW" | "TURN_BRIDGE" | "STOP_AT_LIGHT" | "AVOID" | "WAYPOINT_FOLLOW")
  /lane/center   : std_msgs/Float64MultiArray  data=[offset_m, steering_angle_deg, curvature_px, valid]
                   (lane_detection.cpp 발행 규약. degree→radian 변환은 on_lane 에서 처리)
  /bridge_path   : nav_msgs/Path     (map 프레임, 교차로 회전용. behavior_supervisor 가 발행)
  /waypoint_path : nav_msgs/Path     (map 프레임, 차선 유실 fail-safe 용. behavior_supervisor 가 발행)
  /odometry/filtered_map : nav_msgs/Odometry (EKF+GPS 융합, 절대 map 프레임, pose=map / twist=base_link)
                          (odom 전용 필터는 세션마다 원점/방향이 달라져 course.yaml과
                           어긋나므로, waypoint 기반 기능(TURN_BRIDGE/WAYPOINT_FOLLOW)엔
                           반드시 이 절대 프레임을 써야 한다)
  /velocity        : std_msgs/Float64  (실제 차량 구동용 속도 명령, m/s)
  /steering_angle  : std_msgs/Float64  (실제 차량 구동용 조향각 명령, rad)
"""

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64MultiArray, Float64
from nav_msgs.msg import Odometry, Path
from ackermann_msgs.msg import AckermannDriveStamped


# ----------------------------- 작은 유틸 -----------------------------
def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v

def rate_limit(cur, target, max_step):
    """한 주기에 max_step 이상 못 바뀌게 (조향 급변 방지)."""
    return clamp(target, cur - max_step, cur + max_step)

def ramp(cur, target, up_step, down_step):
    """가속/감속 제한 (속도 명령용)."""
    if target > cur:
        return min(target, cur + up_step)
    return max(target, cur - down_step)

def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


# ----------------------------- 제어 노드 -----------------------------
class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # --- 차량/주기 파라미터 ---
        self.L          = self.declare_parameter('wheelbase', 0.33).value      # 축거(m)
        self.hz         = self.declare_parameter('control_hz', 40.0).value     # 제어 주기
        self.max_steer  = self.declare_parameter('max_steer', 0.5).value       # 조향 한계(rad, ≈28°)
        self.steer_rate = self.declare_parameter('steer_rate', 4.0).value      # 조향 변화율 한계(rad/s)

        # --- 속도 파라미터 ---
        self.cruise     = self.declare_parameter('cruise_speed', 1.5).value    # 평상시 목표(m/s)
        self.bridge_spd = self.declare_parameter('bridge_speed', 1.0).value    # 교차로 통과 속도
        self.wp_speed   = self.declare_parameter('waypoint_speed', 1.0).value   # 차선 유실 fail-safe 속도
        self.min_speed  = self.declare_parameter('min_speed', 0.5).value       # 코너 최저 속도
        self.accel      = self.declare_parameter('accel_limit', 2.0).value     # 가속 한계(m/s^2)
        self.decel      = self.declare_parameter('decel_limit', 4.0).value     # 감속 한계(m/s^2)
        self.curve_slow = self.declare_parameter('curve_slow_k', 1.2).value    # |조향|↑ → 감속 정도

        # --- Stanley(차선추종) 게인 ---
        self.k_stanley  = self.declare_parameter('stanley_k', 0.4).value       # 횡오차 게인
        self.v_soft     = self.declare_parameter('stanley_v_min', 0.5).value   # 저속 발산 방지(분모)
        # 부호는 차마다 다름. 벤치에서 바퀴 들고 offset/heading 한쪽씩 줘보며 맞춘다.
        self.sign_off   = self.declare_parameter('lane_offset_sign', 0.4).value   # +면 차가 우측 → 좌조향(+)
        self.sign_head  = self.declare_parameter('lane_heading_sign', 0.4).value  # +면 차가 좌향 → 우조향(-)
        self.avoid_bias = self.declare_parameter('avoid_offset_bias', 0.4).value  # 회피 시 횡 목표 이동(m)
        # 카메라/차선검출 프레임 노이즈로 조향이 흔들리는 것(꿀렁거림) 방지용 저역통과 필터.
        # 0에 가까울수록 필터 세짐(반응 느려짐), 1이면 필터 없음(원본 그대로).
        self.lane_alpha = self.declare_parameter('lane_filter_alpha', 0.3).value

        # --- Pure Pursuit(브리지) ---
        self.Ld = self.declare_parameter('lookahead', 1.0).value               # lookahead 거리(m)

        # --- 워치독 ---
        self.timeout = self.declare_parameter('input_timeout', 0.3).value      # 입력 정지 판정(s)

        # --- 캐시 상태 ---
        self.mode = 'LANE_FOLLOW'
        self.lane_off = 0.0
        self.lane_head = 0.0
        self.lane_valid = False
        self.lane_init = False   # 첫 유효 프레임 전엔 필터 초기화 필요
        self.x = self.y = self.yaw = 0.0
        self.v = 0.0
        self.bridge = None                      # [[x,y], ...] (map) — TURN_BRIDGE(교차로)용
        self.wp_path = None                     # [[x,y], ...] (map) — WAYPOINT_FOLLOW(차선유실)용

        self.cmd_delta = 0.0                    # 직전 출력(rate-limit 기준)
        self.cmd_speed = 0.0
        self.last_t = self.now_s()
        self.t_odom = self.t_lane = -1e9        # 마지막 수신 시각

        # --- I/O ---  (QoS: 예제 스택과 맞추려 depth 10 기본.
        #     카메라발 고주기 토픽을 best-effort 로 쓰면 SensorDataQoS 로 바꿀 것)
        self.create_subscription(String,            '/driving_mode',      self.on_mode,  10)
        self.create_subscription(Float64MultiArray, '/lane/center',       self.on_lane,  10)
        self.create_subscription(Path,              '/bridge_path',       self.on_bridge, 10)
        self.create_subscription(Path,              '/waypoint_path',     self.on_waypoint_path, 10)
        self.create_subscription(Odometry,          '/odometry/filtered_map', self.on_odom,  10)
        self.cmd_pub = self.create_publisher(AckermannDriveStamped, '/cmd', 10)
        # 실제 차량 구동 토픽(하드웨어가 실제로 구독하는 건 이쪽)
        self.vel_pub = self.create_publisher(Float64, '/velocity', 10)
        self.steer_pub = self.create_publisher(Float64, '/steering_angle', 10)

        self.create_timer(1.0 / self.hz, self.control_step)
        self.get_logger().info('controller(mode mux) 시작')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ----------------------- 콜백: 캐싱만 -----------------------
    def on_mode(self, m):
        self.mode = m.data

    def on_lane(self, m):
        # data = [offset_m, steering_angle_deg, curvature_px, valid]  (lane_detection.cpp 규약)
        d = list(m.data) + [0.0, 0.0, 0.0, 0.0]
        raw_off = d[0]
        raw_head = math.radians(d[1])   # heading은 degree로 오므로 radian 변환
        valid = d[3]                    # d[2]는 curvature_px, valid는 d[3]

        if valid > 0.5:
            if not self.lane_init:
                # 첫 유효 프레임은 그대로 채택(필터 워밍업 중 0으로 끌려가는 것 방지)
                self.lane_off = raw_off
                self.lane_head = raw_head
                self.lane_init = True
            else:
                # 지수이동평균(EMA)으로 프레임간 노이즈를 죽여 조향이 튀는 것(꿀렁거림) 완화.
                # alpha=1이면 원본 그대로, 작을수록 부드러워지되 반응은 느려짐.
                a = self.lane_alpha
                self.lane_off = a * raw_off + (1.0 - a) * self.lane_off
                self.lane_head = a * raw_head + (1.0 - a) * self.lane_head

        self.lane_valid = valid > 0.5
        self.t_lane = self.now_s()

    def on_bridge(self, m):
        self.bridge = [[p.pose.position.x, p.pose.position.y] for p in m.poses]

    def on_waypoint_path(self, m):
        self.wp_path = [[p.pose.position.x, p.pose.position.y] for p in m.poses]

    def on_odom(self, m):
        self.x = m.pose.pose.position.x
        self.y = m.pose.pose.position.y
        self.yaw = yaw_from_quat(m.pose.pose.orientation)
        self.v = m.twist.twist.linear.x        # base_link 전진 속도
        self.t_odom = self.now_s()

    # ----------------------- 모드별 조향 -----------------------
    def lane_follow_steer(self, bias=0.0):
        """Stanley: δ = (heading 항) + atan2(k·e, v).
        e=횡오차(offset), bias 만큼 목표 차선을 옆으로 민다(AVOID 용)."""
        e = (self.lane_off - bias) * self.sign_off
        psi = self.lane_head * self.sign_head
        v = max(abs(self.v), self.v_soft)
        return psi + math.atan2(self.k_stanley * e, v)

    def pure_pursuit_on(self, path):
        """문서 11.2: 주어진 path 위에서 Ld 앞 목표점 → δ=atan2(2L·sinα, Ld).
        TURN_BRIDGE(self.bridge)와 WAYPOINT_FOLLOW(self.wp_path) 양쪽에서 공용으로 쓴다."""
        if not path:
            return 0.0, False
        x, y, yaw = self.x, self.y, self.yaw
        # Ld 이상 떨어진 첫 점(없으면 마지막 점)
        ti = -1
        for k, (px, py) in enumerate(path):
            if math.hypot(px - x, py - y) >= self.Ld:
                ti = k
                break
        tx, ty = path[ti]
        dx, dy = tx - x, ty - y
        local_x = math.cos(yaw) * dx + math.sin(yaw) * dy
        local_y = -math.sin(yaw) * dx + math.cos(yaw) * dy
        alpha = math.atan2(local_y, local_x)
        delta = math.atan2(2.0 * self.L * math.sin(alpha), self.Ld)
        return delta, True

    def pure_pursuit_steer(self):
        """TURN_BRIDGE(교차로 회전) 전용 래퍼: bridge_path 사용."""
        return self.pure_pursuit_on(self.bridge)

    def speed_for_curve(self, delta):
        """조향이 클수록 감속해 코너 안정화."""
        v = self.cruise * max(0.0, 1.0 - self.curve_slow * abs(delta))
        return max(self.min_speed, v)

    # ----------------------- 메인 제어 루프 -----------------------
    def control_step(self):
        now = self.now_s()
        dt = now - self.last_t
        if dt <= 0.0:
            dt = 1.0 / self.hz
        self.last_t = now

        odom_ok = (now - self.t_odom) < self.timeout
        lane_ok = (now - self.t_lane) < self.timeout and self.lane_valid

        tgt_delta = self.cmd_delta             # 기본은 직전 조향 유지
        tgt_speed = 0.0

        if not odom_ok:
            # 위치를 모르면(EKF 끊김) 무조건 정지
            tgt_delta, tgt_speed = 0.0, 0.0

        elif self.mode == 'TURN_BRIDGE':
            d, ok = self.pure_pursuit_steer()
            if ok:
                tgt_delta, tgt_speed = d, self.bridge_spd
            else:
                tgt_speed = 0.0                # 경로 아직 못 받음 → 대기

        elif self.mode == 'STOP_AT_LIGHT':
            if lane_ok:                        # 정지하면서도 차선 중앙은 유지
                tgt_delta = self.lane_follow_steer()
            tgt_speed = 0.0

        elif self.mode == 'AVOID':
            if lane_ok:
                tgt_delta = self.lane_follow_steer(bias=self.avoid_bias)
            tgt_speed = self.speed_for_curve(tgt_delta) * 0.6   # 회피 중 감속

        elif self.mode == 'WAYPOINT_FOLLOW':
            # 차선 유실 fail-safe: 감독 노드가 보내주는 코스 좌표(/waypoint_path)를
            # Pure Pursuit로 따라간다. 정지 대신 완주를 우선.
            d, ok = self.pure_pursuit_on(self.wp_path)
            if ok:
                tgt_delta, tgt_speed = d, self.wp_speed
            else:
                tgt_speed = 0.0             # 경로 아직 못 받음 → 대기(정지)

        else:  # LANE_FOLLOW (그리고 알 수 없는 모드는 보수적으로)
            if lane_ok:
                tgt_delta = self.lane_follow_steer()
                tgt_speed = self.speed_for_curve(tgt_delta)
            else:
                # 차선 소실인데 아직 LANE_FOLLOW → 감속 대기.
                # (감독 노드가 곧 WAYPOINT_FOLLOW/TURN_BRIDGE 로 바꿔주거나 차선이 다시 잡힌다)
                tgt_speed = 0.0

        # 한계/평활화 후 출력
        tgt_delta = clamp(tgt_delta, -self.max_steer, self.max_steer)
        self.cmd_delta = rate_limit(self.cmd_delta, tgt_delta, self.steer_rate * dt)
        self.cmd_speed = ramp(self.cmd_speed, tgt_speed,
                              self.accel * dt, self.decel * dt)

        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'base_link'
        cmd.drive.steering_angle = float(self.cmd_delta)
        cmd.drive.speed = float(self.cmd_speed)
        self.cmd_pub.publish(cmd)

        # 실제 차량 구동용 출력 (하드웨어가 구독하는 형식)
        vel_msg = Float64()
        vel_msg.data = float(self.cmd_speed)
        self.vel_pub.publish(vel_msg)

        steer_msg = Float64()
        steer_msg.data = float(self.cmd_delta)
        self.steer_pub.publish(steer_msg)


def main():
    rclpy.init()
    node = Controller()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()