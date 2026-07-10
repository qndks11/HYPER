#!/usr/bin/env python3
"""
lane_follower.py — 라인 추종 전용 노드

입력:
  /lane/center              : std_msgs/Float64MultiArray
                              data=[offset_m, steering_angle_deg, curvature_px, valid]
  /odometry/filtered_odom   : nav_msgs/Odometry

출력:
  /cmd                      : ackermann_msgs/AckermannDriveStamped
  /velocity                 : std_msgs/Float64
  /steering_angle           : std_msgs/Float64
"""

import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, Float64
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped


# ----------------------------- 작은 유틸 -----------------------------
def clamp(v, lo, hi):
    return lo if v < lo else hi if v > hi else v


def rate_limit(cur, target, max_step):
    """한 주기에 max_step 이상 못 바뀌게 한다."""
    return clamp(target, cur - max_step, cur + max_step)


def ramp(cur, target, up_step, down_step):
    """속도 가속/감속 제한."""
    if target > cur:
        return min(target, cur + up_step)
    return max(target, cur - down_step)


# ----------------------------- 라인 추종 노드 -----------------------------
class LaneFollower(Node):
    def __init__(self):
        super().__init__('lane_follower')

        # ---------------- 파라미터 ----------------
        self.hz = self.declare_parameter('control_hz', 40.0).value

        # 조향 관련
        self.max_steer = self.declare_parameter('max_steer', 0.3).value
        self.steer_rate = self.declare_parameter('steer_rate', 1.0).value

        # 속도 관련
        self.cruise = self.declare_parameter('cruise_speed', 10.0).value
        # min_speed는 급커브에서의 속도 하한선이므로 cruise_speed보다 낮아야 한다.
        # (이전 기본값 0.5는 cruise_speed=0.3보다 커서 speed_for_curve()의 커브 감속이 항상
        # 무시되고 속도가 0.5로 고정되는 버그였음)
        self.min_speed = self.declare_parameter('min_speed', 0.1).value
        self.accel = self.declare_parameter('accel_limit', 1.0).value
        self.decel = self.declare_parameter('decel_limit', 2.0).value
        self.curve_slow = self.declare_parameter('curve_slow_k', 0.5).value

        # 곡률 기반 감속: lane_detection.cpp가 알려주는 곡률 반경(픽셀)이 이 값 이상이면
        # (거의 직선) 감속하지 않고, 작을수록(급커브) cruise_speed를 비례해서 줄인다.
        # 조향각 기반 감속(curve_slow_k)은 이미 계산된 delta에 반응하는 반면, 이건 차선의
        # 곡률 자체를 보므로 조향이 커지기 전에 미리 감속할 수 있다.
        self.curve_radius_ref = self.declare_parameter('curve_radius_ref_px', 300.0).value

        # Stanley 제어 게인
        self.k_stanley = self.declare_parameter('stanley_k', 5.0).value
        self.v_soft = self.declare_parameter('stanley_v_min', 0.3).value

        # 부호 보정 (±1.0 이어야 함. lane_detection.cpp의 우측 차선 기준 offset/heading
        # 부호 규약과 vehicle_controller.cpp의 "양수 조향각 = 좌회전" 규약을 맞추기 위한 값.
        # 이전 기본값 0.5는 부호도 아니고 이득도 아닌 값이라 조향이 절반 세기로 잘못된
        # 방향에 걸리는 버그였음. 실차/시뮬레이션에서 차량이 차선 중앙으로 수렴하지 않고
        # 반대로 벗어나면 lane_offset_sign을 -1.0으로 뒤집어 볼 것)
        self.sign_off = self.declare_parameter('lane_offset_sign', -1.0).value
        self.sign_head = self.declare_parameter('lane_heading_sign', 1.0).value

        # 입력 타임아웃
        self.timeout = self.declare_parameter('input_timeout', 0.3).value

        # ---------------- 상태 변수 ----------------
        self.lane_off = 0.0
        self.lane_head = 0.0
        self.lane_curv_radius_px = float('inf')
        self.lane_valid = False

        self.v = 0.0

        self.cmd_delta = 0.0
        self.cmd_speed = 0.0

        self.last_t = self.now_s()
        self.t_odom = -1e9
        self.t_lane = -1e9

        # ---------------- Subscriber ----------------
        self.create_subscription(
            Float64MultiArray,
            '/lane/center',
            self.on_lane,
            10
        )

        self.create_subscription(
            Odometry,
            '/odometry/filtered_odom',
            self.on_odom,
            10
        )

        # ---------------- Publisher ----------------
        # 기존 Ackermann 명령
        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped,
            '/cmd',
            10
        )

        # 실제 차량 컨트롤러용 명령
        self.vel_pub = self.create_publisher(
            Float64,
            '/velocity',
            10
        )

        self.steer_pub = self.create_publisher(
            Float64,
            '/steering_angle',
            10
        )

        # ---------------- Timer ----------------
        self.create_timer(1.0 / self.hz, self.control_step)

        self.get_logger().info('lane_follower 시작: /cmd + /velocity + /steering_angle publish')

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ----------------------- 콜백 -----------------------
    def on_lane(self, msg):
        # data = [offset_m, steering_angle_deg, curvature_px, valid]
        data = list(msg.data) + [0.0, 0.0, 0.0, 0.0]

        self.lane_off = float(data[0])
        # lane_detection.cpp는 heading을 degree로 publish하므로 radian으로 변환
        self.lane_head = math.radians(float(data[1]))
        self.lane_curv_radius_px = float(data[2])
        valid = float(data[3])  # data[2]는 curvature_px, valid는 data[3]

        self.lane_valid = valid > 0.5
        self.t_lane = self.now_s()

    def on_odom(self, msg):
        self.v = float(msg.twist.twist.linear.x)
        self.t_odom = self.now_s()

    # ----------------------- 제어 계산 -----------------------
    def lane_follow_steer(self):
        """
        Stanley 제어:
        delta = heading_error + atan2(k * offset, velocity)
        """
        e = self.lane_off * self.sign_off
        psi = self.lane_head * self.sign_head
        v = max(abs(self.v), self.v_soft)

        delta = psi + math.atan2(self.k_stanley * e, v)
        return delta

    def speed_for_curve(self, delta):
        """
        조향각(반응형)과 차선 곡률 반경(예측형) 중 더 낮은 속도를 최종 목표로 사용.
        """
        speed_from_steer = self.cruise * max(0.0, 1.0 - self.curve_slow * abs(delta))

        curve_factor = clamp(self.lane_curv_radius_px / self.curve_radius_ref, 0.0, 1.0)
        speed_from_curvature = self.cruise * curve_factor

        target_speed = min(speed_from_steer, speed_from_curvature)
        return max(self.min_speed, target_speed)

    # ----------------------- 메인 제어 루프 -----------------------
    def control_step(self):
        now = self.now_s()
        dt = now - self.last_t

        if dt <= 0.0:
            dt = 1.0 / self.hz

        self.last_t = now

        odom_ok = (now - self.t_odom) < self.timeout
        lane_ok = (now - self.t_lane) < self.timeout and self.lane_valid

        tgt_delta = self.cmd_delta
        tgt_speed = 0.0

        if not odom_ok:
            # odom이 안 들어오면 정지
            tgt_delta = 0.0
            tgt_speed = 0.0

        elif lane_ok:
            # 차선 정상 인식
            tgt_delta = self.lane_follow_steer()
            tgt_speed = self.speed_for_curve(tgt_delta)

        else:
            # 차선 소실
            # 조향은 직전값 유지, 속도는 0으로 감속
            tgt_delta = self.cmd_delta
            tgt_speed = 0.0

        # 조향 제한
        tgt_delta = clamp(tgt_delta, -self.max_steer, self.max_steer)

        # 조향 변화율 제한
        self.cmd_delta = rate_limit(
            self.cmd_delta,
            tgt_delta,
            self.steer_rate * dt
        )

        # 속도 변화율 제한
        self.cmd_speed = ramp(
            self.cmd_speed,
            tgt_speed,
            self.accel * dt,
            self.decel * dt
        )

        # ---------------- /cmd publish ----------------
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'body_link'
        cmd.drive.steering_angle = float(self.cmd_delta)
        cmd.drive.steering_angle_velocity = 0.0
        cmd.drive.speed = float(self.cmd_speed)
        cmd.drive.acceleration = 0.0
        cmd.drive.jerk = 0.0
        self.cmd_pub.publish(cmd)

        # ---------------- /velocity publish ----------------
        vel_msg = Float64()
        vel_msg.data = float(self.cmd_speed)
        self.vel_pub.publish(vel_msg)

        # ---------------- /steering_angle publish ----------------
        steer_msg = Float64()
        steer_msg.data = float(self.cmd_delta)
        self.steer_pub.publish(steer_msg)

    def publish_stop(self):
        """
        노드 종료 시 차량 정지 명령.
        """
        cmd = AckermannDriveStamped()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.header.frame_id = 'body_link'
        cmd.drive.steering_angle = 0.0
        cmd.drive.speed = 0.0
        self.cmd_pub.publish(cmd)

        vel_msg = Float64()
        vel_msg.data = 0.0
        self.vel_pub.publish(vel_msg)

        steer_msg = Float64()
        steer_msg.data = 0.0
        self.steer_pub.publish(steer_msg)


def main():
    rclpy.init()
    node = LaneFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()