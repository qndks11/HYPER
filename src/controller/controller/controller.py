#!/usr/bin/env python3
"""
controller.py — 제어 노드 (모드 mux)

LANE_FOLLOW에서는 /lane/center 메시지 뒤에 포함된 분홍색 중앙 경로 점들을
차량 좌표계(base_link 형태: x=전방, y=좌측)로 받아 Pure Pursuit로 직접 추종한다.

/lane/center 규약
  data[0] = center offset [m]
  data[1] = center heading [deg]
  data[2] = curvature radius [px]
  data[3] = valid (1.0/0.0)
  data[4:] = center path points [forward_m, left_m, forward_m, left_m, ...]

이전 4개 값만 들어오는 lane_detection과도 호환된다. 경로 점이 없으면
기존 Stanley 제어로 자동 폴백한다.
"""

import math

import rclpy
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from std_msgs.msg import Float64, Float64MultiArray, String


def clamp(value, low, high):
    return low if value < low else high if value > high else value


def rate_limit(current, target, max_step):
    """한 제어 주기 동안 조향 명령이 max_step 이상 변하지 않게 한다."""
    return clamp(target, current - max_step, current + max_step)


def ramp(current, target, up_step, down_step):
    """속도 명령에 가속·감속 한계를 적용한다."""
    if target > current:
        return min(target, current + up_step)
    return max(target, current - down_step)


def yaw_from_quat(quaternion):
    return math.atan2(
        2.0 * (
            quaternion.w * quaternion.z +
            quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y +
            quaternion.z * quaternion.z
        )
    )


class Controller(Node):
    def __init__(self):
        super().__init__('controller')

        # ---------------- 차량 및 제어 주기 ----------------
        self.L = float(
            self.declare_parameter('wheelbase', 1.34 * 0.75).value
        )
        self.hz = float(
            self.declare_parameter('control_hz', 40.0).value
        )
        self.max_steer = float(
            self.declare_parameter(
                'max_steering_angle', 0.5235988
            ).value
        )
        self.steer_rate = float(
            self.declare_parameter(
                'max_steering_angular_velocity', 2.0
            ).value
        )
        self.max_velocity = float(
            self.declare_parameter('max_velocity', 5.0).value
        )

        # ---------------- 속도 ----------------
        self.cruise = float(
            self.declare_parameter('cruise_speed', 3.75).value
        )
        self.bridge_spd = float(
            self.declare_parameter('bridge_speed', 2.5).value
        )
        self.wp_speed = float(
            self.declare_parameter('waypoint_speed', 2.5).value
        )
        self.min_speed = float(
            self.declare_parameter('min_speed', 1.25).value
        )
        self.accel = float(
            self.declare_parameter('accel_limit', 2.0).value
        )
        self.decel = float(
            self.declare_parameter('decel_limit', 4.0).value
        )
        self.curve_slow = float(
            self.declare_parameter('curve_slow_k', 1.2).value
        )

        # ---------------- 차선 중앙 경로 추종 ----------------
        # 분홍색 중앙 경로를 직접 따라가기 위한 로컬 Pure Pursuit lookahead.
        self.lane_lookahead = float(
            self.declare_parameter('lane_lookahead', 1.0).value
        )
        self.lane_min_forward = float(
            self.declare_parameter('lane_min_forward', 0.10).value
        )
        self.lane_path_min_points = int(
            self.declare_parameter('lane_path_min_points', 3).value
        )

        # lane_detection이 보내는 로컬 경로의 프레임간 흔들림 완화.
        # 1.0이면 필터 없음, 작을수록 부드러워진다.
        self.lane_target_alpha = float(
            self.declare_parameter('lane_target_filter_alpha', 0.35).value
        )

        # ---------------- Stanley 폴백 ----------------
        # 새 lane_detection이 아닌 예전 4개 데이터만 들어올 때 사용한다.
        self.k_stanley = float(
            self.declare_parameter('stanley_k', 0.8).value
        )
        self.v_soft = float(
            self.declare_parameter('stanley_v_min', 0.5).value
        )

        # 현재 lane_detection의 offset은
        #   + : 목표 중앙 경로가 차량보다 오른쪽
        # 이므로 오른쪽 조향(-)이 필요하다. 기본 부호를 -1로 둔다.
        self.sign_off = float(
            self.declare_parameter('lane_offset_sign', -1.0).value
        )
        self.sign_head = float(
            self.declare_parameter('lane_heading_sign', 1.0).value
        )
        self.lane_alpha = float(
            self.declare_parameter('lane_filter_alpha', 0.3).value
        )

        # ---------------- 장애물 회피 ----------------
        self.avoid_bias = float(
            self.declare_parameter('avoid_offset_bias', 0.45).value
        )

        # ---------------- 지도 경로 Pure Pursuit ----------------
        self.Ld = float(
            self.declare_parameter('lookahead', 1.0).value
        )

        # ---------------- 워치독 ----------------
        self.timeout = float(
            self.declare_parameter('input_timeout', 0.3).value
        )

        # ---------------- 캐시 상태 ----------------
        self.mode = 'LANE_FOLLOW'

        # lane summary 값: 경로 점이 없는 경우의 Stanley 폴백용.
        self.lane_off = 0.0
        self.lane_head = 0.0
        self.lane_radius_px = 1e6
        self.lane_valid = False
        self.lane_init = False

        # 분홍색 중앙 경로의 로컬 좌표 점들.
        # 각 점은 [forward_m, left_m].
        self.lane_path_local = None

        # 선택된 Pure Pursuit 목표점을 EMA로 부드럽게 만든다.
        self.filtered_lane_target = None

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.v = 0.0

        self.bridge = None
        self.wp_path = None

        self.cmd_delta = 0.0
        self.cmd_speed = 0.0
        self.last_t = self.now_s()
        self.t_odom = -1e9
        self.t_lane = -1e9

        # ---------------- ROS I/O ----------------
        self.create_subscription(
            String, '/driving_mode', self.on_mode, 10
        )
        self.create_subscription(
            Float64MultiArray, '/lane/center', self.on_lane, 10
        )
        self.create_subscription(
            Path, '/bridge_path', self.on_bridge, 10
        )
        self.create_subscription(
            Path, '/waypoint_path', self.on_waypoint_path, 10
        )
        self.create_subscription(
            Odometry,
            '/odometry/filtered_map',
            self.on_odom,
            10
        )

        self.cmd_pub = self.create_publisher(
            AckermannDriveStamped, '/cmd', 10
        )
        self.vel_pub = self.create_publisher(
            Float64, '/velocity', 10
        )
        self.steer_pub = self.create_publisher(
            Float64, '/steering_angle', 10
        )

        self.create_timer(1.0 / self.hz, self.control_step)

        self.get_logger().info(
            'controller 시작: LANE_FOLLOW는 분홍색 중앙 경로를 '
            '로컬 Pure Pursuit로 추종'
        )

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------------- 콜백 ----------------
    def on_mode(self, message):
        self.mode = message.data

    def on_lane(self, message):
        data = list(message.data)

        if len(data) < 4:
            self.lane_valid = False
            self.lane_path_local = None
            self.filtered_lane_target = None
            self.t_lane = self.now_s()
            return

        raw_off = float(data[0])
        raw_head = math.radians(float(data[1]))
        self.lane_radius_px = float(data[2])
        valid = float(data[3]) > 0.5

        if valid:
            if not self.lane_init:
                self.lane_off = raw_off
                self.lane_head = raw_head
                self.lane_init = True
            else:
                alpha = clamp(self.lane_alpha, 0.0, 1.0)
                self.lane_off = (
                    alpha * raw_off +
                    (1.0 - alpha) * self.lane_off
                )
                self.lane_head = (
                    alpha * raw_head +
                    (1.0 - alpha) * self.lane_head
                )

            # data[4:]는 [forward_m, left_m] 쌍이다.
            path_values = data[4:]
            local_path = []

            for index in range(0, len(path_values) - 1, 2):
                forward_m = float(path_values[index])
                left_m = float(path_values[index + 1])

                if not (
                    math.isfinite(forward_m) and
                    math.isfinite(left_m)
                ):
                    continue

                if forward_m < self.lane_min_forward:
                    continue

                local_path.append([forward_m, left_m])

            if len(local_path) >= self.lane_path_min_points:
                # lane_detection이 가까운 점부터 먼 점 순서로 발행하지만,
                # 안전하게 전방 거리 기준으로 한 번 정렬한다.
                local_path.sort(key=lambda point: point[0])
                self.lane_path_local = local_path
            else:
                self.lane_path_local = None
                self.filtered_lane_target = None

        else:
            self.lane_path_local = None
            self.filtered_lane_target = None
            self.lane_init = False

        self.lane_valid = valid
        self.t_lane = self.now_s()

    def on_bridge(self, message):
        self.bridge = [
            [pose.pose.position.x, pose.pose.position.y]
            for pose in message.poses
        ]

    def on_waypoint_path(self, message):
        self.wp_path = [
            [pose.pose.position.x, pose.pose.position.y]
            for pose in message.poses
        ]

    def on_odom(self, message):
        self.x = message.pose.pose.position.x
        self.y = message.pose.pose.position.y
        self.yaw = yaw_from_quat(message.pose.pose.orientation)
        self.v = message.twist.twist.linear.x
        self.t_odom = self.now_s()

    # ---------------- 차선 제어 ----------------
    def lane_follow_stanley(self, bias=0.0):
        """경로 점을 받지 못했을 때 사용하는 Stanley 폴백."""
        error = (self.lane_off - bias) * self.sign_off
        heading = self.lane_head * self.sign_head
        speed = max(abs(self.v), self.v_soft)

        return heading + math.atan2(
            self.k_stanley * error,
            speed
        )

    def select_local_lane_target(self):
        """
        분홍색 중앙 경로에서 lane_lookahead 이상 떨어진 첫 점을 고른다.
        좌표 규약은 x=전방, y=좌측이다.
        """
        path = self.lane_path_local

        if not path:
            return None

        target = None

        for forward_m, left_m in path:
            distance = math.hypot(forward_m, left_m)

            if distance >= self.lane_lookahead:
                target = [forward_m, left_m]
                break

        if target is None:
            target = list(path[-1])

        if target[0] <= self.lane_min_forward:
            return None

        # 선택 목표점만 EMA 처리한다. 전체 경로를 점별로 필터링하면
        # 점 개수가 달라질 때 인덱스가 엇갈릴 수 있기 때문이다.
        if self.filtered_lane_target is None:
            self.filtered_lane_target = target
        else:
            alpha = clamp(self.lane_target_alpha, 0.0, 1.0)
            self.filtered_lane_target = [
                alpha * target[0] +
                (1.0 - alpha) * self.filtered_lane_target[0],
                alpha * target[1] +
                (1.0 - alpha) * self.filtered_lane_target[1]
            ]

        return self.filtered_lane_target

    def lane_path_pure_pursuit(self, lateral_bias=0.0):
        """
        분홍색 중앙 경로의 로컬 목표점을 직접 추종한다.

        local_x = 전방 거리
        local_y = 좌측 거리

        Pure Pursuit:
            delta = atan2(2 * L * local_y, Ld^2)
        실제 목표점 거리 제곱을 사용하므로 경로 점 간격 변화에도 대응한다.
        """
        target = self.select_local_lane_target()

        if target is None:
            return 0.0, False

        local_x = target[0]
        local_y = target[1] + lateral_bias
        distance_squared = local_x * local_x + local_y * local_y

        if distance_squared < 1e-6:
            return 0.0, False

        delta = math.atan2(
            2.0 * self.L * local_y,
            distance_squared
        )

        return delta, True

    # ---------------- 지도 경로 제어 ----------------
    def pure_pursuit_on_map_path(self, path):
        """map 좌표의 bridge/waypoint 경로를 Pure Pursuit로 추종한다."""
        if not path:
            return 0.0, False

        target_index = -1

        for index, (path_x, path_y) in enumerate(path):
            if math.hypot(path_x - self.x, path_y - self.y) >= self.Ld:
                target_index = index
                break

        target_x, target_y = path[target_index]
        dx = target_x - self.x
        dy = target_y - self.y

        local_x = math.cos(self.yaw) * dx + math.sin(self.yaw) * dy
        local_y = -math.sin(self.yaw) * dx + math.cos(self.yaw) * dy

        distance_squared = local_x * local_x + local_y * local_y

        if local_x <= 0.0 or distance_squared < 1e-6:
            return 0.0, False

        delta = math.atan2(
            2.0 * self.L * local_y,
            distance_squared
        )

        return delta, True

    def pure_pursuit_steer(self):
        return self.pure_pursuit_on_map_path(self.bridge)

    def speed_for_curve(self, steering):
        speed = self.cruise * max(
            0.0,
            1.0 - self.curve_slow * abs(steering)
        )
        return max(self.min_speed, speed)

    def lane_follow_command(self, lateral_bias=0.0):
        """
        우선 분홍색 중앙 경로를 Pure Pursuit로 추종하고,
        경로 점이 없는 구버전 메시지에서는 Stanley로 폴백한다.
        """
        steering, ok = self.lane_path_pure_pursuit(lateral_bias)

        if ok:
            return steering

        return self.lane_follow_stanley(lateral_bias)

    # ---------------- 메인 제어 루프 ----------------
    def control_step(self):
        now = self.now_s()
        dt = now - self.last_t

        if dt <= 0.0:
            dt = 1.0 / self.hz

        self.last_t = now

        odom_ok = (now - self.t_odom) < self.timeout
        lane_ok = (
            (now - self.t_lane) < self.timeout and
            self.lane_valid
        )

        target_steering = self.cmd_delta
        target_speed = 0.0

        if not odom_ok:
            target_steering = 0.0
            target_speed = 0.0

        elif self.mode == 'TURN_BRIDGE':
            steering, ok = self.pure_pursuit_steer()

            if ok:
                target_steering = steering
                target_speed = self.bridge_spd
            else:
                target_speed = 0.0

        elif self.mode == 'STOP_AT_LIGHT':
            if lane_ok:
                target_steering = self.lane_follow_command()
            target_speed = 0.0

        elif self.mode == 'AVOID':
            if lane_ok:
                # positive bias는 목표 경로를 좌측으로 이동시킨다.
                target_steering = self.lane_follow_command(
                    lateral_bias=self.avoid_bias
                )
                target_speed = (
                    self.speed_for_curve(target_steering) * 0.6
                )
            else:
                target_speed = 0.0

        elif self.mode == 'WAYPOINT_FOLLOW':
            steering, ok = self.pure_pursuit_on_map_path(self.wp_path)

            if ok:
                target_steering = steering
                target_speed = self.wp_speed
            else:
                target_speed = 0.0

        else:  # LANE_FOLLOW
            if lane_ok:
                target_steering = self.lane_follow_command()
                target_speed = self.speed_for_curve(target_steering)
            else:
                target_speed = 0.0

        target_steering = clamp(
            target_steering,
            -self.max_steer,
            self.max_steer
        )
        target_speed = clamp(
            target_speed,
            -self.max_velocity,
            self.max_velocity
        )

        self.cmd_delta = rate_limit(
            self.cmd_delta,
            target_steering,
            self.steer_rate * dt
        )
        self.cmd_speed = ramp(
            self.cmd_speed,
            target_speed,
            self.accel * dt,
            self.decel * dt
        )

        command = AckermannDriveStamped()
        command.header.stamp = self.get_clock().now().to_msg()
        command.header.frame_id = 'base_link'
        command.drive.steering_angle = float(self.cmd_delta)
        command.drive.speed = float(self.cmd_speed)
        self.cmd_pub.publish(command)

        velocity_message = Float64()
        velocity_message.data = float(self.cmd_speed)
        self.vel_pub.publish(velocity_message)

        steering_message = Float64()
        steering_message.data = float(self.cmd_delta)
        self.steer_pub.publish(steering_message)


def main(args=None):
    rclpy.init(args=args)
    node = Controller()

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