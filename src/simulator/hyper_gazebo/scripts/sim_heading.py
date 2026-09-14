#!/usr/bin/env python3
"""gz IMU의 절대 방위를 /imu/heading으로 내보냅니다 -- sim 전용.

왜 필요한가
-----------
ekf_global의 절대 방위 관측은 imu1(= /imu/heading) **하나뿐**입니다
(dual_ekf_navsat.yaml에서 imu0의 yaw는 false, odom0의 yaw도 false).
그런데 /imu/heading을 내보내는 건 실차의 듀얼 GNSS moving-base 수신기뿐이고
(hyper_rtk/launch/rtk.launch.py의 navheading -> imu/heading remap), sim에는
두 번째 안테나가 없습니다(vehicle.xacro의 gps_rover_link 주석 참고).

그래서 sim에서는 ekf_global이 절대 방위를 하나도 못 받고, map yaw가 0에서
시작해 자이로 적분으로만 굴러갑니다. 차는 스폰 yaw(2.846 rad)로 서 있는데
필터는 0이라고 믿으니 map 프레임이 통째로 돌아가고, 그러면 "sim과 실차가
같은 map 좌표를 쓴다"는 전제 자체가 깨집니다.

이 노드는 gz IMU가 이미 ENU 절대 자세로 내보내는 yaw를 실차와 **같은 토픽,
같은 메시지 타입**으로 중계해서 그 구멍을 막습니다. sim 전용 yaml을 따로
두지 않으므로 dual_ekf_navsat.yaml은 sim과 실차가 한 글자도 다르지 않습니다
-- 이번 좌표계 통일 작업의 목적과 같은 이야기입니다.

gz IMU가 절대 방위를 준다는 근거: vehicle.xacro의 imu_sensor가
<orientation_reference_frame><localization>ENU</localization>로 고정돼 있습니다.
이게 CUSTOM(기본값)이면 기준이 "스폰 당시 자세"가 되어 여기서 읽는 yaw가
절대 방위가 아니게 되므로, 그 블록을 지우면 이 노드도 같이 무의미해집니다.

실차와 다른 점: 실차 헤딩은 carrier phase가 fixed로 수렴해야 유효하고 그
전에는 covariance가 커집니다. 여기서는 항상 유효합니다. 즉 이 노드는 RTK가
안 잡혔을 때의 거동까지 흉내내지는 않습니다.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


def yaw_from_quaternion(q):
    """쿼터니언에서 yaw(Z축 회전)만 뽑습니다."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SimHeading(Node):

    def __init__(self):
        super().__init__('sim_heading')
        # gz IMU는 50 Hz이고 ekf_global은 30 Hz입니다. 그대로 1:1 중계합니다.
        self.declare_parameter('input_topic', 'imu')
        self.declare_parameter('output_topic', 'imu/heading')
        # 실차 RTK 헤딩(0.6 m 베이스라인)의 실제 정밀도는 0.3deg 수준입니다.
        # sim IMU는 사실상 무오차지만, 필터가 이 관측 하나만 믿고 다른 센서를
        # 무시하지 않도록 비슷한 크기로 둡니다. (0.3deg)^2 ~= 2.7e-5 rad^2.
        self.declare_parameter('yaw_variance', 2.7e-5)

        self._yaw_variance = self.get_parameter('yaw_variance').value
        out = self.get_parameter('output_topic').value
        src = self.get_parameter('input_topic').value

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        self._pub = self.create_publisher(Imu, out, qos)
        self._sub = self.create_subscription(Imu, src, self._on_imu, qos)
        self.get_logger().info(f"sim heading relay: {src} -> {out}")

    def _on_imu(self, msg):
        out = Imu()
        # 스탬프와 frame_id를 그대로 물려받습니다. frame_id는 body_link이고
        # (vehicle.xacro의 gz_frame_id), robot_localization이 IMU를
        # base_link_frame 기준으로 돌려놓을 때 이 프레임을 씁니다. 실차는
        # gps_rover_link로 찍히지만 그 링크의 rpy가 0이라 결과는 같습니다.
        out.header = msg.header

        # yaw만 남긴 쿼터니언 -- 실차 /imu/heading과 같은 의미입니다(roll/pitch는
        # imu0이 중력으로 관측하므로 여기서 또 주면 관측이 겹칩니다).
        yaw = yaw_from_quaternion(msg.orientation)
        out.orientation.z = math.sin(yaw / 2.0)
        out.orientation.w = math.cos(yaw / 2.0)

        # ekf_global의 imu1_config는 인덱스 5(yaw)만 true입니다. roll/pitch에는
        # "믿지 마라"는 뜻으로 큰 분산을 넣어 둡니다.
        out.orientation_covariance[0] = 1e9
        out.orientation_covariance[4] = 1e9
        out.orientation_covariance[8] = self._yaw_variance

        # 각속도/가속도는 이 메시지의 관심사가 아닙니다. imu0(/imu)이 이미
        # 내보내므로 여기서 또 주면 같은 관측을 두 번 먹입니다. -1은 ROS 관례로
        # "이 항목 없음"입니다.
        out.angular_velocity_covariance[0] = -1.0
        out.linear_acceleration_covariance[0] = -1.0

        self._pub.publish(out)


def main():
    rclpy.init()
    node = SimHeading()
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
