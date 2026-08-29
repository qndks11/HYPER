import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Real-car equivalent of simulation.launch.py: sensors replace Gazebo, but the
# same staggered stage delays apply so odometry/perception/behavior attach
# once sensor drivers (RTK fix lock, D435i firmware init, etc.) are up.
ODOMETRY_DELAY_S = 5.0
PERCEPTION_DELAY_S = 7.0
BEHAVIOR_DELAY_S = 9.0


def generate_launch_description():
    hyper_launch_share = get_package_share_directory('hyper_launch')

    def stage(name, **launch_arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hyper_launch_share, 'launch', name)),
            launch_arguments=launch_arguments.items())

    # Publishes body_link -> camera_link/lidar_link/gps_link TF from vehicle.xacro, same as
    # hyper_gazebo's vehicle.launch.py does for sim. Lives in hyper_control (not hyper_launch),
    # since vehicle.xacro is owned there.
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_control'),
            'launch', 'robot_state_publisher.launch.py')),
    )

    # Full nav2 view (costmaps, planner/MPPI paths, footprint) -- same config
    # hyper_planner's README points at for follow_path debugging, shared here since real.launch.py
    # runs the same behavior.launch.py stage.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('hyper_planner'),
            'config', 'follow_path.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # GPS 정확도 모니터. /ublox_gps_node/navpvt의 hAcc/vAcc를 큰 글씨로 띄웁니다.
    # 조건 없이 항상 뜹니다 -- 어떤 토픽도 publish하지 않는 순수 구독자라 nav2든
    # 조이스틱이든 아무것과도 충돌하지 않고, 실차에서 "지금 GPS를 믿어도 되는가"는
    # 항상 봐야 하는 값이기 때문입니다. sensors 스테이지(ublox_gps_node)보다 먼저
    # 떠도 무방합니다: NavPVT가 안 오는 동안은 NO DATA (stale)로 표시됩니다.
    gps_accuracy_gui = Node(
        package='hyper_localization',
        executable='gps_accuracy_gui.py',
        name='gps_accuracy_gui',
        output='screen',
    )

    return LaunchDescription([
        # 어떤 미션을 실을지. hyper_planner/config/<이름>.yaml로 풀립니다.
        # mission:=simple 이면 코스 한 바퀴만 도는 단일 골 미션입니다.
        DeclareLaunchArgument('mission', default_value='mission'),
        # navsat_transform의 GPS 원점. hyper_localization/config/datums.yaml의 키입니다.
        # 스테이지 기본값은 시뮬레이션 원점(sim)이라 실차 진입점에서는 여기서 덮어써야
        # 합니다 -- 대회장이 아닌 곳에서 돌릴 때는 datum_site:=school 처럼 바꾸세요.
        DeclareLaunchArgument(
            'datum_site', default_value='track',
            description='GPS 원점 (datums.yaml의 sim | school | track)'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz with the follow_path nav2 view'),
        # 수동 주행용 조이스틱. 기본값 false인 이유가 있습니다: joystick_controller_node는
        # 스틱을 안 건드려도 100Hz로 /velocity + /steering_angle을 계속 내보내는데, 이는
        # behavior 스테이지의 cmd_vel_to_ackermann이 쓰는 토픽과 똑같습니다. 둘을 같이
        # 띄우면 스틱 중립(0.0) 명령이 nav2의 명령과 번갈아 arduino_interface에 도착해
        # 차가 자율주행을 못 합니다. 수동으로 몰 때만 켜세요:
        #   ros2 launch hyper_launch real.launch.py use_joystick:=true
        # (이때 nav2도 같이 도는 게 싫으면 mission 스테이지를 따로 내리거나, 조이스틱
        #  단독으로 sensors + interface만 띄우는 편이 낫습니다.)
        DeclareLaunchArgument(
            'use_joystick', default_value='false',
            description='Xbox/joy 수동 조작 노드를 함께 띄웁니다 (nav2와 /velocity 충돌 주의)'),
        robot_state_publisher,
        rviz,
        gps_accuracy_gui,
        stage('sensors.launch.py'),
        # use_sim_time=false가 핵심입니다. 실차에는 /clock을 내보내는 노드가 없으므로,
        # dual_ekf_navsat.yaml에 박혀 있는 use_sim_time: true를 그대로 두면 ekf_local /
        # ekf_global / navsat_transform이 멈춘 시계 위에서 돌며 아무것도 publish하지 않습니다.
        TimerAction(period=ODOMETRY_DELAY_S, actions=[
            stage('odometry.launch.py',
                  datum_site=LaunchConfiguration('datum_site'),
                  use_sim_time='false')]),
        # Real vehicle: hyper_camera owns both physical cameras and publishes plain image
        # topics -- the ELP publisher loads into the same component container as
        # lane_detection_node for zero-copy intra-process delivery (lane_input_backend
        # intra_process); the Logitech publisher just feeds object_detection_node's ordinary
        # subscription (object_input_backend usb_camera). See perception.launch.py.
        TimerAction(period=PERCEPTION_DELAY_S, actions=[
            stage(
                'perception.launch.py',
                lane_input_backend='intra_process',
                object_input_backend='usb_camera')]),
        # interface.launch.py (hyper_interface's Arduino serial bridge) starts alongside
        # behavior since it only needs /velocity + /steering_angle to exist -- late subscriber
        # join works fine with ROS 2 discovery either way. It subscribes to those topics
        # directly (see hyper_interface/README.md), so no vehicle_controller_node/ros2_control
        # translation step sits in between on the real car.
        TimerAction(period=BEHAVIOR_DELAY_S, actions=[
            stage('behavior.launch.py',
                  mission=LaunchConfiguration('mission'),
                  # 위와 같은 이유. 이쪽은 nav2_controller.launch.py가 RewrittenYaml로
                  # nav2_controller.yaml의 use_sim_time을 덮어쓰므로 인자만 넘기면 됩니다.
                  use_sim_time='false'),
            stage('interface.launch.py'),
            # interface.launch.py와 같은 타이밍에 올립니다. 조이스틱 명령이 가는 곳이
            # arduino_interface_node이므로, 그보다 먼저 떠 있어 봐야 받을 노드가 없습니다.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(os.path.join(
                    get_package_share_directory('hyper_control'),
                    'launch', 'joystick.launch.py')),
                condition=IfCondition(LaunchConfiguration('use_joystick'))),
        ]),
    ])
