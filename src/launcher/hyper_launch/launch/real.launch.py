import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# Real-car equivalent of simulation.launch.py: sensors replace Gazebo, but the
# same staggered stage delays apply so odometry/perception/behavior attach
# once sensor drivers (RTK fix lock, USB camera enumeration, etc.) are up.
#
# 이 파일은 **미션(자율주행) 전용**입니다. 사람이 스틱으로 모는 수동 주행은
# joystick_control_real.launch.py로 분리했습니다. 둘을 한 트리에 담으면
# joystick_controller_node와 cmd_vel_to_ackermann_node가 /velocity +
# /steering_angle을 동시에 100Hz로 쏘면서 차가 자율주행을 못 합니다.
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

    # GPS 정확도 모니터. /ublox_gps_node/navpvt의 hAcc/vAcc를 큰 글씨로 띄우고,
    # IMU(E2BOX EBIMU-9DOFV5) 링크 상태(/imu 수신 여부와 Hz)도 같이 보여줍니다.
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

    # hyper_rqt의 "HYPER Panel" -- simulation.launch.py와 동일하게 mission_manager
    # start/cancel/skip/restart 버튼을 띄웁니다. model_service/teleport_service 그룹은
    # 실차에 없는 서비스라 그냥 비활성(회색)으로 뜰 뿐 문제 없습니다(panel_widget.py의
    # _poll()이 service_is_ready()로 버튼을 껐다 켰다 합니다).
    #
    # name=을 주지 않는 것이 중요합니다: launch_ros가 name을 붙이면 --ros-args -r
    # __node:=... 가 argv에 끼는데, 이 실행 파일은 rqt_gui의 Main()이 argv를 직접
    # 파싱하므로 알 수 없는 인자로 보고 죽습니다.
    mission_panel = Node(
        package='hyper_rqt',
        executable='hyper_panel',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_panel')),
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
        DeclareLaunchArgument(
            'use_panel', default_value='true',
            description='Launch the hyper_rqt HYPER Panel (mission start/cancel)'),
        # 미션이 실제로 따라갈 코스 CSV. 기본값이 sim.csv라는 점이 중요합니다 --
        # 실차에서는 반드시 녹화한 파일로 덮어쓰세요:
        #   ros2 launch hyper_launch real.launch.py waypoint_csv:=$HOME/HYPER/src/planning/hyper_waypoint/waypoints/real.csv
        # 이 인자를 여기서 선언하고 behavior 스테이지로 넘겨주지 않으면, 넘긴 값이
        # 조용히 무시된 채 시뮬레이션 코스가 실차에 실립니다.
        DeclareLaunchArgument(
            'waypoint_csv',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), 'HYPER', 'src', 'planning', 'hyper_waypoint',
                'waypoints', 'sim.csv']),
            description='미션이 따를 웨이포인트 CSV (실차는 real.csv로 덮어쓰세요)'),
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
                  waypoint_csv=LaunchConfiguration('waypoint_csv'),
                  # 위와 같은 이유. 이쪽은 nav2_controller.launch.py가 RewrittenYaml로
                  # nav2_controller.yaml의 use_sim_time을 덮어쓰므로 인자만 넘기면 됩니다.
                  use_sim_time='false'),
            stage('interface.launch.py'),
            mission_panel,
        ]),
    ])
