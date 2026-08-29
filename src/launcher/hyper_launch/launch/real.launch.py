import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

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
        robot_state_publisher,
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
        ]),
    ])
