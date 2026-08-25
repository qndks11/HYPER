import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
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

    # Same Ackermann-to-controller-command bridge as hyper_gazebo's vehicle.launch.py, but with
    # single_output:=true: the real interface exposes one virtual steering joint and one virtual
    # rear-axle joint (see Stm32SystemInterface), not four independently commandable wheels, so
    # per-wheel Ackermann geometry doesn't apply on the output end here.
    vehicle_controller_node = Node(
        package='hyper_control',
        executable='vehicle_controller_node',
        parameters=[
            os.path.join(
                get_package_share_directory('hyper_control'), 'config', 'parameters.yaml'),
            {'single_output': True},
        ],
        output='screen',
    )

    return LaunchDescription([
        # 어떤 미션을 실을지. hyper_planner/config/<이름>.yaml로 풀립니다.
        # mission:=simple 이면 코스 한 바퀴만 도는 단일 골 미션입니다.
        DeclareLaunchArgument('mission', default_value='mission'),
        robot_state_publisher,
        vehicle_controller_node,
        stage('sensors.launch.py'),
        TimerAction(period=ODOMETRY_DELAY_S, actions=[stage('odometry.launch.py')]),
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
        TimerAction(period=BEHAVIOR_DELAY_S, actions=[
            stage('behavior.launch.py', mission=LaunchConfiguration('mission'))]),
    ])
