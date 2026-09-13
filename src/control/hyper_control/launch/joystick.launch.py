import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    package_name = "hyper_control"

    vehicle_params_path = os.path.join(get_package_share_directory(package_name),
                                       'config', 'parameters.yaml')

    # Seconds between /velocity + /steering_angle publishes -- default (0.0)
    # leaves joystick_controller_node's own 0.01s (100Hz) default in place.
    # Raise this (e.g. 0.1 for 10Hz) to slow down how often commands go out,
    # without recompiling: `ros2 launch hyper_control joystick.launch.py
    # joystick_publish_period:=0.1`
    joystick_publish_period_arg = DeclareLaunchArgument(
        'joystick_publish_period', default_value='0.0')

    joy_node = Node(package="joy", executable="joy_node")

    joystick_controller_node = Node(
        package=package_name,
        executable='joystick_controller_node',
        parameters=[vehicle_params_path, {
            'joystick_publish_period': LaunchConfiguration('joystick_publish_period'),
        }],
        output='screen')

    # 로컬 코스트맵(/local_costmap/costmap)을 미션 없이 보기 위한 nav2 controller_server +
    # lifecycle_manager. controller_server는 목표가 없어도 활성화되면 코스트맵을
    # publish하고, 목표를 받기 전까지 /cmd_vel은 내지 않습니다.
    # use_cmd_vel_to_ackermann:=false가 핵심입니다 -- 그 노드는 /cmd_vel이 없으면 워치독이
    # /velocity + /steering_angle에 0.0을 계속 내보내 위 joystick_controller_node와 싸웁니다.
    # 코스트맵은 odom -> body_link TF(odometry)와 /scan(sensors)이 나올 때까지 기다립니다.
    # 실차 전용이라 use_sim_time은 false입니다.
    use_costmap_arg = DeclareLaunchArgument(
        'use_costmap', default_value='true',
        description='nav2 로컬 코스트맵(/local_costmap/costmap)을 같이 띄웁니다')

    costmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_planner'),
            'launch', 'nav2_controller.launch.py')),
        condition=IfCondition(LaunchConfiguration('use_costmap')),
        launch_arguments={
            'use_sim_time': 'false',
            'use_cmd_vel_to_ackermann': 'false',
        }.items())

    return LaunchDescription([joystick_publish_period_arg,
                              use_costmap_arg,
                              joy_node,
                              joystick_controller_node,
                              costmap])