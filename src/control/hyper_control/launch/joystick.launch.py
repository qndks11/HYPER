import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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

    return LaunchDescription([joystick_publish_period_arg,
                              joy_node,
                              joystick_controller_node])