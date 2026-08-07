#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('hyper_waypoint_tracker'), 'config', 'waypoint_tracker_params.yaml'])
    default_waypoints = PathJoinSubstitution([
        EnvironmentVariable('HOME'), 'HYPER', 'src', 'planning',
        'hyper_waypoint_tracker', 'waypoints', 'waypoints.yaml'])

    params_file = LaunchConfiguration('params_file')
    waypoints_yaml = LaunchConfiguration('waypoints_yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('waypoints_yaml', default_value=default_waypoints),
        Node(
            package='hyper_waypoint_tracker', executable='waypoint_tracker_node',
            name='waypoint_tracker', output='screen',
            parameters=[params_file, {'waypoints_yaml': waypoints_yaml}],
        ),
    ])
