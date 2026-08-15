#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('hyper_planner'), 'config', 'parking_params.yaml'])
    default_course = PathJoinSubstitution([
        EnvironmentVariable('HOME'), 'HYPER', 'src', 'waypoint', 'course.yaml'])

    params_file = LaunchConfiguration('params_file')
    course_yaml = LaunchConfiguration('course_yaml')

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('course_yaml', default_value=default_course),
        Node(
            package='hyper_planner', executable='behavior_supervisor_with_parking_node',
            name='behavior_supervisor', output='screen',
            parameters=[params_file, {'event_path_yaml': course_yaml}],
        ),
        Node(
            package='hyper_planner', executable='controller_with_parking_node',
            name='controller', output='screen', parameters=[params_file],
        ),
    ])
