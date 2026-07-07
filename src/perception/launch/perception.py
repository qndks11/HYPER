from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='perception',
            executable='camera_subscriber',
            name='camera_subscriber',
            output='screen'
        ),

        Node(
            package='perception',
            executable='lane_detector',
            name='lane_detector',
            output='screen'
        ),

        Node(
            package='perception',
            executable='sign_detector',
            name='sign_detector',
            output='screen'
        ),
    ])