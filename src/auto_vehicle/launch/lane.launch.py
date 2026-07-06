from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    lane_detection_node = Node(
        package='auto_vehicle',
        executable='lane_detection',
        remappings=[('/image_raw', '/camera/image_raw')],
        output='screen'
    )

    return LaunchDescription([lane_detection_node])
