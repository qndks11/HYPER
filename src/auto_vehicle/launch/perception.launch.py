import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('auto_vehicle'), 'models', 'best.pt')

    bev_producer_node = Node(
        package='auto_vehicle',
        executable='bev_producer',
        remappings=[('/image_raw', '/camera/image_raw')],
        output='screen'
    )

    lane_detection_node = Node(
        package='auto_vehicle',
        executable='lane_detection',
        output='screen'
    )

    stopline_detection_node = Node(
        package='auto_vehicle',
        executable='stopline_detection',
        output='screen'
    )

    visualizer_node = Node(
        package='auto_vehicle',
        executable='visualizer',
        output='screen'
    )

    object_detection_node = Node(
        package='auto_vehicle',
        executable='object_detection',
        remappings=[('/image_raw', '/camera/image_raw')],
        parameters=[{'model_path': model_path}],
        output='screen'
    )

    return LaunchDescription([
        bev_producer_node, lane_detection_node, stopline_detection_node, visualizer_node,
        object_detection_node
    ])