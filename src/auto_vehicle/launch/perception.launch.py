import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('auto_vehicle'), 'models', 'best.pt')

    perception_node = Node(
        package='auto_vehicle',
        executable='perception',
        remappings=[('/image_raw', '/camera/image_raw')],
        output='screen'
    )

    object_detection_node = Node(
        package='auto_vehicle',
        executable='object_detection',
        remappings=[('/image_raw', '/camera/image_raw')],
        parameters=[{'model_path': model_path}],
        output='screen'
    )

    return LaunchDescription([perception_node, object_detection_node])