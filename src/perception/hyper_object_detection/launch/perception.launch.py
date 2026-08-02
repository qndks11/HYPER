import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('hyper_object_detection'), 'models', 'best.pt')

    lane_detection_node = Node(
        package='hyper_lane_detection',
        executable='lane_detection',
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/rear_image_raw', '/camera_rear/image_raw'),
        ],
        output='screen'
    )

    object_detection_node = Node(
        package='hyper_object_detection',
        executable='object_detection',
        remappings=[('/image_raw', '/camera_object/image_raw')],
        parameters=[{'model_path': model_path}],
        output='screen'
    )

    return LaunchDescription([lane_detection_node, object_detection_node])