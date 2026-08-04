import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    imu_config = os.path.join(
        get_package_share_directory('hyper_imu'),
        'config',
        'imu_params.yaml'
    )

    imu_node = Node(
        package='hyper_imu',
        executable='witmotion_ble_node',
        name='witmotion_ble_node',
        output='screen',
        parameters=[imu_config],
    )

    return LaunchDescription([imu_node])
