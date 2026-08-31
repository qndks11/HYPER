import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('hyper_ebimu'),
        'config', 'ebimu.yaml')

    ebimu_node = Node(
        package='hyper_ebimu',
        executable='ebimu_node',
        name='ebimu_node',
        output='screen',
        parameters=[config],
    )

    return LaunchDescription([ebimu_node])
