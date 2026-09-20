import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('hyper_lidar')
    params_file = os.path.join(share, 'config', 'rplidar_params.yaml')
    filter_file = os.path.join(share, 'config', 'scan_filter.yaml')

    # Full 360 deg goes to /scan_raw; only scan_filter below publishes /scan.
    rplidar_node = Node(
        package='rplidar_ros',
        executable='rplidar_node',
        name='rplidar_node',
        parameters=[params_file],
        remappings=[('scan', 'scan_raw')],
        output='screen',
    )

    # Front half only, matching the sim's forward-180 deg sensor (see scan_filter.yaml).
    scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_filter',
        parameters=[filter_file],
        remappings=[('scan', 'scan_raw'), ('scan_filtered', 'scan')],
        output='screen',
    )

    return LaunchDescription([rplidar_node, scan_filter])
