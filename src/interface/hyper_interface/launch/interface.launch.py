import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('hyper_interface'), 'config', 'parameters.yaml')

    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_params)
    # CH340-based board -- shows up as /dev/ttyUSB*, not /dev/ttyACM* (see
    # parameters.yaml's serial_port comment).
    serial_port_arg = DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0')

    arduino_interface_node = Node(
        package='hyper_interface',
        executable='arduino_interface_node',
        parameters=[
            LaunchConfiguration('params_file'),
            {'serial_port': LaunchConfiguration('serial_port')},
        ],
        output='screen',
    )

    return LaunchDescription([
        params_file_arg,
        serial_port_arg,
        arduino_interface_node,
    ])
