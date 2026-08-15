import os

import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def load_robot_description(serial_port, baud_rate):
    """
    Same vehicle.xacro + parameters.yaml pipeline robot_state_publisher.launch.py uses, but with
    use_sim forced to false so vehicle.xacro emits the real <ros2_control> block (hyper_control/
    Stm32SystemInterface) instead of the sim one (gz_ros2_control/GazeboSimSystem).
    """
    package_path = get_package_share_directory('hyper_control')
    xacro_path = os.path.join(package_path, 'urdf', 'vehicle.xacro')
    params_path = os.path.join(package_path, 'config', 'parameters.yaml')

    with open(params_path, 'r') as file:
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']

    mappings = {key: str(value) for key, value in vehicle_params.items()}
    mappings['use_sim'] = 'false'
    mappings['serial_port'] = serial_port
    mappings['baud_rate'] = baud_rate

    robot_description = xacro.process_file(xacro_path, mappings=mappings)
    return robot_description.toxml()


def launch_setup(context, *args, **kwargs):
    # LaunchConfiguration substitutions only resolve to real strings once a LaunchContext
    # exists, so the xacro pipeline (which needs plain strings) has to run in here rather than
    # directly in generate_launch_description().
    serial_port = LaunchConfiguration('serial_port').perform(context)
    baud_rate = LaunchConfiguration('baud_rate').perform(context)

    package_path = get_package_share_directory('hyper_control')
    controllers_yaml_path = os.path.join(package_path, 'config', 'stm32_ros2_control.yaml')

    robot_description = {'robot_description': load_robot_description(serial_port, baud_rate)}

    controller_manager_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml_path],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    forward_position_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_position_controller'],
        output='screen',
    )

    forward_velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['forward_velocity_controller'],
        output='screen',
    )

    return [
        controller_manager_node,
        joint_state_broadcaster_spawner,
        forward_position_controller_spawner,
        forward_velocity_controller_spawner,
    ]


def generate_launch_description():
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/stm32',
        description='STM32 serial device (fixed by udev rule, see README)',
    )
    baud_rate_arg = DeclareLaunchArgument(
        'baud_rate',
        default_value='115200',
        description='STM32 serial link baud rate',
    )

    return LaunchDescription([
        serial_port_arg,
        baud_rate_arg,
        OpaqueFunction(function=launch_setup),
    ])
