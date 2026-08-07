import os

import xacro
import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def load_robot_description():
    """
    Same vehicle.xacro + parameters.yaml pipeline hyper_gazebo's vehicle.launch.py uses to
    spawn the sim robot, minus anything Gazebo-specific -- the <gazebo>/<ros2_control> tags
    in vehicle.xacro are inert here, robot_state_publisher only reads <link>/<joint>.
    """
    package_path = get_package_share_directory('hyper_control')
    xacro_path = os.path.join(package_path, 'urdf', 'vehicle.xacro')
    params_path = os.path.join(package_path, 'config', 'parameters.yaml')

    with open(params_path, 'r') as file:
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']

    robot_description = xacro.process_file(
        xacro_path,
        mappings={key: str(value) for key, value in vehicle_params.items()}
    )

    return robot_description.toxml()


def generate_launch_description():
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': load_robot_description(),
            'use_sim_time': False,
        }],
        output='screen',
    )

    return LaunchDescription([robot_state_publisher_node])
