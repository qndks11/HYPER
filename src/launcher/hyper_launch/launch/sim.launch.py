import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    default_world_path = os.path.join(
        get_package_share_directory('hyper_gazebo'), 'worlds', 'track.world')

    return LaunchDescription([
        DeclareLaunchArgument(
            'world', default_value=default_world_path,
            description='Specify the world file for Gazebo'),
        DeclareLaunchArgument('x', default_value='41.0866', description='Initial X position'),
        DeclareLaunchArgument('y', default_value='-45.6842', description='Initial Y position'),
        DeclareLaunchArgument('z', default_value='0.36', description='Initial Z position'),
        DeclareLaunchArgument('R', default_value='0.00', description='Initial Roll'),
        DeclareLaunchArgument('P', default_value='0.00', description='Initial Pitch'),
        DeclareLaunchArgument('Y', default_value='1.64', description='Initial Yaw'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('hyper_gazebo'),
                'launch', 'vehicle.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'R': LaunchConfiguration('R'),
                'P': LaunchConfiguration('P'),
                'Y': LaunchConfiguration('Y'),
            }.items(),
        ),
    ])
