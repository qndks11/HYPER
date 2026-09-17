import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # track | school. 월드(worlds/<site>.world), datum 대조, 스폰 기본값을 고릅니다.
        DeclareLaunchArgument(
            'site', default_value='track', choices=['track', 'school'],
            description='Which site to simulate (track | school)'),
        # 빈 값이면 hyper_gazebo/vehicle.launch.py가 site 프리셋으로 채웁니다.
        DeclareLaunchArgument(
            'world', default_value='',
            description='World file for Gazebo (empty = worlds/<site>.world)'),
        DeclareLaunchArgument('x', default_value='', description='Initial X position (empty = site preset)'),
        DeclareLaunchArgument('y', default_value='', description='Initial Y position (empty = site preset)'),
        DeclareLaunchArgument('z', default_value='0.36', description='Initial Z position'),
        DeclareLaunchArgument('R', default_value='0.00', description='Initial Roll'),
        DeclareLaunchArgument('P', default_value='0.00', description='Initial Pitch'),
        DeclareLaunchArgument('Y', default_value='', description='Initial Yaw (empty = site preset)'),
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo without the 3D GUI window (sensors still render offscreen)'),
        DeclareLaunchArgument(
            'software_rendering', default_value='false',
            description='Force llvmpipe software rendering. Only needed on WSL2.'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('hyper_gazebo'),
                'launch', 'vehicle.launch.py')),
            launch_arguments={
                'site': LaunchConfiguration('site'),
                'world': LaunchConfiguration('world'),
                'x': LaunchConfiguration('x'),
                'y': LaunchConfiguration('y'),
                'z': LaunchConfiguration('z'),
                'R': LaunchConfiguration('R'),
                'P': LaunchConfiguration('P'),
                'Y': LaunchConfiguration('Y'),
                'headless': LaunchConfiguration('headless'),
                'software_rendering': LaunchConfiguration('software_rendering'),
            }.items(),
        ),
    ])
