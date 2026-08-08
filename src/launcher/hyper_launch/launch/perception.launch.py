import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # Forwarded to hyper_lane_detection's input_backend parameter -- real.launch.py and
        # simulation.launch.py each override this with their own default when including this
        # stage; see stage()'s launch_arguments in both.
        DeclareLaunchArgument('lane_input_backend', default_value='intra_process'),

        # Forwarded to object_detection_node's input_backend parameter -- real.launch.py
        # overrides this with its own default when including this stage; see stage()'s
        # launch_arguments there.
        DeclareLaunchArgument('object_input_backend', default_value='ros_raw'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('hyper_object_detection'),
                'launch', 'perception.launch.py')),
            launch_arguments={
                'lane_input_backend': LaunchConfiguration('lane_input_backend'),
                'object_input_backend': LaunchConfiguration('object_input_backend'),
            }.items(),
        ),
    ])
