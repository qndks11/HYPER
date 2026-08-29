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

        # Forwarded to hyper_lane_detection's drivable.enabled parameter. Off by default -- see
        # the argument's own comment in hyper_object_detection's perception.launch.py, and note
        # that the consumer (drivable_area_layer in hyper_planner's nav2_controller.yaml) has to
        # be enabled separately for this to change what the vehicle does.
        DeclareLaunchArgument('drivable_area', default_value='false'),

        # Forwarded to the depthimage_to_laserscan node that turns the rear camera's depth
        # stream into /scan_rear. Off by default -- see the argument's own comment in
        # hyper_object_detection's perception.launch.py. Only the sim publishes rear depth
        # today, so only simulation.launch.py has a reason to pass this true.
        DeclareLaunchArgument('rear_scan', default_value='false'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('hyper_object_detection'),
                'launch', 'perception.launch.py')),
            launch_arguments={
                'lane_input_backend': LaunchConfiguration('lane_input_backend'),
                'object_input_backend': LaunchConfiguration('object_input_backend'),
                'drivable_area': LaunchConfiguration('drivable_area'),
                'rear_scan': LaunchConfiguration('rear_scan'),
            }.items(),
        ),
    ])
