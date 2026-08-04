import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Real-car equivalent of simulation.launch.py: sensors replace Gazebo, but the
# same staggered stage delays apply so odometry/perception/behavior attach
# once sensor drivers (RTK fix lock, D435i firmware init, etc.) are up.
ODOMETRY_DELAY_S = 5.0
PERCEPTION_DELAY_S = 7.0
BEHAVIOR_DELAY_S = 9.0


def generate_launch_description():
    hyper_launch_share = get_package_share_directory('hyper_launch')

    def stage(name, **launch_arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hyper_launch_share, 'launch', name)),
            launch_arguments=launch_arguments.items())

    return LaunchDescription([
        stage('sensors.launch.py'),
        TimerAction(period=ODOMETRY_DELAY_S, actions=[stage('odometry.launch.py')]),
        # Real vehicle: lane_detection_node and object_detection_node each own their camera
        # directly (input_backend direct_usb) -- hyper_camera is no longer started for either,
        # see sensors.launch.py.
        TimerAction(period=PERCEPTION_DELAY_S, actions=[
            stage(
                'perception.launch.py',
                lane_input_backend='direct_usb',
                object_input_backend='direct_usb')]),
        TimerAction(period=BEHAVIOR_DELAY_S, actions=[stage('behavior.launch.py')]),
    ])
