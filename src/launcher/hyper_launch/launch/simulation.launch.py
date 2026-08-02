import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

# Stages are staggered to give Gazebo time to come up before the nodes that
# depend on it attach -- mirrors run_all.sh's sleep 5 / sleep 2 / sleep 2 gaps
# between terminals.
ODOMETRY_DELAY_S = 5.0
PERCEPTION_DELAY_S = 7.0
BEHAVIOR_DELAY_S = 9.0


def generate_launch_description():
    hyper_launch_share = get_package_share_directory('hyper_launch')

    def stage(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hyper_launch_share, 'launch', name)))

    return LaunchDescription([
        stage('sim.launch.py'),
        TimerAction(period=ODOMETRY_DELAY_S, actions=[stage('odometry.launch.py')]),
        TimerAction(period=PERCEPTION_DELAY_S, actions=[stage('perception.launch.py')]),
        TimerAction(period=BEHAVIOR_DELAY_S, actions=[stage('behavior.launch.py')]),
    ])
