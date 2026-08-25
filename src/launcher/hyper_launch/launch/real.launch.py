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

    # Publishes body_link -> camera_link/lidar_link/gps_link TF from vehicle.xacro, same as
    # hyper_gazebo's vehicle.launch.py does for sim. Lives in hyper_control (not hyper_launch),
    # since vehicle.xacro is owned there. Real vehicle has no ros2_control hardware interface,
    # so wheel/steering joint transforms are not published (no /joint_states source yet) --
    # only the fixed sensor-mount joints resolve.
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_control'),
            'launch', 'robot_state_publisher.launch.py')),
    )

    return LaunchDescription([
        robot_state_publisher,
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
        # interface.launch.py (hyper_interface's Arduino serial bridge) starts alongside
        # behavior since it only needs /velocity + /steering_angle to exist -- late subscriber
        # join works fine with ROS 2 discovery either way.
        TimerAction(period=BEHAVIOR_DELAY_S, actions=[
            stage('behavior.launch.py'),
            stage('interface.launch.py'),
        ]),
    ])
