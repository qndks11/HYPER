import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

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
    # since vehicle.xacro is owned there.
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_control'),
            'launch', 'robot_state_publisher.launch.py')),
    )

    # Real vehicle equivalent of Gazebo's gz_ros2_control plugin: starts ros2_control_node
    # against the STM32 hardware interface (use_sim:=false) and spawns joint_state_broadcaster/
    # forward_position_controller/forward_velocity_controller, so wheel/steering joint
    # transforms (/joint_states) and the controller command topics vehicle_controller_node
    # publishes to actually exist on the real vehicle.
    real_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_control'),
            'launch', 'real_control.launch.py')),
    )

    # Same Ackermann-to-controller-command bridge as hyper_gazebo's vehicle.launch.py, but with
    # single_output:=true: the real interface exposes one virtual steering joint and one virtual
    # rear-axle joint (see Stm32SystemInterface), not four independently commandable wheels, so
    # per-wheel Ackermann geometry doesn't apply on the output end here.
    vehicle_controller_node = Node(
        package='hyper_control',
        executable='vehicle_controller_node',
        parameters=[
            os.path.join(
                get_package_share_directory('hyper_control'), 'config', 'parameters.yaml'),
            {'single_output': True},
        ],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        real_control,
        vehicle_controller_node,
        stage('sensors.launch.py'),
        TimerAction(period=ODOMETRY_DELAY_S, actions=[stage('odometry.launch.py')]),
        # Real vehicle: hyper_camera owns both physical cameras and publishes plain image
        # topics -- the ELP publisher loads into the same component container as
        # lane_detection_node for zero-copy intra-process delivery (lane_input_backend
        # intra_process); the Logitech publisher just feeds object_detection_node's ordinary
        # subscription (object_input_backend usb_camera). See perception.launch.py.
        TimerAction(period=PERCEPTION_DELAY_S, actions=[
            stage(
                'perception.launch.py',
<<<<<<< HEAD
                lane_input_backend='intra_process',
                object_input_backend='usb_camera')]),
        TimerAction(period=BEHAVIOR_DELAY_S, actions=[stage('behavior.launch.py')]),
=======
                lane_input_backend='direct_usb',
                object_input_backend='direct_usb')]),
        # interface.launch.py (hyper_interface's Arduino serial bridge) starts alongside
        # behavior since it only needs /velocity + /steering_angle to exist -- late subscriber
        # join works fine with ROS 2 discovery either way.
        TimerAction(period=BEHAVIOR_DELAY_S, actions=[
            stage('behavior.launch.py'),
            stage('interface.launch.py'),
        ]),
>>>>>>> 6f6b24572a732012e5e0cfd43b1be11618faaf9a
    ])
