import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Real-car equivalent of phone_control_sim.launch.py: same rosbridge_websocket bridge so a phone
# app can publish /steering_angle and /velocity (std_msgs/Float64) directly over
# ws://<host-ip>:<port>, but drives the STM32 hardware interface instead of Gazebo. Deliberately
# does NOT include real.launch.py's sensors/odometry/perception/behavior stages -- hyper_planner's
# controller_with_parking_node also publishes to /steering_angle and /velocity, so running both
# at once would have the phone and the autonomy stack fight over the same two topics.


def generate_launch_description():
    hyper_control_share = get_package_share_directory('hyper_control')

    rosbridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch', 'rosbridge_websocket_launch.xml')),
        launch_arguments={
            'port': LaunchConfiguration('rosbridge_port'),
        }.items(),
    )

    # Publishes body_link -> camera_link/lidar_link/gps_link TF, same as real.launch.py.
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            hyper_control_share, 'launch', 'robot_state_publisher.launch.py')),
    )

    # Starts ros2_control_node against Stm32SystemInterface (use_sim:=false) and spawns
    # joint_state_broadcaster/forward_position_controller/forward_velocity_controller.
    real_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            hyper_control_share, 'launch', 'real_control.launch.py')),
    )

    # single_output:=true: the real interface exposes one virtual steering joint and one virtual
    # rear-axle joint (see Stm32SystemInterface), not four independently commandable wheels, so
    # per-wheel Ackermann geometry doesn't apply on the output end here.
    vehicle_controller_node = Node(
        package='hyper_control',
        executable='vehicle_controller_node',
        parameters=[
            os.path.join(hyper_control_share, 'config', 'parameters.yaml'),
            {'single_output': True},
        ],
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rosbridge_port', default_value='9090',
            description='TCP port for the rosbridge websocket server'),

        robot_state_publisher,
        real_control,
        vehicle_controller_node,
        rosbridge_launch,
    ])
