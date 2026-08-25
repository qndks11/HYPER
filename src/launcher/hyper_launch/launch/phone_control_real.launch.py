import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Real-car equivalent of phone_control_sim.launch.py: same rosbridge_websocket bridge so a phone
# app can publish /steering_angle and /velocity (std_msgs/Float64) directly over
# ws://<host-ip>:<port>, but drives the Arduino motor-driver interface instead of Gazebo.
# arduino_interface_node subscribes to those same two topics directly (see
# hyper_interface/README.md), so no vehicle_controller_node/ros2_control translation step sits
# in between here. Deliberately does NOT include real.launch.py's sensors/odometry/perception/
# behavior stages -- hyper_planner's controller_with_parking_node also publishes to
# /steering_angle and /velocity, so running both at once would have the phone and the autonomy
# stack fight over the same two topics.


def generate_launch_description():
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
            get_package_share_directory('hyper_control'),
            'launch', 'robot_state_publisher.launch.py')),
    )

    # ROS <-> Arduino serial bridge, same as real.launch.py's interface.launch.py stage.
    arduino_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_interface'),
            'launch', 'interface.launch.py')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rosbridge_port', default_value='9090',
            description='TCP port for the rosbridge websocket server'),

        robot_state_publisher,
        arduino_interface,
        rosbridge_launch,
    ])
