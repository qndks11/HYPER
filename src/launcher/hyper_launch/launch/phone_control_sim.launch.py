import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Same Gazebo sim as sim.launch.py (spawns the vehicle + vehicle_controller_node, which already
# subscribes to /velocity and /steering_angle std_msgs/Float64), plus a rosbridge_websocket
# server so a phone app can publish those two topics directly over ws://<host-ip>:<port>
# without needing a native ROS 2 node. Also brings up hyper_localization's dual EKF +
# navsat_transform (odometry/filtered_map etc.) so GPS/pose consumers work under phone teleop too.


def generate_launch_description():
    default_world_path = os.path.join(
        get_package_share_directory('hyper_gazebo'), 'worlds', 'track.world')

    rosbridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch', 'rosbridge_websocket_launch.xml')),
        launch_arguments={
            'port': LaunchConfiguration('rosbridge_port'),
        }.items(),
    )

    odometry_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_localization'),
            'launch', 'odometry.launch.py')),
        launch_arguments={
            'datum_site': LaunchConfiguration('datum_site'),
        }.items(),
    )

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
        DeclareLaunchArgument(
            'rosbridge_port', default_value='9090',
            description='TCP port for the rosbridge websocket server'),
        DeclareLaunchArgument(
            'datum_site', default_value='sim',
            description="GPS origin for navsat_transform, keyed into hyper_localization's "
                        "config/datums.yaml (e.g. 'sim', 'school', 'track')."),

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

        odometry_launch,
        rosbridge_launch,
    ])
