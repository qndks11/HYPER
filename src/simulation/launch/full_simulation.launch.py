import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim_share = get_package_share_directory('simulation')

    # ---------- arguments ----------
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(sim_share, 'worlds', 'track.sdf'),
        description='Path to Gazebo world SDF',
    )
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='hyper',
        description='Name of the spawned robot entity',
    )
    x_arg = DeclareLaunchArgument('x', default_value='0.0', description='Spawn X')
    y_arg = DeclareLaunchArgument('y', default_value='0.0', description='Spawn Y')
    z_arg = DeclareLaunchArgument('z', default_value='0.1', description='Spawn Z')

    world     = LaunchConfiguration('world')
    robot_name = LaunchConfiguration('robot_name')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')

    # ---------- Gazebo ----------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            ])
        ]),
        launch_arguments={
            'gz_args': '-r ' + os.path.join(sim_share, 'worlds', 'track.sdf'),
        }.items(),
    )

    # ---------- robot model ----------
    urdf_path = os.path.join(sim_share, 'models', 'hyper.urdf.xacro')
    robot_description_content = xacro.process_file(
        urdf_path,
        mappings={'robot_name': robot_name}
    ).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
    )

    # ---------- spawn entity ----------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_hyper',
        arguments=[
            '-name', robot_name,
            '-topic', 'robot_description',
            '-x', x, '-y', y, '-z', z,
        ],
        output='screen',
    )

    # ---------- ROS ↔ Gazebo bridge ----------
    bridge_config = os.path.join(sim_share, 'config', 'bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
        output='screen',
    )

    # ---------- perception nodes ----------
    camera_subscriber = Node(
        package='perception',
        executable='camera_subscriber',
        name='camera_subscriber',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    lane_detector = Node(
        package='perception',
        executable='lane_detector',
        name='lane_detector',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    sign_detector = Node(
        package='perception',
        executable='sign_detector',
        name='sign_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'model_path': os.path.join(sim_share, 'models', 'best.pt'),
        }],
    )

    visualizer = Node(
        package='perception',
        executable='visualizer',
        name='visualizer',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    return LaunchDescription([
        world_arg,
        robot_name_arg,
        x_arg, y_arg, z_arg,
        gazebo,
        robot_state_publisher,
        TimerAction(period=3.0, actions=[spawn_entity]),
        TimerAction(period=4.0, actions=[ros_gz_bridge]),
        TimerAction(period=5.0, actions=[
            camera_subscriber,
            lane_detector,
            sign_detector,
            visualizer,
        ]),
    ])
