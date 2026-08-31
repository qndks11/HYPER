#!/usr/bin/env python3
"""nav2 controller_server(follow_path 액션 서버) + /cmd_vel -> 아커만 변환 노드.

follow_path_client.launch.py는 이 launch가 띄운 액션 서버로 경로를 보냅니다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('hyper_planner'), 'config', 'nav2_controller.yaml'])

    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # use_sim_time은 controller_server 본체와 그 안의 local_costmap 양쪽에 들어가야 하므로
    # 중첩된 키까지 한 번에 바꿔주는 RewrittenYaml을 씁니다.
    configured_params = RewrittenYaml(
        source_file=params_file,
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='시뮬레이션은 true, 실차는 false'),
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='hyper_planner', executable='cmd_vel_to_ackermann_node',
            name='cmd_vel_to_ackermann', output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_control', output='screen',
            parameters=[configured_params],
        ),
    ])
