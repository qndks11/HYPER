#!/usr/bin/env python3
"""nav2 controller_server(follow_path 액션 서버) + /cmd_vel -> 아커만 변환 노드.

follow_path_client.launch.py는 이 launch가 띄운 액션 서버로 경로를 보냅니다.

use_cmd_vel_to_ackermann:=false면 controller_server + lifecycle_manager만 뜹니다 -- 조이스틱으로
몰면서 로컬 코스트맵(/local_costmap/costmap)만 보고 싶을 때 씁니다. 코스트맵은 목표 없이도
controller_server가 활성화되면 publish되지만, cmd_vel_to_ackermann_node는 /cmd_vel이 없으면
워치독이 /velocity + /steering_angle에 0.0을 계속 내보내 스틱 명령과 싸우므로 빼야 합니다.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
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
        DeclareLaunchArgument(
            'use_cmd_vel_to_ackermann', default_value='true',
            description='false면 /velocity + /steering_angle을 내지 않습니다 (조이스틱 주행 중 코스트맵만 볼 때)'),
        Node(
            package='nav2_controller', executable='controller_server',
            name='controller_server', output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='hyper_planner', executable='cmd_vel_to_ackermann_node',
            name='cmd_vel_to_ackermann', output='screen',
            parameters=[configured_params],
            condition=IfCondition(LaunchConfiguration('use_cmd_vel_to_ackermann')),
        ),
        Node(
            package='nav2_lifecycle_manager', executable='lifecycle_manager',
            name='lifecycle_manager_control', output='screen',
            parameters=[configured_params],
        ),
    ])
