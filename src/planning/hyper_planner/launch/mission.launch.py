#!/usr/bin/env python3
"""미션 매니저 -- config/mission.yaml의 스텝 큐를 follow_path 액션으로 실행합니다.

nav2 controller_server가 이미 떠 있어야 합니다(nav2_controller.launch.py).
follow_path_client.launch.py의 대체재입니다: 그 노드는 코스 전체를 골 하나로 보내
한 바퀴 도는 용도로 남겨 두고, 미션 주행은 이 launch를 쓰세요.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_csv = PathJoinSubstitution([
        EnvironmentVariable('HOME'), 'HYPER', 'src', 'planning', 'hyper_waypoint',
        'waypoints', 'sim.csv'])
    default_mission = PathJoinSubstitution([
        FindPackageShare('hyper_planner'), 'config', 'mission.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('mission_yaml', default_value=default_mission),
        DeclareLaunchArgument('waypoint_csv', default_value=default_csv),
        DeclareLaunchArgument('action_name', default_value='follow_path'),
        DeclareLaunchArgument('sign_topic', default_value='/perception/sign'),
        # mission.yaml에서 controller/goal_checker를 안 적은 스텝이 쓰는 기본값.
        DeclareLaunchArgument('controller_id', default_value='FollowPath'),
        DeclareLaunchArgument('goal_checker_id', default_value='general_goal_checker'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('robot_base_frame', default_value='body_link'),
        DeclareLaunchArgument('lead_in_spacing_m', default_value='0.5'),
        # 골 직전에 경로를 이 간격(m)으로 다시 깝니다. 0이면 끕니다.
        # 녹화 CSV의 약 0.59 m 간격은 MPPI PathAlignCritic에 ±0.3 m 코너 컷 사각지대를
        # 만듭니다(path_loader.hpp의 resample_path 주석 참고). 이 값을 바꾸면
        # nav2_controller.yaml의 offset_from_furthest 값들도 함께 조정하세요.
        DeclareLaunchArgument('path_resample_spacing_m', default_value='0.25'),
        DeclareLaunchArgument('max_start_distance_m', default_value='0.0'),
        # 컨트롤러가 abort 했어도 차가 목표 이 거리 안이면 도착으로 칩니다.
        DeclareLaunchArgument('arrival_slack_m', default_value='0.6'),
        DeclareLaunchArgument('goal_retry_limit', default_value='2'),
        # 신호를 못 보고 timeout_s가 지나면 그냥 출발할지(true), 미션을 세울지(false).
        DeclareLaunchArgument('proceed_on_signal_timeout', default_value='true'),
        # false: '~/start'를 부를 때까지 대기합니다. 미션 주행은 사람이 시작하는 게 안전합니다.
        DeclareLaunchArgument('auto_start', default_value='false'),
        # 경로 헤더 stamp가 이 노드의 시계로 찍힙니다. 시뮬레이션에서 빼먹으면 벽시계
        # 시각이 들어가 controller_server의 tf 변환이 "Transform data too old"로 실패합니다.
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='시뮬레이션은 true, 실차는 false'),
        Node(
            package='hyper_planner', executable='mission_manager_node',
            name='mission_manager', output='screen',
            parameters=[{
                'mission_yaml': LaunchConfiguration('mission_yaml'),
                'waypoint_csv': LaunchConfiguration('waypoint_csv'),
                'action_name': LaunchConfiguration('action_name'),
                'sign_topic': LaunchConfiguration('sign_topic'),
                'controller_id': LaunchConfiguration('controller_id'),
                'goal_checker_id': LaunchConfiguration('goal_checker_id'),
                'frame_id': LaunchConfiguration('frame_id'),
                'robot_base_frame': LaunchConfiguration('robot_base_frame'),
                'lead_in_spacing_m': ParameterValue(
                    LaunchConfiguration('lead_in_spacing_m'), value_type=float),
                'path_resample_spacing_m': ParameterValue(
                    LaunchConfiguration('path_resample_spacing_m'), value_type=float),
                'max_start_distance_m': ParameterValue(
                    LaunchConfiguration('max_start_distance_m'), value_type=float),
                'arrival_slack_m': ParameterValue(
                    LaunchConfiguration('arrival_slack_m'), value_type=float),
                'goal_retry_limit': ParameterValue(
                    LaunchConfiguration('goal_retry_limit'), value_type=int),
                'proceed_on_signal_timeout': ParameterValue(
                    LaunchConfiguration('proceed_on_signal_timeout'), value_type=bool),
                'auto_start': ParameterValue(
                    LaunchConfiguration('auto_start'), value_type=bool),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ),
    ])
