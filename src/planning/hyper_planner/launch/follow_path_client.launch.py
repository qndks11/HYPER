#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_csv = PathJoinSubstitution([
        EnvironmentVariable('HOME'), 'HYPER', 'src', 'planning', 'hyper_waypoint',
        'waypoints', 'sim.csv'])

    return LaunchDescription([
        DeclareLaunchArgument('waypoint_csv', default_value=default_csv),
        DeclareLaunchArgument('action_name', default_value='follow_path'),
        DeclareLaunchArgument('controller_id', default_value='FollowPath'),
        DeclareLaunchArgument('goal_checker_id', default_value='general_goal_checker'),
        DeclareLaunchArgument('frame_id', default_value='map'),
        DeclareLaunchArgument('min_spacing_m', default_value='0.0'),
        # 경로의 시작점이 아니라 차량에서 가장 가까운 웨이포인트부터 따라갑니다.
        # false로 두면 CSV 첫 점이 로컬 costmap(20 x 20 m) 밖일 때 컨트롤러가 바로 abort 합니다.
        DeclareLaunchArgument('start_from_nearest', default_value='true'),
        DeclareLaunchArgument('robot_base_frame', default_value='body_link'),
        # 가장 가까운 점까지 직선 진입 경로를 이 간격으로 깔아 줍니다(0이면 끔).
        DeclareLaunchArgument('lead_in_spacing_m', default_value='0.5'),
        # 가장 가까운 웨이포인트가 이보다 멀면 전송을 거부합니다(0이면 제한 없음).
        DeclareLaunchArgument('max_start_distance_m', default_value='0.0'),
        DeclareLaunchArgument('auto_start', default_value='true'),
        # 경로 헤더 stamp가 이 노드의 시계로 찍히므로, 시뮬레이션에서 이 값을 빼먹으면
        # 벽시계 시각이 들어가 controller_server의 목표 지점 tf 변환이 실패합니다
        # ("Transform data too old") -> 변환 실패 시 목표가 (0,0)으로 남아 출발 직후
        # 목표 도달로 판정되어 액션이 바로 성공해 버립니다.
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='시뮬레이션은 true, 실차는 false'),
        Node(
            package='hyper_planner', executable='follow_path_client_node',
            name='follow_path_client', output='screen',
            parameters=[{
                'waypoint_csv': LaunchConfiguration('waypoint_csv'),
                'action_name': LaunchConfiguration('action_name'),
                'controller_id': LaunchConfiguration('controller_id'),
                'goal_checker_id': LaunchConfiguration('goal_checker_id'),
                'frame_id': LaunchConfiguration('frame_id'),
                'min_spacing_m': ParameterValue(
                    LaunchConfiguration('min_spacing_m'), value_type=float),
                'start_from_nearest': ParameterValue(
                    LaunchConfiguration('start_from_nearest'), value_type=bool),
                'robot_base_frame': LaunchConfiguration('robot_base_frame'),
                'lead_in_spacing_m': ParameterValue(
                    LaunchConfiguration('lead_in_spacing_m'), value_type=float),
                'max_start_distance_m': ParameterValue(
                    LaunchConfiguration('max_start_distance_m'), value_type=float),
                'auto_start': ParameterValue(
                    LaunchConfiguration('auto_start'), value_type=bool),
                'use_sim_time': ParameterValue(
                    LaunchConfiguration('use_sim_time'), value_type=bool),
            }],
        ),
    ])
