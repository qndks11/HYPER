#!/usr/bin/env python3
"""미션 매니저 -- config/<mission>.yaml의 스텝 큐를 follow_path 액션으로 실행합니다.

nav2 controller_server가 이미 떠 있어야 합니다(nav2_controller.launch.py).

  ros2 launch hyper_planner mission.launch.py                  # config/mission.yaml (대회 미션)
  ros2 launch hyper_planner mission.launch.py mission:=simple  # config/simple.yaml (한 바퀴)

simple은 코스 전체를 골 하나로 보냅니다 -- follow_path_client_node가 하던 일이지만
실주행과 같은 경로 처리(path_loader.hpp)를 거치므로 컨트롤러 튜닝 확인에 그대로 씁니다.
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
    # config/<mission>.yaml. 이름만 받는 이유는 전체 경로를 손으로 적기 번거롭기 때문입니다
    # (share 디렉터리는 `ros2 pkg prefix`를 거쳐야 나옵니다). 절대 경로가 필요하면
    # mission_yaml을 직접 주면 됩니다 -- 그쪽이 이 기본값을 덮어씁니다.
    default_mission = PathJoinSubstitution([
        FindPackageShare('hyper_planner'), 'config',
        [LaunchConfiguration('mission'), '.yaml']])

    return LaunchDescription([
        DeclareLaunchArgument(
            'mission', default_value='mission',
            description='config/<이름>.yaml 중 실행할 미션. 예: mission:=simple (한 바퀴)'),
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
        # cancel-on-arrival: 골까지 이만큼 남고 속도도 아래 값 이하로 떨어지면, 골 판정을
        # 기다리지 않고 취소해 도착으로 칩니다(MPPI가 마지막 수십 cm를 기어가는 것 방지).
        # 여기 값은 mission.yaml의 drive 스텝이 cancel_on_arrival_m를 안 적었을 때의
        # 기본값이고, 0 = 끔입니다. 주차 스텝에서 실수로 켜지면 그만큼 짧게 서 버리므로
        # 기본을 끔으로 두고 필요한 스텝만 mission.yaml에서 켭니다.
        DeclareLaunchArgument('cancel_on_arrival_m', default_value='0.0'),
        DeclareLaunchArgument('cancel_on_arrival_speed', default_value='0.5'),
        # decel 프로파일. mission.yaml의 drive 스텝이 decel_profile_a를 안 적었을 때의
        # 기본값이고, 0 = 끔입니다. 켜면 골 경로를 라벨보다 뒤까지 늘려 MPPI가 스스로
        # 감속하지 않게 하고, 대신 /speed_limit으로 v = sqrt(2*a*d)를 내려보냅니다.
        # 자세한 설명은 mission.yaml의 decel_profile_a 주석을 보세요.
        DeclareLaunchArgument('decel_profile_a', default_value='0.0'),
        # 라벨보다 이만큼 뒤까지 경로를 늘립니다. nav2_controller.yaml의 local_costmap
        # width/height의 절반(= 10 m)보다 커야 MPPI가 경로 끝을 못 봅니다.
        DeclareLaunchArgument('decel_profile_lookahead_m', default_value='12.0'),
        # 프로파일이 내려보내는 속도의 하한. 0이면 안 됩니다 -- SpeedLimit의 0.0은
        # nav2에서 "제한 해제"라 정반대로 동작합니다. cancel_on_arrival_speed보다
        # 낮게 두세요(그래야 정지점에서 취소 조건이 열립니다).
        DeclareLaunchArgument('decel_profile_min_speed', default_value='0.4'),
        # nav2_controller.yaml의 FollowPath.vx_max와 같아야 합니다. MPPI의 setSpeedLimit은
        # ratio = speed_limit / vx_max를 곱하는 방식이라, 더 큰 값을 보내면 vx_max를
        # 오히려 올려 버립니다.
        DeclareLaunchArgument('controller_vx_max', default_value='2.22'),
        # controller_server의 speed_limit_topic과 같아야 합니다.
        DeclareLaunchArgument('speed_limit_topic', default_value='/speed_limit'),
        # 프로파일 주행 중 tf가 이 시간 동안 끊기면 골을 취소해 차를 세웁니다.
        DeclareLaunchArgument('progress_stale_cancel_sec', default_value='1.0'),
        # 경로 위 진행 커서를 옮길 때 앞으로 훑는 거리(m). 커서는 뒤로 가지 않습니다.
        DeclareLaunchArgument('progress_search_window_m', default_value='5.0'),
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
                'cancel_on_arrival_m': ParameterValue(
                    LaunchConfiguration('cancel_on_arrival_m'), value_type=float),
                'cancel_on_arrival_speed': ParameterValue(
                    LaunchConfiguration('cancel_on_arrival_speed'), value_type=float),
                'decel_profile_a': ParameterValue(
                    LaunchConfiguration('decel_profile_a'), value_type=float),
                'decel_profile_lookahead_m': ParameterValue(
                    LaunchConfiguration('decel_profile_lookahead_m'), value_type=float),
                'decel_profile_min_speed': ParameterValue(
                    LaunchConfiguration('decel_profile_min_speed'), value_type=float),
                'controller_vx_max': ParameterValue(
                    LaunchConfiguration('controller_vx_max'), value_type=float),
                'speed_limit_topic': LaunchConfiguration('speed_limit_topic'),
                'progress_stale_cancel_sec': ParameterValue(
                    LaunchConfiguration('progress_stale_cancel_sec'), value_type=float),
                'progress_search_window_m': ParameterValue(
                    LaunchConfiguration('progress_search_window_m'), value_type=float),
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
