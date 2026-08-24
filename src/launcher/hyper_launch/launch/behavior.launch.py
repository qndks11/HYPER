from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_nav2_params = PathJoinSubstitution([
        FindPackageShare('hyper_planner'), 'config', 'nav2_controller.yaml'])
    default_mission_yaml = PathJoinSubstitution([
        FindPackageShare('hyper_planner'), 'config', 'mission.yaml'])
    default_waypoint_csv = PathJoinSubstitution([
        EnvironmentVariable('HOME'), 'HYPER', 'src', 'planning', 'hyper_waypoint',
        'waypoints', 'sim.csv'])

    return LaunchDescription([
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params),
        DeclareLaunchArgument('mission_yaml', default_value=default_mission_yaml),
        DeclareLaunchArgument('waypoint_csv', default_value=default_waypoint_csv),
        # mission_manager_node의 '~/start'는 여전히 사람이 직접 호출해야 합니다
        # (mission.launch.py의 auto_start 기본값 false -- 미션 주행은 사람이 시작하는 게 안전).
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='시뮬레이션은 true, 실차는 false'),

        # controller_server(follow_path 액션 서버) + lifecycle_manager + cmd_vel 변환.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('hyper_planner'), 'launch', 'nav2_controller.launch.py'])),
            launch_arguments={
                'params_file': LaunchConfiguration('nav2_params_file'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),

        # config/mission.yaml의 스텝 큐를 실행하는 미션 매니저.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('hyper_planner'), 'launch', 'mission.launch.py'])),
            launch_arguments={
                'mission_yaml': LaunchConfiguration('mission_yaml'),
                'waypoint_csv': LaunchConfiguration('waypoint_csv'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])
