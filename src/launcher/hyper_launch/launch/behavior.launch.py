from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_nav2_params = PathJoinSubstitution([
        FindPackageShare('hyper_planner'), 'config', 'nav2_controller.yaml'])
    # mission.launch.py와 같은 규칙입니다: mission은 config/<이름>.yaml을 고르고,
    # mission_yaml은 그 결과를 절대 경로로 덮어씁니다.
    default_mission_yaml = PathJoinSubstitution([
        FindPackageShare('hyper_planner'), 'config',
        [LaunchConfiguration('mission'), '.yaml']])
    default_waypoint_csv = PathJoinSubstitution([
        EnvironmentVariable('HOME'), 'HYPER', 'src', 'planning', 'hyper_waypoint',
        'waypoints', 'sim.csv'])

    return LaunchDescription([
        DeclareLaunchArgument('nav2_params_file', default_value=default_nav2_params),
        DeclareLaunchArgument(
            'mission', default_value='mission',
            description='config/<이름>.yaml 중 실행할 미션. 예: mission:=simple (한 바퀴)'),
        DeclareLaunchArgument('mission_yaml', default_value=default_mission_yaml),
        DeclareLaunchArgument('waypoint_csv', default_value=default_waypoint_csv),
        # mission_manager_node의 '~/start'는 여전히 사람이 직접 호출해야 합니다
        # (mission.launch.py의 auto_start 기본값 false -- 미션 주행은 사람이 시작하는 게 안전).
        DeclareLaunchArgument(
            'use_sim_time', default_value='true',
            description='시뮬레이션은 true, 실차는 false'),

        # 조이스틱 비상정지. 미션 스택에는 /joy를 내보내는 노드가 없으므로
        # launch_joy_node의 기본값을 estop.launch.py(false)와 달리 true로 둡니다.
        # 수동 주행(joystick.launch.py)과 같이 띄울 때는 joy_node가 둘이 되지
        # 않도록 estop_launch_joy_node:=false로 끄세요.
        DeclareLaunchArgument(
            'estop', default_value='true',
            description='조이스틱 비상정지 노드를 같이 띄울지'),
        DeclareLaunchArgument('estop_button_index', default_value='1'),
        DeclareLaunchArgument('resume_button_index', default_value='0'),
        DeclareLaunchArgument('estop_launch_joy_node', default_value='true'),

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

        # /estop(Bool, latched)만 내보냅니다. 실제 정지는 arduino_interface_node가
        # 그 래치를 보고 속도/조향을 0으로 만드는 것으로 이뤄집니다.
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('hyper_control'), 'launch', 'estop.launch.py'])),
            condition=IfCondition(LaunchConfiguration('estop')),
            launch_arguments={
                'estop_button_index': LaunchConfiguration('estop_button_index'),
                'resume_button_index': LaunchConfiguration('resume_button_index'),
                'launch_joy_node': LaunchConfiguration('estop_launch_joy_node'),
            }.items(),
        ),
    ])
