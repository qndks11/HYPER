import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# Stages are staggered to give Gazebo time to come up before the nodes that
# depend on it attach -- mirrors run_all.sh's sleep 5 / sleep 2 / sleep 2 gaps
# between terminals.
ODOMETRY_DELAY_S = 5.0
PERCEPTION_DELAY_S = 7.0
BEHAVIOR_DELAY_S = 9.0


def generate_launch_description():
    hyper_launch_share = get_package_share_directory('hyper_launch')

    def stage(name, **launch_arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hyper_launch_share, 'launch', name)),
            launch_arguments=launch_arguments.items())

    # real.launch.py의 rviz와 같은 설정(costmap, planner/MPPI 경로, footprint)을 씁니다.
    # headless:=true로 Gazebo 3D 창을 끈 채 이걸 켜면 한 명령으로 "창 없는 시뮬 + rviz"가
    # 됩니다. real.launch.py와 마찬가지로 기본값 true입니다 -- 별도 터미널에서 rviz2를
    # 직접 띄우고 싶으면 use_rviz:=false로 끄세요.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('hyper_planner'),
            'config', 'follow_path.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    # hyper_rqt의 "HYPER Panel" -- mission_manager의 start/cancel/skip/restart,
    # model_service의 spawn/remove, teleport_service를 버튼으로 모아 둔 창입니다
    # (버튼 목록은 hyper_rqt/config/panel.yaml). auto_start가 false라 미션은 사람이
    # Start를 눌러야 출발하므로, headless 시뮬에서는 이 패널이 사실상 조작 창입니다.
    #
    # name=을 주지 않는 것이 중요합니다: launch_ros가 name을 붙이면 --ros-args -r
    # __node:=... 가 argv에 끼는데, 이 실행 파일은 rqt_gui의 Main()이 argv를 직접
    # 파싱하므로 알 수 없는 인자로 보고 죽습니다.
    #
    # behavior 스테이지와 같은 타이밍에 띄웁니다. panel.yaml의 select들이
    # apply_on_start로 /model_service와 /teleport_service의 파라미터를 건드리고
    # 상단 상태줄은 /mission_manager/status를 읽으므로, 그 노드들보다 먼저 떠 봐야
    # 대상이 없습니다.
    def mission_panel():
        return Node(
            package='hyper_rqt',
            executable='hyper_panel',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_panel')),
        )

    # waypoint_csv:=real.csv 처럼 파일명만 준 경우 hyper_waypoint/waypoints/ 아래로 풀어 줍니다.
    waypoint_csv_resolved = PathJoinSubstitution([
        EnvironmentVariable('HOME'), 'HYPER', 'src', 'planning', 'hyper_waypoint',
        'waypoints', LaunchConfiguration('waypoint_csv')])

    def behavior_stage(**extra):
        return stage('behavior.launch.py',
                     mission=LaunchConfiguration('mission'), **extra)

    return LaunchDescription([
        # 어떤 미션을 실을지. hyper_planner/config/<이름>.yaml로 풀립니다.
        # mission:=simple 이면 코스 한 바퀴만 도는 단일 골 미션입니다.
        DeclareLaunchArgument('mission', default_value='mission'),
        # 차량 스폰 위치/방위(map 프레임). 기본은 sim.csv 시작점입니다. real.csv처럼
        # 다른 곳에서 녹화한 경로를 시뮬에서 따라가려면 그 CSV의 0번 행 x/y/yaw로
        # 스폰시켜야 리드인이 코스 전체를 가로지르는 직선으로 안 잡힙니다.
        # 예: real.csv 시작점 -> x:=-18.7494 y:=27.8460 Y:=-1.8681
        DeclareLaunchArgument('x', default_value='41.0866', description='Initial X position'),
        DeclareLaunchArgument('y', default_value='-45.6842', description='Initial Y position'),
        DeclareLaunchArgument('Y', default_value='1.64', description='Initial Yaw (rad)'),
        # navsat_transform 원점. hyper_localization/config/datums.yaml의 키
        # (sim | school | track). 기본값 sim은 track.world의 <spherical_coordinates>와
        # 맞는 시뮬 원점입니다. datum_site:=track이면 실차 트랙 좌표로 시뮬을 돌립니다.
        DeclareLaunchArgument('datum_site', default_value='sim'),
        # behavior 스테이지가 mission_manager에 넘길 웨이포인트 CSV. 기본은
        # hyper_waypoint/waypoints/sim.csv (behavior.launch.py의 기본값). 절대 경로로도,
        # waypoints/ 아래 파일명(real.csv 등)으로도 넘길 수 있게 아래에서 풀어 줍니다.
        DeclareLaunchArgument(
            'waypoint_csv', default_value='',
            description='웨이포인트 CSV. 파일명만 주면 hyper_waypoint/waypoints/ 아래에서 찾습니다'),
        # headless:=true면 Gazebo 3D 창을 띄우지 않습니다. 센서 렌더링은 오프스크린으로
        # 그대로 돌아가므로 카메라/라이다 토픽은 동일하게 나오고, 시각화는 rviz로 하면 됩니다.
        DeclareLaunchArgument(
            'headless', default_value='true',
            description='Run Gazebo without the 3D GUI window (sensors still render offscreen)'),
        # 렌더링은 기본이 네이티브 GPU입니다. WSL2에서만 software_rendering:=true가 필요합니다
        # (WSL2 가상 GPU가 ign gazebo를 죽이는 문제 우회 -- vehicle.launch.py 주석 참고).
        DeclareLaunchArgument(
            'software_rendering', default_value='false',
            description='Force llvmpipe software rendering. Only needed on WSL2.'),
        # 전방 카메라의 색 기반 주행가능영역 분류(/lane/drivable_area)를 켭니다. 기본은 off --
        # 이 토픽을 실제로 읽는 쪽(hyper_planner/config/nav2_controller.yaml의 local_costmap
        # plugins에 있는 drivable_area_layer)도 따로 켜야 주행이 달라집니다. 켜기 전에
        # /lane/drivable/image_raw를 rqt_image_view로 먼저 확인하세요.
        DeclareLaunchArgument(
            'drivable_area', default_value='false',
            description="Publish the camera drivable-area grid for nav2's DrivableAreaLayer"),
        # headless:=true와 짝지어 쓰는 인자. Gazebo 창 대신 rviz로 봅니다. 끄려면 false.
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz with the follow_path nav2 view'),
        # 미션 조작 GUI (hyper_rqt HYPER Panel). Start를 눌러야 미션이 출발하므로, 이 패널이
        # 없으면 서비스 콜을 직접 해야 합니다 -- 그래서 기본값 true. 끄려면 false.
        DeclareLaunchArgument(
            'use_panel', default_value='true',
            description='Launch the hyper_rqt HYPER Panel (mission start/cancel, teleport)'),
        rviz,
        stage('sim.launch.py',
              headless=LaunchConfiguration('headless'),
              software_rendering=LaunchConfiguration('software_rendering'),
              x=LaunchConfiguration('x'),
              y=LaunchConfiguration('y'),
              Y=LaunchConfiguration('Y')),
        TimerAction(period=ODOMETRY_DELAY_S, actions=[
            stage('odometry.launch.py',
                  datum_site=LaunchConfiguration('datum_site'))]),
        # Gazebo bridges plain sensor_msgs/Image already (see ros_gz_bridge.yaml), so
        # lane_detection_node runs input_backend ros_raw here -- no rectification, no
        # image_transport/compressed subscription. object_detection_node's own default is
        # already ros_raw, so no override needed for it.
        TimerAction(period=PERCEPTION_DELAY_S, actions=[
            stage('perception.launch.py', lane_input_backend='ros_raw',
                  drivable_area=LaunchConfiguration('drivable_area'))]),
        # waypoint_csv 미지정: behavior.launch.py 기본값(sim.csv)을 씁니다.
        TimerAction(period=BEHAVIOR_DELAY_S, actions=[
            behavior_stage(),
            mission_panel(),
        ], condition=LaunchConfigurationEquals('waypoint_csv', '')),
        # waypoint_csv 지정: waypoints/ 아래로 풀어 넘깁니다.
        TimerAction(period=BEHAVIOR_DELAY_S, actions=[
            behavior_stage(waypoint_csv=waypoint_csv_resolved),
            mission_panel(),
        ], condition=LaunchConfigurationNotEquals('waypoint_csv', '')),
    ])
