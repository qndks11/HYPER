import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# 실차 수동 주행. real.launch.py에서 조이스틱을 떼어낸 짝입니다 -- real.launch.py는
# 미션(자율주행) 전용이고, 이 파일은 사람이 스틱으로 모는 전용입니다.
#
# 둘을 나눈 이유는 취향이 아니라 토픽 충돌입니다. joystick_controller_node는 스틱을
# 건드리지 않아도 /velocity + /steering_angle을 100Hz로 계속 내보내는데, 이는
# behavior.launch.py의 cmd_vel_to_ackermann_node가 nav2의 /cmd_vel을 변환해 내보내는
# 토픽과 정확히 같습니다. 한 프로세스 트리에 둘 다 있으면 스틱 중립(0.0) 명령과 nav2
# 명령이 번갈아 arduino_interface_node에 도착해서 차가 자율주행을 못 합니다. 예전
# real.launch.py의 use_joystick 인자는 그 사고를 한 글자로 낼 수 있는 스위치였습니다.
# (phone_control_real.launch.py가 rosbridge로 같은 두 토픽을 모는 것과 같은 구조입니다.)
#
# 주 용도는 웨이포인트 녹화라서 레코더와 그 조작판 GUI가 같이 뜹니다
# (hyper_waypoint/record.launch.py). 레코더는 auto_start:=false로 떠서 GUI의
# Record를 누를 때까지 기다립니다 -- 기록 시작이 곧 CSV truncate이므로, 스택을
# 띄우는 것만으로 지난 녹화본이 날아가지 않습니다. 녹화를 끝낼 때도 Ctrl-C가 아니라
# Stop을 누르면 되므로 스택을 내리지 않고 다시 녹화할 수 있습니다.
#
# 카메라와 차선/객체 인식은 아예 포함하지 않습니다. 수동 주행과 녹화는
# /odometry/filtered_map만 쓰므로 인지 스택이 하는 일이 없고, USB 카메라 두 대와
# 추론 노드는 놀고 있지 않습니다(시뮬 실측 기준 lane_detection 46%,
# object_detection 17% CPU). 인지를 실차에서 보려면 perception.launch.py를 따로
# 띄우세요.
ODOMETRY_DELAY_S = 5.0  # real.launch.py와 같은 지연 -- 센서 드라이버가 먼저 뜨도록


def generate_launch_description():
    hyper_launch_share = get_package_share_directory('hyper_launch')

    def stage(name, **launch_arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hyper_launch_share, 'launch', name)),
            launch_arguments=launch_arguments.items())

    # body_link -> camera_link/lidar_link/gps_link TF. real.launch.py와 동일.
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_control'),
            'launch', 'robot_state_publisher.launch.py')),
    )

    # joy_node + joystick_controller_node. /velocity + /steering_angle을 직접 내보내며,
    # arduino_interface_node가 그 두 토픽을 그대로 구독합니다(hyper_interface/README.md)
    # -- 실차에는 vehicle_controller_node/ros2_control 변환 단계가 없습니다.
    joystick = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_control'),
            'launch', 'joystick.launch.py')),
        launch_arguments={
            'joystick_publish_period': LaunchConfiguration('joystick_publish_period'),
        }.items(),
    )

    # 웨이포인트 레코더 + 조작판 GUI. auto_start:=false라 Record를 누를 때까지
    # 기다립니다(record.launch.py 주석 참고).
    waypoint_record = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_waypoint'),
            'launch', 'record.launch.py')),
        launch_arguments={
            'waypoint_csv': LaunchConfiguration('waypoint_csv'),
            'min_spacing_m': LaunchConfiguration('min_spacing_m'),
            'use_record_gui': LaunchConfiguration('use_record_gui'),
        }.items(),
    )

    # GPS 정확도 모니터. real.launch.py와 마찬가지로 조건 없이 항상 띄웁니다 -- 아무
    # 토픽도 publish하지 않는 순수 구독자이고, 녹화 중에는 "지금 GPS를 믿어도 되는가"가
    # 곧 녹화 품질이라 계속 보고 있어야 하는 값입니다.
    gps_accuracy_gui = Node(
        package='hyper_localization',
        executable='gps_accuracy_gui.py',
        name='gps_accuracy_gui',
        output='screen',
    )

    return LaunchDescription([
        # GPS 원점. datums.yaml의 키입니다. 대회장이 아닌 곳에서 돌릴 때는
        # datum_site:=school 처럼 바꾸세요 (real.launch.py와 동일).
        DeclareLaunchArgument(
            'datum_site', default_value='track',
            description='GPS 원점 (datums.yaml의 sim | school | track)'),
        DeclareLaunchArgument(
            'joystick_publish_period', default_value='0.0',
            description='/velocity + /steering_angle 발행 주기(초). 0.0이면 노드 기본값 100Hz'),
        # 녹화 인자는 hyper_waypoint/record.launch.py로 그대로 넘어갑니다.
        DeclareLaunchArgument(
            'waypoint_csv',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), 'HYPER', 'src', 'planning', 'hyper_waypoint',
                'waypoints', 'real.csv']),
            description='녹화 결과를 쓸 CSV (Record를 누르는 순간 truncate)'),
        DeclareLaunchArgument(
            'min_spacing_m', default_value='0.5',
            description='이 거리(m) 이상 이동했을 때만 한 점 기록'),
        DeclareLaunchArgument(
            'use_record_gui', default_value='true',
            description='녹화 조작판 GUI를 함께 띄웁니다'),

        robot_state_publisher,
        gps_accuracy_gui,
        stage('sensors.launch.py'),
        stage('interface.launch.py'),
        joystick,
        # use_sim_time=false가 핵심입니다. 실차에는 /clock을 내보내는 노드가 없으므로
        # dual_ekf_navsat.yaml에 박혀 있는 use_sim_time: true를 그대로 두면 ekf_local /
        # ekf_global / navsat_transform이 멈춘 시계 위에서 돌며 아무것도 publish하지
        # 않습니다 -- 녹화할 /odometry/filtered_map 자체가 안 나옵니다.
        TimerAction(period=ODOMETRY_DELAY_S, actions=[
            stage('odometry.launch.py',
                  datum_site=LaunchConfiguration('datum_site'),
                  use_sim_time='false')]),
        waypoint_record,
    ])
