from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# GUI로 조작하는 웨이포인트 녹화. joystick_control_real.launch.py와
# phone_control_real.launch.py가 이 파일을 include합니다 -- 수동 주행 스택은 어느
# 쪽으로 몰든 "몰면서 코스를 딴다"가 목적이라 녹화가 따라붙는 게 맞습니다.
#
# 조작판은 hyper_waypoint_studio의 녹화 모드입니다(예전 waypoint_record_gui.py).
# 미니맵 대신 공용 캔버스라, 새로 따는 코스를 트랙 항공사진과 이전 녹화본 위에
# 겹쳐 보면서 몰 수 있습니다.
#
# 주의: 이 머신의 VSCode 통합 터미널에서는 snap이 주입하는 GTK_PATH 때문에 GUI가
# 즉시 죽습니다. env -u GTK_PATH -u GTK_EXE_PREFIX -u GDK_PIXBUF_MODULE_FILE
# -u GDK_PIXBUF_MODULEDIR 를 앞에 붙여 실행하세요.
#
# 핵심은 auto_start:=false입니다. 레코더는 뜨자마자 기록하지 않고 GUI의 Record를
# 누를 때까지 기다립니다. 기록 시작이 곧 CSV truncate이므로, 스택을 띄우는 것만으로
# 지난 녹화본이 날아가지 않는다는 뜻입니다.


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoint_csv',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), 'HYPER', 'src', 'planning', 'hyper_waypoint',
                'waypoints', 'track', 'real.csv']),
            description='녹화 결과를 쓸 CSV. Record를 누르는 순간 truncate됩니다'),
        DeclareLaunchArgument(
            'min_spacing_m', default_value='0.5',
            description='이 거리(m) 이상 이동했을 때만 한 점 기록. 시간 간격이 아닙니다'),
        DeclareLaunchArgument(
            'use_record_gui', default_value='true',
            description='녹화 조작판 GUI를 함께 띄웁니다'),

        Node(
            package='hyper_waypoint',
            executable='waypoint_recorder_node',
            name='waypoint_recorder',
            output='screen',
            parameters=[{
                'output_csv': LaunchConfiguration('waypoint_csv'),
                'min_spacing_m': LaunchConfiguration('min_spacing_m'),
                'auto_start': False,
            }],
        ),
        Node(
            package='hyper_waypoint_studio',
            executable='waypoint_studio',
            name='waypoint_studio',
            output='screen',
            # 저장 파일 칸을 레코더와 같은 기본 경로로 채워 둡니다. 그대로 Record하면
            # 이 값으로, 바꿔서 누르면 그 값으로 갑니다. 같은 파일을 코스로도 올려
            # 두므로, Record가 무엇을 덮어쓰는지 누르기 전에 화면에 보입니다.
            arguments=[
                LaunchConfiguration('waypoint_csv'),
                '--mode', 'record',
                '--destination', LaunchConfiguration('waypoint_csv'),
                '--recorder', '/waypoint_recorder',
            ],
            condition=IfCondition(LaunchConfiguration('use_record_gui')),
        ),
    ])
