from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# GUI로 조작하는 웨이포인트 녹화. joystick_control_real.launch.py와
# phone_control_real.launch.py가 이 파일을 include합니다 -- 수동 주행 스택은 어느
# 쪽으로 몰든 "몰면서 코스를 딴다"가 목적이라 녹화가 따라붙는 게 맞습니다.
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
                'waypoints', 'real.csv']),
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
            package='hyper_waypoint',
            executable='waypoint_record_gui.py',
            name='waypoint_record_gui',
            output='screen',
            parameters=[{'recorder': '/waypoint_recorder'}],
            condition=IfCondition(LaunchConfiguration('use_record_gui')),
        ),
    ])
