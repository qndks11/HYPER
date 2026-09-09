#!/usr/bin/env python3
"""waypoint studio -- 코스 보기/편집/녹화/주행을 한 창에서.

  ros2 launch hyper_waypoint_studio studio.launch.py
  ros2 launch hyper_waypoint_studio studio.launch.py mode:=drive \
      mission_yaml:=$HOME/HYPER/src/planning/hyper_planner/config/mission_sim.yaml

주의: 이 머신의 VSCode 통합 터미널에서 GUI 노드를 띄우면 snap이 주입하는 GTK_PATH
때문에 즉시 죽습니다. 그럴 때는 아래처럼 환경 변수를 걷어내고 실행하세요.

  env -u GTK_PATH -u GTK_EXE_PREFIX -u GDK_PIXBUF_MODULE_FILE -u GDK_PIXBUF_MODULEDIR \
      ros2 launch hyper_waypoint_studio studio.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    default_csv = PathJoinSubstitution([
        EnvironmentVariable('HOME'), 'HYPER', 'src', 'planning', 'hyper_waypoint',
        'waypoints', 'simulation', 'sim1.csv'])

    return LaunchDescription([
        DeclareLaunchArgument(
            'waypoint_csv', default_value=default_csv,
            description='창이 뜨자마자 올려 둘 코스 CSV'),
        DeclareLaunchArgument(
            'mission_yaml', default_value='',
            description='열어 둘 mission.yaml (비우면 안 엽니다)'),
        DeclareLaunchArgument(
            'overlay', default_value='',
            description="배경 이미지 경로, 또는 'gazebo'"),
        DeclareLaunchArgument(
            'mode', default_value='view',
            description='시작 모드: view | edit | record | drive'),
        DeclareLaunchArgument(
            'destination', default_value='',
            description='녹화 모드의 저장 파일 초기값'),
        DeclareLaunchArgument('recorder', default_value='/waypoint_recorder'),

        Node(
            package='hyper_waypoint_studio',
            executable='waypoint_studio',
            name='waypoint_studio',
            output='screen',
            arguments=[
                LaunchConfiguration('waypoint_csv'),
                '--mode', LaunchConfiguration('mode'),
                '--mission', LaunchConfiguration('mission_yaml'),
                '--overlay', LaunchConfiguration('overlay'),
                '--destination', LaunchConfiguration('destination'),
                '--recorder', LaunchConfiguration('recorder'),
            ],
        ),
    ])
