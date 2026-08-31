import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        # navsat_transform의 원점. hyper_localization/config/datums.yaml의 키
        # (sim | school | track)로 풀립니다. real.launch.py는 자기 기본값으로 덮어씁니다
        # -- 여기 기본값 sim은 시뮬레이션 원점이라 실차에서 그대로 쓰면 위치가 통째로
        # 엉뚱한 곳에 찍힙니다.
        DeclareLaunchArgument('datum_site', default_value='sim'),

        # dual_ekf_navsat.yaml의 use_sim_time: true를 덮어쓸 값. 실차는 /clock을 내는
        # 노드가 없으므로 반드시 false여야 세 노드가 동작합니다 (real.launch.py가 넘깁니다).
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory('hyper_localization'),
                'launch', 'odometry.launch.py')),
            launch_arguments={
                'datum_site': LaunchConfiguration('datum_site'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])
