"""실차 RPLidar 드라이버 + 전방 섹터 필터.

드라이버는 360도를 다 훑지만, nav2 local_costmap이 실제로 보는 `/scan`은 전방 180도만
담습니다(`scan_front_filter.py`). 그 노드의 헤더 주석에 이유가 길게 적혀 있는데 요약하면:
뒤를 보는 빔이 있으면 obstacle_layer가 차 뒤쪽을 raytrace로 계속 지워서, 지나온 콘이
코스트맵에 남지 않습니다. 시뮬레이션 라이다(vehicle.xacro)가 전방 180도인 것도 같은
이유이므로, 이 필터가 있어야 실차와 시뮬레이션의 코스트맵이 같게 동작합니다.

토픽 배선:
    rplidar_node -> /scan_raw (360도 원본)
    scan_front_filter -> /scan (전방 180도, 나머지 빔은 NaN)

use_front_filter:=false면 드라이버가 `/scan`을 직접 내고 필터는 뜨지 않습니다 --
360도 원본을 확인할 때만 쓰세요(그 상태로 미션을 돌리면 뒤쪽 코스트맵이 지워집니다).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = os.path.join(
        get_package_share_directory('hyper_lidar'), 'config', 'rplidar_params.yaml')

    use_front_filter = LaunchConfiguration('use_front_filter')

    # 같은 드라이버를 두 벌 선언하는 이유는 remapping이 조건부로 못 들어가기 때문입니다.
    # 둘 중 하나만 실제로 뜹니다.
    def rplidar(scan_topic, condition):
        return Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[params_file],
            remappings=[('scan', scan_topic)],
            condition=condition,
            output='screen',
        )

    scan_front_filter = Node(
        package='hyper_lidar',
        executable='scan_front_filter.py',
        name='scan_front_filter',
        condition=IfCondition(use_front_filter),
        output='screen',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_front_filter', default_value='true',
            description='true면 /scan을 전방 180도로 자릅니다 (뒤쪽 코스트맵 유지). '
                        'false면 드라이버의 360도 원본이 그대로 /scan으로 나갑니다.'),
        rplidar('scan_raw', IfCondition(use_front_filter)),
        rplidar('scan', UnlessCondition(use_front_filter)),
        scan_front_filter,
    ])
