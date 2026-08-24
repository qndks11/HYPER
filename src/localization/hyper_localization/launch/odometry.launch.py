#!/usr/bin/env python3
# =====================================================================
# ROS2 Humble - Dual EKF + navsat_transform 통합 launch 파일
#
# 띄우는 노드 3개:
#   1) ekf_local        (robot_localization/ekf_node)
#   2) ekf_global       (robot_localization/ekf_node)
#   3) navsat_transform (robot_localization/navsat_transform_node)
#
# 센서 토픽:  /odom  /imu  /gps/fix
# =====================================================================

import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, config, datums_yaml):
    datum_site = LaunchConfiguration('datum_site').perform(context)

    with open(datums_yaml) as f:
        datums = yaml.safe_load(f)['datums']
    if datum_site not in datums:
        raise RuntimeError(
            f"Unknown datum_site '{datum_site}' -- choices are {sorted(datums.keys())} "
            f"(see {datums_yaml})")
    datum = datums[datum_site]

    navsat_overrides = {
        'wait_for_datum': True,
        'datum': [
            datum['latitude_deg'],
            datum['longitude_deg'],
            math.radians(datum['heading_deg']),
        ],
        'magnetic_declination_radians': math.radians(datum['magnetic_declination_deg']),
        'yaw_offset': math.radians(datum['yaw_offset_deg']),
    }

    return [
        # -----------------------------------------------------------------
        # 노드 1) 로컬 EKF (엔코더 + IMU -> odom)
        # -----------------------------------------------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[config],
            remappings=[
                ('odometry/filtered', 'odometry/filtered_odom'),
            ],
        ),

        # -----------------------------------------------------------------
        # 노드 2) 글로벌 EKF (엔코더 + IMU + GPS -> map)
        # -----------------------------------------------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_global',
            output='screen',
            parameters=[config],
            remappings=[
                ('odometry/filtered', 'odometry/filtered_map'),
            ],
        ),

        # -----------------------------------------------------------------
        # 노드 3) navsat_transform (GPS 위경도 -> 직교좌표)
        #   navsat은 글로벌 EKF 출력(filtered_map)을 받아야 함. 로컬(filtered_odom)을 주면
        #   odometry/gps가 odom 프레임으로 찍히고, ekf_global이 이를 map으로 변환할 때
        #   자기 자신이 publish한 map->odom(=현재 추정치)을 더해 버려 추정치가 발산함.
        #   datum_site로 고른 원점(datums.yaml)을 config 위에 덮어씀
        # -----------------------------------------------------------------
        Node(
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[config, navsat_overrides],
            remappings=[
                ('imu', 'imu'),
                ('gps/fix', 'gps/fix'),
                ('odometry/filtered', 'odometry/filtered_map'),
                ('odometry/gps', 'odometry/gps'),
            ],
        ),
    ]


def generate_launch_description():

    pkg_name = 'hyper_localization'
    share_dir = get_package_share_directory(pkg_name)

    config = os.path.join(share_dir, 'config', 'dual_ekf_navsat.yaml')
    datums_yaml = os.path.join(share_dir, 'config', 'datums.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'datum_site', default_value='sim',
            description="GPS origin to use, keyed into config/datums.yaml (e.g. 'sim', "
                        "'school', 'track')."),
        OpaqueFunction(function=lambda context: _launch_setup(context, config, datums_yaml)),
    ])