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

    # dual_ekf_navsat.yaml은 시뮬레이션 기준으로 use_sim_time: true를 박아 두고 있습니다.
    # 실차에는 /clock을 내보내는 노드가 없어서 그대로 두면 세 노드 모두 멈춘 시계 위에서
    # 돌며 아무것도 publish하지 않습니다. YAML 뒤에 dict로 얹어 덮어씁니다
    # (파라미터는 나중에 오는 쪽이 이깁니다).
    use_sim_time = LaunchConfiguration('use_sim_time').perform(context).lower() in (
        'true', '1', 'yes')
    clock_override = {'use_sim_time': use_sim_time}

    with open(datums_yaml) as f:
        datums = yaml.safe_load(f)['datums']
    if datum_site not in datums:
        raise RuntimeError(
            f"Unknown datum_site '{datum_site}' -- choices are {sorted(datums.keys())} "
            f"(see {datums_yaml})")
    datum = datums[datum_site]

    navsat_overrides = {
        **clock_override,
        'wait_for_datum': True,
        'datum': [
            datum['latitude_deg'],
            datum['longitude_deg'],
            math.radians(datum['heading_deg']),
        ],
        'magnetic_declination_radians': math.radians(datum['magnetic_declination_deg']),
        'yaw_offset': math.radians(datum['yaw_offset_deg']),
    }

    # imu_enu_relay는 WitMotion WT901BLE 전용이라 지금은 띄우지 않는다.
    # 실차 IMU는 E2BOX EBIMU-9DOFV5(hyper_ebimu)로 바뀌었고, 그 드라이버가 body_link
    # 프레임으로 /imu에 바로 publish한다(sensors.launch.py -> ebimu.launch.py).
    # WitMotion이 쓰던 /imu/raw를 이제 아무도 내보내지 않으므로 relay를 띄우면
    # 입력 없는 빈 노드가 될 뿐이다.
    #
    # EBIMU 축이 REP-103 ENU와 맞는지는 실차에서 확인해야 한다(hyper_ebimu/README.md:
    # 센서 body frame 그대로 내보내며 재매핑하지 않음). yaw 부호/원점이나
    # angular_velocity.z 부호가 틀린 것으로 나오면 두 가지 방법이 있다:
    #   1) hyper_ebimu/config/ebimu.yaml의 topic을 imu/raw로 바꾸고 아래 relay를 되살린다
    #      (datums.yaml의 imu_yaw_sign / imu_yaw_offset_deg / imu_flip_gyro_z가 보정값)
    #   2) ebimu_node.py에서 축을 직접 고친다
    # 확인(실차): 차를 반시계로 돌리며 `ros2 topic echo /imu --field angular_velocity`의
    # z가 양수인지, 아는 방위로 세웠을 때 orientation yaw가 그 방위(북=90deg)인지 본다.
    # ekf_global이 /imu의 yaw를 절대 방위로 먹으므로(dual_ekf_navsat.yaml의 imu0_config
    # 인덱스 5 = true) 여기가 틀리면 map heading이 통째로 돌아간다.
    #
    # imu_relay = [] if use_sim_time else [
    #     Node(
    #         package='hyper_localization',
    #         executable='imu_enu_relay.py',
    #         name='imu_enu_relay',
    #         output='screen',
    #         parameters=[clock_override, {
    #             'yaw_sign': float(datum.get('imu_yaw_sign', -1.0)),
    #             'yaw_offset_rad': math.radians(datum.get('imu_yaw_offset_deg', 0.0)),
    #             'flip_angular_velocity_z': bool(datum.get('imu_flip_gyro_z', False)),
    #         }],
    #     ),
    # ]

    # -----------------------------------------------------------------
    # 절대 방위(yaw) 공급원: 듀얼 GNSS moving-base 헤딩.
    # 이 launch 파일은 더 이상 헤딩 노드를 띄우지 않는다 -- imu/heading은 이제
    # hyper_rtk/launch/rtk.launch.py의 rover 쪽 ublox_gps_node가 직접 낸다
    # (dual_ekf_navsat.yaml의 ekf_global imu1 참고). 실차에서는 odometry.launch.py와
    # 별도로 hyper_rtk를 같이 띄워야 imu/heading이 나온다.
    #
    # 과거에 쓰던 진행방향(course-over-ground) 기반 gps_heading 노드는
    # scripts/gps_heading.py에 남아 있지만 지금은 어디서도 실행하지 않는다 --
    # 그 파일의 docstring에 배경이 정리되어 있다.
    # -----------------------------------------------------------------

    return [
        # -----------------------------------------------------------------
        # 노드 1) 로컬 EKF (엔코더 + IMU -> odom)
        # -----------------------------------------------------------------
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_local',
            output='screen',
            parameters=[config, clock_override],
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
            parameters=[config, clock_override],
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
            'use_sim_time', default_value='true',
            description='시뮬레이션은 true, 실차는 false'),
        DeclareLaunchArgument(
            'datum_site', default_value='sim',
            description="GPS origin to use, keyed into config/datums.yaml (e.g. 'sim', "
                        "'school', 'track')."),
        OpaqueFunction(function=lambda context: _launch_setup(context, config, datums_yaml)),
    ])