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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
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

    # -----------------------------------------------------------------
    # rviz_satellite 앵커: datum 위경도를 STATUS_FIX로 고정 publish.
    #   실 GPS(/gps/fix)가 NO_FIX면 rviz_satellite가 타일을 안 그리므로, 위성
    #   배경만을 위한 별도 고정 fix를 /aerial_map/anchor로 계속 내보낸다.
    #   frame_id=map -> 타일 격자 중심이 map 원점(=navsat datum)에 놓여 EKF map
    #   프레임과 정렬된다. ekf_global의 map->odom TF가 있어야 렌더된다.
    # -----------------------------------------------------------------
    anchor_msg = (
        "{header: {frame_id: map}, "
        "status: {status: 0, service: 1}, "
        f"latitude: {datum['latitude_deg']}, "
        f"longitude: {datum['longitude_deg']}, "
        "altitude: 0.0, position_covariance_type: 0}"
    )

    # 실차 IMU(WitMotion WT901BLE) 드라이버는 센서의 나침반식 각도(yaw가 시계방향 증가,
    # 센서 자체 0점 기준)를 그대로 orientation에 넣어 REP-103 ENU(0=East, 반시계 증가)와
    # 부호도 원점도 다르다. navsat_transform의 yaw_offset/declination은 GPS 오도메트리만
    # 돌릴 뿐 ekf_global이 직접 먹는 IMU 절대 yaw는 못 고치므로, EKF 앞에 relay를 끼워
    # out_yaw = yaw_sign*in_yaw + yaw_offset_rad 로 변환한다 (드라이버는 /imu/raw로 내보내도록
    # sensors.launch.py에서 remap). 시뮬레이션 IMU는 이미 ENU이므로 relay를 띄우지 않는다.
    #
    # 캘리브레이션(실차): 차를 정북으로 세우고 `ros2 topic echo /imu`의 yaw가 +1.5708이
    # 되도록 imu_yaw_offset_deg를 맞춘다. 차를 반시계로 돌렸을 때 yaw가 증가해야 하며,
    # 감소하면 imu_yaw_sign을 뒤집는다. datums.yaml에서 사이트별로 덮어쓴다.
    imu_relay = [] if use_sim_time else [
        Node(
            package='hyper_localization',
            executable='imu_enu_relay.py',
            name='imu_enu_relay',
            output='screen',
            parameters=[clock_override, {
                'yaw_sign': float(datum.get('imu_yaw_sign', -1.0)),
                'yaw_offset_rad': math.radians(datum.get('imu_yaw_offset_deg', 0.0)),
                'flip_angular_velocity_z': bool(datum.get('imu_flip_gyro_z', True)),
            }],
        ),
    ]

    return imu_relay + [
        ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '-r', '1',
                 '/aerial_map/anchor', 'sensor_msgs/msg/NavSatFix', anchor_msg],
            output='screen',
        ),

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