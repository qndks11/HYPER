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

    # 실차 IMU(WitMotion WT901BLE) 드라이버는 센서의 나침반식 각도(yaw가 시계방향 증가,
    # 센서 자체 0점 기준)를 그대로 orientation에 넣어 REP-103 ENU(0=East, 반시계 증가)와
    # 부호도 원점도 다르다. navsat_transform의 yaw_offset/declination은 GPS 오도메트리만
    # 돌릴 뿐 ekf_global이 직접 먹는 IMU 절대 yaw는 못 고치므로, EKF 앞에 relay를 끼워
    # out_yaw = yaw_sign*in_yaw + yaw_offset_rad 로 변환한다 (드라이버는 /imu/raw로 내보내도록
    # sensors.launch.py에서 remap). 시뮬레이션 IMU는 이미 ENU이므로 relay를 띄우지 않는다.
    #
    #
    # 단, EKF는 더 이상 이 yaw를 절대 방위로 먹지 않는다(datums.yaml 참조 -- WT901BLE가
    # 6축 모드라 yaw가 분당 0.5도씩 밀린다). 절대 방위는 아래 gps_heading이 담당하고,
    # relay가 여전히 중요한 이유는 flip_angular_velocity_z 하나다: yaw 전파를 자이로 z
    # 적분이 통째로 짊어지므로 부호가 반대면 좌회전이 우회전으로 추정된다.
    # 확인(실차): 차를 반시계로 돌리며 `ros2 topic echo /imu --field angular_velocity`의
    # z가 양수인지 본다. 음수면 datums.yaml의 imu_flip_gyro_z를 뒤집는다.
    imu_relay = [] if use_sim_time else [
        Node(
            package='hyper_localization',
            executable='imu_enu_relay.py',
            name='imu_enu_relay',
            output='screen',
            parameters=[clock_override, {
                'yaw_sign': float(datum.get('imu_yaw_sign', -1.0)),
                'yaw_offset_rad': math.radians(datum.get('imu_yaw_offset_deg', 0.0)),
                'flip_angular_velocity_z': bool(datum.get('imu_flip_gyro_z', False)),
            }],
        ),
    ]

    # -----------------------------------------------------------------
    # 절대 yaw 공급원. 지자기 yaw를 어디에서도 쓰지 않으므로(datums.yaml 참조)
    # ekf_global의 방위 기준은 이 노드 하나뿐이다:
    #   정지 중 -> datums.yaml의 initial_heading_deg를 그대로 실어 yaw를 고정
    #   주행 중 -> GPS 진행방향(course over ground)으로 자이로 드리프트를 교정
    # 실차에서는 /ublox_gps_node/navpvt의 heading/headAcc를, 시뮬레이션처럼 NavPVT가
    # 없으면 /gps/fix 연속 측정값 차분을 쓴다. 그래서 sim/실차 모두 띄운다.
    # -----------------------------------------------------------------
    gps_heading = Node(
        package='hyper_localization',
        executable='gps_heading.py',
        name='gps_heading',
        output='screen',
        parameters=[clock_override, {
            'initial_heading_deg': float(datum.get('initial_heading_deg', 0.0)),
            # sim GPS 노이즈는 ~1 cm(vehicle.xacro)라 실차 기본값 0.3 m는 지나치게
            # 비관적이다. 그대로 두면 기본 기선 0.5 m에서 각도 불확실도가
            # atan(0.3/0.5)=31deg로 튀어 max_head_acc(25deg)에 전부 기각되고 GPS
            # 코스 보정이 한 번도 발동하지 못한다. 0.05 m면 atan(0.05/0.5)=6deg.
            # (기선 자체는 gps_heading이 min_fix_displacement를 넘을 때까지 누적하므로
            #  거리 임계값은 sim에서도 기본값 0.5 m를 그대로 쓴다.)
            # 실차는 /ublox_gps_node/navpvt가 우선 소스이므로 기본값을 유지한다.
            **({'fix_position_stddev': 0.05} if use_sim_time else {}),
        }],
    )

    return imu_relay + [
        gps_heading,

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