import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    ntrip_config = os.path.join(
        get_package_share_directory('hyper_rtk'),
        'config',
        'ntrip_params.yaml'
    )

    # ---------------- u-blox GPS 드라이버 ----------------
    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node',
        output='log',
        remappings=[
            # hyper_localization의 navsat_transform_node가 구독하는 /gps/fix로 직접 발행
            ('~/fix', 'gps/fix'),
        ],
        parameters=[{
            'debug': 0,
            'device': '/dev/tty_Ardusimple',
            'frame_id': 'gps',
            'uart1': {'baudrate': 115200},
            'inf': {'all': False},
            'publish': {
                'all': True,
                'aid': {'hui': False},
                'nav': {'posecef': False},
            },
            'tmode3': 0,
            'rate': 5.0,
        }],
    )

    set_debug_env = SetEnvironmentVariable(
        name='NTRIP_CLIENT_DEBUG', value='false'
    )

    # ---------------- NTRIP 클라이언트 ----------------
    # host/mountpoint/username/password 등 민감정보는
    # config/ntrip_params.yaml (gitignore 대상)에서 불러옴
    ntrip_node = Node(
        package='ntrip_client',
        executable='ntrip_ros.py',
        name='ntrip_client',
        output='screen',
        parameters=[ntrip_config],
        remappings=[
            # NTRIP caster에 보낼 현재 위치(NMEA/GGA) 소스
            ('/fix', '/gps/fix'),
        ],
    )

    return LaunchDescription([set_debug_env, ublox_node, ntrip_node])