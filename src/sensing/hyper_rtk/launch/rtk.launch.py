import os

from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

# base UART2 out / rover UART2 in -- 두 보드 다 u-center로 이 값에 맞춰 두고
# BBR+Flash로 저장해 둔다(README의 "u-center 사전 설정" 참고). 여기서는 값을
# 밀어넣지 않는다 -- 그냥 재확인용 상수이자 문서용 주석이다.
# 460800: 이 값으로 u-center에서 moving-base 헤딩이 실제로 되는 걸 확인했다 -- 115200은
# 검증 없이 UART1과 맞춰본 임의의 값이었으니 재현 안 되면 이 값부터 의심할 것.
UART2_BAUD = 460800  # noqa: F841 -- 문서용, README/u-center 설정과 맞춰 둘 것


def generate_launch_description():
    ntrip_config = os.path.join(
        get_package_share_directory('hyper_rtk'),
        'config',
        'ntrip_params.yaml'
    )

    common_params = {
        'debug': 0,
        'uart1': {'baudrate': 115200},
        'inf': {'all': False},
        'publish': {
            'all': True,
            # ZED-F9P (protocol version 27+) dropped the legacy AID class in favor of
            # MGA-*; leaving aid.alm/aid.eph on publish.all's default makes the driver
            # send CFG-MSG for message IDs the board no longer recognizes, so both
            # boards NACK (0x06 / 0x01 = CFG-MSG itself) right after every startup.
            'aid': {'all': False},
            'nav': {'posecef': False},
        },
        # HpPosRecProduct(moving base)는 HpgRefProduct의 getRosParams()를 그대로 물려받는데,
        # 거기서 nav_rate(cycles) * meas_rate(ms)가 정확히 1000이 아니면(=1Hz 내비게이션
        # 해가 아니면) "For HPG Ref devices, nav_rate should be exactly 1 Hz." 경고를 낸다.
        # 이게 경고로만 끝나는 게 아니라 실제로 moving-base 상대위치 엔진이 그 이상 rate에서
        # 안정적으로 안 풀린다(REL_POS_VALID/HEAD_VALID가 영영 안 뜨는 원인이었음, RTCM은
        # 정상 수신/사용되는데도). u-center로 그냥 붙여서 볼 땐 보드가 원래 있던 1Hz 설정을
        # 안 건드리니 되고, ROS는 launch마다 이 rate를 강제로 밀어 넣으니 매번 깨졌다.
        'rate': 1.0,
    }

    # ---------------- u-blox GPS 드라이버: base (뒤쪽 안테나) ----------------
    # UART2 출력(RTCM3 + 4072.0) 구성은 ublox_gps 드라이버가 자동으로 밀어주지
    # 않으므로(hpg_ref_product.cpp: configRtcm()이 tmode3=FIXED/SURVEY_IN에서만
    # 호출되고, moving base가 쓰는 DISABLED 경로에서는 호출되지 않는다), u-center로
    # 보드 플래시에 미리 저장해 둔 값을 그대로 신뢰한다 -- README의 "u-center 사전
    # 설정" 절 순서대로 먼저 두 보드를 설정해 두지 않으면 이 launch만으로는 UART2가
    # 안 켜진다. (예전엔 launch마다 UBX-CFG-PRT/CFG-MSG를 직접 재주입하는
    # push_uart2_config.py를 자동으로 돌렸는데, 지금은 안 쓴다 -- 플래시 설정이
    # 지워진 것 같을 때 수동 복구용으로만 hyper_rtk/hyper_rtk/push_uart2_config.py가
    # 남아 있다.)
    ublox_base = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node_base',
        output='log',
        remappings=[
            # hyper_localization의 navsat_transform_node가 구독하는 /gps/fix로 직접 발행
            ('~/fix', 'gps/fix'),
            # hp_pos_rec_product.cpp가 create_publisher(node, "navheading"/"navrelposned", ..)로
            # *상대* 이름(~ 없이)을 쓴다 -- 네임스페이스를 안 줬으니 그대로 두면 base/rover 둘 다
            # 전역 토픽 /navheading, /navrelposned에 동시에 publish해서 서로 섞인다. base 쪽은
            # 안 쓰므로 노드 프라이빗 이름으로 치워서 rover 것과 충돌하지 않게 한다.
            ('navheading', '~/navheading'),
            ('navrelposned', '~/navrelposned'),
            # ublox_firmware8.hpp도 같은 문제: create_publisher(node, "rxmrtcm", ..)가
            # *상대* 이름이라 안 옮기면 base/rover가 전역 /rxmrtcm 하나에 같이 publish된다.
            # UART2 진단(base가 보내는 RTCM3를 rover가 실제로 받는지)에 rover 쪽만 따로
            # 봐야 해서 여기도 노드 프라이빗으로 치운다.
            ('rxmrtcm', '~/rxmrtcm'),
        ],
        parameters=[{
            **common_params,
            'device': '/dev/tty_ublox_base',
            'frame_id': 'gps_base_link',
            'tmode3': 0,  # Disabled -- moving base는 고정좌표 기준국이 아니다
        }],
    )

    # ---------------- u-blox GPS 드라이버: rover (앞쪽 안테나) ----------------
    # UART2로 base의 RTCM3(+4072.0)를 받아 상대위치/헤딩(NAV-RELPOSNED9)을 푼다.
    # publish.all=True라 publish.nav.heading이 같이 켜져서, 드라이버가 알아서
    # NED->ENU 변환과 REL_POS_HEAD_VALID 기반 covariance까지 채운 sensor_msgs/Imu를
    # navheading 토픽으로 낸다 (ublox_gps의 hp_pos_rec_product.cpp) -- 별도 변환
    # 노드가 필요 없다. dual_ekf_navsat.yaml의 imu1이 이 imu/heading을 그대로 먹는다.
    ublox_rover = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps_node_rover',
        output='log',
        remappings=[
            # 드라이버가 create_publisher(node, "navheading", ..)로 *상대* 이름(~ 아님)을
            # 쓰므로, 매칭시키려면 remap의 from도 '~/navheading'이 아니라 'navheading'이어야
            # 한다 -- '~/navheading'로 걸면 아예 안 걸려서 기본 해석(네임스페이스 없음 =
            # 전역 /navheading)으로 새고, base와 rover가 같은 전역 토픽에 같이 publish해
            # 버린다(실제로 관찰됨).
            ('navheading', 'imu/heading'),
            ('navrelposned', '~/navrelposned'),
            ('rxmrtcm', '~/rxmrtcm'),
            # node.cpp의 RTCM 입력 구독은 create_subscription(..., "/rtcm", ...)로
            # *절대* 경로로 하드코딩돼 있다 -- 위의 상대 이름들과 달리 이건 remap
            # 대상으로 안 잡힐 것 같지만 실제로는 걸린다(정확히 그 문자열을 from으로
            # 주면 됨). 이걸 안 걸면 rover도 ntrip_client가 내는 전역 /rtcm(NTRIP
            # CORS망 보정, base용)을 그대로 받아서 gps_->sendRtcm()으로 자기 USB
            # 링크에 곧장 흘려넣는다 -- UART2로 받는 moving-base 전용 보정(base가
            # 4072.0으로 보내는, baseline 수 미터짜리)과 NTRIP망 보정(기준국까지
            # 수 km, 완전히 다른 baseline)이 동시에 들어가 서로 경쟁하면서
            # diffSoln은 뜨는데(NTRIP 보정 자체는 유효하니까) relPosValid/headValid는
            # 영영 안 서는 현상으로 나타났다(rxmrtcm에 1013/1033/MSM5 같이 UART2용으로
            # 설정한 적 없는 메시지가 crcFailed=0으로 깨끗하게 섞여 들어온 것으로 확인).
            # rover는 NTRIP이 전혀 필요 없으므로 안 쓰는 토픽으로 치워 버린다.
            ('/rtcm', 'unused/rtcm'),
        ],
        parameters=[{
            **common_params,
            'device': '/dev/tty_ublox_rover',
            'frame_id': 'gps_rover_link',
            'tmode3': 0,
        }],
    )

    set_debug_env = SetEnvironmentVariable(
        name='NTRIP_CLIENT_DEBUG', value='false'
    )

    # ---------------- NTRIP 클라이언트 ----------------
    # base 하나에만 필요하다 -- rover는 UART2로 base에서 직접 보정을 받는다.
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

    return LaunchDescription([
        set_debug_env,
        ublox_base,
        ublox_rover,
        ntrip_node,
    ])
