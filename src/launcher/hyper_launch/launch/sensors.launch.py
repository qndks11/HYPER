import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Real-car sensor bring-up: assigns each physical sensor to the topic role
# hyper_localization / hyper_object_detection already expect from simulation
# (see hyper_gazebo's ros_gz_bridge.yaml for the sim-side equivalents).
#   ELP usb_cam                         -> owned by hyper_camera's ElpCameraPublisherNode
#                                          component (input_backend:=intra_process, loaded into
#                                          lane_detection_node's container -- see
#                                          perception.launch.py)
#   Logitech C920                       -> owned by hyper_camera's
#                                          logitech_camera_publisher_node
#                                          (object_input_backend:=usb_camera, plain topic -- see
#                                          perception.launch.py)
#   WitMotion WT901BLE (witmotion_ros2) -> /imu                    (EKF)
#   RPLidar (hyper_lidar)              -> /scan                   (already unremapped default)
#   u-blox + NTRIP (hyper_rtk)         -> /gps/fix                (navsat_transform)
#
# No camera is launched here at all -- usb_cam has been removed from this workspace entirely
# (see deps.repos); hyper_camera now provides both USB cameras' driver nodes (see
# perception.launch.py) plus every camera's calibration/config files.


def generate_launch_description():
    # witmotion_ros2의 witmotion.launch.py를 include하지 않고 노드를 직접 선언합니다.
    # 그 launch 파일은 params 파일을 하드코딩해서 오버라이드 지점을 주지 않는데,
    # 아래 orientation_covariance는 드라이버의 속성이 아니라 "이 센서를 EKF가 얼마나
    # 믿을지"에 대한 튜닝 값이므로 EKF 설정과 같은 저장소(HYPER)에서 관리해야 합니다.
    # witmotion_ros2는 deps.repos로 가져오는 별도 저장소라 HYPER git이 추적하지 않으며,
    # 거기 config를 고치면 다음 vcs import 때 사라집니다.
    witmotion_share = get_package_share_directory('witmotion_ros2')
    imu = Node(
        package='witmotion_ros2',
        executable='witmotion_ros2',
        name='witmotion_node',
        # 드라이버 기본값('log')으로 두면 BLE 스레드가 내는 진단이 전부 ~/.ros/log/로만
        # 갑니다: "No BLE device found matching ...", "BLE error: ...", "IMU disconnected",
        # "Connected to IMU at ...". 같은 트리의 rplidar_node/ntrip_node/gps_accuracy_gui는
        # 모두 screen이라 IMU만 아무 말 없이 실패하는 것처럼 보이지만, 실패 이유는 내내
        # 출력되고 있었습니다. BLE는 붙었는지 아닌지가 전부라 반드시 터미널에서 봐야 합니다.
        output='screen',
        # BLE 스레드에는 자체 재시도 루프가 있지만 프로세스가 죽으면 아무도 살리지 않습니다.
        # onNotify -> publishBatch -> publish()는 SimpleBLE 내부 D-Bus 스레드에서 try/catch
        # 없이 도는 경로라, 거기서 rclcpp 예외가 나면 std::terminate로 프로세스가 통째로
        # 사라집니다. 그때 미션 도중 IMU만 조용히 빠지는 대신 2초 뒤 다시 붙게 합니다.
        respawn=True,
        respawn_delay=2.0,
        # 이 파일에서 제일 중요한 한 줄입니다.
        #
        # 드라이버 소멸자는 BLE 스레드를 join하는데, 그 스레드는 running_ 플래그를 루프
        # 사이에서만 확인합니다. 그래서 Ctrl-C 시점에 adapter.scan_for(10초) 안이나 그
        # 직후의 sleep(5초) 안에 들어가 있으면 최대 15초가 지나야 빠져나옵니다. launch의
        # 기본값은 sigterm_timeout=5라 그 전에 SIGTERM -> SIGKILL로 죽여 버리고, 그러면
        # peripheral.disconnect()가 실행되지 않은 채 BlueZ 쪽에 GATT 연결이 그대로 남습니다.
        # 연결된 WT901BLE는 advertise를 멈추므로 "다음" 실행의 스캔은 센서를 아예 못 찾고
        # "No BLE device found"만 반복합니다 -- 한 번 띄웠다 내린 뒤부터 계속 안 붙는
        # 증상의 정체가 이것입니다. 20초를 주어 정상 종료 경로가 끝까지 돌게 합니다.
        #
        # 그래도 연결이 남았다면(강제 종료 등) 다음 실행 전에 직접 끊으세요:
        #   bluetoothctl info FD:C0:E8:FE:A9:58        # Connected: yes 면 이 문제
        #   bluetoothctl disconnect FD:C0:E8:FE:A9:58
        #
        # 드라이버 자체를 고치면(patches/witmotion_ros2-ble-reliability.patch) 대기가 전부
        # 중단 가능해져 종료가 즉시 끝나므로, 이 20초는 그때부터 그냥 여유분이 됩니다.
        sigterm_timeout='20',
        parameters=[
            os.path.join(witmotion_share, 'config', 'witmotion.yaml'),
            {
                # 드라이버 기본 frame_id는 base_link인데 이 URDF에는 base_link가 없고
                # 차체 프레임은 body_link 하나뿐입니다(vehicle.xacro). ekf_local/ekf_global의
                # base_link_frame도 body_link이므로, 그대로 두면 robot_localization이 IMU를
                # body_link로 변환하지 못해 실차에서 IMU 메시지를 전부 조용히 버립니다.
                'frame_id': 'body_link',
                # 드라이버 yaw는 나침반식(시계방향 증가)이라 ROS ENU와 부호가 반대다.
                # 원본을 /imu/raw로 내보내고, hyper_localization의 imu_enu_relay가 yaw축을
                # 뒤집어 EKF가 먹는 /imu로 다시 publish한다 (odometry.launch.py).
                'topic': '/imu/raw',
                # yaw 분산을 드라이버 기본값 0.02 rad^2(1sigma≈8.1deg)에서 크게 올립니다.
                # WT901BLE의 yaw는 지자기 기반이라 차량 모터/철제 구조물 근처에서 수십 도씩
                # 틀어질 수 있는데, 8.1deg로 신고하면 ekf_global이 그 값을 거의 진리로
                # 받아들여 RTK 위치해와 싸웁니다. 0.25 rad^2(1sigma≈28.6deg)로 두면 주행 중에는
                # RTK 위치가 방위를 지배하고, 정지 상태에서는 자이로 드리프트를 묶어주는
                # 약한 기준으로만 작동합니다. roll/pitch는 중력으로 관측되므로 그대로 둡니다.
                # 실차에서 /imu yaw와 RTK 진행방향을 비교해 실측값으로 다시 조정하세요.
                'orientation_covariance': [0.01, 0.0, 0.0,
                                           0.0, 0.01, 0.0,
                                           0.0, 0.0, 0.25],
            },
        ],
    )

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_lidar'),
            'launch', 'rplidar.launch.py')),
    )

    rtk = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_rtk'),
            'launch', 'rtk.launch.py')),
    )

    return LaunchDescription([imu, lidar, rtk])
