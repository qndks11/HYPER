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
# /camera_rear/image_raw has no physical source yet, and input_backend:=intra_process has no
# rear-camera path at all -- lane_detection_node simply never receives rear frames on the real
# vehicle until a rear camera and its own backend wiring are added.
#
# Neither camera goes through usb_cam here anymore -- usb_cam has been removed from this
# workspace entirely (see deps.repos); hyper_camera now provides both cameras' driver nodes
# (see perception.launch.py) plus their calibration/config files.


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
        parameters=[
            os.path.join(witmotion_share, 'config', 'witmotion.yaml'),
            {
                # 드라이버 기본 frame_id는 base_link인데 이 URDF에는 base_link가 없고
                # 차체 프레임은 body_link 하나뿐입니다(vehicle.xacro). ekf_local/ekf_global의
                # base_link_frame도 body_link이므로, 그대로 두면 robot_localization이 IMU를
                # body_link로 변환하지 못해 실차에서 IMU 메시지를 전부 조용히 버립니다.
                'frame_id': 'body_link',
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
