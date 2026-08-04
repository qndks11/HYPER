import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, SetRemap

# Real-car sensor bring-up: assigns each physical sensor to the topic role
# hyper_localization / hyper_object_detection already expect from simulation
# (see hyper_gazebo's ros_gz_bridge.yaml for the sim-side equivalents).
#   ELP usb_cam                         -> owned directly by lane_detection_node itself
#                                          (input_backend:=direct_usb, see perception.launch.py)
#   Logitech C920 (hyper_camera)        -> /camera_object/image_raw (YOLO object detection)
#   RealSense D435i (realsense2_camera) -> /imu                    (EKF)
#   RPLidar (hyper_lidar)              -> /scan                   (already unremapped default)
#   u-blox + NTRIP (hyper_rtk)         -> /gps/fix                (navsat_transform)
# /camera_rear/image_raw has no physical source yet, and input_backend:=direct_usb has no
# rear-camera path at all -- lane_detection_node simply never receives rear frames on the real
# vehicle until a rear camera and its own backend wiring are added.
#
# hyper_camera's rectify pipeline (image_proc, used for the ELP camera) is not needed for the
# Logitech camera: object_detection_node consumes raw sensor_msgs/Image with no rectification, and
# there's no intrinsic calibration for this unit yet.


def generate_launch_description():
    object_camera = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera_object',
        parameters=[os.path.join(
            get_package_share_directory('hyper_camera'), 'config', 'params_logitech.yaml')],
        remappings=[
            ('image_raw', 'camera_object/image_raw'),
            ('image_raw/compressed', 'camera_object/image_raw/compressed'),
        ],
    )

    realsense_launch = os.path.join(
        get_package_share_directory('realsense2_camera'), 'launch', 'rs_launch.py')

    realsense_camera = GroupAction(actions=[
        SetRemap('camera_object/imu', '/imu'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(realsense_launch),
            launch_arguments={
                'camera_name': 'camera_object',
                'camera_namespace': '',
                'enable_gyro': 'true',
                'enable_accel': 'true',
                'unite_imu_method': '1',
                'enable_color': 'false',
                'enable_depth': 'false',
            }.items(),
        ),
    ])

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

    return LaunchDescription([object_camera, realsense_camera, lidar, rtk])
