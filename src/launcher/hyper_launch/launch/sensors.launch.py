import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetRemap

# Real-car sensor bring-up: assigns each physical sensor to the topic role
# hyper_localization / hyper_object_detection already expect from simulation
# (see hyper_gazebo's ros_gz_bridge.yaml for the sim-side equivalents).
#   ELP usb_cam                         -> owned directly by lane_detection_node itself
#                                          (input_backend:=direct_usb, see perception.launch.py)
#   Logitech C920                       -> owned directly by object_detection_node itself
#                                          (no ROS topic, see perception.launch.py)
#   RealSense D435i (realsense2_camera) -> /imu                    (EKF)
#   RPLidar (hyper_lidar)              -> /scan                   (already unremapped default)
#   u-blox + NTRIP (hyper_rtk)         -> /gps/fix                (navsat_transform)
# /camera_rear/image_raw has no physical source yet, and input_backend:=direct_usb has no
# rear-camera path at all -- lane_detection_node simply never receives rear frames on the real
# vehicle until a rear camera and its own backend wiring are added.
#
# Neither camera goes through usb_cam here anymore -- usb_cam has been removed from this
# workspace entirely (see deps.repos), and hyper_camera is now just the config file
# distribution point for both cameras; see hyper_camera's README.


def generate_launch_description():
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

    return LaunchDescription([realsense_camera, lidar, rtk])
