import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

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
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('witmotion_ros2'),
            'launch', 'witmotion.launch.py')),
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
