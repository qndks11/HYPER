import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# Real-car sensor bring-up: assigns each physical sensor to the topic role
# hyper_localization / hyper_object_detection already expect from simulation
# (see hyper_gazebo's ros_gz_bridge.yaml for the sim-side equivalents).
#   Logitech C920 (hyper_camera)        -> /camera/image_raw   (lane + object detection)
#                                          launched by perception.launch.py, not here: its
#                                          publisher component loads into lane_detection's
#                                          container for zero-copy delivery
#   E2BOX EBIMU-9DOFV5 (hyper_ebimu)    -> /imu                    (EKF)
#   RPLidar (hyper_lidar)              -> /scan                   (already unremapped default)
#   u-blox + NTRIP (hyper_rtk)         -> /gps/fix                (navsat_transform)
#
# No camera is launched here at all -- usb_cam has been removed from this workspace entirely
# (see deps.repos); hyper_camera provides the driver node for the vehicle's one camera (see
# perception.launch.py) plus its config files.


def generate_launch_description():
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_ebimu'),
            'launch', 'ebimu.launch.py')),
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
