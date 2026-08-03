import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('hyper_object_detection'), 'models', 'best.pt')

    # hyper_lane_detection's input_backend: direct_usb (real vehicle -- lane_detection_node opens
    # the ELP camera itself), ros_raw (Gazebo simulation -- plain sensor_msgs/Image, no
    # rectification), or ros_compressed (legacy real-car image_transport path, kept only for A/B
    # comparison and rollback). See hyper_launch's real.launch.py / simulation.launch.py for which
    # value each entrypoint passes.
    lane_input_backend_arg = DeclareLaunchArgument(
        'lane_input_backend',
        default_value='ros_compressed',
    )

    # The /image_raw(/compressed) and /rear_image_raw(/compressed) remaps below are only actually
    # subscribed to under input_backend:=ros_compressed/ros_raw -- direct_usb never creates those
    # subscriptions, so the remaps are simply unused (not harmful) in that mode.
    lane_detection_node = Node(
        package='hyper_lane_detection',
        executable='lane_detection_node',
        parameters=[{'input_backend': LaunchConfiguration('lane_input_backend')}],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/image_raw/compressed', '/camera/image_raw/compressed'),
            ('/rear_image_raw', '/camera_rear/image_raw'),
            ('/rear_image_raw/compressed', '/camera_rear/image_raw/compressed'),
        ],
        output='screen'
    )

    object_detection_node = Node(
        package='hyper_object_detection',
        executable='object_detection_node',
        remappings=[('/image_raw', '/camera_object/image_raw')],
        parameters=[{'model_path': model_path}],
        output='screen'
    )

    return LaunchDescription([lane_input_backend_arg, lane_detection_node, object_detection_node])