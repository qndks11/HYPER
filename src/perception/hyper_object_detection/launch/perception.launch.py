import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    model_path = os.path.join(
        get_package_share_directory('hyper_object_detection'), 'models', 'best.pt')

    # Bird's-eye-view geometry for the real ELP front camera. lane_detection's built-in defaults
    # describe the simulated cameras, so only the intra_process (real vehicle) path loads this --
    # it replaces the sim's ideal-pinhole lens model with the ELP's measured rectified
    # intrinsics, leaving the ground region and scale identical between sim and car.
    real_bev_params = os.path.join(
        get_package_share_directory('hyper_lane_detection'), 'config', 'bev_real.yaml')

    # Rear RealSense D435i settings -- see the file for what is enabled and why.
    d435i_params = os.path.join(
        get_package_share_directory('hyper_camera'), 'config', 'params_d435i.yaml')

    # hyper_lane_detection's input_backend: intra_process (real vehicle -- hyper_camera's
    # ElpCameraPublisherNode component and LaneDetection are loaded into one
    # ComposableNodeContainer, so the frame is handed over by pointer instead of a serialized
    # topic) or ros_raw (Gazebo simulation -- plain sensor_msgs/Image from ros_gz_bridge, no
    # rectification). See hyper_launch's real.launch.py / simulation.launch.py for which value
    # each entrypoint passes.
    lane_input_backend_arg = DeclareLaunchArgument(
        'lane_input_backend',
        default_value='intra_process',
    )

    # object_detection_node's camera source: usb_camera (real vehicle -- hyper_camera's
    # logitech_camera_publisher_node owns the Logitech C920 and publishes to this stage's
    # /image_raw remap) or ros_raw (Gazebo simulation, also selectable on the real car for
    # A/B/rollback: no camera node is launched, some other plain sensor_msgs/Image source must
    # already publish the remap target). object_detection_node itself no longer knows the
    # difference -- it always just subscribes; this argument only decides whether
    # logitech_camera_publisher_node is launched to feed that subscription.
    object_input_backend_arg = DeclareLaunchArgument(
        'object_input_backend',
        default_value='ros_raw',
    )

    # intra_process: hyper_camera's ElpCameraPublisherNode and LaneDetection load into one
    # process. rclcpp's intra-process manager hands the publisher's std::unique_ptr<Image>
    # straight to LaneDetection's subscription instead of serializing it over a DDS topic.
    lane_detection_container = ComposableNodeContainer(
        name='lane_detection_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hyper_camera',
                plugin='hyper_camera::ElpCameraPublisherNode',
                name='elp_camera_publisher',
                remappings=[('image_raw', '/camera/image_raw')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            # Rear camera. realsense2_camera's own component, not a hyper_camera node -- the
            # upstream driver already publishes each frame as a std::unique_ptr<Image> through a
            # native rclcpp publisher when use_intra_process_comms is set, so it drops into this
            # container on the same zero-copy terms as the ELP publisher above. It creates its
            # topics under "~/", so with this name/namespace the colour stream would land on
            # /camera_rear/color/image_raw -- remapped here to the /camera_rear/image_raw that
            # LaneDetection's rear subscription and the sim's ros_gz_bridge already agree on.
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='camera_rear',
                namespace='',
                parameters=[d435i_params],
                remappings=[('/camera_rear/color/image_raw', '/camera_rear/image_raw')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='hyper_lane_detection',
                plugin='LaneDetection',
                name='lane_detection',
                parameters=[real_bev_params, {'input_backend': 'intra_process'}],
                remappings=[
                    ('/image_raw', '/camera/image_raw'),
                    ('/rear_image_raw', '/camera_rear/image_raw'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        condition=LaunchConfigurationEquals('lane_input_backend', 'intra_process'),
    )

    # ros_raw: no physical camera to publish here -- lane_detection_node runs standalone and
    # subscribes to whatever already publishes the remap target (ros_gz_bridge in sim).
    lane_detection_node = Node(
        package='hyper_lane_detection',
        executable='lane_detection_node',
        parameters=[{'input_backend': LaunchConfiguration('lane_input_backend')}],
        remappings=[
            ('/image_raw', '/camera/image_raw'),
            ('/rear_image_raw', '/camera_rear/image_raw'),
        ],
        output='screen',
        condition=LaunchConfigurationNotEquals('lane_input_backend', 'intra_process'),
    )

    # usb_camera: hyper_camera owns the Logitech C920 and publishes to the same remap target
    # object_detection_node subscribes to below -- a plain topic, not intra-process (rclpy has no
    # zero-copy path to load it into the same process as object_detection_node).
    logitech_camera_publisher_node = Node(
        package='hyper_camera',
        executable='logitech_camera_publisher_node',
        remappings=[('image_raw', '/camera_object/image_raw')],
        output='screen',
        condition=LaunchConfigurationEquals('object_input_backend', 'usb_camera'),
    )

    # The /image_raw remap below is object_detection_node's only camera input now -- fed by
    # logitech_camera_publisher_node (usb_camera) or ros_gz_bridge (ros_raw, sim).
    object_detection_node = Node(
        package='hyper_object_detection',
        executable='object_detection_node',
        parameters=[{
            'model_path': model_path,
        }],
        remappings=[('/image_raw', '/camera_object/image_raw')],
        output='screen'
    )

    return LaunchDescription([
        lane_input_backend_arg,
        object_input_backend_arg,
        lane_detection_container,
        lane_detection_node,
        logitech_camera_publisher_node,
        object_detection_node,
    ])
