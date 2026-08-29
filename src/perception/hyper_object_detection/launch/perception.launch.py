import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals, LaunchConfigurationNotEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue


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

    # Turns the rear RGBD camera's depth stream into a pseudo-LaserScan on /scan_rear, which
    # nav2's obstacle_layer folds into the local costmap as a second observation source (see
    # hyper_planner/config/nav2_controller.yaml). Off by default for the same reason
    # drivable_area is: it steers the vehicle.
    #
    # Only the simulation currently has a depth stream to convert -- params_d435i.yaml leaves
    # enable_depth false on the real car -- so real.launch.py has no reason to pass this true
    # until that changes.
    rear_scan_arg = DeclareLaunchArgument(
        'rear_scan',
        default_value='false',
    )

    # depthimage_to_laserscan slices a band of image rows centered on cy and keeps the nearest
    # valid range per column. That band is a constant-height horizontal plane ONLY because the
    # rear camera is mounted level (parameters.yaml: rear_camera_pitch 0.0, 0.6 m above ground);
    # tilt it down and the centre rows hit the road, and shortest-range-wins collapses the whole
    # scan into a ring at that distance. Do not re-pitch that camera without revisiting this.
    #
    #   scan_height 40  -> +/-20 rows = +/-2.56 deg about level (fy = 446.8 at 848x480, 87 deg
    #                      HFOV). The lowest ray in that band would not reach the ground until
    #                      0.6 / tan(2.56 deg) = 13.4 m, comfortably past range_max, so no ground
    #                      return can win a column. Widening this shortens that distance fast
    #                      (scan_height 80 -> 6.7 m), which is when ground starts leaking in.
    #   range_max 8.0   -> under the sensor's own 10 m depth clip (rear_depth_far) and well under
    #                      the 13.4 m above.
    #   range_min 0.3   -> the camera is bumper-flush, so this is ~0.28 m behind the car. The
    #                      D435i's min-Z is 0.2 m (rear_depth_near); nothing closer is measurable.
    #   output_frame    -> rear_camera_link, NOT an optical frame. A LaserScan is x-along-view /
    #                      y-left, which is this link's convention; the node reads only the depth
    #                      image's pixels and cx/fx, never its frame_id, so the sim stamping every
    #                      rear stream with the x-forward link (vehicle.xacro's gz_frame_id) is
    #                      not a problem here.
    rear_depth_to_scan_node = Node(
        package='depthimage_to_laserscan',
        executable='depthimage_to_laserscan_node',
        name='rear_depth_to_scan',
        parameters=[{
            'scan_time': 0.033,
            'range_min': 0.3,
            'range_max': 8.0,
            'scan_height': 40,
            'output_frame': 'rear_camera_link',
        }],
        remappings=[
            ('depth', '/camera_rear/depth/image_raw'),
            ('depth_camera_info', '/camera_rear/camera_info'),
            ('scan', '/scan_rear'),
        ],
        output='screen',
        condition=LaunchConfigurationEquals('rear_scan', 'true'),
    )

    # Publishes /lane/drivable_area, the colour-based drivable-ground classification that
    # hyper_costmap_plugins' DrivableAreaLayer folds into the local costmap. Off by default: it is
    # the only output of this stage that steers the vehicle rather than just being watchable, so
    # enabling it is a decision an entrypoint launch file makes on purpose. Turning it on here
    # without also adding drivable_area_layer to the local_costmap plugins in
    # hyper_planner/config/nav2_controller.yaml just publishes a topic nobody reads.
    drivable_area_arg = DeclareLaunchArgument(
        'drivable_area',
        default_value='false',
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
            # The point cloud needs the same treatment for the same reason: this driver hardcodes
            # its cloud topic to "~/depth/color/points" (it is the only cloud topic string in
            # librealsense2_camera.so -- the name does not follow pointcloud.stream_filter), so
            # /camera_rear/depth/points, the name the sim's ros_gz_bridge publishes and RViz's
            # follow_path config expects, only exists if it is remapped here.
            ComposableNode(
                package='realsense2_camera',
                plugin='realsense2_camera::RealSenseNodeFactory',
                name='camera_rear',
                namespace='',
                parameters=[d435i_params],
                remappings=[
                    ('/camera_rear/color/image_raw', '/camera_rear/image_raw'),
                    ('/camera_rear/depth/color/points', '/camera_rear/depth/points'),
                ],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='hyper_lane_detection',
                plugin='LaneDetection',
                name='lane_detection',
                parameters=[
                    real_bev_params,
                    {
                        'input_backend': 'intra_process',
                        'drivable.enabled': ParameterValue(
                            LaunchConfiguration('drivable_area'), value_type=bool),
                    },
                ],
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

    # Grafts realsense2_camera's TF subtree onto the URDF. The driver publishes its own frames
    # (camera_rear_depth_frame -> camera_rear_depth_optical_frame, and the colour pair) under a
    # root named <camera_name>_<base_frame_id> == camera_rear_link, with nothing joining it to
    # body_link. The point cloud is stamped camera_rear_depth_optical_frame, so without this link
    # it is untransformable and RViz/costmaps drop every message.
    # camera_name is set in params_d435i.yaml for exactly this reason -- at its default the root
    # is the bare "camera_link" the URDF already uses for the FRONT camera, which silently
    # re-parents the rear cloud to the front of the car.
    # Identity is correct rather than approximate in orientation: both rear_camera_link and the
    # driver's root are x-forward REP-103 body frames, and rear_camera_link already carries the
    # rear-facing yaw (the resulting body_link -> camera_depth_optical_frame maps optical +z to
    # body -x). The residual translation is the sub-centimetre offset between the D435i's case
    # origin and its left imager, which is below this camera's depth noise at any usable range.
    rear_camera_tf_graft_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rear_camera_tf_graft',
        arguments=[
            '--frame-id', 'rear_camera_link',
            '--child-frame-id', 'camera_rear_link',
        ],
        condition=LaunchConfigurationEquals('lane_input_backend', 'intra_process'),
        output='screen',
    )

    # ros_raw: no physical camera to publish here -- lane_detection_node runs standalone and
    # subscribes to whatever already publishes the remap target (ros_gz_bridge in sim).
    lane_detection_node = Node(
        package='hyper_lane_detection',
        executable='lane_detection_node',
        parameters=[{
            'input_backend': LaunchConfiguration('lane_input_backend'),
            'drivable.enabled': ParameterValue(
                LaunchConfiguration('drivable_area'), value_type=bool),
        }],
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
        drivable_area_arg,
        rear_scan_arg,
        rear_depth_to_scan_node,
        rear_camera_tf_graft_node,
        lane_detection_container,
        lane_detection_node,
        logitech_camera_publisher_node,
        object_detection_node,
    ])
