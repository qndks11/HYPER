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

    # Bird's-eye-view geometry for the real vehicle's camera. lane_detection's built-in defaults
    # describe that same camera as the simulator renders it, so only the intra_process (real
    # vehicle) path loads this -- and all it overrides is the camera's height above the ground,
    # which differs because the sim's body_link rides higher than the real car's.
    real_bev_params = os.path.join(
        get_package_share_directory('hyper_lane_detection'), 'config', 'bev_real.yaml')

    # Where this whole stage's camera frames come from. The vehicle has exactly one camera now --
    # a Logitech C920 that feeds both detectors -- so this single argument decides who owns it:
    #
    #   intra_process (real vehicle) -- hyper_camera's LogitechCameraPublisherNode component and
    #     LaneDetection are loaded into one ComposableNodeContainer, so the frame is handed to
    #     lane_detection by pointer instead of a serialized topic. The same publish still goes out
    #     over DDS, which is where object_detection_node (a separate rclpy process, with no
    #     zero-copy path available to it) picks up the identical frames.
    #   ros_raw (Gazebo simulation) -- no camera driver is launched at all; ros_gz_bridge already
    #     publishes /camera/image_raw and both detectors just subscribe.
    #
    # Note this now gates the camera for object detection too: running this stage with ros_raw on
    # the real car leaves *both* detectors without a source unless something else publishes
    # /camera/image_raw.
    #
    # See hyper_launch's real.launch.py / simulation.launch.py for which value each entrypoint
    # passes.
    lane_input_backend_arg = DeclareLaunchArgument(
        'lane_input_backend',
        default_value='intra_process',
    )

    # Publishes /lane/drivable_area, the colour-based drivable-ground classification that
    # hyper_costmap_plugins' DrivableAreaLayer folds into the local costmap. Off by default: it is
    # the only output of this stage that steers the vehicle rather than just being watchable, so
    # enabling it is a decision an entrypoint launch file makes on purpose. Turning it on here
    # without also adding drivable_area_layer to the local_costmap plugins in
    # hyper_planner/config/nav2_controller.yaml just publishes a topic nobody reads.
    sign_class_map_arg = DeclareLaunchArgument(
        'sign_class_map', default_value="['']",
        description="YOLO 클래스 이름 -> 신호 값 추가/덮어쓰기. \"['LaneBan:ban']\" 형태")

    drivable_area_arg = DeclareLaunchArgument(
        'drivable_area',
        default_value='false',
    )

    # intra_process: hyper_camera's LogitechCameraPublisherNode and LaneDetection load into one
    # process. rclcpp's intra-process manager hands the publisher's std::unique_ptr<Image>
    # straight to LaneDetection's subscription instead of serializing it over a DDS topic. The
    # publish is still visible on /camera/image_raw for out-of-process subscribers, which is how
    # object_detection_node below is fed off this very same camera.
    lane_detection_container = ComposableNodeContainer(
        name='lane_detection_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='hyper_camera',
                plugin='hyper_camera::LogitechCameraPublisherNode',
                name='logitech_camera_publisher',
                remappings=[('image_raw', '/camera/image_raw')],
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
                remappings=[('/image_raw', '/camera/image_raw')],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        condition=LaunchConfigurationEquals('lane_input_backend', 'intra_process'),
    )

    # ros_raw: no physical camera to open here -- lane_detection_node runs standalone and
    # subscribes to whatever already publishes the remap target (ros_gz_bridge in sim).
    lane_detection_node = Node(
        package='hyper_lane_detection',
        executable='lane_detection_node',
        parameters=[{
            'input_backend': LaunchConfiguration('lane_input_backend'),
            'drivable.enabled': ParameterValue(
                LaunchConfiguration('drivable_area'), value_type=bool),
        }],
        remappings=[('/image_raw', '/camera/image_raw')],
        output='screen',
        condition=LaunchConfigurationNotEquals('lane_input_backend', 'intra_process'),
    )

    # object_detection_node reads the same /camera/image_raw the lane detector does -- one
    # physical camera, two consumers. It is a separate rclpy process either way: rclpy has no
    # equivalent of rclcpp's intra-process comms, so this end is always a plain topic
    # subscription, whoever is publishing.
    #
    # sign_class_map maps YOLO class names onto the sign values mission_manager consumes
    # (red/green/left_arrow/ban/allow). The node's built-in map covers the class names the
    # current model uses; set this when a retrained model renames a class, so the sign does
    # not silently vanish. Entries are "<YoloClass>:<sign>", e.g.
    #   ros2 launch ... sign_class_map:="['LaneBan:ban','LaneAllow:allow']"
    object_detection_node = Node(
        package='hyper_object_detection',
        executable='object_detection_node',
        parameters=[{
            'model_path': model_path,
            'sign_class_map': LaunchConfiguration('sign_class_map'),
        }],
        remappings=[('/image_raw', '/camera/image_raw')],
        output='screen'
    )

    # On-demand frame grabber: subscribes to the same /camera/image_raw the detector
    # sees and writes the latest frame to images/ as shot_<timestamp>.png (auto-named, no
    # collisions) whenever its `~/save` std_srvs/Trigger service is called -- the same folder
    # lane_detection's `~/image_saving` recording fills with rec_<timestamp>.png, since there is
    # one camera and these are its frames either way. Relative, so it lands under whatever
    # directory the launch was started from. Handy for building up a labelling set from live
    # runs:
    #   ros2 service call /image_saver_service/save std_srvs/srv/Trigger
    image_saver_service_node = Node(
        package='hyper_object_detection',
        executable='image_saver_service',
        output='screen',
    )

    return LaunchDescription([
        lane_input_backend_arg,
        sign_class_map_arg,
        drivable_area_arg,
        lane_detection_container,
        lane_detection_node,
        object_detection_node,
        image_saver_service_node,
    ])
