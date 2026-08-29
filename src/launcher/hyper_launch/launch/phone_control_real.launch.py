import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import FrontendLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

# Real-car equivalent of phone_control_sim.launch.py: same rosbridge_websocket bridge so a phone
# app can publish /steering_angle and /velocity (std_msgs/Float64) directly over
# ws://<host-ip>:<port>, but drives the Arduino motor-driver interface instead of Gazebo.
# arduino_interface_node subscribes to those same two topics directly (see
# hyper_interface/README.md), so no vehicle_controller_node/ros2_control translation step sits
# in between here. Includes real.launch.py's sensors, odometry and perception stages (IMU,
# lidar, RTK GPS, ekf_local/ekf_global/navsat_transform, cameras/lane+object detection) but
# deliberately does NOT include its behavior stage -- hyper_planner's cmd_vel_to_ackermann_node
# (started by behavior.launch.py) also publishes to /steering_angle and /velocity, converted
# from nav2's /cmd_vel, so running both at once would have the phone and the autonomy stack
# fight over the same two topics. Odometry itself doesn't touch either topic, so it's safe to
# run alongside phone control.
ODOMETRY_DELAY_S = 5.0  # same delay real.launch.py uses, so sensor drivers are up first
PERCEPTION_DELAY_S = 7.0  # same delay real.launch.py uses, so USB camera enumeration finishes


def generate_launch_description():
    rosbridge_launch = IncludeLaunchDescription(
        FrontendLaunchDescriptionSource(os.path.join(
            get_package_share_directory('rosbridge_server'),
            'launch', 'rosbridge_websocket_launch.xml')),
        launch_arguments={
            'port': LaunchConfiguration('rosbridge_port'),
        }.items(),
    )

    # Publishes body_link -> camera_link/lidar_link/gps_link TF, same as real.launch.py.
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_control'),
            'launch', 'robot_state_publisher.launch.py')),
    )

    # ROS <-> Arduino serial bridge, same as real.launch.py's interface.launch.py stage.
    arduino_interface = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_interface'),
            'launch', 'interface.launch.py')),
    )

    # IMU + lidar + RTK GPS, same as real.launch.py's sensors stage. Cameras aren't part of
    # this (they're owned by perception.launch.py, which this file doesn't include).
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_launch'),
            'launch', 'sensors.launch.py')),
    )

    # EKF localization, same as real.launch.py's odometry stage. use_sim_time=false is required
    # on the real car -- nothing publishes /clock, so leaving dual_ekf_navsat.yaml's
    # use_sim_time: true would stall ekf_local/ekf_global/navsat_transform on a frozen clock.
    odometry = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_launch'),
            'launch', 'odometry.launch.py')),
        launch_arguments={
            'datum_site': LaunchConfiguration('datum_site'),
            'use_sim_time': 'false',
        }.items(),
    )

    # Cameras + lane/object detection, same as real.launch.py's perception stage.
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_launch'),
            'launch', 'perception.launch.py')),
        launch_arguments={
            'lane_input_backend': 'intra_process',
            'object_input_backend': 'usb_camera',
        }.items(),
    )

    # 웨이포인트 레코더 + 조작판 GUI. joystick_control_real.launch.py와 같은 구성입니다
    # -- 폰으로 몰든 스틱으로 몰든 수동 주행의 목적은 코스를 따는 것이라, 녹화가 따라붙는
    # 게 맞습니다. auto_start:=false라 GUI의 Record를 누를 때까지 기다립니다: 기록 시작이
    # 곧 CSV truncate이므로 스택을 띄우는 것만으로 지난 녹화본이 날아가지 않습니다.
    waypoint_record = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('hyper_waypoint'),
            'launch', 'record.launch.py')),
        launch_arguments={
            'waypoint_csv': LaunchConfiguration('waypoint_csv'),
            'min_spacing_m': LaunchConfiguration('min_spacing_m'),
            'use_record_gui': LaunchConfiguration('use_record_gui'),
        }.items(),
    )

    # Shows the robot's current position (RobotModel/TF) and the trail it has driven so far
    # (an Odometry display on /odometry/filtered_map with a large Keep count, since nothing in
    # this stack publishes an accumulated nav_msgs/Path). Fixed Frame is map, so the trail stays
    # put as the robot moves through it.
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            get_package_share_directory('hyper_launch'),
            'config', 'phone_control_real.rviz')],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'rosbridge_port', default_value='9090',
            description='TCP port for the rosbridge websocket server'),
        # GPS 원점. hyper_localization/config/datums.yaml의 키입니다. 대회장이 아닌 곳에서
        # 돌릴 때는 datum_site:=school 처럼 바꾸세요 (real.launch.py와 동일).
        DeclareLaunchArgument(
            'datum_site', default_value='track',
            description='GPS 원점 (datums.yaml의 sim | school | track)'),
        DeclareLaunchArgument(
            'use_rviz', default_value='true',
            description='Launch RViz showing robot position + driven trail'),
        # 녹화 인자는 hyper_waypoint/record.launch.py로 그대로 넘어갑니다.
        DeclareLaunchArgument(
            'waypoint_csv',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), 'HYPER', 'src', 'planning', 'hyper_waypoint',
                'waypoints', 'real.csv']),
            description='녹화 결과를 쓸 CSV (Record를 누르는 순간 truncate)'),
        DeclareLaunchArgument(
            'min_spacing_m', default_value='0.5',
            description='이 거리(m) 이상 이동했을 때만 한 점 기록'),
        DeclareLaunchArgument(
            'use_record_gui', default_value='true',
            description='녹화 조작판 GUI를 함께 띄웁니다'),

        robot_state_publisher,
        arduino_interface,
        sensors,
        TimerAction(period=ODOMETRY_DELAY_S, actions=[odometry]),
        TimerAction(period=PERCEPTION_DELAY_S, actions=[perception]),
        rosbridge_launch,
        rviz,
        waypoint_record,
    ])
