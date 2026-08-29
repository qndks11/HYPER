import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# Stages are staggered to give Gazebo time to come up before the nodes that
# depend on it attach -- mirrors run_all.sh's sleep 5 / sleep 2 / sleep 2 gaps
# between terminals.
ODOMETRY_DELAY_S = 5.0
PERCEPTION_DELAY_S = 7.0
BEHAVIOR_DELAY_S = 9.0


def generate_launch_description():
    hyper_launch_share = get_package_share_directory('hyper_launch')

    def stage(name, **launch_arguments):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(hyper_launch_share, 'launch', name)),
            launch_arguments=launch_arguments.items())

    return LaunchDescription([
        # 어떤 미션을 실을지. hyper_planner/config/<이름>.yaml로 풀립니다.
        # mission:=simple 이면 코스 한 바퀴만 도는 단일 골 미션입니다.
        DeclareLaunchArgument('mission', default_value='mission'),
        # headless:=true면 Gazebo 3D 창을 띄우지 않습니다. 센서 렌더링은 오프스크린으로
        # 그대로 돌아가므로 카메라/라이다 토픽은 동일하게 나오고, 시각화는 rviz로 하면 됩니다.
        DeclareLaunchArgument(
            'headless', default_value='false',
            description='Run Gazebo without the 3D GUI window (sensors still render offscreen)'),
        # 렌더링은 기본이 네이티브 GPU입니다. WSL2에서만 software_rendering:=true가 필요합니다
        # (WSL2 가상 GPU가 ign gazebo를 죽이는 문제 우회 -- vehicle.launch.py 주석 참고).
        DeclareLaunchArgument(
            'software_rendering', default_value='false',
            description='Force llvmpipe software rendering. Only needed on WSL2.'),
        # 전방 카메라의 색 기반 주행가능영역 분류(/lane/drivable_area)를 켭니다. 기본은 off --
        # 이 토픽을 실제로 읽는 쪽(hyper_planner/config/nav2_controller.yaml의 local_costmap
        # plugins에 있는 drivable_area_layer)도 따로 켜야 주행이 달라집니다. 켜기 전에
        # /lane/drivable/image_raw를 rqt_image_view로 먼저 확인하세요.
        DeclareLaunchArgument(
            'drivable_area', default_value='false',
            description="Publish the camera drivable-area grid for nav2's DrivableAreaLayer"),
        # 후방 RGBD 카메라의 깊이 영상을 유사 라이다(/scan_rear)로 변환해 local costmap의 두 번째
        # 관측 소스로 씁니다. 기본은 off. 카메라가 수평(rear_camera_pitch 0.0)이라는 전제 위에
        # 서 있으니 마운트를 바꿀 거면 perception.launch.py의 주석을 먼저 읽으세요.
        DeclareLaunchArgument(
            'rear_scan', default_value='false',
            description="Publish /scan_rear from the rear camera's depth stream"),
        stage('sim.launch.py',
              headless=LaunchConfiguration('headless'),
              software_rendering=LaunchConfiguration('software_rendering')),
        TimerAction(period=ODOMETRY_DELAY_S, actions=[stage('odometry.launch.py')]),
        # Gazebo bridges plain sensor_msgs/Image already (see ros_gz_bridge.yaml), so
        # lane_detection_node runs input_backend ros_raw here -- no rectification, no
        # image_transport/compressed subscription. object_detection_node's own default is
        # already ros_raw, so no override needed for it.
        TimerAction(period=PERCEPTION_DELAY_S, actions=[
            stage('perception.launch.py', lane_input_backend='ros_raw',
                  drivable_area=LaunchConfiguration('drivable_area'),
                  rear_scan=LaunchConfiguration('rear_scan'))]),
        TimerAction(period=BEHAVIOR_DELAY_S, actions=[
            stage('behavior.launch.py', mission=LaunchConfiguration('mission'))]),
    ])
