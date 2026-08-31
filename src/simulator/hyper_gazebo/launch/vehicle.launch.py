import os
import xacro
import yaml

from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    RegisterEventHandler,
    ExecuteProcess,
    SetEnvironmentVariable
)
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node


def load_robot_description(robot_description_path, vehicle_params_path):
    """
    Loads the robot description from a Xacro file, using parameters from a YAML file.
    """
    with open(vehicle_params_path, 'r') as file:
        vehicle_params = yaml.safe_load(file)['/**']['ros__parameters']

    robot_description = xacro.process_file(
        robot_description_path,
        mappings={key: str(value) for key, value in vehicle_params.items()}
    )

    return robot_description.toxml()


def start_vehicle_control():
    """
    Starts the necessary controllers for the vehicle's operation in ROS 2.
    """
    joint_state_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'joint_state_broadcaster'
        ],
        output='screen'
    )

    forward_velocity_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'forward_velocity_controller'
        ],
        output='screen'
    )

    forward_position_controller = ExecuteProcess(
        cmd=[
            'ros2', 'control', 'load_controller',
            '--set-state', 'active',
            'forward_position_controller'
        ],
        output='screen'
    )

    return (
        joint_state_controller,
        forward_velocity_controller,
        forward_position_controller
    )


def generate_launch_description():
    # 시뮬레이션 전용 리소스(월드, gz 플러그인, ros_gz_bridge 설정)는 hyper_gazebo에,
    # 차량 자체(urdf, 파라미터, vehicle_controller_node 실행 파일)는 실차에서도 쓰는 hyper_control에 있다.
    sim_package_name = "hyper_gazebo"
    sim_package_path = get_package_share_directory(sim_package_name)

    vehicle_package_name = "hyper_control"
    vehicle_package_path = get_package_share_directory(vehicle_package_name)

    # 기본 Gazebo world 파일 경로
    default_world_path = os.path.join(
        sim_package_path,
        'worlds',
        'track.world'
    )

    # Gazebo Sim에서 model:// 경로를 찾을 수 있도록 설정
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            sim_package_path,
            os.pathsep,
            os.path.join(sim_package_path, 'worlds'),
            os.pathsep,
            # 💡 worlds 폴더 안의 models 폴더를 직접 바라보도록 수정합니다.
            os.path.join(sim_package_path, 'worlds', 'models'),
            os.pathsep,
            os.environ.get('GZ_SIM_RESOURCE_PATH', '')
        ]
    )

    # Gazebo Sim에서 커스텀 System 플러그인(신호등 자동 점멸 등)을 찾을 수 있도록 설정
    gz_plugin_path = SetEnvironmentVariable(
        name='GZ_SIM_SYSTEM_PLUGIN_PATH',
        value=[
            os.path.join(get_package_prefix(sim_package_name), 'lib', sim_package_name),
            os.pathsep,
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', '')
        ]
    )

    # WSL2의 가상 GPU(D3D12 변환) OpenGL 드라이버는 OGRE-Next GL3Plus 백엔드가 밉맵 생성에
    # 쓰는 glCopyImageSubData 일부 경로를 구현하지 않아 "GL3PlusTextureGpu::copyTo
    # UnimplementedException"으로 ign gazebo가 죽는다. llvmpipe 소프트웨어 렌더러로 강제 전환하면
    # 해당 경로가 완전히 구현되어 있어 크래시가 사라진다.
    #
    # 다만 이 우회는 WSL2에서만 필요하고, 네이티브 리눅스 + 실제 GPU에서는 모든 렌더링을
    # CPU로 끌어내려 손해만 본다(측정상 GUI를 켠 상태에서 RTF 0.833 -> 0.918). 그래서 기본값은
    # 네이티브 GPU 렌더링이고, WSL2에서 돌릴 때만 software_rendering:=true로 켜면 된다.
    software_rendering_arg = DeclareLaunchArgument(
        'software_rendering',
        default_value='false',
        description=(
            'Force llvmpipe software rendering. Only needed on WSL2, whose virtual GPU '
            'crashes ign gazebo in the OGRE-Next GL3Plus backend. Costs performance on '
            'native Linux with a real GPU.'
        )
    )

    gz_render_software = SetEnvironmentVariable(
        name='LIBGL_ALWAYS_SOFTWARE',
        value='1',
        condition=IfCondition(LaunchConfiguration('software_rendering'))
    )

    gz_gallium_driver = SetEnvironmentVariable(
        name='GALLIUM_DRIVER',
        value='llvmpipe',
        condition=IfCondition(LaunchConfiguration('software_rendering'))
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Specify the world file for Gazebo'
    )

    # headless:=true는 ign gazebo를 서버 전용(-s)으로 띄우고, 카메라/라이다가 쓰는 오프스크린
    # 렌더링만 --headless-rendering으로 남깁니다. 3D 창을 그리던 `ign gazebo gui` 프로세스가
    # 통째로 사라지므로(측정상 코어 하나를 상시 점유했음) 시뮬레이터가 실시간에 훨씬 가깝게
    # 돕니다. 센서 토픽은 그대로 나오므로 시각화가 필요하면 rviz를 쓰면 됩니다.
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without the 3D GUI window (sensors still render offscreen)'
    )

    headless_flags = PythonExpression([
        "'-s --headless-rendering ' if '",
        LaunchConfiguration('headless'),
        "'.lower() in ('true', '1') else ''"
    ])

    x_arg = DeclareLaunchArgument(
        'x',
        default_value='41.0866',
        description='Initial X position'
    )

    y_arg = DeclareLaunchArgument(
        'y',
        default_value='-45.6842',
        description='Initial Y position'
    )

    z_arg = DeclareLaunchArgument(
        'z',
        default_value='0.36',
        description='Initial Z position'
    )

    roll_arg = DeclareLaunchArgument(
        'R',
        default_value='0.00',
        description='Initial Roll'
    )

    pitch_arg = DeclareLaunchArgument(
        'P',
        default_value='0.00',
        description='Initial Pitch'
    )

    yaw_arg = DeclareLaunchArgument(
        'Y',
        default_value='1.64',
        description='Initial Yaw'
    )

    world_file = LaunchConfiguration('world')
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('R')
    pitch = LaunchConfiguration('P')
    yaw = LaunchConfiguration('Y')

    robot_description_path = os.path.join(
        vehicle_package_path,
        'urdf',
        'vehicle.xacro'
    )

    gz_bridge_params_path = os.path.join(
        sim_package_path,
        'config',
        'ros_gz_bridge.yaml'
    )

    vehicle_params_path = os.path.join(
        vehicle_package_path,
        'config',
        'parameters.yaml'
    )

    robot_description = load_robot_description(
        robot_description_path,
        vehicle_params_path
    )

    gazebo_pkg_launch = PythonLaunchDescriptionSource(
        os.path.join(
            get_package_share_directory('ros_gz_sim'),
            'launch',
            'gz_sim.launch.py'
        )
    )

    gazebo_launch = IncludeLaunchDescription(
        gazebo_pkg_launch,
        launch_arguments={
            'gz_args': ['-r -v 4 ', headless_flags, world_file],
            'on_exit_shutdown': 'true'
        }.items()
    )

    robot_name = "ackermann_steering_vehicle"

    spawn_model_gazebo_node = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', robot_name,
            '-string', robot_description,
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
            '-allow_renaming', 'false'
        ],
        output='screen'
    )

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': True
            }
        ],
        output='screen'
    )

    gz_bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={gz_bridge_params_path}'
        ],
        output='screen'
    )

    model_service_node = Node(
        package='hyper_gazebo',
        executable='model_service.py',
        output='screen'
    )

    # 차량을 mission.yaml의 라벨 위치로 옮기는 테스트용 서비스 (sim 전용).
    teleport_service_node = Node(
        package='hyper_gazebo',
        executable='teleport_service.py',
        output='screen'
    )

    # 두 차로 제어 표지판(초록 화살표 / 빨간 X)의 위치를 맞바꾸는 테스트용 서비스.
    lane_sign_service_node = Node(
        package='hyper_gazebo',
        executable='lane_sign_service.py',
        output='screen'
    )

    joint_state, forward_velocity, forward_position = start_vehicle_control()

    vehicle_controller_node = Node(
        package='hyper_control',
        executable='vehicle_controller_node',
        parameters=[vehicle_params_path],
        output='screen'
    )

    # ros_gz_bridge's parameter_bridge publishes camera/image_raw with a plain rclcpp
    # publisher, not image_transport, so no camera/image_raw/compressed topic exists
    # unless something republishes it explicitly.
    camera_compressed_republisher_node = Node(
        package='image_transport',
        executable='republish',
        name='camera_compressed_republisher',
        arguments=['raw', 'compressed'],
        remappings=[
            ('in', 'camera/image_raw'),
            ('out/compressed', 'camera/image_raw/compressed'),
        ],
        output='screen'
    )

    launch_description = LaunchDescription([
        software_rendering_arg,
        gz_render_software,
        gz_gallium_driver,
        gz_resource_path,
        gz_plugin_path,

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=spawn_model_gazebo_node,
                on_exit=[joint_state]
            )
        ),

        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=joint_state,
                on_exit=[
                    forward_velocity,
                    forward_position
                ]
            )
        ),

        world_arg,
        headless_arg,
        gazebo_launch,

        x_arg,
        y_arg,
        z_arg,
        roll_arg,
        pitch_arg,
        yaw_arg,

        spawn_model_gazebo_node,
        robot_state_publisher_node,
        vehicle_controller_node,
        gz_bridge_node,
        camera_compressed_republisher_node,
        model_service_node,
        teleport_service_node,
        lane_sign_service_node,
    ])

    return launch_description