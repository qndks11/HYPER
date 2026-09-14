from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# 조이스틱 버튼 비상정지. joystick.launch.py와 나란한 짝이지만, 이 트리는
# /velocity + /steering_angle을 절대 publish하지 않습니다 -- joy 버튼만 보고
# /estop(Bool, latched)만 내보내므로 미션(자율주행) 스택과 같이 떠도
# cmd_vel_to_ackermann_node와 토픽이 겹치지 않습니다.
#
# 실제 정지는 arduino_interface_node가 /estop을 보고 속도/조향을 0으로
# 강제하는 것으로 이뤄집니다(hyper_interface). 이 노드는 래치만 겁니다.
#
# joy_node는 기본적으로 띄우지 않습니다(launch_joy_node:=false). 한 트리에
# joy_node가 둘이면 같은 장치를 두 번 열고, 각자 20Hz autorepeat로 /joy에 같은
# 토픽을 쏘면서 joystick_controller_node(큐 깊이 1, 마지막 메시지를 그대로 들고
# 있음)의 스틱 반응이 눈에 띄게 느려집니다 -- 실제로 겪은 증상입니다.
#
# 그래서 기본값은 "장치를 열지 않는다"입니다. 수동 주행에서는
# joystick.launch.py가 이미 joy_node를 띄우므로 그대로 두면 되고, 미션처럼
# /joy를 내보내는 노드가 하나도 없는 스택에서만 launch_joy_node:=true로
# 켜세요. 안 켜면 버튼이 조용히 아무 일도 하지 않습니다.


def generate_launch_description():
    joy_node = Node(
        package='joy',
        executable='joy_node',
        condition=IfCondition(LaunchConfiguration('launch_joy_node')),
    )

    estop_controller_node = Node(
        package='hyper_control',
        executable='estop_controller_node',
        parameters=[{
            'estop_button_index': LaunchConfiguration('estop_button_index'),
            'resume_button_index': LaunchConfiguration('resume_button_index'),
        }],
        output='screen')

    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_joy_node', default_value='false',
            description='/joy를 내보내는 노드가 이 스택에 없을 때만 true (미션 스택)'),
        DeclareLaunchArgument(
            'estop_button_index', default_value='1',
            description='비상정지 버튼 인덱스 (ros2 topic echo /joy로 확인)'),
        DeclareLaunchArgument(
            'resume_button_index', default_value='0',
            description='해제 버튼 인덱스 (ros2 topic echo /joy로 확인)'),
        joy_node,
        estop_controller_node,
    ])
