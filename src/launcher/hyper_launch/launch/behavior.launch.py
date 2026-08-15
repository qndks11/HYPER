from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    default_params = PathJoinSubstitution([
        FindPackageShare('hyper_planner'), 'config', 'parking_params.yaml'])
    default_course = PathJoinSubstitution([
        EnvironmentVariable('HOME'), 'HYPER', 'src', 'waypoint', 'course.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('course_yaml', default_value=default_course),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('hyper_planner'), 'launch', 'parking_system_cpp.launch.py'])),
            launch_arguments={
                'params_file': LaunchConfiguration('params_file'),
                'course_yaml': LaunchConfiguration('course_yaml'),
            }.items(),
        ),
    ])
