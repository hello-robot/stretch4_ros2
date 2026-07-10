from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('stretch_base_hazard')
    config = PathJoinSubstitution([pkg, 'config', 'hazard_teleop_filter.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config),
        Node(
            package='stretch_base_hazard',
            executable='hazard_cmd_vel_filter_node',
            name='hazard_cmd_vel_filter_node',
            output='screen',
            parameters=[LaunchConfiguration('config_file')],
        ),
    ])
