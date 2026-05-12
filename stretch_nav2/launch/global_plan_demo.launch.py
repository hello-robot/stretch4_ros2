from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    stretch_nav2 = FindPackageShare('stretch_nav2')
    ld = LaunchDescription()

    ld.add_action(DeclareLaunchArgument('map_yaml'))

    ld.add_action(Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[PathJoinSubstitution([stretch_nav2, 'config', 'planner_demo.yaml'])],
    ))

    ld.add_action(Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': LaunchConfiguration('map_yaml')}],
    ))

    ld.add_action(Node(
        package='stretch_nav2',
        executable='global_plan_demo.py',
        output='screen',
    ))

    ld.add_action(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        output='screen',
        parameters=[{'autostart': True},
                    {'node_names': ['map_server', 'planner_server']}]))

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([stretch_nav2, 'rviz', 'global.rviz'])],
        output='screen'
    ))

    return ld
