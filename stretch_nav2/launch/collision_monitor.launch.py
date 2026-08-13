"""Collision monitor + stretch driver + lidar.

Params used are the same ones used in navigation.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml

from hello_helpers.multi_yaml import MultiYaml


def generate_launch_description():
    stretch_core_path = FindPackageShare('stretch_core')
    stretch_navigation_path = FindPackageShare('stretch_nav2')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    lifecycle_nodes = ['collision_monitor']
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    default_params = MultiYaml([
        PathJoinSubstitution([stretch_navigation_path, 'config', 'original_nav2_params.yaml']),
        PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_core.yaml']),
        PathJoinSubstitution([stretch_navigation_path, 'config', 'collision_monitor_standalone.yaml']),
    ])

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    stretch_driver_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'stretch_driver.launch.py']),
        launch_arguments={
            'broadcast_odom_tf': 'True',
            # navigation mode: accepts /cmd_vel (after collision_monitor)
            'mode': 'navigation',
        }.items(),
    )

    hlidar_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'dual_hesai.launch.py']),
        launch_arguments={
            'filter_type': 'sor_ransac',
            'tool_preset': LaunchConfiguration('tool_preset'),
        }.items(),
    )

    footprint_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'robot_footprint.launch.py']),
        launch_arguments={
            'tool_preset': LaunchConfiguration('tool_preset'),
            'joystick_control': 'true',
            'publish_footprint_stamped': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Top-level namespace',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically configure and activate collision_monitor',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description=(
                'Params file(s) for collision_monitor. Default matches navigation '
                'MultiYaml (original_nav2_params.yaml + nav2_params_core.yaml).'
            ),
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='Whether to respawn if a node crashes',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Log level',
        ),
        DeclareLaunchArgument(
            'tool_preset',
            default_value='auto',
            description='Mounted tool preset for lidar self-filter: auto, sg4, pg4, tablet, or nil',
        ),
        stretch_driver_launch,
        hlidar_launch,
        footprint_launch,
        Node(
            package='nav2_collision_monitor',
            executable='collision_monitor',
            name='collision_monitor',
            output='screen',
            respawn=use_respawn,
            respawn_delay=2.0,
            parameters=[configured_params, {'use_sim_time': use_sim_time}],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_collision_monitor',
            output='screen',
            emulate_tty=True,
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': lifecycle_nodes},
            ],
            arguments=['--ros-args', '--log-level', log_level],
            remappings=remappings,
        ),
    ])
