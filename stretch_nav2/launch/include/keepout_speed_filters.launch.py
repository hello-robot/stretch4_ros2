"""Launch keepout + speed costmap filter mask/info servers."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    pkg = get_package_share_directory('stretch_nav2')
    params_file = LaunchConfiguration('filter_params_file')
    keepout_mask = LaunchConfiguration('keepout_mask')
    speed_mask = LaunchConfiguration('speed_mask')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    lifecycle_nodes = [
        'keepout_filter_mask_server',
        'keepout_costmap_filter_info_server',
        'speed_filter_mask_server',
        'speed_costmap_filter_info_server',
    ]

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key='',
        param_rewrites={'use_sim_time': use_sim_time},
        convert_types=True,
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'filter_params_file',
            default_value=os.path.join(pkg, 'config', 'keepout_speed_filter_servers.yaml'),
        ),
        DeclareLaunchArgument('keepout_mask', description='Full path to keepout mask yaml'),
        DeclareLaunchArgument('speed_mask', description='Full path to speed mask yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_filter_mask_server',
            output='screen',
            parameters=[configured_params, {'yaml_filename': keepout_mask}],
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='keepout_costmap_filter_info_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='speed_filter_mask_server',
            output='screen',
            parameters=[configured_params, {'yaml_filename': speed_mask}],
        ),
        Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='speed_costmap_filter_info_server',
            output='screen',
            parameters=[configured_params],
        ),
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_costmap_filters',
            output='screen',
            parameters=[
                {'use_sim_time': use_sim_time},
                {'autostart': autostart},
                {'node_names': lifecycle_nodes},
            ],
        ),
    ])
