"""Launch keepout and/or speed costmap filter mask/info servers.

Args enable_keepout / enable_speed select which servers start.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import RewrittenYaml


def _bool_cfg(context, name, default=True):
    raw = LaunchConfiguration(name).perform(context).strip().lower()
    if raw in ('', 'default'):
        return default
    return raw in ('true', '1', 'yes')


def _launch_setup(context, *args, **kwargs):
    pkg = get_package_share_directory('stretch_nav2')
    params_file = LaunchConfiguration('filter_params_file').perform(context)
    keepout_mask = LaunchConfiguration('keepout_mask').perform(context)
    speed_mask = LaunchConfiguration('speed_mask').perform(context)
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    enable_keepout = _bool_cfg(context, 'enable_keepout', True)
    enable_speed = _bool_cfg(context, 'enable_speed', True)

    configured_params = RewrittenYaml(
        source_file=params_file if params_file else os.path.join(
            pkg, 'config', 'nav2_filter_servers.yaml'),
        root_key='',
        param_rewrites={'use_sim_time': LaunchConfiguration('use_sim_time')},
        convert_types=True,
    )

    nodes = []
    lifecycle_nodes = []

    if enable_keepout:
        if not keepout_mask:
            raise RuntimeError('enable_keepout=true but keepout_mask is empty')
        lifecycle_nodes.extend([
            'keepout_filter_mask_server',
            'keepout_costmap_filter_info_server',
        ])
        nodes.append(Node(
            package='nav2_map_server',
            executable='map_server',
            name='keepout_filter_mask_server',
            output='screen',
            parameters=[configured_params, {'yaml_filename': keepout_mask}],
        ))
        nodes.append(Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='keepout_costmap_filter_info_server',
            output='screen',
            parameters=[configured_params],
        ))

    if enable_speed:
        if not speed_mask:
            raise RuntimeError('enable_speed=true but speed_mask is empty')
        lifecycle_nodes.extend([
            'speed_filter_mask_server',
            'speed_costmap_filter_info_server',
        ])
        nodes.append(Node(
            package='nav2_map_server',
            executable='map_server',
            name='speed_filter_mask_server',
            output='screen',
            parameters=[configured_params, {'yaml_filename': speed_mask}],
        ))
        nodes.append(Node(
            package='nav2_map_server',
            executable='costmap_filter_info_server',
            name='speed_costmap_filter_info_server',
            output='screen',
            parameters=[configured_params],
        ))

    if not lifecycle_nodes:
        return []

    nodes.append(Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_costmap_filters',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': autostart},
            {'node_names': lifecycle_nodes},
        ],
    ))
    return nodes


def generate_launch_description():
    pkg = get_package_share_directory('stretch_nav2')
    return LaunchDescription([
        DeclareLaunchArgument(
            'filter_params_file',
            default_value=os.path.join(pkg, 'config', 'nav2_filter_servers.yaml'),
        ),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value='',
            description='Full path to keepout mask yaml (required if enable_keepout)',
        ),
        DeclareLaunchArgument(
            'speed_mask',
            default_value='',
            description='Full path to speed mask yaml (required if enable_speed)',
        ),
        DeclareLaunchArgument(
            'enable_keepout',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'enable_speed',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('autostart', default_value='true'),
        OpaqueFunction(function=_launch_setup),
    ])
