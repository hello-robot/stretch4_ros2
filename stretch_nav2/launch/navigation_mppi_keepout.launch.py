"""Navigation with MPPI + optional KeepoutFilter / SpeedFilter masks."""

import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from hello_helpers.multi_yaml import MultiYaml


def _bool_cfg(context, name, default=True):
    raw = LaunchConfiguration(name).perform(context).strip().lower()
    if raw in ('', 'default'):
        return default
    return raw in ('true', '1', 'yes')


def _filter_overlay_with_flags(enable_keepout: bool, enable_speed: bool) -> str:
    """Load keepout/speed overlay and set each filter's enabled flag."""
    pkg = get_package_share_directory('stretch_nav2')
    base_path = os.path.join(pkg, 'config', 'nav2_params_keepout_speed.yaml')
    with open(base_path, 'r', encoding='utf-8') as f:
        data = yaml.safe_load(f)

    global_p = data['global_costmap']['global_costmap']['ros__parameters']
    global_p['keepout_filter']['enabled'] = enable_keepout
    global_p['speed_filter']['enabled'] = enable_speed

    local_p = data['local_costmap']['local_costmap']['ros__parameters']
    local_p['keepout_filter']['enabled'] = enable_keepout

    fd, path = tempfile.mkstemp(prefix='nav2_filters_', suffix='.yaml')
    with os.fdopen(fd, 'w', encoding='utf-8') as out:
        yaml.safe_dump(data, out, default_flow_style=False)
    return path


def _launch_setup(context, *args, **kwargs):
    stretch_core_path = FindPackageShare('stretch_core')
    stretch_navigation_path = FindPackageShare('stretch_nav2')

    enable_keepout = _bool_cfg(context, 'enable_keepout', True)
    enable_speed = _bool_cfg(context, 'enable_speed', True)

    yaml_list = [
        PathJoinSubstitution([stretch_navigation_path, 'config', 'original_nav2_params.yaml']),
        PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_core.yaml']),
        PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_mppi.yaml']),
        PathJoinSubstitution([stretch_navigation_path, 'config', 'mppi_params.yaml']),
    ]
    if enable_keepout or enable_speed:
        yaml_list.append(_filter_overlay_with_flags(enable_keepout, enable_speed))

    actions = [
        IncludeLaunchDescription(
            PathJoinSubstitution([stretch_core_path, 'launch', 'stretch_driver.launch.py']),
            launch_arguments={'broadcast_odom_tf': 'True', 'mode': 'navigation'}.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([stretch_core_path, 'launch', 'dual_hesai.launch.py']),
            launch_arguments={
                'filter_type': 'sor_ransac',
                'tool_preset': LaunchConfiguration('tool_preset'),
            }.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([stretch_core_path, 'launch', 'robot_footprint.launch.py']),
            launch_arguments={'tool_preset': LaunchConfiguration('tool_preset')}.items(),
        ),
        IncludeLaunchDescription(
            PathJoinSubstitution([stretch_navigation_path, 'launch', 'include', 'nav_core.launch.py']),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': MultiYaml(yaml_list),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'use_composition': LaunchConfiguration('use_composition'),
            }.items(),
        ),
    ]

    if enable_keepout or enable_speed:
        actions.append(IncludeLaunchDescription(
            PathJoinSubstitution([
                stretch_navigation_path, 'launch', 'include', 'keepout_speed_filters.launch.py'
            ]),
            launch_arguments={
                'keepout_mask': LaunchConfiguration('keepout_mask'),
                'speed_mask': LaunchConfiguration('speed_mask'),
                'enable_keepout': 'true' if enable_keepout else 'false',
                'enable_speed': 'true' if enable_speed else 'false',
                'use_sim_time': 'false',
                'autostart': 'true',
            }.items(),
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Full path to the occupancy map.yaml',
        ),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value='',
            description='Full path to keepout mask yaml (when enable_keepout)',
        ),
        DeclareLaunchArgument(
            'speed_mask',
            default_value='',
            description='Full path to speed mask yaml (when enable_speed)',
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
        DeclareLaunchArgument(
            'tool_preset',
            default_value='auto',
            description='Mounted tool preset for lidar self-filter: auto, sg4, pg4, tablet, or nil',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=['true', 'false'],
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            choices=['True', 'False'],
        ),
        OpaqueFunction(function=_launch_setup),
    ])
