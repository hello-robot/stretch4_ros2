from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os
import sys


def launch_setup(context, *args, **kwargs):
    stretch_core = get_package_share_directory('stretch_core')
    sys.path.insert(0, os.path.join(stretch_core, 'launch'))
    from self_filter_config import (
        dual_lidar_self_filter_parameters,
        robot_self_filter_yaml,
        tool_preset_yaml,
        validate_tool_preset,
    )

    tool_preset = LaunchConfiguration('tool_preset').perform(context)
    validate_tool_preset(tool_preset)

    launch_lidar = LaunchConfiguration('launch_lidar').perform(context)
    launch_filter_node = LaunchConfiguration('launch_filter_node').perform(context)
    launch_viz_node = LaunchConfiguration('launch_viz_node').perform(context)
    use_rviz = LaunchConfiguration('use_rviz').perform(context)
    pub_pc = LaunchConfiguration('pub_pc').perform(context).lower() == 'true'

    robot_filter_yaml = robot_self_filter_yaml(stretch_core)
    preset_yaml = tool_preset_yaml(stretch_core, tool_preset)

    nodes = []

    if launch_lidar.lower() == 'true':
        nodes.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(stretch_core, 'launch', 'dual_hesai.launch.py')
                ),
                launch_arguments={
                    'filter_type': 'region',
                    'launch_filter_node': 'false',
                    'use_rviz': 'false',
                    'tool_preset': tool_preset,
                }.items(),
            )
        )

    viz_params = [
        robot_filter_yaml,
        preset_yaml,
        {
            'lidar1_frame': 'lidar_right_link',
            'lidar2_frame': 'lidar_left_link',
            'pub_pc': pub_pc,
            'pub_self_filter_markers': True,
        },
    ]

    if launch_filter_node.lower() == 'true':
        nodes.append(
            Node(
                package='stretch_core',
                executable='dual_lidar_laserscan',
                name='pointcloud_to_laserscan',
                output='screen',
                parameters=[
                    *dual_lidar_self_filter_parameters(stretch_core, tool_preset),
                    {
                        'lidar1_frame': 'lidar_right_link',
                        'lidar2_frame': 'lidar_left_link',
                        'pub_pc': pub_pc,
                        'pub_self_filter_markers': True,
                    },
                ],
            )
        )

    if launch_viz_node.lower() == 'true':
        nodes.append(
            Node(
                package='stretch_core',
                executable='self_filter_viz_node',
                name='self_filter_viz_node',
                output='screen',
                parameters=[
                    *viz_params,
                ],
            )
        )

    if use_rviz.lower() == 'true':
        nodes.append(
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', os.path.join(stretch_core, 'rviz', 'self_filter_debug.rviz')],
            )
        )

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'launch_lidar',
            default_value='true',
            description='Launch Hesai dual-lidar driver.',
        ),
        DeclareLaunchArgument(
            'launch_filter_node',
            default_value='true',
            description='Launch dual_lidar_laserscan with self-filter enabled.',
        ),
        DeclareLaunchArgument(
            'launch_viz_node',
            default_value='false',
            description='Launch marker-only self_filter_viz_node instead of filter node.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            description='Launch RViz with self_filter_debug.rviz.',
        ),
        DeclareLaunchArgument(
            'pub_pc',
            default_value='false',
            description='Publish /filtered_points debug cloud.',
        ),
        DeclareLaunchArgument(
            'tool_preset',
            default_value='sg4',
            description='Self-filter attachment preset: sg4, pg4, tablet, or nil.',
        ),
        OpaqueFunction(function=launch_setup),
    ])
