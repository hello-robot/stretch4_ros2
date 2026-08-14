import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.insert(0, os.path.dirname(__file__))
from self_filter_config import footprint_self_filter_parameters, validate_tool_preset


def launch_setup(context, *args, **kwargs):
    stretch_core = get_package_share_directory('stretch_core')
    tool_preset = LaunchConfiguration('tool_preset').perform(context)
    validate_tool_preset(tool_preset)
    joystick_control = LaunchConfiguration('joystick_control').perform(context).lower() in (
        'true', '1', 'yes',
    )
    publish_footprint_stamped = LaunchConfiguration(
        'publish_footprint_stamped',
    ).perform(context).lower() in ('true', '1', 'yes')
    return [
        Node(
            package='stretch_core',
            executable='robot_footprint_publisher',
            name='robot_footprint_publisher',
            output='screen',
            parameters=footprint_self_filter_parameters(stretch_core, tool_preset) + [
                {'joystick_control': joystick_control},
                {'publish_footprint_stamped': publish_footprint_stamped},
            ],
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tool_preset',
            default_value='auto',
            description='Self-filter preset: auto, sg4, pg4, tablet, or nil',
        ),
        DeclareLaunchArgument(
            'joystick_control',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'If true, publish base-only footprint (no arm), same as calling '
                '~/joystick_control with data=true. Toggle later via that service.'
            ),
        ),
        DeclareLaunchArgument(
            'publish_footprint_stamped',
            default_value='false',
            choices=['true', 'false'],
            description=(
                'If true, also publish PolygonStamped on footprint_stamped_topic '
                '(for collision_monitor FootprintApproach without local_costmap).'
            ),
        ),
        OpaqueFunction(function=launch_setup),
    ])
