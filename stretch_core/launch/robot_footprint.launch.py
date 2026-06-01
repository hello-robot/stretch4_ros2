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
    return [
        Node(
            package='stretch_core',
            executable='robot_footprint_publisher',
            name='robot_footprint_publisher',
            output='screen',
            parameters=footprint_self_filter_parameters(stretch_core, tool_preset),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'tool_preset',
            default_value='sg4',
            description='Self-filter preset: sg4, pg4, tablet, or nil',
        ),
        OpaqueFunction(function=launch_setup),
    ])
