import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, LogInfo
import launch.logging as logger
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def launch_setup(context, *args, **kwargs): 

    additional_marker_dict = LaunchConfiguration("aruco_config_filepath").perform(context)
    if not additional_marker_dict:
        additional_marker_dict = os.path.join(get_package_share_directory('stretch_tag_perception'), 'config', 'user_aruco_dictionary.yaml')
        
    stretch_marker_dict = os.path.join(get_package_share_directory('stretch_tag_perception'), 'config', 'stretch_marker_dict.yaml')

    return [
        Node(
                package='stretch_tag_perception',
                executable='aruco_detection.py',
                output='screen',
                parameters=[
                    stretch_marker_dict,
                    additional_marker_dict,
                    {'cameras': LaunchConfiguration("cameras")}
                ],
            )
        ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "aruco_config_filepath",
            default_value="",
            description="Filepath to a yaml file with additional aruco configuration parameters (optional)."
        ),
        DeclareLaunchArgument(
            "cameras",
            default_value="center",
            description="Camera(s) to use for detection (comma-separated list of: left, right, center, or 'all')."
        ),
        OpaqueFunction(function=launch_setup)
    ])