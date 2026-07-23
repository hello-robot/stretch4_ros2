import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def launch_setup(context, *args, **kwargs): 

    additional_marker_dict = LaunchConfiguration("aruco_config_filepath").perform(context)
    if not additional_marker_dict:
        additional_marker_dict = os.path.join(get_package_share_directory('stretch_tag_perception'), 'config', 'user_aruco_dict.yaml')
        
    stretch_marker_dict = os.path.join(get_package_share_directory('stretch_tag_perception'), 'config', 'stretch_marker_dict.yaml')
    
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_path = os.path.join(get_package_share_directory('stretch_tag_perception'), 'rviz', 'wrist_tag.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(use_rviz),
    )

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
        DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="If true, launch Rviz2 automatically.",
    ),
        # DeclareLaunchArgument(
        #     "publish_markers",
        #     default_value="false",
        #     description="Publish the markers topic. If you do not publish this, the detections will still be available via TF."
        # ),
        OpaqueFunction(function=launch_setup)
    ])