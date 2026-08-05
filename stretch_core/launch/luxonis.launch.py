import os
from pathlib import Path

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from stretch_core.vision.vision_topics import (
    VisionTopics,
    get_camera_calibration_file_path,
)
from hello_helpers.launch_utils import get_rviz_node

def is_launch_config_true(context, name):
    return LaunchConfiguration(name).perform(context) == "true"


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="If true, launch Rviz2 automatically.",
    )

    use_left = DeclareLaunchArgument(
        "use_left",
        default_value="true",
    )
    use_right = DeclareLaunchArgument(
        "use_right",
        default_value="true",
    )
    use_center = DeclareLaunchArgument(
        "use_center",
        default_value="false",
    )

    launch_args = [
        use_rviz_arg,
        use_left,
        use_right,
        use_center,
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    is_use_left = is_launch_config_true(context, "use_left")
    is_use_right = is_launch_config_true(context, "use_right")
    is_use_center = is_launch_config_true(context, "use_center")

    camera_node = Node(
        package="stretch_core",
        executable="luxonis_camera_node",
        name="luxonis_camera_node",
        parameters=[
            {
                "use_left": is_use_left,
                "use_right": is_use_right,
                "use_center": is_use_center,
                "is_gripper": False,
            }
        ],
        output="both",
    )

    camera_info_nodes = []
    for camera_name in ["left", "right", "center"]:
        calibration_file_path = get_camera_calibration_file_path(camera_name)
        if is_launch_config_true(context, f"use_{camera_name}") and Path(calibration_file_path).exists():
            camera_info_nodes.append(
                Node(
                    package="stretch_core",
                    executable="camera_info_publisher",
                    name=f"camera_info_publisher_{camera_name}",
                    parameters=[
                        {
                            "camera_name": camera_name,
                        }
                    ],
                )
            )
        else:
            print(f"{camera_name} does not have a calibration file at {calibration_file_path}.")

    rviz_config_path = os.path.join(
        get_package_share_directory("stretch_core"), "rviz", "cameras.rviz"
    )

    return [camera_node] + camera_info_nodes + get_rviz_node(rviz_config_path)
