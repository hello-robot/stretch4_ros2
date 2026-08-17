import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from stretch_core.vision.vision_topics import (
    VisionTopics
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

    use_compressed = DeclareLaunchArgument(
        "use_compressed",
        default_value="true",
        description=(
            "If true, publish the camera's MJPEG stream on the compressed topics. The raw topics are "
            "then only published while something is subscribed, since serving them costs a decode per "
            "frame. The 16-bit depth image is always published raw."
        ),
    )

    launch_args = [
        use_rviz_arg,
        use_compressed,
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    camera_node = Node(
        package="stretch_core",
        executable="luxonis_camera_node",
        name="luxonis_camera_node_gripper",
        parameters=[
            {
                "is_gripper": True,
                "use_compressed": is_launch_config_true(context, "use_compressed"),
            }
        ],
        output="both",
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("stretch_core"), "rviz", "cameras_gripper.rviz"
    )

    return [camera_node] + get_rviz_node(rviz_config_path)
