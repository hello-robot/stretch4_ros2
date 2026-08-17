import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
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
        default_value="true",
    )
    publish_rotated = DeclareLaunchArgument(
        "publish_rotated",
        default_value="true",
        description="If true, publish rotated image frames on rotated_image topic.",
    )

    launch_args = [
        use_rviz_arg,
        use_left,
        use_right,
        use_center,
        publish_rotated,
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    is_use_left = is_launch_config_true(context, "use_left")
    is_use_right = is_launch_config_true(context, "use_right")
    is_use_center = is_launch_config_true(context, "use_center")
    is_publish_rotated = is_launch_config_true(context, "publish_rotated")

    camera_node = Node(
        package="stretch_core",
        executable="luxonis_camera_node",
        name="luxonis_camera_node",
        parameters=[
            {
                "use_left": is_use_left,
                "use_right": is_use_right,
                "use_center": is_use_center,
                "publish_rotated": is_publish_rotated,
                "is_gripper": False,
            }
        ],
        output="both",
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("stretch_core"), "rviz", "cameras_head.rviz"
    )

    return [camera_node] + get_rviz_node(rviz_config_path)
