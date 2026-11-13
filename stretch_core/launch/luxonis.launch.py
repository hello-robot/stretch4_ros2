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
    use_compressed = DeclareLaunchArgument(
        "use_compressed",
        default_value="true",
        description=(
            "If true, capture MJPEG on the camera and publish it on the compressed topics. The raw and "
            "rotated topics are then only published while something is subscribed, since serving them "
            "costs a decode per frame."
        ),
    )

    fps = DeclareLaunchArgument(
        "fps",
        default_value="0",
        description=(
            "Frame rate to open the left and right head cameras at. 0 keeps the rate each camera is "
            "configured with in stretch4_body's CAMERA_CONFIGS. The two are a synced stereo pair, so "
            "they share one rate; the center camera always keeps its own."
        ),
    )

    use_stereo = DeclareLaunchArgument(
        "use_stereo",
        default_value="false",
        description=(
            "If true, publish the left and right head cameras as a rectified, row-aligned stereo pair "
            "on /cameras_head/stereo/{left,right}/, only while something subscribes. Needs both "
            "use_left and use_right."
        ),
    )

    use_system_timestamp = DeclareLaunchArgument(
        "use_system_timestamp",
        default_value="true",
        description=(
            "If true, shift the camera's device timestamps onto the system clock before stamping "
            "messages, so they can be compared against the rest of the system."
        ),
    )

    stereo_args = [
        DeclareLaunchArgument(
            "stereo_pitch_deg", default_value="25.0",
            description="Pitch of the rectified head stereo pair, degrees (> 0 up)."),
        DeclareLaunchArgument(
            "stereo_focal_px", default_value="400.0",
            description="Rectified head stereo focal length, pixels. 0 keeps OpenCV's."),
        DeclareLaunchArgument(
            "stereo_output_size", default_value="[1600,1200]",
            description="[width,height] of the rectified head stereo images. [0,0] = raw size."),
        DeclareLaunchArgument(
            "stereo_max_rate", default_value="0.0",
            description="Max rate of the head stereo pair, Hz. 0 = every frame."),
    ]

    launch_args = stereo_args + [
        use_rviz_arg,
        use_left,
        use_right,
        use_center,
        use_compressed,
        use_system_timestamp,
        fps,
        use_stereo,
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    is_use_left = is_launch_config_true(context, "use_left")
    is_use_right = is_launch_config_true(context, "use_right")
    is_use_center = is_launch_config_true(context, "use_center")
    is_use_compressed = is_launch_config_true(context, "use_compressed")
    is_use_system_timestamp = is_launch_config_true(context, "use_system_timestamp")
    fps = int(LaunchConfiguration("fps").perform(context))
    is_use_stereo = is_launch_config_true(context, "use_stereo")
    stereo_pitch_deg = float(LaunchConfiguration("stereo_pitch_deg").perform(context))
    stereo_focal_px = float(LaunchConfiguration("stereo_focal_px").perform(context))
    stereo_max_rate = float(LaunchConfiguration("stereo_max_rate").perform(context))
    stereo_output_size = [int(v) for v in
                          LaunchConfiguration("stereo_output_size").perform(context).strip("[] ").split(",")]

    camera_node = Node(
        package="stretch_core",
        executable="luxonis_camera_node",
        name="luxonis_camera_node",
        parameters=[
            {
                "use_left": is_use_left,
                "use_right": is_use_right,
                "use_center": is_use_center,
                "use_compressed": is_use_compressed,
                "use_system_timestamp": is_use_system_timestamp,
                "fps": fps,
                "use_stereo": is_use_stereo,
                "stereo_pitch_deg": stereo_pitch_deg,
                "stereo_focal_px": stereo_focal_px,
                "stereo_output_size": stereo_output_size,
                "stereo_max_rate": stereo_max_rate,
                "is_gripper": False,
            }
        ],
        output="both",
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("stretch_core"), "rviz", "cameras_head.rviz"
    )

    return [camera_node] + get_rviz_node(rviz_config_path)
