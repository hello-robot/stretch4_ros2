import os
from pathlib import Path
import time

import rclpy
from rclpy.node import Node as ROSNode

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
        description="If true, stream the left head camera.",
    )
    use_right = DeclareLaunchArgument(
        "use_right",
        default_value="true",
        description="If true, stream the right head camera.",
    )
    use_center = DeclareLaunchArgument(
        "use_center",
        default_value="false",
        description="If true, stream the center head camera.",
    )
    use_gripper = DeclareLaunchArgument(
        "use_gripper",
        default_value="false",
        description="If true, stream the gripper camera.",
    )
    publish_rotated = DeclareLaunchArgument(
        "publish_rotated",
        default_value="false",
        description="If true, publish rotated color image frames on rotated_image topic.",
    )

    launch_args = [
        use_rviz_arg,
        use_left,
        use_right,
        use_center,
        use_gripper,
        publish_rotated,
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])


def check_active_topics():
    initialized_here = False
    if not rclpy.ok():
        rclpy.init()
        initialized_here = True

    node = ROSNode('rgbd_launch_topic_detector')

    camera_topics = [
        VisionTopics.image_raw("left"),
        VisionTopics.image_raw("right"),
        VisionTopics.image_raw("center"),
    ]
    gripper_topics = [
        VisionTopics.gripper_image_raw("right"),
        VisionTopics.gripper_image_raw("stereo")
    ]
    lidar_topics = [
        VisionTopics.lidar_points_left(),
        VisionTopics.lidar_points_right()
    ]

    has_active_head_camera = False
    has_active_gripper_camera = False
    has_active_lidar = False

    start_time = time.time()
    timeout = 1.0

    while (time.time() - start_time) < timeout:
        try:
            topic_names_and_types = node.get_topic_names_and_types()
            active_topics = {name for name, _ in topic_names_and_types}

            if not has_active_head_camera:
                for topic in camera_topics:
                    # node.get_logger().info(f"{topic} {node.count_publishers(topic)=}")
                    if topic in active_topics and node.count_publishers(topic) > 0:
                        has_active_head_camera = True
                        break

            if not has_active_gripper_camera:
                for topic in gripper_topics:
                    if topic in active_topics and node.count_publishers(topic) > 0:
                        has_active_gripper_camera = True
                        break

            if not has_active_lidar:
                for topic in lidar_topics:
                    if topic in active_topics and node.count_publishers(topic) > 0:
                        has_active_lidar = True
                        break

            if has_active_head_camera and has_active_gripper_camera and has_active_lidar:
                break
        except Exception as e:
            node.get_logger().warn(f"Error checking topics in launch: {e}")

        time.sleep(0.1)

    node.destroy_node()
    if initialized_here:
        rclpy.shutdown()

    return has_active_head_camera, has_active_gripper_camera, has_active_lidar


def launch_setup(context, *args, **kwargs):
    is_use_left = is_launch_config_true(context, "use_left")
    is_use_right = is_launch_config_true(context, "use_right")
    is_use_center = is_launch_config_true(context, "use_center")
    is_use_gripper = is_launch_config_true(context, "use_gripper")
    is_publish_rotated = is_launch_config_true(context, "publish_rotated")

    has_active_head, has_active_gripper, has_active_lidar = check_active_topics()

    extra_launches = []

    if not has_active_head and (is_use_left or is_use_right or is_use_center):
        print("Head cameras not publishing. Launching luxonis.launch.py...")
        extra_launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('stretch_core'), 'launch', 'luxonis.launch.py')
                ),
                launch_arguments={
                    'use_left': 'true' if is_use_left else 'false',
                    'use_right': 'true' if is_use_right else 'false',
                    'use_center': 'true' if is_use_center else 'false',
                    'publish_rotated': 'true' if is_publish_rotated else 'false',
                }.items()
            )
        )

    if not has_active_gripper and is_use_gripper:
        print("Gripper camera not publishing. Launching gripper_camera.launch.py...")
        extra_launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('stretch_core'), 'launch', 'gripper_camera.launch.py')
                )
            )
        )

    if not has_active_lidar:
        print("Dual Lidar not publishing. Launching dual_hesai.launch.py...")
        extra_launches.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory('stretch_core'), 'launch', 'dual_hesai.launch.py')
                )
            )
        )

    rgbd_node = Node(
        package="stretch_core",
        executable="rgbd_node",
        name="rgbd_camera_node",
        parameters=[
            {
                "use_left": is_use_left,
                "use_right": is_use_right,
                "use_center": is_use_center,
                "use_gripper": is_use_gripper,
                "publish_rotated": is_publish_rotated,
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
        get_package_share_directory("stretch_core"), "rviz", "rgbd.rviz"
    )

    return [rgbd_node] + camera_info_nodes + get_rviz_node(rviz_config_path) + extra_launches
