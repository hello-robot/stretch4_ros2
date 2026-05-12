import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from stretch4_body.subsystem.cameras.adapters.luxonis_camera_adapter import (
    get_device_port_by_product_name,
)
from stretch_core.vision.vision_topics import (
    VisionTopics
)


def is_launch_config_true(context, name):
    return LaunchConfiguration(name).perform(context) == "true"


def generate_launch_description():
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="If true, launch Rviz2 automatically.",
    )

    launch_args = [
        use_rviz_arg
    ]

    return LaunchDescription(launch_args + [OpaqueFunction(function=launch_setup)])


def launch_setup(context, *args, **kwargs):
    params = {
        # Parameters should match keys in ~/ament_ws/src/depthai-ros/depthai_ros_driver/include/depthai_ros_driver/param_handlers/base_param_handler.hpp
        "pipeline_gen": {
            "i_pipeline_type": "Depth",
            "i_nn_type": "none",
        },
        "driver":{
            "i_usb_port_id": get_device_port_by_product_name("OAK-D-SR")
        },
        "right": {
            "i_publish_topic": True
        },
        "stereo": {
            "i_extended_disp":True,
            "i_enable_left_rgbd": True,
            "i_board_socket_id": 1,
            "i_set_input_size": True,
            # "i_input_width": 1280,
            # "i_input_height": 720
        },
        "left": {
            # Options for full FOV: (640x400), (800x500), (960x600), (1024x640), (1280x800)
            "i_width": 640,
            "i_height": 400,
            # "i_fps": 30.0,
            "i_publish_topic": False
        },
        "right": {
            # Options for full FOV: (640x400), (800x500), (960x600), (1024x640), (1280x800)
            "i_width": 640,
            "i_height": 400,
            # "i_fps": 30.0,
            "i_publish_topic": True
        },
    }

    camera_node = ComposableNodeContainer(
        name=f"luxonis_gripper_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="depthai_ros_driver",
                plugin="depthai_ros_driver::Driver",
                name=VisionTopics.GRIPPER_CAMERA_NAMESPACE.value[1:],
                namespace="",
                parameters=[
                    params,
                ],
                remappings=[
                ],
            )
        ],
        output="both",
    )

    rviz_config_path = os.path.join(
        get_package_share_directory("stretch_core"), "rviz", "cameras.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    return [camera_node, rviz_node]
