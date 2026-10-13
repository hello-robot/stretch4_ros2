"""Runs the MuJoCo sim as a digital twin of a running Stretch driver.

The sim's outputs (topics, TF frames) go in their own namespace (twin_namespace), while
it listens to the same root command topics as the primary (real) driver. The
stretch4_twin_relay makes the sim follow what command topics can't share: trajectory
goals, stop/home/stow and runstop, and parameter changes. Clients keep talking to the
primary driver and get its results.

Start the primary driver (e.g. stretch_core stretch_driver.launch.py on the robot) with
no namespace, then this launch on the desktop.
"""
from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Sim launch arguments passed through unchanged.
PASSTHROUGH_ARGS = ["use_rviz", "use_mujoco_viewer", "use_cameras", "use_robocasa", "scene_xml", "scene_name"]


def launch(context):
    twin_ns = LaunchConfiguration("twin_namespace").perform(context).strip("/")

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path("stretch_simulation") / "launch" / "stretch_mujoco_driver.launch.py")),
        launch_arguments={
            "driver_namespace": twin_ns,
            "command_namespace": "/",
            # the primary's joint_jog_converter already turns joint_vel into root commands
            "deprecated_interfaces": "false",
            **{arg: LaunchConfiguration(arg) for arg in PASSTHROUGH_ARGS},
        }.items(),
    )

    relay = Node(
        package="hello_helpers",
        executable="stretch4_twin_relay",
        name="stretch4_twin_relay",
        namespace=twin_ns,
        emulate_tty=True,
        output="screen",
        parameters=[{
            "primary_node": LaunchConfiguration("primary_node"),
            "mirror_nodes": [f"/{twin_ns}/stretch_mujoco_driver"],
            "mirror_parameters": True,
        }],
    )
    return [sim, relay]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("twin_namespace", default_value="sim",
                              description="Namespace for the twin's outputs"),
        DeclareLaunchArgument("primary_node", default_value="/stretch_driver",
                              description="Fully qualified name of the driver the twin follows"),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_mujoco_viewer", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_cameras", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("use_robocasa", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("scene_xml", default_value="", description="The absolute path to a Mujoco scene xml"),
        DeclareLaunchArgument("scene_name", default_value="",
                              description="The name of a scene xml within stretch4_mujoco/models, e.g. some_rooms"),
        OpaqueFunction(function=launch),
    ])
