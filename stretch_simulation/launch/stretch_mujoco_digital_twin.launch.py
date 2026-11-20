"""Runs the MuJoCo sim as a digital twin of a running Stretch driver.

The sim's outputs (topics, TF frames) go in their own namespace (twin_namespace). How it
follows the primary (real) driver depends on `follow`:

- commands (default): the sim listens to the same root command topics as the primary,
  and the stretch4_twin_relay makes it follow what command topics can't share:
  trajectory goals, stop/home/stow and runstop, and parameter changes. Clients keep
  talking to the primary driver and get its results.
- puppet: the sim runs in puppet mode, following the primary's joint states (and,
  with follow_base, its base velocity), whatever moved the primary.

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
PASSTHROUGH_ARGS = ["use_rviz", "use_mujoco_viewer", "use_cameras", "use_robocasa", "scene_xml", "scene_name", "tool"]


def launch(context):
    twin_ns = LaunchConfiguration("twin_namespace").perform(context).strip("/")
    follow = LaunchConfiguration("follow").perform(context)
    primary_node = LaunchConfiguration("primary_node").perform(context)
    primary_ns = primary_node.rstrip("/").rsplit("/", 1)[0]

    if follow == "puppet":
        follow_args = {
            "mode": "puppet",
            "puppet_joint_states_topic": f"{primary_ns}/joint_states",
            "puppet_odom_topic": f"{primary_ns}/wheel_odom",
            "follow_base": LaunchConfiguration("follow_base"),
        }
    else:
        follow_args = {"command_namespace": "/"}

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(get_package_share_path("stretch_simulation") / "launch" / "stretch_mujoco_driver.launch.py")),
        launch_arguments={
            "driver_namespace": twin_ns,
            # the primary's joint_jog_converter already handles joint_vel commands
            "deprecated_interfaces": "false",
            **follow_args,
            **{arg: LaunchConfiguration(arg) for arg in PASSTHROUGH_ARGS},
        }.items(),
    )
    if follow == "puppet":
        return [sim]

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
        DeclareLaunchArgument("follow", default_value="commands", choices=["commands", "puppet"],
                              description="Follow the primary's commands (with the relay), or its joint states in puppet mode"),
        DeclareLaunchArgument("follow_base", default_value="true", choices=["true", "false"],
                              description="With follow:=puppet, also follow the primary's base velocity"),
        DeclareLaunchArgument("use_rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_mujoco_viewer", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("use_cameras", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("use_robocasa", default_value="false", choices=["true", "false"]),
        DeclareLaunchArgument("scene_xml", default_value="", description="The absolute path to a Mujoco scene xml"),
        DeclareLaunchArgument("scene_name", default_value="",
                              description="The name of a scene xml within stretch4_mujoco/models, e.g. some_rooms"),
        DeclareLaunchArgument("tool", default_value="sg4",
                              description="Tool to simulate, matching the primary's: 'sg4', 'pg4', or a full stretch4_urdf tool name"),
        OpaqueFunction(function=launch),
    ])
