from platform import system
import os

from ament_index_python import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, Shutdown
from launch.substitutions import Command, LaunchConfiguration
import launch_ros.parameter_descriptions
from launch_ros.actions import Node
import launch_ros

from launch.conditions import IfCondition
from stretch4_urdf import get_urdf
from stretch4_mujoco import Stretch4MujocoSimulator
from stretch_mujoco_driver.sim_tools import resolve_sim_tool

from hello_helpers.launch_utils import get_rviz_node, namespaced_rviz_config

if system() == "Linux":
    # this fixes rviz launch issue
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = (
        "/usr/lib/x86_64-linux-gnu/qt5/plugins/platforms/libqxcb.so"
    )
    os.environ["GTK_PATH"] = ""

stretch_driver_params = {
    "use_cameras": LaunchConfiguration("use_cameras"),
    "use_mujoco_viewer": LaunchConfiguration("use_mujoco_viewer"),
    "use_robocasa": LaunchConfiguration("use_robocasa"),
    "scene_xml": LaunchConfiguration("scene_xml"),
    "scene_name": LaunchConfiguration("scene_name"),
    "mode": LaunchConfiguration("mode"),
    "broadcast_odom_tf": LaunchConfiguration("broadcast_odom_tf"),
    "controller_calibration_file": LaunchConfiguration("controller_calibration_file"),
    "fail_out_of_range_goal": LaunchConfiguration("fail_out_of_range_goal"),
    "fail_if_motor_initial_point_is_not_trajectory_first_point": LaunchConfiguration("fail_if_motor_initial_point_is_not_trajectory_first_point"),
    "action_server_rate": LaunchConfiguration("action_server_rate"),
    "joint_state_rate": LaunchConfiguration("joint_state_rate"),
    "timeout": LaunchConfiguration("timeout"),
    "default_goal_timeout_s": LaunchConfiguration("default_goal_timeout_s"),
    "command_namespace": LaunchConfiguration("command_namespace"),
    "puppet.joint_states_topic": LaunchConfiguration("puppet_joint_states_topic"),
    "puppet.odom_topic": LaunchConfiguration("puppet_odom_topic"),
    "puppet.follow_base": LaunchConfiguration("follow_base"),
    "tool": LaunchConfiguration("tool"),
}


def launch(context):

    stretch_simulation_path = get_package_share_path("stretch_simulation")

    use_robocasa = context.perform_substitution(stretch_driver_params["use_robocasa"]) == "true"

    if use_robocasa:
        from stretch4_mujoco.robocasa_gen import choose_layout, choose_style, get_styles, layouts

        robocasa_layout = choose_layout()
        robocasa_layout = layouts[robocasa_layout]
        print(f"{robocasa_layout=}")

        robocasa_style = choose_style()
        robocasa_style = get_styles()[robocasa_style]
        print(f"{robocasa_style=}")

        stretch_driver_params["robocasa_task"] = LaunchConfiguration("robocasa_task")
        stretch_driver_params["robocasa_layout"] = (
            robocasa_layout
            if robocasa_layout is not None
            else LaunchConfiguration("robocasa_layout")
        )
        stretch_driver_params["robocasa_style"] = (
            robocasa_style
            if robocasa_style is not None
            else LaunchConfiguration("robocasa_style")
        )

    model_name, batch_name, _ = Stretch4MujocoSimulator.get_default_model_batch_tool_names()
    tool_name = resolve_sim_tool(context.perform_substitution(LaunchConfiguration("tool")))
    robot_description_content = get_urdf(model_name=model_name, batch_name=batch_name, tool_name=tool_name)

    # As for the real driver: only passed when given, so the driver's default applies otherwise.
    driver_params = [stretch_driver_params]
    action_timeout = context.perform_substitution(LaunchConfiguration("action_timeout"))
    if action_timeout:
        driver_params.append({"action_timeout": float(action_timeout)})
    puppet_engage = context.perform_substitution(LaunchConfiguration("puppet_engage"))
    if puppet_engage:
        driver_params.append({"puppet.engage": puppet_engage})

    # Outputs (topics, TF frames) go in driver_namespace; command topics follow
    # command_namespace (see Stretch4ROSDriver).
    prefix = context.perform_substitution(LaunchConfiguration("driver_namespace"))
    ns = prefix.strip("/") if prefix != "UNSET" else ""

    # The driver publishes every movable URDF joint itself, so no joint_state_publisher.
    rate = float(context.perform_substitution(stretch_driver_params["joint_state_rate"]))
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=ns,
        output="both",
        parameters=[
            {"robot_description": robot_description_content},
            {"publish_frequency": rate},
            {"frame_prefix": f"{ns}/" if ns else ""},
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    stretch_sim = Node(
        package="stretch_simulation",
        executable="stretch_mujoco_driver",
        namespace=ns,
        emulate_tty=True,
        output="screen",
        parameters=driver_params,
        # arguments=["--ros-args", "--log-level", "debug"],
        on_exit=Shutdown(),
    )

    # Forwarding shim for the deprecated joint_vel (control_msgs/JointJog) topic.
    joint_jog_converter = Node(
        package="stretch_core",
        executable="joint_jog_converter",
        name="joint_jog_converter",
        namespace=ns,
        emulate_tty=True,
        output="screen",
        parameters=[{"command_namespace": LaunchConfiguration("command_namespace")}],
        condition=IfCondition(LaunchConfiguration("deprecated_interfaces")),
    )

    rviz_config = str(stretch_simulation_path / "rviz" / "stretch_sim.rviz")
    rviz_remappings = []
    if ns:
        rviz_config = namespaced_rviz_config(rviz_config, ns)
        rviz_remappings = [("/clock", f"/{ns}/clock")]

    return robot_state_publisher, stretch_sim, joint_jog_converter, *get_rviz_node(
        rviz_config, rviz_params={"use_sim_time": True}, remappings=rviz_remappings)


def generate_launch_description():
    launch_func = OpaqueFunction(function=launch)

    launch_args = [
        # Not named "namespace", which collides with Nav2 bringup (as in stretch_core's launch).
        DeclareLaunchArgument(
            "driver_namespace",
            default_value="UNSET",
            description="Namespace for the driver's outputs (topics, TF frame prefix), defaults to no namespace",
        ),
        DeclareLaunchArgument(
            "command_namespace",
            default_value="",
            description="Namespace of the command topics: '' = same as driver_namespace, '/' = root (e.g. for a digital twin)",
        ),
        DeclareLaunchArgument(
            "broadcast_odom_tf",
            default_value="true",
            choices=["true", "false"],
            description="Whether to broadcast the odom TF",
        ),
        DeclareLaunchArgument(
            "fail_out_of_range_goal",
            default_value="false",
            choices=["true", "false"],
            description="Whether the motion action servers fail on out-of-range commands",
        ),
        DeclareLaunchArgument(
            "fail_if_motor_initial_point_is_not_trajectory_first_point",
            default_value="false",
            choices=["true", "false"],
            description="Whether the motion action servers fail on mismatched starting points",
        ),
        DeclareLaunchArgument(
            "controller_calibration_file",
            default_value=os.path.join(get_package_share_path("stretch_core"), "config", "controller_calibration_head.yaml")
        ),
        DeclareLaunchArgument(
            "mode",
            default_value="active",
            choices=["active", "teleop", "puppet"],
            description="The mode in which the ROS driver commands the robot",
        ),
        DeclareLaunchArgument(
            "puppet_joint_states_topic", default_value="",
            description="JointState topic the robot follows in puppet mode (e.g. another robot's /joint_states)",
        ),
        DeclareLaunchArgument(
            "puppet_odom_topic", default_value="",
            description="Odometry topic whose base velocity the robot follows in puppet mode when follow_base is set",
        ),
        DeclareLaunchArgument(
            "follow_base", default_value="false", choices=["true", "false"],
            description="In puppet mode, also follow the base velocity on puppet_odom_topic",
        ),
        DeclareLaunchArgument(
            "action_server_rate", default_value='30.0', description="Action server update rate",
        ),
        DeclareLaunchArgument(
            "action_timeout", default_value="",
            description="If set, the trajectory server's timeout (sec), trajectory_server.timeout. Unset keeps the driver's default.",
        ),
        DeclareLaunchArgument(
            "puppet_engage", default_value="",
            description="How the sim enters puppet mode: 'immediate' (the sim's default: jumps to the leader's pose), 'require_match' (only once it matches), or 'slow' (not implemented yet)",
        ),
        DeclareLaunchArgument(
            "tool", default_value="sg4",
            description="Tool to simulate: 'sg4' (Stretch gripper), 'pg4' (parallel gripper), or a full stretch4_urdf tool name",
        ),
        DeclareLaunchArgument(
            "deprecated_interfaces",
            default_value="True",
            choices=["True", "False", "true", "false"],
            description="Launch joint_jog_converter to republish deprecated joint_vel (JointJog) commands as joint_velocity_cmd (JointState) commands.",
        ),
        DeclareLaunchArgument(
            "use_rviz", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "use_mujoco_viewer", default_value="true", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "use_cameras", default_value="false", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "use_robocasa", default_value="false", choices=["true", "false"]
        ),
        DeclareLaunchArgument(
            "scene_xml", default_value="", description='The absolute path to a Mujoco scene xml',
        ),
        DeclareLaunchArgument(
            "scene_name", default_value="", description='The name of a scene xml within stretch4_mujoco/models, e.g. some_rooms',
        ),
        DeclareLaunchArgument(
            "robocasa_task", default_value="PnPCounterToCab"
        ),
        DeclareLaunchArgument(
            "joint_state_rate", default_value="30.0", description="Joint state update rate",
        ),
        DeclareLaunchArgument(
            "timeout", default_value="0.5", description="Timeout (sec) after which Twist/Joy commands are considered stale",
        ),
        DeclareLaunchArgument(
            "default_goal_timeout_s", default_value='10.0', description="Timeout (sec) for goal execution",
        ),
    ]

    ld = LaunchDescription(launch_args)
    ld.add_action(launch_func)

    return ld
