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

from hello_helpers.launch_utils import get_rviz_node

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

    model_name, batch_name, tool_name = Stretch4MujocoSimulator.get_default_model_batch_tool_names()
    robot_description_content = get_urdf(model_name=model_name, batch_name=batch_name, tool_name=tool_name)

    rate = float(context.perform_substitution(stretch_driver_params["joint_state_rate"]))
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="log",
        parameters=[
            {"source_list": ["/joint_states"]},
            {"rate": rate},
            {"robot_description": robot_description_content},
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {"robot_description": robot_description_content},
            {"publish_frequency": rate},
        ],
        arguments=["--ros-args", "--log-level", "error"],
    )

    stretch_sim = Node(
        package="stretch_simulation",
        executable="stretch_mujoco_driver",
        emulate_tty=True,
        output="screen",
        # remappings=[
        #     ("cmd_vel", "/stretch/cmd_vel"),
        #     ("joint_states", "/stretch/joint_states"),
        # ],
        parameters=[stretch_driver_params],
        # arguments=["--ros-args", "--log-level", "debug"],
        on_exit=Shutdown(),
    )

    return joint_state_publisher, robot_state_publisher, stretch_sim, *get_rviz_node( str(stretch_simulation_path / "rviz" / "stretch_sim.rviz"), rviz_params={"use_sim_time": True})


def generate_launch_description():
    launch_func = OpaqueFunction(function=launch)

    launch_args = [
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
            choices=["active", "teleop"],
            description="The mode in which the ROS driver commands the robot",
        ),
        DeclareLaunchArgument(
            "action_server_rate", default_value='30.0', description="Action server update rate",
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
