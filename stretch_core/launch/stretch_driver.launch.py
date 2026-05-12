import os
import sys

import launch_ros
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from stretch4_urdf import get_urdf_from_robot_params


def compile_robot_description(context, *args, **kwargs):
    """This OpaqueFunction allows the launch file to compile
    whatever XACRO or URDF the user provides to `description_filepath`
    for use in the robot state publisher. It uses the robot's default
    URDF if the arg is unspecified.
    """

    robot_description_content = get_urdf_from_robot_params()

    prefix = LaunchConfiguration('namespace').perform(context)
    ns = prefix if prefix != 'UNSET' else ''
    frame_prefix = prefix + '/' if prefix != 'UNSET' else ''
    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 namespace=ns,
                                 output='both',
                                 parameters=[{'robot_description': robot_description_content},
                                             {'publish_frequency': 100.0},
                                             {'frame_prefix': frame_prefix}],
                                 arguments=['--ros-args', '--log-level', 'error'],)
    return [robot_state_publisher]

def generate_launch_description():
    # Check is robot
    if 'HELLO_FLEET_ID' not in os.environ:
        print("\nERROR: Must be run on a robot.")
        sys.exit(1)

    ld = LaunchDescription()

    # namespace
    declare_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='UNSET',
        description='The prefix to use as a namespace, defaults to no namespace'
    )
    ld.add_action(declare_namespace_arg)

    # Wheel odom TF
    declare_broadcast_odom_tf_arg = DeclareLaunchArgument(
        'broadcast_odom_tf',
        default_value='False', choices=['True', 'False'],
        description='Whether to broadcast the odom TF (based on wheel odometry)'
    )
    ld.add_action(declare_broadcast_odom_tf_arg)

    compile_description_fn = OpaqueFunction(function=compile_robot_description)
    ld.add_action(compile_description_fn)

    # Driver mode
    declare_mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='navigation', choices=['position', 'velocity', 'navigation', 'teleop'],
        description='The mode in which the ROS driver commands the robot'
    )
    ld.add_action(declare_mode_arg)

    # Log level for StretchDriver node
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for the StretchDriver node'
    )
    ld.add_action(log_level_arg)

    # Convert the robot_id LaunchConfiguration into a python string so we can check it
    def add_stretch_driver(context, *args, **kwargs):
        prefix = LaunchConfiguration('namespace').perform(context)
        ns = prefix if prefix != 'UNSET' else ''
        stretch_driver = Node(package='stretch_core',
                              executable='stretch_driver',
                              name='stretch_driver',
                              namespace=ns,
                              emulate_tty=True,
                              output='screen',
                              parameters=[{'broadcast_odom_tf': LaunchConfiguration('broadcast_odom_tf')},
                                          {'mode': LaunchConfiguration('mode')}],
                              ros_arguments=['--log-level', ['stretch_driver:=', LaunchConfiguration('log_level')]],)
        return [stretch_driver]

    add_stretch_driver_fn = OpaqueFunction(function=add_stretch_driver)
    ld.add_action(add_stretch_driver_fn)

    return ld
