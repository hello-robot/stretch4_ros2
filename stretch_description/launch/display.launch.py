import launch_ros
from ament_index_python.packages import get_package_share_path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import (Command, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from stretch4_urdf import get_robot_params, get_urdf


def compile_robot_description(context, *args, **kwargs):
    """This OpaqueFunction allows the launch file to compile
    whatever XACRO or URDF the user provides to `description_filepath`
    for use in the robot state publisher. It uses the robot's default
    URDF if the arg is unspecified.
    """

    model_name = LaunchConfiguration('model').perform(context)
    batch_name = LaunchConfiguration('batch').perform(context)
    tool_name = LaunchConfiguration('tool').perform(context)

    robot_description_content = get_urdf(model_name=model_name, batch_name=batch_name, tool_name=tool_name)

    robot_state_publisher = Node(package='robot_state_publisher',
                                 executable='robot_state_publisher',
                                 output='both',
                                 parameters=[{'robot_description': robot_description_content},
                                             {'publish_frequency': 15.0}],
                                 arguments=['--ros-args', '--log-level', 'error'],)
    return [robot_state_publisher]



def generate_launch_description():
    stretch_description_path = get_package_share_path('stretch_description')

    ld = LaunchDescription()

    model_name, batch_name, tool_name = get_robot_params()

    # Description for robot_state_publisher
    declare_model_arg = DeclareLaunchArgument(
        'model',
        default_value=model_name,
        description='Model name for the robot URDF (e.g. se4)'
    )

    declare_batch_arg = DeclareLaunchArgument(
        'batch',
        default_value=batch_name,
        description='Batch name for the robot URDF (e.g. eames)'
    )

    declare_tool_arg = DeclareLaunchArgument(
        'tool',
        default_value=tool_name,
        description='Tool name for the robot URDF (e.g. eoa_wrist_dw4_tool_sg4)'
    )

    compile_description_fn = OpaqueFunction(function=compile_robot_description)
    ld.add_action(declare_model_arg)
    ld.add_action(declare_batch_arg)
    ld.add_action(declare_tool_arg) 
    ld.add_action(compile_description_fn)

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        parameters=[{'zeros.joint_lift': 0.2, 'zeros.joint_wrist_yaw': 3.4}],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', str(stretch_description_path / 'rviz' / 'stretch.rviz')]
    )

    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)
    return ld
