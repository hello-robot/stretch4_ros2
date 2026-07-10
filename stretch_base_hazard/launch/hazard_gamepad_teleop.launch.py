from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _add_node(context, *args, **kwargs):
    direct = LaunchConfiguration('direct').perform(context).lower() == 'true'
    no_singleton_lock = (
        LaunchConfiguration('no_singleton_lock').perform(context).lower() == 'true'
    )
    node_args = []
    if direct:
        node_args.append('--direct')
    if no_singleton_lock:
        node_args.append('--no-singleton-lock')

    return [
        Node(
            package='stretch_base_hazard',
            executable='hazard_gamepad_teleop',
            name='hazard_gamepad_teleop',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('config_file')],
            arguments=node_args,
        ),
    ]


def generate_launch_description():
    pkg = FindPackageShare('stretch_base_hazard')
    config = PathJoinSubstitution([pkg, 'config', 'hazard_teleop_filter.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config),
        DeclareLaunchArgument(
            'direct',
            default_value='false',
            choices=['true', 'false'],
            description='Use direct stretch4_body API instead of RobotClient/server',
        ),
        DeclareLaunchArgument(
            'no_singleton_lock',
            default_value='false',
            choices=['true', 'false'],
            description='Do not take the stretch_gamepad_teleop singleton lock',
        ),
        OpaqueFunction(function=_add_node),
    ])
