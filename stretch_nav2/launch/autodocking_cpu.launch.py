from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    stretch_navigation_path = FindPackageShare('stretch_nav2')

    log_level_arg = DeclareLaunchArgument(
        'autodocking_log_level',
        default_value='info',
        description='Log level for the autodocking nodes'
    )

    navigation_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_navigation_path, 'launch', 'navigation_mppi.launch.py']),
        launch_arguments={'rviz_config': LaunchConfiguration('rviz_config')}.items(),
    )

    docking_server = Node(
        package='stretch_nav2',
        executable='docking_server.py',
        output='screen',
        ros_arguments=['--log-level', ['docking_server:=', LaunchConfiguration('autodocking_log_level')]],
    )

    blind_undock_arg = DeclareLaunchArgument(
        'blind_undock',
        default_value='False',
        description=(
            'Undock without any lidar clearance checking. Only safe where the space'
            'beside the dock is known to be clear.'
        ),
    )

    undocking_server = Node(
        package='stretch_nav2',
        executable='undocking_server.py',
        output='screen',
        parameters=[{
            'blind_undock': ParameterValue(
                LaunchConfiguration('blind_undock'), value_type=bool),
        }],
        ros_arguments=['--log-level', ['undocking_server:=', LaunchConfiguration('autodocking_log_level')]],
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            stretch_navigation_path, 'rviz', 'autodocking.rviz'
        ]),
        description='Full path to the RViz config to load when use_rviz is true',
    )

    return LaunchDescription([
        log_level_arg,
        blind_undock_arg,
        rviz_config_arg,
        navigation_launch,
        docking_server,
        undocking_server,
    ])
