from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from hello_helpers.multi_yaml import MultiYaml


def scoped(action):
    """Keep an include's launch_arguments from leaking into the includes that follow it.

    IncludeLaunchDescription sets its launch_arguments in the *current* scope rather than the
    included one, so `use_rviz: 'false'` on one include silently becomes the value every later
    sibling reads. Wrapping each include in a (scoped) GroupAction contains its arguments.
    """
    return GroupAction([action])


def generate_launch_description():
    stretch_core_path = FindPackageShare('stretch_core')
    stretch_navigation_path = FindPackageShare('stretch_nav2')

    stretch_driver_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'stretch_driver.launch.py']),
        condition=IfCondition(LaunchConfiguration('launch_driver')),
        launch_arguments={'broadcast_odom_tf': 'True', 'mode': 'navigation'}.items())

    hlidar_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'dual_hesai.launch.py']),
        launch_arguments={
            'filter_type': 'sor_ransac',
            'tool_preset': LaunchConfiguration('tool_preset'),
            # Without this the lidar bringup inherits the top-level use_rviz and opens a second
            # RViz on lidars.rviz alongside the navigation one.
            'use_rviz': 'false',
        }.items(),
    )

    footprint_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'robot_footprint.launch.py']),
        launch_arguments={'tool_preset': LaunchConfiguration('tool_preset')}.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_navigation_path, 'launch', 'include', 'nav_core.launch.py']),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'params_file': MultiYaml([
                PathJoinSubstitution([stretch_navigation_path, 'config', 'original_nav2_params.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_core.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_mppi.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'mppi_params.yaml']),
            ]),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'use_composition': LaunchConfiguration('use_composition'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                stretch_navigation_path, 'maps', 'dual_ds3.yaml'
            ]),
            description='Full path to the map.yaml file to use for navigation',
        ),
        DeclareLaunchArgument(
            'launch_driver',
            default_value='true',
            choices=['true', 'false'],
            description='Start stretch_driver; set false when the caller already runs one',
        ),
        DeclareLaunchArgument(
            'tool_preset',
            default_value='auto',
            description='Mounted tool preset for lidar self-filter: auto, sg4, pg4, tablet, or nil',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Start RViz with navigation; requires a graphical display',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution([
                stretch_navigation_path, 'rviz', 'navigation.rviz'
            ]),
            description='Full path to the RViz config to load when use_rviz is true',
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            choices=['True', 'False'],
            description='Run Nav2 as composed components in a container (False = separate nodes for debugging)',
        ),
        scoped(stretch_driver_launch),
        scoped(hlidar_launch),
        scoped(footprint_launch),
        scoped(navigation_launch),
    ])
