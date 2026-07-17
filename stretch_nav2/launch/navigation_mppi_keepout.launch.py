"""Navigation with MPPI + KeepoutFilter + SpeedFilter masks."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from hello_helpers.multi_yaml import MultiYaml


def generate_launch_description():
    stretch_core_path = FindPackageShare('stretch_core')
    stretch_navigation_path = FindPackageShare('stretch_nav2')

    stretch_driver_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'stretch_driver.launch.py']),
        launch_arguments={'broadcast_odom_tf': 'True', 'mode': 'navigation'}.items())

    hlidar_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'dual_hesai.launch.py']),
        launch_arguments={
            'filter_type': 'sor_ransac',
            'tool_preset': LaunchConfiguration('tool_preset'),
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
                PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_keepout_speed.yaml']),
            ]),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_composition': LaunchConfiguration('use_composition'),
        }.items(),
    )

    filters_launch = IncludeLaunchDescription(
        PathJoinSubstitution([
            stretch_navigation_path, 'launch', 'include', 'keepout_speed_filters.launch.py'
        ]),
        launch_arguments={
            'keepout_mask': LaunchConfiguration('keepout_mask'),
            'speed_mask': LaunchConfiguration('speed_mask'),
            'use_sim_time': 'false',
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            description='Full path to the occupancy map.yaml',
        ),
        DeclareLaunchArgument(
            'keepout_mask',
            description='Full path to keepout mask yaml',
        ),
        DeclareLaunchArgument(
            'speed_mask',
            description='Full path to speed mask yaml',
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
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            choices=['True', 'False'],
        ),
        stretch_driver_launch,
        hlidar_launch,
        footprint_launch,
        navigation_launch,
        filters_launch,
    ])
