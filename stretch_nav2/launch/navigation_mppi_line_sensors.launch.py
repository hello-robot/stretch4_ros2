from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from hello_helpers.multi_yaml import MultiYaml


def generate_launch_description():
    stretch_core_path = FindPackageShare('stretch_core')
    stretch_navigation_path = FindPackageShare('stretch_nav2')
    line_sensor_path = FindPackageShare('stretch4_line_sensor')

    stretch_driver_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'stretch_driver.launch.py']),
        launch_arguments={'broadcast_odom_tf': 'True', 'mode': 'navigation'}.items(),
    )

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
            'params_file': MultiYaml([
                PathJoinSubstitution([stretch_navigation_path, 'config', 'original_nav2_params.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_core.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_mppi.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'mppi_params.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_line_sensor.yaml']),
            ]),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_composition': LaunchConfiguration('use_composition'),
        }.items(),
    )

    line_sensor_launch = IncludeLaunchDescription(
        PathJoinSubstitution([line_sensor_path, 'launch', 'line_sensor.launch.py']),
        launch_arguments={'use_rviz': 'false'}.items(),
    )

    return LaunchDescription([
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
            'use_composition',
            default_value='True',
            choices=['True', 'False'],
            description='Run Nav2 as composed components in a container (False = separate nodes for debugging)',
        ),
        stretch_driver_launch,
        hlidar_launch,
        footprint_launch,
        navigation_launch,
        line_sensor_launch,
    ])
