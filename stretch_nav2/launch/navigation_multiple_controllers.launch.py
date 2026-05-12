from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from hello_helpers.multi_yaml import MultiYaml
from nav2_common.launch import ReplaceString


def generate_launch_description():
    stretch_core_path = FindPackageShare('stretch_core')
    stretch_navigation_path = FindPackageShare('stretch_nav2')

    stretch_driver_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'stretch_driver.launch.py']),
        launch_arguments={'broadcast_odom_tf': 'True', 'mode': 'navigation'}.items())

    rslidar_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'airy_dual_rslidar.launch.py']),
    )

    dwb_params_file = ReplaceString(
        source_file=PathJoinSubstitution([stretch_navigation_path, 'config', 'dwb_params.yaml']),
        replacements={
            'FollowPath': 'DWBController',
        },
    )
    mppi_params_file = ReplaceString(
        source_file=PathJoinSubstitution([stretch_navigation_path, 'config', 'mppi_params.yaml']),
        replacements={
            'FollowPath': 'MPPIController',
        },
    )

    navigation_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_navigation_path, 'launch', 'nav_core.launch.py']),
        launch_arguments={
            'params_file': MultiYaml([
                PathJoinSubstitution([stretch_navigation_path, 'config', 'original_nav2_params.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_core.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_switch_controller.yaml']),
                dwb_params_file,
                mppi_params_file,
            ]),
        }.items(),
    )

    switch_controller_config = Node(
        package='stretch_nav2',
        executable='switch_controller_config.py',
        name='switch_controller_config',
        output='screen'
    )

    return LaunchDescription([
        stretch_driver_launch,
        rslidar_launch,
        navigation_launch,
        switch_controller_config,
    ])
