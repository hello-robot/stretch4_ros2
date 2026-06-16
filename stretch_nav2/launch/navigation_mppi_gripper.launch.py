from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from hello_helpers.multi_yaml import MultiYaml
from launch_ros.actions import Node


def generate_launch_description():
    stretch_core_path = FindPackageShare('stretch_core')
    stretch_navigation_path = FindPackageShare('stretch_nav2')

    stretch_driver_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'stretch_driver.launch.py']),
        launch_arguments={'broadcast_odom_tf': 'True', 'mode': 'navigation'}.items())

    # gripper_camera_launch = IncludeLaunchDescription(
    #     PathJoinSubstitution([stretch_core_path, 'launch', 'gripper_camera.launch.py']),
    #     launch_arguments={'use_rviz': 'false'}.items(),
    # )

    hlidar_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'dual_hesai.launch.py']),
        launch_arguments={
            'filter_type': 'sor',
            'tool_preset': LaunchConfiguration('tool_preset'),
        }.items(),
    )

    footprint_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'robot_footprint.launch.py']),
        launch_arguments={'tool_preset': LaunchConfiguration('tool_preset')}.items(),
    )

    gripper_interceptor_node = Node(
        package='stretch_nav2',
        executable='gripper_interceptor.py',
        name='gripper_interceptor_node',
        output='screen',
    )

    navigation_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_navigation_path, 'launch', 'include', 'nav_core.launch.py']),
        launch_arguments={
            'params_file': MultiYaml([
                PathJoinSubstitution([stretch_navigation_path, 'config', 'original_nav2_params.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_core.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_mppi.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'mppi_params.yaml']),
            ]),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_composition': LaunchConfiguration('use_composition'),
            'rviz_config': LaunchConfiguration('rviz_config'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'tool_preset',
            default_value='sg4',
            description='Mounted tool preset for lidar self-filter: sg4, pg4, tablet, or nil',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=PathJoinSubstitution(
                [stretch_navigation_path, 'rviz', 'gripper_nav2.rviz']),
            description='RViz config file for Nav2 (passed through to nav_core.launch.py)',
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
        gripper_interceptor_node,
        # gripper_camera_launch,
        hlidar_launch,
        footprint_launch,
        navigation_launch,
    ])
