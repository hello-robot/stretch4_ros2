import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_context import LaunchContext
from launch_ros.substitutions import FindPackageShare
import rclpy


def generate_launch_description():
    logger = rclpy.logging.get_logger('navigation_launch')

    stretch_navigation_path = FindPackageShare('stretch_nav2')
    navigation_bringup_path = FindPackageShare('nav2_bringup')

    map_path_param = DeclareLaunchArgument(
        'map',
        default_value=PathJoinSubstitution([stretch_navigation_path,
                                            'maps', 'dual_ds3.yaml']),
        description='Full path to the map.yaml file to use for navigation')

    use_slam = DeclareLaunchArgument(
        'use_slam',
        default_value='False',
        choices=['True', 'False'],
        description='Whether run a SLAM')

    # Error out if the map file does not exist
    def map_file_check(context: LaunchContext):
        if LaunchConfiguration('use_slam').perform(context):
            return
        map_path = LaunchConfiguration('map').perform(context)
        if not os.path.exists(map_path):
            msg = 'Map file not found in given path: {}'.format(map_path)
            logger.error(msg)
            raise FileNotFoundError(msg)
        if not map_path.endswith('.yaml'):
            msg = 'Map file is not a yaml file: {}'.format(map_path)
            logger.error(msg)
            raise FileNotFoundError(msg)

    map_path_check_action = OpaqueFunction(function=map_file_check)

    rviz_param = DeclareLaunchArgument('use_rviz', default_value='true', choices=['true', 'false'])

    params_file = DeclareLaunchArgument(
        'params_file',
        description='Full path to the ROS2 parameters file to use for Nav2',
    )

    use_composition_param = DeclareLaunchArgument(
        'use_composition',
        default_value='True',
        choices=['True', 'False'],
        description='Run Nav2 as composed components in a container (False = separate nodes for debugging)',
    )

    navigation_bringup_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_navigation_path, 'launch', 'include', 'bringup_launch.py']),
        launch_arguments={
            'map': LaunchConfiguration('map'),
            'slam': LaunchConfiguration('use_slam'),
            'params_file': LaunchConfiguration('params_file'),
            'use_composition': LaunchConfiguration('use_composition'),
        }.items())

    rviz_launch = IncludeLaunchDescription(
        PathJoinSubstitution([navigation_bringup_path, 'launch', 'rviz_launch.py']),
        launch_arguments={
            'rviz_config': PathJoinSubstitution([stretch_navigation_path, 'rviz', 'navigation.rviz']),
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_rviz')))

    return LaunchDescription([
        map_path_param,
        use_slam,
        rviz_param,
        params_file,
        use_composition_param,
        navigation_bringup_launch,
        rviz_launch,
        map_path_check_action,
    ])
