from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('publish_rate_hz', default_value='30.0'),
        DeclareLaunchArgument('stale_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('use_tare', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('publish_debug', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('points_topic', default_value='/line_sensor/points'),
        DeclareLaunchArgument('obstacle_topic', default_value='/line_sensor/obstacle_points'),
        DeclareLaunchArgument('small_drop_topic', default_value='/line_sensor/small_drop_points'),
        Node(
            package='stretch_core',
            executable='line_sensor_publisher',
            name='line_sensor_publisher',
            output='screen',
            parameters=[{
                'base_frame': LaunchConfiguration('base_frame'),
                'publish_rate_hz': ParameterValue(LaunchConfiguration('publish_rate_hz'), value_type=float),
                'stale_timeout_s': ParameterValue(LaunchConfiguration('stale_timeout_s'), value_type=float),
                'use_tare': ParameterValue(LaunchConfiguration('use_tare'), value_type=bool),
                'publish_debug': ParameterValue(LaunchConfiguration('publish_debug'), value_type=bool),
                'points_topic': LaunchConfiguration('points_topic'),
                'obstacle_topic': LaunchConfiguration('obstacle_topic'),
                'small_drop_topic': LaunchConfiguration('small_drop_topic'),
            }],
        ),
    ])
