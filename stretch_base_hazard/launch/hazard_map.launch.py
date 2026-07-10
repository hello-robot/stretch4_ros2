from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare('stretch_base_hazard')
    config = PathJoinSubstitution([pkg, 'config', 'hazard_map.yaml'])

    return LaunchDescription([
        DeclareLaunchArgument('config_file', default_value=config),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_topic', default_value='wheel_odom'),
        DeclareLaunchArgument('lidar_topic', default_value='/lidar_pointcloud'),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='',
            description='Override source frame. Empty uses the PointCloud2 header frame.',
        ),
        DeclareLaunchArgument('line_obstacle_topic', default_value='/line_sensor/obstacle_points'),
        DeclareLaunchArgument(
            'line_small_drop_topic',
            default_value='/line_sensor/small_drop_points',
        ),
        DeclareLaunchArgument(
            'line_frame',
            default_value='',
            description='Override line point source frame. Empty uses each PointCloud2 header.',
        ),
        DeclareLaunchArgument('line_topic_timeout_s', default_value='0.5'),
        DeclareLaunchArgument('detector_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('publish_debug', default_value='false', choices=['true', 'false']),
        Node(
            package='stretch_base_hazard',
            executable='hazard_map_node',
            name='hazard_map_node',
            output='screen',
            parameters=[
                LaunchConfiguration('config_file'),
                {
                    'base_frame': LaunchConfiguration('base_frame'),
                    'odom_topic': LaunchConfiguration('odom_topic'),
                    'lidar_topic': LaunchConfiguration('lidar_topic'),
                    'lidar_frame': LaunchConfiguration('lidar_frame'),
                    'line_obstacle_topic': LaunchConfiguration('line_obstacle_topic'),
                    'line_small_drop_topic': LaunchConfiguration('line_small_drop_topic'),
                    'line_frame': LaunchConfiguration('line_frame'),
                    'line_topic_timeout_s': ParameterValue(
                        LaunchConfiguration('line_topic_timeout_s'),
                        value_type=float,
                    ),
                    'detector_rate_hz': ParameterValue(
                        LaunchConfiguration('detector_rate_hz'),
                        value_type=float,
                    ),
                    'publish_debug': ParameterValue(
                        LaunchConfiguration('publish_debug'),
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
