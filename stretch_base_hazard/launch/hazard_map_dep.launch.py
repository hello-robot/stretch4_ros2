from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = FindPackageShare('stretch_base_hazard')
    pkg_core = FindPackageShare('stretch_core')
    config = PathJoinSubstitution([pkg, 'config', 'hazard_map.yaml'])

    stretch_driver_launch = IncludeLaunchDescription(
        PathJoinSubstitution([pkg_core, 'launch', 'stretch_driver.launch.py']),
        launch_arguments={'broadcast_odom_tf': 'True', 'mode': 'navigation'}.items()
    )

    line_sensor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_core, 'launch', 'line_sensor.launch.py'])
        ),
        launch_arguments={'publish_debug': 'true'}.items(),
    )

    lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_core, 'launch', 'dual_hesai_hazard.launch.py'])
        ),
        launch_arguments={
            'filter_type': 'sor',
            'z_min': '0.135',
            'pub_laserscan': 'True',
            'pub_pointcloud': 'True',
            'pointcloud_z_min': 'none',
            'pointcloud_range_max': '2.0',
            'frame_id': 'base_link',
            'tool_preset': LaunchConfiguration('tool_preset'),
        }.items(),
    )

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
        DeclareLaunchArgument(
            'tool_preset',
            default_value='auto',
            description='Mounted tool preset for lidar self-filter: auto, sg4, pg4, tablet, or nil',
        ),
        stretch_driver_launch,
        line_sensor_launch,
        lidar_launch,
        DeclareLaunchArgument('hazard_publish_debug', default_value='true', choices=['true', 'false']),
        Node(
            package='stretch_base_hazard',
            executable='hazard_map_node',
            name='hazard_map_node',
            output='screen',
            additional_env={
                'OPENBLAS_NUM_THREADS': '1',
                'OMP_NUM_THREADS': '1',
                'MKL_NUM_THREADS': '1',
                'NUMEXPR_NUM_THREADS': '1',
            },
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
                        LaunchConfiguration('hazard_publish_debug'),
                        value_type=bool,
                    ),
                },
            ],
        ),
    ])
