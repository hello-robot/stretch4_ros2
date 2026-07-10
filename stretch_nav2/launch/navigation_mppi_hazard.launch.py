from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from hello_helpers.multi_yaml import MultiYaml


def generate_launch_description():
    stretch_core_path = FindPackageShare('stretch_core')
    stretch_hazard_path = FindPackageShare('stretch_base_hazard')
    stretch_navigation_path = FindPackageShare('stretch_nav2')

    stretch_driver_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'stretch_driver.launch.py']),
        launch_arguments={'broadcast_odom_tf': 'True', 'mode': 'navigation'}.items(),
    )

    line_sensor_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'line_sensor.launch.py']),
        launch_arguments={'publish_debug': 'false'}.items(),
    )

    lidar_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'dual_hesai_hazard.launch.py']),
        launch_arguments={
            'filter_type': 'sor',
            'z_min': '0.135',
            'pub_laserscan': 'True',
            'pub_pointcloud': 'True',
            'pointcloud_z_min': 'none',
            'pointcloud_range_max': LaunchConfiguration('pointcloud_range_max'),
            'use_rviz': 'true',
            # 'frame_id': LaunchConfiguration('lidar_output_frame'),
            'tool_preset': LaunchConfiguration('tool_preset'),
            'scan_angle_increment_deg': LaunchConfiguration('scan_angle_increment_deg'),
        }.items(),
    )

    footprint_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'robot_footprint.launch.py']),
        launch_arguments={'tool_preset': LaunchConfiguration('tool_preset')}.items(),
    )

    hazard_map_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_hazard_path, 'launch', 'hazard_map.launch.py']),
        launch_arguments={
            'config_file': LaunchConfiguration('hazard_config_file'),
            'base_frame': LaunchConfiguration('base_frame'),
            'odom_topic': LaunchConfiguration('odom_topic'),
            'lidar_topic': LaunchConfiguration('hazard_lidar_topic'),
            'lidar_frame': LaunchConfiguration('lidar_frame'),
            'line_obstacle_topic': LaunchConfiguration('line_obstacle_topic'),
            'line_small_drop_topic': LaunchConfiguration('line_small_drop_topic'),
            'line_frame': LaunchConfiguration('line_frame'),
            'line_topic_timeout_s': LaunchConfiguration('line_topic_timeout_s'),
            'detector_rate_hz': LaunchConfiguration('hazard_detector_rate_hz'),
            'publish_debug': LaunchConfiguration('hazard_publish_debug'),
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_navigation_path, 'launch', 'include', 'nav_core.launch.py']),
        launch_arguments={
            'params_file': MultiYaml([
                PathJoinSubstitution([stretch_navigation_path, 'config', 'original_nav2_params.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_core.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_mppi.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'nav2_params_mppi_hazard.yaml']),
                PathJoinSubstitution([stretch_navigation_path, 'config', 'mppi_params.yaml']),
            ]),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'use_composition': LaunchConfiguration('use_composition'),
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'tool_preset',
            default_value='auto',
            description='Mounted tool preset for lidar self-filter: auto, sg4, pg4, tablet, or nil',
        ),
        DeclareLaunchArgument(
            'hazard_config_file',
            default_value=PathJoinSubstitution([
                stretch_hazard_path, 'config', 'hazard_map.yaml'
            ]),
            description='Hazard map node parameter file.',
        ),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('odom_topic', default_value='wheel_odom'),
        DeclareLaunchArgument(
            'hazard_lidar_topic',
            default_value='/lidar_pointcloud',
            description='PointCloud2 topic produced by dual_hesai_hazard and consumed by hazard_map_node.',
        ),
        DeclareLaunchArgument(
            'lidar_frame',
            default_value='',
            description='Override lidar source frame. Empty uses the PointCloud2 header frame.',
        ),
        DeclareLaunchArgument(
            'line_obstacle_topic',
            default_value='/line_sensor/obstacle_points',
            description='Line sensor obstacle PointCloud2 topic consumed by hazard_map_node.',
        ),
        DeclareLaunchArgument(
            'line_small_drop_topic',
            default_value='/line_sensor/small_drop_points',
            description='Line sensor small-drop PointCloud2 topic consumed by hazard_map_node.',
        ),
        DeclareLaunchArgument(
            'line_frame',
            default_value='',
            description='Override line point source frame. Empty uses each PointCloud2 header.',
        ),
        DeclareLaunchArgument('line_topic_timeout_s', default_value='0.5'),
        DeclareLaunchArgument(
            'hazard_detector_rate_hz',
            default_value='10.0',
            description='Hazard map detector update rate.',
        ),
        DeclareLaunchArgument(
            'hazard_publish_debug',
            default_value='false',
            choices=['true', 'false'],
            description='Publish /under_base_hazard/debug/* point clouds and source counts.',
        ),
        DeclareLaunchArgument(
            'lidar_output_frame',
            default_value='base_link',
            description='Frame used by dual_hesai_hazard for /scan_filtered and /lidar_pointcloud.',
        ),
        DeclareLaunchArgument(
            'pointcloud_range_max',
            default_value='2.0',
            description='PointCloud-only horizontal radius crop passed to dual_hesai_hazard.',
        ),
        DeclareLaunchArgument(
            'scan_angle_increment_deg',
            default_value='0.1',
            description='LaserScan angular bin width passed to dual_hesai_hazard.',
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
        line_sensor_launch,
        lidar_launch,
        footprint_launch,
        hazard_map_launch,
        navigation_launch,
    ])
