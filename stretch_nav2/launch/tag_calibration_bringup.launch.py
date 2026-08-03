from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from hello_helpers.multi_yaml import MultiYaml


def generate_launch_description():
    stretch_core_path = FindPackageShare('stretch_core')
    stretch_navigation_path = FindPackageShare('stretch_nav2')
    stretch_tag_perception_path = FindPackageShare('stretch_tag_perception')

    # 1. Stretch Driver Launch
    stretch_driver_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'stretch_driver.launch.py']),
        launch_arguments={'broadcast_odom_tf': 'True', 'mode': 'navigation'}.items()
    )

    # 2. Dual Hesai Lidar Launch (Crucial for AMCL scan matching; suppress auxiliary RViz)
    hlidar_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'dual_hesai.launch.py']),
        launch_arguments={
            'filter_type': 'sor_ransac',
            'tool_preset': LaunchConfiguration('tool_preset'),
            'use_rviz': 'false',
        }.items(),
    )

    # 3. Footprint Launch
    footprint_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'robot_footprint.launch.py']),
        launch_arguments={'tool_preset': LaunchConfiguration('tool_preset')}.items(),
    )

    # 4. Cameras Launch (Must explicitly enable use_center; suppress auxiliary RViz)
    luxonis_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'luxonis.launch.py']),
        launch_arguments={
            'use_center': 'true',
            'use_left': 'true',
            'use_right': 'true',
            'use_rviz': 'false',
        }.items()
    )

    # 5. ArUco Tag Perception Launch (Run for all cameras; suppress auxiliary RViz)
    aruco_perception_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_tag_perception_path, 'launch', 'stretch_aruco.launch.py']),
        launch_arguments={
            'cameras': 'all',
            'publish_markers': 'true',
            'use_rviz': 'false',
        }.items()
    )

    # 6. Navigation Core Launch (Brings up map_server, amcl, planner_server, controller_server, and single nav RViz2)
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
            'use_rviz': 'true',
            'use_composition': LaunchConfiguration('use_composition'),
        }.items(),
    )

    # 7. Tag Localization Service Node
    tag_localization_node = Node(
        package='stretch_nav2',
        executable='aruco_tag_localization.py',
        output='screen',
        parameters=[{
            'map_name': 'map'
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([stretch_navigation_path, 'maps', 'map_ds_rp.yaml']),
            description='Full path to the map YAML file to load.'
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
            'use_composition',
            default_value='True',
            choices=['True', 'False'],
            description='Run Nav2 as composed components in a container (False = separate nodes for debugging)',
        ),
        stretch_driver_launch,
        hlidar_launch,
        footprint_launch,
        luxonis_launch,
        aruco_perception_launch,
        navigation_launch,
        tag_localization_node,
    ])
