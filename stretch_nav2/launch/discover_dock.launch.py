from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from hello_helpers.multi_yaml import MultiYaml


def scoped(action):
    """Keep an include's launch_arguments from leaking into the includes that follow it.

    IncludeLaunchDescription sets its launch_arguments in the *current* scope rather than the
    included one, so `use_rviz: 'false'` on one include silently becomes the value every later
    sibling reads. Wrapping each include in a (scoped) GroupAction contains its arguments.
    """
    return GroupAction([action])


def generate_launch_description():
    stretch_core_path = FindPackageShare('stretch_core')
    stretch_navigation_path = FindPackageShare('stretch_nav2')
    stretch_tag_perception_path = FindPackageShare('stretch_tag_perception')

    stretch_driver_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'stretch_driver.launch.py']),
        launch_arguments={'broadcast_odom_tf': 'True', 'mode': 'navigation'}.items()
    )

    hlidar_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'dual_hesai.launch.py']),
        launch_arguments={
            'filter_type': 'sor_ransac',
            'tool_preset': LaunchConfiguration('tool_preset'),
            'use_rviz': 'false',
        }.items(),
    )

    footprint_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'robot_footprint.launch.py']),
        launch_arguments={'tool_preset': LaunchConfiguration('tool_preset')}.items(),
    )

    luxonis_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_core_path, 'launch', 'luxonis.launch.py']),
        launch_arguments={
            'use_center': 'true',
            'use_left': 'false',
            'use_right': 'false',
            'use_rviz': 'false',
        }.items()
    )

    aruco_perception_launch = IncludeLaunchDescription(
        PathJoinSubstitution([stretch_tag_perception_path, 'launch', 'stretch_aruco.launch.py']),
        launch_arguments={
            'cameras': 'center',
            'publish_markers': 'false',
            'show_debug_images': LaunchConfiguration('show_debug_images'),
            'use_rviz': 'false',
        }.items()
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
            ]),
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': PathJoinSubstitution([stretch_navigation_path, 'rviz', 'discover_dock.rviz']),
            'use_composition': LaunchConfiguration('use_composition'),
        }.items(),
    )

    discover_dock_node = Node(
        package='stretch_nav2',
        executable='discover_dock.py',
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
            'show_debug_images',
            default_value='true',
            choices=['true', 'false'],
            description=('Publish /aruco/debug_image: the center camera with detected ArUco markers drawn'),
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Start RViz with the discover_dock config; requires a graphical display',
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            choices=['True', 'False'],
            description='Run Nav2 as composed components in a container (False = separate nodes for debugging)',
        ),
        scoped(stretch_driver_launch),
        scoped(hlidar_launch),
        scoped(footprint_launch),
        scoped(luxonis_launch),
        scoped(aruco_perception_launch),
        scoped(navigation_launch),
        discover_dock_node,
    ])
