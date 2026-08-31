from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import os
import sys
import yaml
import tempfile
from pathlib import Path

from hello_helpers.launch_utils import get_rviz_node

sys.path.insert(0, os.path.dirname(__file__))
from self_filter_config import (
    dual_lidar_fused_parameters,
    dual_lidar_self_filter_parameters,
    validate_tool_preset,
)


def launch_setup(context, *args, **kwargs):
    stretch_core = get_package_share_directory('stretch_core')
    tool_preset = LaunchConfiguration('tool_preset').perform(context)
    validate_tool_preset(tool_preset)
    self_filter_params = dual_lidar_self_filter_parameters(stretch_core, tool_preset)
    fused_params = dual_lidar_fused_parameters(stretch_core, tool_preset)
    use_fused = LaunchConfiguration('use_fused_lidar_pipeline').perform(context).lower() == 'true'

    template_file = Path(stretch_core) / 'config' / 'hesai_dual_lidar.yaml'
    with open(template_file, "r") as f:
        cfg = yaml.safe_load(f)
    fleet_dir = os.environ.get("HELLO_FLEET_PATH", "")
    fleet_id = os.environ.get("HELLO_FLEET_ID", "")
    cfg["lidar"][0]["driver"]["lidar_udp_type"]["correction_file_path"] = (
        f"{fleet_dir}/{fleet_id}/calibration_hesais/left_lidar_calibration.dat"
    )
    cfg["lidar"][1]["driver"]["lidar_udp_type"]["correction_file_path"] = (
        f"{fleet_dir}/{fleet_id}/calibration_hesais/right_lidar_calibration.dat"
    )
    with tempfile.NamedTemporaryFile(mode="w", delete=False, suffix=".yaml", dir="/tmp") as tmp_file:
        yaml.dump(cfg, tmp_file, sort_keys=False)
        temp_yaml_path = tmp_file.name

    pub_pointcloud = LaunchConfiguration('pub_pointcloud').perform(context).lower() == 'true'

    hesai_node = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        output='screen',
        parameters=[{'config_path': temp_yaml_path}]
    )

    filter_type = LaunchConfiguration('filter_type')
    scan_angle_increment_deg = LaunchConfiguration('scan_angle_increment_deg')
    launch_filter_node = LaunchConfiguration('launch_filter_node')

    dual_lidar_filter_node = Node(
        package='stretch_core',
        executable='dual_lidar_laserscan',
        name='pointcloud_to_laserscan',
        output='screen',
        parameters=[
            *self_filter_params,
            {
                'filter_type': filter_type,
                'scan_angle_increment_deg': scan_angle_increment_deg,
                'lidar1_frame': 'lidar_right_link',
                'lidar2_frame': 'lidar_left_link',
                'pub_pointcloud': pub_pointcloud,
            },
        ],
        condition=IfCondition(launch_filter_node),
    )

    # One node in place of dual_lidar_laserscan + dual_lidar_pointcloud_merger. 
    fused_pipeline_node = Node(
        package='stretch_core',
        executable='dual_lidar_fused_pipeline',
        name='dual_lidar_fused_pipeline',
        output='screen',
        parameters=[
            *fused_params,
            {
                'scan_angle_increment_deg': scan_angle_increment_deg,
                'lidar1_frame': 'lidar_right_link',
                'lidar2_frame': 'lidar_left_link',
                'publish_cloud': True,
                'publish_scan': True,
                'enable_self_robot_filter': ParameterValue(
                    LaunchConfiguration('enable_self_robot_filter'), value_type=bool),
                'enable_floor_ransac_filter': ParameterValue(
                    LaunchConfiguration('enable_floor_ransac_filter'), value_type=bool),
                'enable_sor_filter': ParameterValue(
                    LaunchConfiguration('enable_sor_filter'), value_type=bool),
                'scan_range_max': ParameterValue(
                    LaunchConfiguration('scan_range_max'), value_type=float),
                'log_stats_period_sec': ParameterValue(
                    LaunchConfiguration('log_stats_period_sec'), value_type=float),
                'pub_self_filter_markers': ParameterValue(
                    LaunchConfiguration('pub_self_filter_markers'), value_type=bool),
            },
        ],
    )

    rviz_config_path = os.path.join(stretch_core, 'rviz', 'lidars.rviz')
    lidar_processing_node = fused_pipeline_node if use_fused else dual_lidar_filter_node
    return [
        hesai_node,
        lidar_processing_node,
        *get_rviz_node(rviz_config_path),
    ]


def generate_launch_description():
    filter_type_arg = DeclareLaunchArgument(
        'filter_type',
        default_value='sor_ransac',
        description='Pipeline preset: region | sor | sor_ransac | self | none | custom',
    )

    filter_type = LaunchConfiguration('filter_type')

    print_filter_cmd = LogInfo(
        msg=['==== USING FILTER TYPE: ', filter_type]
    )

    error_log = LogInfo(
        condition=UnlessCondition(PythonExpression([
            "'", filter_type, "' in ['region', 'sor', 'sor_ransac', 'self', 'none', 'custom']"
        ])),
        msg="Invalid filter_type! Must be region, sor, sor_ransac, self, none, or custom.",
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="false",
        description="If true, launch Rviz2 automatically.",
    )

    launch_filter_node_arg = DeclareLaunchArgument(
        'launch_filter_node',
        default_value='true',
        description='If true, launch the dual_lidar_laserscan filter node.',
    )

    tool_preset_arg = DeclareLaunchArgument(
        'tool_preset',
        default_value='auto',
        description='Self-filter attachment preset: auto, sg4, pg4, tablet, or nil.',
    )

    scan_angle_increment_arg = DeclareLaunchArgument(
        'scan_angle_increment_deg',
        default_value='0.1',
        description='LaserScan angular bin width in degrees. Try 0.1 or 0.2 for Nav2.',
    )

    pub_pointcloud_arg = DeclareLaunchArgument('pub_pointcloud', default_value='false', description='Publish a pointcloud from the filter node.')

    fused_stage_args = [
        DeclareLaunchArgument(
            'enable_self_robot_filter', default_value='true', choices=['true', 'false'],
            description='Fused pipeline: cut the robot body out of both outputs.'),
        DeclareLaunchArgument(
            'enable_floor_ransac_filter', default_value='true', choices=['true', 'false'],
            description=(
                'Fused pipeline: remove the floor FROM THE SCAN with a fitted plane. '
                'false falls back to the plain z_min cut.'),
        ),
        DeclareLaunchArgument(
            'enable_sor_filter', default_value='false', choices=['true', 'false'],
            description=(
                'Fused pipeline: StatisticalOutlierRemoval on the scan band.'),
        ),
        DeclareLaunchArgument(
            'scan_range_max', default_value='30.0',
            description=(
                'Fused pipeline: max scan range. '),
        ),
        DeclareLaunchArgument(
            'log_stats_period_sec', default_value='0.0',
            description='Fused pipeline: seconds between point-count lines. 0 disables.'),
        DeclareLaunchArgument(
            'pub_self_filter_markers', default_value='false', choices=['true', 'false'],
            description='Publish the self-filter volumes on /self_filter_markers for RViz.'),
    ]

    use_fused_lidar_pipeline_arg = DeclareLaunchArgument(
        'use_fused_lidar_pipeline',
        default_value='false',
        choices=['true', 'false'],
        description=(
            'Run the single fused node that publishes BOTH /lidar_points and '
            '/scan_filtered from one pass, instead of dual_lidar_laserscan. Callers must '
            'also stop launching dual_lidar_pointcloud_merger when this is true, since '
            'the fused node publishes /lidar_points itself.'
        ),
    )

    return LaunchDescription([
        filter_type_arg,
        tool_preset_arg,
        print_filter_cmd,
        error_log,
        scan_angle_increment_arg,
        pub_pointcloud_arg,
        use_fused_lidar_pipeline_arg,
        *fused_stage_args,
        use_rviz_arg,
        launch_filter_node_arg,
        OpaqueFunction(function=launch_setup),
    ])