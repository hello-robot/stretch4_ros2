from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import UnlessCondition
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

LIDAR_PIPELINES = ('laserscan', 'fused', 'none')

def resolve_lidar_pipeline(context):
    """The lidar_pipeline to run, honouring the deprecated launch_filter_node and
    use_fused_lidar_pipeline args when lidar_pipeline is not given."""
    pipeline = LaunchConfiguration('lidar_pipeline').perform(context).strip().lower()
    use_fused = LaunchConfiguration('use_fused_lidar_pipeline').perform(context).strip().lower()
    launch_filter = LaunchConfiguration('launch_filter_node').perform(context).strip().lower()

    notes = []
    if use_fused or launch_filter:
        notes.append(LogInfo(msg=(
            'launch_filter_node and use_fused_lidar_pipeline are deprecated; '
            'use lidar_pipeline:=laserscan|fused|none.')))

    if not pipeline:
        if use_fused == 'true':
            pipeline = 'fused'
        elif launch_filter == 'false':
            pipeline = 'none'
        else:
            pipeline = 'laserscan'

    if pipeline not in LIDAR_PIPELINES:
        raise ValueError(
            f"lidar_pipeline must be one of {', '.join(LIDAR_PIPELINES)}, got '{pipeline}'.")
    return pipeline, notes


def launch_setup(context, *args, **kwargs):
    stretch_core = get_package_share_directory('stretch_core')
    tool_preset = LaunchConfiguration('tool_preset').perform(context)
    validate_tool_preset(tool_preset)
    self_filter_params = dual_lidar_self_filter_parameters(stretch_core, tool_preset)
    fused_params = dual_lidar_fused_parameters(stretch_core, tool_preset)
    lidar_pipeline, notes = resolve_lidar_pipeline(context)

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
    )

    # Publishes both /lidar_points and /scan_filtered.
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
    processing_nodes = {
        'laserscan': [
            LogInfo(msg=['==== lidar_pipeline: laserscan, filter_type: ', filter_type]),
            dual_lidar_filter_node,
        ],
        'fused': [LogInfo(msg='==== lidar_pipeline: fused'), fused_pipeline_node],
        'none': [LogInfo(msg='==== lidar_pipeline: none (driver only)')],
    }
    return [
        *notes,
        hesai_node,
        *processing_nodes[lidar_pipeline],
        *get_rviz_node(rviz_config_path),
    ]


def generate_launch_description():
    filter_type_arg = DeclareLaunchArgument(
        'filter_type',
        default_value='sor_ransac',
        description=(
            'lidar_pipeline:=laserscan only. '
            'Preset: region | sor | sor_ransac | self | none | custom'),
    )

    filter_type = LaunchConfiguration('filter_type')

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

    lidar_pipeline_arg = DeclareLaunchArgument(
        'lidar_pipeline',
        default_value='',
        description=(
            'What runs after the lidar driver (default laserscan). '
            'laserscan: dual_lidar_laserscan publishes /scan_filtered; filter_type and '
            'pub_pointcloud apply. '
            'fused: one node publishes /lidar_points and /scan_filtered; the enable_* args apply. '
            'none: driver only, raw /lidar_points_left and /lidar_points_right.'),
    )

    launch_filter_node_arg = DeclareLaunchArgument(
        'launch_filter_node',
        default_value='',
        description='Deprecated: use lidar_pipeline. false means lidar_pipeline:=none.',
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

    pub_pointcloud_arg = DeclareLaunchArgument(
        'pub_pointcloud', default_value='false',
        description='lidar_pipeline:=laserscan only. Publish a pointcloud from the filter node.')

    fused_stage_args = [
        DeclareLaunchArgument(
            'enable_self_robot_filter', default_value='true', choices=['true', 'false'],
            description='lidar_pipeline:=fused only. Cut the robot body out of both outputs.'),
        DeclareLaunchArgument(
            'enable_floor_ransac_filter', default_value='true', choices=['true', 'false'],
            description=(
                'lidar_pipeline:=fused only. Remove the floor from the scan with a fitted plane. '
                'false uses the z_min cut instead.'),
        ),
        DeclareLaunchArgument(
            'enable_sor_filter', default_value='false', choices=['true', 'false'],
            description='lidar_pipeline:=fused only. StatisticalOutlierRemoval on the scan band.',
        ),
        DeclareLaunchArgument(
            'scan_range_max', default_value='30.0',
            description='lidar_pipeline:=fused only. Max scan range in meters.',
        ),
        DeclareLaunchArgument(
            'log_stats_period_sec', default_value='0.0',
            description=(
                'lidar_pipeline:=fused only. Seconds between point-count lines. 0 disables.')),
        DeclareLaunchArgument(
            'pub_self_filter_markers', default_value='false', choices=['true', 'false'],
            description='lidar_pipeline:=fused only. Publish the self-filter volumes for RViz.'),
    ]

    use_fused_lidar_pipeline_arg = DeclareLaunchArgument(
        'use_fused_lidar_pipeline',
        default_value='',
        description='Deprecated: use lidar_pipeline. true means lidar_pipeline:=fused.',
    )

    return LaunchDescription([
        filter_type_arg,
        tool_preset_arg,
        error_log,
        scan_angle_increment_arg,
        pub_pointcloud_arg,
        lidar_pipeline_arg,
        *fused_stage_args,
        use_rviz_arg,
        launch_filter_node_arg,
        use_fused_lidar_pipeline_arg,
        OpaqueFunction(function=launch_setup),
    ])