from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch_ros.actions import Node

import math
import os
import sys
import yaml
import tempfile
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))
from self_filter_config import dual_lidar_self_filter_parameters, validate_tool_preset


def optional_float_arg(value):
    text = str(value).strip().lower()
    if text in ('none', 'null', 'nan', ''):
        return math.nan
    return float(value)


def launch_setup(context, *args, **kwargs):
    stretch_core = get_package_share_directory('stretch_core')
    tool_preset = LaunchConfiguration('tool_preset').perform(context)
    validate_tool_preset(tool_preset)
    self_filter_params = dual_lidar_self_filter_parameters(stretch_core, tool_preset)

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

    z_min = float(LaunchConfiguration('z_min').perform(context))
    z_max = float(LaunchConfiguration('z_max').perform(context))
    pub_laserscan = LaunchConfiguration('pub_laserscan').perform(context).lower() == 'true'
    pub_pointcloud = LaunchConfiguration('pub_pointcloud').perform(context).lower() == 'true'
    pointcloud_z_min = optional_float_arg(LaunchConfiguration('pointcloud_z_min').perform(context))
    pointcloud_z_max = optional_float_arg(LaunchConfiguration('pointcloud_z_max').perform(context))
    pointcloud_range_max = optional_float_arg(LaunchConfiguration('pointcloud_range_max').perform(context))

    hesai_node = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        output='screen',
        parameters=[{'config_path': temp_yaml_path}]
    )

    filter_type = LaunchConfiguration('filter_type')
    scan_angle_increment_deg = LaunchConfiguration('scan_angle_increment_deg')
    frame_id = LaunchConfiguration('frame_id')
    launch_filter_node = LaunchConfiguration('launch_filter_node')

    dual_lidar_filter_node = Node(
        package='stretch_core',
        executable='dual_lidar_hazard',
        name='dual_lidar_hazard',
        output='screen',
        parameters=[
            *self_filter_params,
            {
                'filter_type': filter_type,
                'scan_angle_increment_deg': scan_angle_increment_deg,
                'frame_id': frame_id,
                'lidar1_frame': 'lidar_right_link',
                'lidar2_frame': 'lidar_left_link',
                'z_min': z_min,
                'z_max': z_max,
                'pointcloud_z_min': pointcloud_z_min,
                'pointcloud_z_max': pointcloud_z_max,
                'pointcloud_range_max': pointcloud_range_max,
                'pub_laserscan': pub_laserscan,
                'pub_pointcloud': pub_pointcloud,
            },
        ],
        condition=IfCondition(launch_filter_node),
    )

    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config_path = os.path.join(stretch_core, 'rviz', 'lidars.rviz')
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        condition=IfCondition(use_rviz),
    )

    os.environ.pop('QT_QPA_PLATFORM_PLUGIN_PATH', None)
    os.environ.pop('QT_QPA_FONTDIR', None)
    os.environ.pop('QT_PLUGIN_PATH', None)

    return [
        hesai_node,
        dual_lidar_filter_node,
        rviz_node,
    ]


def generate_launch_description():
    filter_type_arg = DeclareLaunchArgument(
        'filter_type',
        default_value='sor',
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
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='base_footprint',
        description='Output TF frame for the filtered point cloud and internal lidar processing.',
    )

    z_min_arg = DeclareLaunchArgument(
        'z_min',
        default_value='0.135',
        description='Point with z value less than z_min are pruned when using region without ransac',
    )
    z_max_arg = DeclareLaunchArgument(
        'z_max',
        default_value='1.5',
        description='Point with z value less than z_min are pruned when using region without ransac',
    )
    pointcloud_z_min_arg = DeclareLaunchArgument(
        'pointcloud_z_min',
        default_value='none',
        description='PointCloud-only minimum z in the output frame; use none to disable the lower bound.',
    )
    pointcloud_z_max_arg = DeclareLaunchArgument(
        'pointcloud_z_max',
        default_value='1.5',
        description='PointCloud-only maximum z in the output frame; use none to disable the upper bound.',
    )
    pointcloud_range_max_arg = DeclareLaunchArgument(
        'pointcloud_range_max',
        default_value='2.0',
        description='PointCloud-only horizontal radius crop in meters; <= 0 disables the crop.',
    )
    pub_laserscan_arg = DeclareLaunchArgument('pub_laserscan', default_value='true', description='Publish a LaserScan from the filter node.')
    pub_pointcloud_arg = DeclareLaunchArgument('pub_pointcloud', default_value='true', description='Publish a pointcloud from the hazard filter node.')

    return LaunchDescription([
        filter_type_arg,
        tool_preset_arg,
        print_filter_cmd,
        error_log,
        scan_angle_increment_arg,
        frame_id_arg,
        z_min_arg,
        z_max_arg,
        pointcloud_z_min_arg,
        pointcloud_z_max_arg,
        pointcloud_range_max_arg,
        pub_laserscan_arg,
        pub_pointcloud_arg,
        use_rviz_arg,
        launch_filter_node_arg,
        OpaqueFunction(function=launch_setup),
    ])