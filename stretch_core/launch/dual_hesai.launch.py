from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction, UnsetEnvironmentVariable
from launch.conditions import IfCondition, UnlessCondition
from launch import LaunchDescription
from launch_ros.actions import Node

import os
import sys
import yaml
import tempfile
from pathlib import Path

sys.path.insert(0, os.path.dirname(__file__))
from self_filter_config import dual_lidar_self_filter_parameters, validate_tool_preset


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

    hesai_node = Node(
        package='hesai_ros_driver',
        executable='hesai_ros_driver_node',
        output='screen',
        parameters=[{'config_path': temp_yaml_path}]
    )

    filter_type = LaunchConfiguration('filter_type')
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
                'lidar1_frame': 'lidar_right_link',
                'lidar2_frame': 'lidar_left_link',
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

    return [
        hesai_node,
        dual_lidar_filter_node,
        UnsetEnvironmentVariable('QT_QPA_PLATFORM_PLUGIN_PATH'),
        UnsetEnvironmentVariable('QT_QPA_FONTDIR'),
        UnsetEnvironmentVariable('QT_PLUGIN_PATH'),
        rviz_node,
    ]


def generate_launch_description():
    filter_type_arg = DeclareLaunchArgument(
        'filter_type',
        default_value='region',
        description='Pipeline preset: region | sor | sor_ransac | self_voxel | none | custom',
    )

    filter_type = LaunchConfiguration('filter_type')

    print_filter_cmd = LogInfo(
        msg=['==== USING FILTER TYPE: ', filter_type]
    )

    error_log = LogInfo(
        condition=UnlessCondition(PythonExpression([
            "'", filter_type, "' in ['region', 'sor', 'sor_ransac', 'self_voxel', 'none', 'custom']"
        ])),
        msg="Invalid filter_type! Must be region, sor, sor_ransac, self_voxel, none, or custom.",
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
        default_value='sg4',
        description='Self-filter attachment preset: sg4, pg4, tablet, or nil.',
    )

    return LaunchDescription([
        filter_type_arg,
        tool_preset_arg,
        print_filter_cmd,
        error_log,
        use_rviz_arg,
        launch_filter_node_arg,
        OpaqueFunction(function=launch_setup),
    ])
