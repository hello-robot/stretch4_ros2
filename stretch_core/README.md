# Overview

*stretch_core* provides the drivers to the Stretch mobile manipulator.

## Dual Hesai LiDAR filtering

The `dual_lidar_laserscan` node (`pointcloud_to_laserscan`) fuses the left and right Hesai point clouds, runs a configurable filter pipeline, and always publishes `/scan_filtered` (`LaserScan`) in `base_footprint`.

- `pub_pointcloud` — optional debug output: publish a filtered merged xyz cloud on `pointcloud_topic` (default `/lidar_pointcloud`, off by default)

### Filter pipeline

Processing is handled by `DualLidarPipeline`. Each stage runs only when enabled. Choose stages with `filter_type` presets or `filter_type:=custom` plus `enable_*` booleans:

| `filter_type` | Stages |
|---------------|--------|
| `region` | SelfRobot, Region |
| `sor` | SelfRobot, Region, SOR |
| `sor_ransac` | SelfRobot, Region, SOR, FloorRansac |
| `self` | SelfRobot |
| `none` | No filters |
| `custom` | Use `enable_self_robot_filter`, `enable_region_filter`, `enable_sor_filter`, `enable_floor_ransac_filter` |

For each lidar point (in `base_footprint`): optionally apply region crop, then a cheap spatial gate, then TF-driven robot geometry checks only inside that gate. Optional SOR denoises near-field points, then points are projected to LaserScan with optional speckle filtering.

Use `region` for mapping. Use `sor` for navigation. Use `self` with `pub_pointcloud:=true` to visualize a filtered merged cloud without region crop.

### Launch

```bash
# Mapping-style filtering (region + self-filter)
ros2 launch stretch_core dual_hesai.launch.py filter_type:=region tool_preset:=sg4

# Navigation-style filtering (adds SOR near the robot)
ros2 launch stretch_core dual_hesai.launch.py filter_type:=sor tool_preset:=sg4

# With RViz
ros2 launch stretch_core dual_hesai.launch.py filter_type:=sor use_rviz:=true
```

`tool_preset` selects the mounted-tool attachment box: `sg4`, `pg4`, `tablet`, or `nil`. It must match the hardware on the robot.

Nav2 navigation launch (`stretch_nav2`) typically includes this stack with `filter_type:=sor` and starts `robot_footprint_publisher` for a dynamic costmap footprint.

### Debug self-filter geometry

```bash
ros2 launch stretch_core self_filter_debug.launch.py tool_preset:=sg4 use_rviz:=true
```

RViz markers are published on `/self_filter_markers` when `pub_self_filter_markers` is true.

### Configuration

| File | Purpose |
|------|---------|
| `config/dual_lidar_filter.yaml` | `filter_type`, region limits, SOR, speckle, floor RANSAC |
| `config/robot_self_filter.yaml` | Base/arm/wrist geometry, spatial gate |
| `config/self_filter_<tool>.yaml` | Tool-specific attachment box |
| `config/robot_footprint.yaml` | Dynamic footprint publisher (topics, base polygon) |

Tuning notes (filter order, gate radius, marker colors): see [config/README.md](config/README.md).

## API

For comprehensive API documentation, please refer to [Coming soon](#TODO).

## Testing

Colcon is used to run the system/perf tests in the */test* folder. The command to run the entire suite of tests is:

```bash
$ cd ~/ament_ws
$ colcon test --packages-select stretch_core
```

You can run individual tests using the following command:

```bash
$ colcon test --packages-select stretch_core --pytest-args -k test_trajectory_server -s --event-handlers console_direct+

Test suites:
  - test_trajectory_server
  - test_pub_topics
  - test_sub_topics
  - test_services
  - test_parameters
```

## License

Please see the [LICENSE](./LICENSE.md) file.
