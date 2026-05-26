# Overview

*stretch_core* provides the drivers to the Stretch mobile manipulator.

## Dual Hesai LiDAR filtering

C++ nodes convert synchronized dual `PointCloud2` topics into `/scan_filtered`:

- `region_dual_lidar_laserscan` — region / height filtering only
- `voxel_dual_lidar_laserscan` — voxel grid + statistical outlier removal

Launch with `ros2 launch stretch_core dual_hesai.launch.py filter_type:=region` or `filter_type:=sor`.

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
