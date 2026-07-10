# stretch_base_hazard

Base hazard mapping for Stretch4. This package uses direct-style
detector/grid/extractor logic behind ROS 2 topic inputs. It builds a
robot-centric rolling hazard map from:

- One already-merged 3D lidar `PointCloud2` topic
- Line-sensor obstacle and small-drop `PointCloud2` topics

It publishes:

- `/under_base_hazard/points`
- `/under_base_hazard/obstacle_points`
- `/under_base_hazard/cliff_points`
- `/under_base_hazard/occluded_points`
- debug point clouds under `/under_base_hazard/debug/*`

## Typical Use

Consume already-published point-cloud and line-sensor topics. The node does not
start a Hesai reader, `stretch_core`, or Stretch body APIs; it only subscribes
to topics.

The default
`lidar_topic` is `/lidar_pointcloud`, matching the optional point-cloud output
from `stretch_core dual_lidar_laserscan`:

```bash
ros2 launch stretch_base_hazard hazard_map.launch.py \
  lidar_topic:=/lidar_pointcloud \
  line_obstacle_topic:=/line_sensor/obstacle_points \
  line_small_drop_topic:=/line_sensor/small_drop_points \
  detector_rate_hz:=10.0
```

If your merged cloud is already in the desired base frame, leave `lidar_frame`
empty so the node uses `PointCloud2.header.frame_id`. Set `lidar_frame:=base_link`
only when you want to override a missing or wrong header. `line_frame` behaves
the same way for the line-sensor point-cloud topics. Line-sensor range cleanup
is handled upstream by the `stretch_core` line-sensor publisher.

Run the dual-Hesai filter once and share both outputs by enabling the point-cloud
publisher on the existing `stretch_core` launch:

```bash
ros2 launch stretch_core dual_hesai.launch.py \
  launch_filter_node:=true \
  filter_type:=sor_ransac \
  pub_pointcloud:=true
```

The hazard map does not subscribe to `LaserScan` directly because the cliff and
floor logic needs 3D `z` evidence. The useful `stretch_core` setup is
`pub_pointcloud:=true`, then pass that point-cloud topic into `hazard_map_node`.
If you feed a `sor_ransac` cloud, remember that the published cloud is after
floor removal; obstacle evidence will still be useful, but cliff/floor-clear
evidence is better when the shared cloud retains near-floor points.

## Hazard-Aware Teleop

For the Stretch gamepad path, launch the hazard-aware gamepad wrapper:

```bash
ros2 launch stretch_base_hazard hazard_gamepad_teleop.launch.py
```

This starts `hazard_gamepad_teleop`, reads the same hazard point topics from
`config/hazard_teleop_filter.yaml`, and wraps the Stretch gamepad base velocity
commands before they reach the base.

For ROS `Twist` teleop pipelines, launch the command-velocity filter:

```bash
ros2 launch stretch_base_hazard hazard_cmd_vel_filter.launch.py
```

That node subscribes to `cmd_vel_unfiltered` and publishes filtered commands on
`cmd_vel`. The upstream teleop node must publish to `cmd_vel_unfiltered` for
this path to do anything.

Obstacle handling uses two zones so tight doorways can be crossed slowly:

- `hard_obstacle_buffer_m` - obstacle points inside `footprint_m + hard_obstacle_buffer_m` stop linear motion.
- `soft_obstacle_buffer_m` - obstacle points inside `footprint_m + soft_obstacle_buffer_m` slow linear motion when the hard zone is clear.
- `min_clearance_speed_scale` and `creep_linear_speed_mps` set the slow-zone scale and speed cap.

Cliff points still hard-stop linear motion using `cliff_buffer_m`.
