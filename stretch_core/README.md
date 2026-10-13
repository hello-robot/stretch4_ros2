# Overview

*stretch_core* provides the drivers and navigation-facing sensor processing for the Stretch mobile manipulator.

## stretch_driver

`stretch_driver` is the main hardware node. It wraps `stretch4_body`'s `RobotClient` and exposes the robot's joints, sensors, and end-of-arm tool over ROS 2. Its ROS interface (topics, services, parameters, modes, and the `follow_joint_trajectory` action) comes from the `Stretch4ROSDriver` base class in `hello_helpers/stretch4_ros_api.py`, which the simulation's `stretch_mujoco_driver` shares, so the same clients work against either.

```bash
ros2 launch stretch_core stretch_driver.launch.py
```

### Modes

The `mode` parameter is the driver mode. Control modes, which a client chooses:

| Mode | Accepts |
|---|---|
| `active` (default) | `cmd_vel`, `cmd_vel_stiff`, `joint_position_cmd`, `joint_velocity_cmd`, `follow_joint_trajectory` |
| `teleop` | `joy` (gamepad) |
| `puppet` | Follows another robot's `/joint_states` (`puppet.*` parameters) |

Priority modes, which the driver enters by itself while it is busy: `homing`, `stowing`, `seating` (`seat_into_dock`), and `runstopped`. Routines are refused unless the robot is in a control mode, and the driver returns to the previous control mode afterwards. `position`, `navigation`, and `velocity` are deprecated: setting one selects `active`, with `velocity` also putting the joints into velocity mode.

Whether each joint takes position or velocity commands is set per joint with `joint_mode.<joint>` (`position` or `velocity`).

### Parameters

All parameters live on the `/stretch_driver` node. Read them with the `<node>:<param>` form used by `ros2 param` and rosbridge's `get_param`:

```bash
ros2 param get /stretch_driver joint_velocity.lift_joint
ros2 param get /stretch_driver tool_info.is_actuated
ros2 param list /stretch_driver
```

Joint parameters use the command joint names: `lift_joint`, `arm_joint`, `wrist_yaw_joint`, `wrist_pitch_joint`, `wrist_roll_joint`, and `gripper_joint` (the attached tool, when it is actuated).

| Parameter | Type | Description |
|---|---|---|
| `mode` | string | Driver mode; see *Modes*. Writable. |
| `joint_mode.<joint>` | string | `position` or `velocity`. Writable. |
| `sensitivity` | string | Guarded-contact sensitivity preset (e.g. `default`, `high_sensitivity_manipulation`). Writable. |
| `tool_info.name` | string | The configured end-of-arm tool (e.g. `eoa_wrist_dw4_tool_sg4`), or `unknown`. Read-only. |
| `tool_info.is_actuated` | bool | Whether the attached tool has any actuated joints. `false` for passive tools (tablet holder, NIL) and when no tool is configured. Read-only. |
| `tool_info.tool_joints` | string[] | The tool's own URDF joint names, from its `ToolMetadata`. Empty for a passive tool. Read-only. |
| `tool_info.{urdf_range,aperture_range,position_tolerance}` | double[] / double | The tool's range in URDF units and aperture meters (each as `[closed, open]`), and the tolerance within which a position counts as reached. Declared only for an actuated tool. Read-only. |
| `joint_velocity.<joint>` | double | Default speed of a position command that doesn't give one (m/s or rad/s; URDF units for `gripper_joint`). Writable. |
| `joint_velocity.omnibase.{linear,angular}` | double | Default base speeds. |
| `joint_acceleration.<joint>`, `joint_acceleration.omnibase.{linear,angular}` | double | Default accelerations. |
| `joint_limit.<joint>.{lower,upper}` | double | Position limits applied to position commands and trajectories. Set for `gripper_joint` from the tool's URDF range. |
| `joint_limit.<joint>.{velocity,acceleration}` | double | Maximum speed and acceleration. Faster velocity commands are limited to `velocity`. |
| `trajectory_server.*` | | The `follow_joint_trajectory` server's settings, e.g. `timeout` and the per-joint `trajectory_tolerance.<joint>` (for `gripper_joint`, the tool's `position_tolerance`). |
| `action_timeout` | double | Kept for launch files written for the old driver: if set (at launch or later), it sets `trajectory_server.timeout`. The navigation launch sets 30. |
| `broadcast_odom_tf` | bool | Broadcast the wheel-odometry TF. Read-only. |
| `velocity_timeout` | double | Timeout for velocity commands, seconds. |
| `command_namespace` | string | Namespace of the command topics (`''` = the node's own). Read-only. |

#### Tool info

`tool_info.*` is resolved once at node startup from `stretch4_body`'s `get_tool_metadata()`. A tool that raises `ToolConfigurationError` — none configured, or a passive tool with no `ToolMetadata` — yields `is_actuated: false` and an empty `tool_joints` rather than a startup failure, and no `gripper_joint` is created.

Clients use `tool_info.is_actuated` to decide whether to offer gripper controls at all; Web Teleop greys out its open/close buttons when it reads `false`.

#### The gripper joint

Whatever tool is attached is commanded as `gripper_joint`, in the tool's URDF units: one finger's value (radians for the Stretch gripper, meters for the parallel gripper). `/joint_states` reports `gripper_joint` in the same units, alongside the tool's own joints (e.g. `gripper_finger_left_joint` and `gripper_finger_right_joint`), so a position read from `/joint_states` can be sent straight back as a command. The tool's own names (e.g. `stretch_gripper`, `stretch_gripper_joint`, or a finger joint) are accepted for `gripper_joint` on command topics and trajectories.

#### Where limits live

Limits are split across surfaces according to how they change. A consumer should read each kind from the surface that owns it:

| Surface | Holds | Consumer guidance |
|---|---|---|
| URDF, on `/robot_description` | **Hardware limits** — each joint's physical envelope, generated by `stretch4_urdf` from robot params. | This is the robot *model*. Consumers should **not** parse it to discover limits; it is not a client API. |
| Topics — `joint_states_diagnostics` | **Current soft limits** — the `soft_motion_limits` status, the hard range narrowed by any user override. These change at runtime. | Subscribe here for position bounds. |
| Parameters — `joint_velocity.*`, `joint_acceleration.*`, `joint_limit.*` | **Speeds and accelerations** — defaults and maximums. | Read at startup; `joint_velocity.*` can be changed. |

#### Joint velocities (parameters)

Values come from `stretch4_body`'s robot params (`robot_params[<joint>]['motion']['default']['vel']`, and `vel_m` / `vel_xy_m` / `vel_w_r` for lift, arm, and base; the `max` profile for `joint_limit.<joint>.velocity`), so they follow whatever tool and calibration the robot is actually configured with. For the tool joint the velocity is in URDF units, while the acceleration stays in the tool's actuator units (rad/s^2).

- The gripper value comes from `tool_metadata.position_independent_velocity_limit('urdf')`: the tool's actuator limit expressed in URDF units, so a client can consume it exactly like every other joint. A rate does not convert like a position, so the published scalar is the minimum over the range — a rate achievable at any position rather than one that holds only near a particular opening.
- `joint_velocity.gripper_joint` is declared only when the tool is actuated. A passive tool has no velocity to report, and its absence is the signal a client should treat the joint as unavailable.

#### Joint position limits (topic)

Position bounds are soft limits that can be narrowed at runtime, so they are published on a topic rather than fixed in a parameter or read from the URDF. `stretch_driver` publishes them on `joint_states_diagnostics` (`diagnostic_msgs/DiagnosticArray`), which carries one `DiagnosticStatus` per concern, each with a `KeyValue` per command joint:

| Status name | Meaning |
|---|---|
| `soft_motion_limits` | **The joint's current position bounds** — `(min, max)`, the hard range from robot params narrowed by any user-set limit. This is the numeric source of truth for a consumer. |
| `at_limit` | Whether the joint is against a stop right now, as `{'pos': bool, 'neg': bool}`. |
| `braking_distance` | Current braking distance. |
| `is_homed` / `is_homing` | Homing state. |
| `is_runstopped` | Runstop state. |
| `in_collision` | Self-collision stop state. |

`soft_motion_limits` and `at_limit` answer different questions and a client generally wants both: the former is *where the bounds are*, the latter is *am I against one now*. Values are `DiagnosticStatus` `KeyValue` pairs, so they arrive as strings and need parsing.

### Topics

Command topics are in `command_namespace` (by default the node's own namespace).

| Topic | Type | Notes |
|---|---|---|
| `joint_states` | `sensor_msgs/JointState` | The command joints (the arm split into its `arm_l1..4_joint` links), the tool's own joints, and `gripper_joint`. |
| `joint_states_diagnostics` | `diagnostic_msgs/DiagnosticArray` | Per-joint soft limits, at-limit state, homing, runstop, collision. See *Joint position limits* above. |
| `mode`, `tool`, `sensitivity`, `is_homed`, `is_runstopped` | `std_msgs` | Latched. |
| `wheel_odom`, `battery` | `nav_msgs/Odometry`, `sensor_msgs/BatteryState` | |
| `joint_position_cmd` (sub) | `sensor_msgs/JointState` | Position targets, for joints in `position` mode. `active` mode. |
| `joint_velocity_cmd` (sub) | `sensor_msgs/JointState` | Velocities, for joints in `velocity` mode. `active` mode. |
| `cmd_vel` (sub) | `geometry_msgs/Twist` | Base velocity. `active` mode. |
| `cmd_vel_stiff` (sub) | `geometry_msgs/TwistWithCovarianceStamped` | Base velocity and stiffness (0–1) in `covariance[0]`; full stiffness if the covariance's last element is 0. Used by the docking server. `active` mode. |
| `joy` (sub) | `sensor_msgs/Joy` | Gamepad. `teleop` mode. |
| `joint_vel` (sub) | `control_msgs/JointJog` | Deprecated; handled by the `joint_jog_converter` node (`deprecated_interfaces:=true`, the default), which forwards it to `joint_velocity_cmd`. Wrist and gripper jogs move by `velocity × duration` (URDF units for the gripper). |

### Services

All `std_srvs/Trigger`, except `runstop_the_robot` (`std_srvs/SetBool`). Each publishes its calls on `<service>/_service_event` (service introspection), which the digital twin relay follows.

| Service | |
|---|---|
| `stop_the_robot` | Stop all joints and the base. |
| `home_the_robot`, `stow_the_robot` | Run the homing (60 s timeout) or stowing (30 s) routine in the `homing` / `stowing` mode; fails if it doesn't complete. |
| `seat_into_dock` | Seat the robot into the dock it is lined up with (`routine_blind_dock`, 60 s timeout) in the `seating` mode. Not supported in simulation yet. |
| `freewheel_the_robot` | Let the base's wheels turn freely. |
| `runstop_the_robot` | Set or clear the runstop. |

### End-of-arm tools

The driver is tool-agnostic: joint names, unit conventions, and position tolerances are resolved at runtime through `stretch4_body`'s `ToolMetadata`, so a new gripper needs no driver changes. See the [custom user tool guide](https://github.com/hello-robot/stretch4_body#custom-user-end-of-arm-tools) in stretch4_body for how a tool declares its joints, units, and tolerance.

## Dual Hesai point cloud merger

The `dual_lidar_pointcloud_merger` node synchronizes the left and right Hesai `PointCloud2` streams, transforms both clouds into a target frame, preserves the original per-point fields such as `ring` and `timestamp`, and publishes one merged cloud.

It is intended for consumers that need a merged point cloud rather than a projected navigation scan. The node caches the static lidar transforms after they become available, supports an optional pre-transform voxel downsample with `merger_voxel_leaf_size`, and can blank points inside a base-centered cylinder with `cylinder_filter_radius`.

Example:

```bash
ros2 run stretch_core dual_lidar_pointcloud_merger --ros-args \
  -p left_topic:=/lidar_points_left \
  -p right_topic:=/lidar_points_right \
  -p output_topic:=/lidar_points \
  -p target_frame:=base_link
```

## Dual Hesai LaserScan filtering

The `dual_lidar_laserscan` node (`pointcloud_to_laserscan`) fuses the left and right Hesai point clouds, runs a configurable filter pipeline, and publishes `/scan_filtered` (`LaserScan`) in `base_footprint`. The scan is only published while synchronized pairs arrive from **both** lidars: if either lidar stops (or their stamps drift more than 0.2 s apart), `/scan_filtered` goes silent and the node reports the stale input on `/diagnostics`.
- `pub_pointcloud` - optional debug output: publish a filtered merged xyz cloud on `pointcloud_topic` (default `/lidar_pointcloud`, off by default)

### Filter presets

Processing is handled by `DualLidarPipeline`. The easiest way to choose behavior is with `filter_type`:

| `filter_type` | What it enables | Typical use |
|---------------|-----------------|-------------|
| `region` | Region crop + robot self-filter | Mapping and general filtered scans |
| `sor` | Region crop + robot self-filter + near-robot SOR | Navigation |
| `sor_ransac` | Region crop + robot self-filter + near-robot SOR + floor RANSAC | Navigation when floor returns need explicit removal |
| `self` | Robot self-filter only | Debugging robot/self-hit removal |
| `none` | No filtering before projection | Baseline/debug comparison |
| `custom` | Uses the individual `enable_*` booleans | Experiments and tuning |

### Available filtering techniques

Each technique is optional except the final LaserScan projection:

| Technique | What it does | User-facing knobs |
|-----------|--------------|-------------------|
| Region crop | Removes points outside the configured height/range limits. This is the simple floor/ceiling/far-range crop. | `z_min`, `z_max`, `range_max` |
| Robot self-filter | Removes returns from the robot itself: base cylinder, arm capsule, and required URDF-derived arm/wrist/gripper/tool boxes. | `base_radius`, `arm_filter_radius`, `self_filter_*_buffer`, `self_filter_box_*` |
| Self-filter spatial gate | Cheap pre-check before robot geometry. Points outside this XY/Z gate skip the expensive robot shape checks. It does not remove points by itself. | `self_filter_spatial_gate_enabled`, `self_filter_gate_radius_m`, `self_filter_gate_z_min_m`, `self_filter_gate_z_max_m` |
| Internal voxel downsample | Near-robot voxel grid reduction used before expensive work. It is not a user-selectable `filter_type`; it runs automatically when robot self-filter or SOR is enabled. | `dist_rob`, `leaf_size` |
| SOR | PCL StatisticalOutlierRemoval for flying/noisy near-robot points. In the `sor` preset, voxel downsample runs first, then SOR runs inside `dist_rob`; far points pass through. | `dist_rob`, `sor_mean_k`, `sor_stddev` |
| Floor RANSAC | Fits the floor plane with RANSAC from the configured floor-detection Z band, then removes points close to that plane. It is only floor removal, not general obstacle detection. | `floor_detect_z_min`, `floor_detect_z_max`, `plane_fitting_threshold`, `angle` |
| Speckle filter | Post-projection cleanup on LaserScan bins. It removes weak isolated bins that do not have angular support from neighboring bins. | `speckle_filter_enabled`, `speckle_min_points`, `speckle_neighbor_window`, `speckle_min_neighbors`, `speckle_range_tolerance` |

The runtime order is:

```text
transform -> region crop -> internal voxel -> robot self-filter -> SOR -> floor RANSAC -> LaserScan projection -> speckle filter
```

Only enabled stages run. Projection always runs because it produces `/scan_filtered`. Use `region` for mapping, `sor` for navigation, and `self` with `pub_pointcloud:=true` to visualize robot/self-hit removal without the region crop.

### Launch

```bash
# Mapping-style filtering: region + self-filter
ros2 launch stretch_core dual_hesai.launch.py filter_type:=region tool_preset:=auto

# Navigation-style filtering: region + self-filter + near-robot SOR
ros2 launch stretch_core dual_hesai.launch.py filter_type:=sor tool_preset:=auto

# With RViz
ros2 launch stretch_core dual_hesai.launch.py filter_type:=sor tool_preset:=auto use_rviz:=true
```

`tool_preset` selects URDF-derived self-filter geometry for the mounted tool. Use `auto` to read Stretch robot params when available, or pass `sg4`, `pg4`, `tablet`, or `nil` explicitly.

Nav2 navigation launch (`stretch_nav2`) typically includes this stack with `filter_type:=sor` and starts `robot_footprint_publisher` for a dynamic costmap footprint.

### Robot self-filter and footprint geometry

Self-filter geometry is generated from URDF collision geometry at launch time. The generated boxes cover the arm/lift details, wrist chain, gripper camera, and the selected SG4/PG4/tablet tool geometry. Empty frames such as grasp frames and ArUco marker frames are not used as collision boxes.

The base cylinder and arm capsule are always active when the self-filter stage runs. The URDF-derived boxes are required; launch through `dual_hesai.launch.py` or `robot_footprint.launch.py` so `self_filter_config.py` can generate the temporary `self_filter_box_*` parameter YAML.

The dynamic footprint publisher receives the same generated geometry as the lidar self-filter. By default, footprint buffers match the self-filter buffers so Nav2 plans conservatively around the same arm/tool volume that lidar filtering treats as robot geometry.

### Debug self-filter geometry

Run the lidar filter and enable marker publication on the node:

```bash
ros2 launch stretch_core dual_hesai.launch.py filter_type:=self tool_preset:=auto pub_pointcloud:=true use_rviz:=true
ros2 param set /pointcloud_to_laserscan pub_self_filter_markers true
ros2 param set /pointcloud_to_laserscan publish_raw_urdf_self_filter_markers true
ros2 param set /pointcloud_to_laserscan publish_buffered_self_filter_markers true
```

RViz markers are published on `/self_filter_markers`. Raw URDF boxes show the collision geometry from the URDF. Buffered boxes show the effective lidar self-filter geometry.

### Configuration

| File | Purpose |
|------|---------|
| `config/dual_lidar_filter.yaml` | `filter_type`, region limits, SOR, speckle, floor RANSAC |
| `config/robot_self_filter.yaml` | Shared self-filter policy: base cylinder, arm capsule, spatial gate, marker controls |
| launch-generated temp YAML | Required URDF-derived `self_filter_box_*` geometry for the selected `tool_preset` |
| `config/robot_footprint.yaml` | Dynamic footprint publisher topics, base polygon, and joint update thresholds |

Tuning notes for filter order, gate radius, URDF box buffers, and markers: see [config/README.md](config/README.md).

### Head lidar PTP check

Read-only verification of JT128 return mode (Last + Strongest), point-cloud filter (Strong), PTP lock offset (350 µs), locked PTP status (single read), and jitter p95 ≤ 350 µs over 30 s (direct PTC TCP, no ROS topics):

```bash
ros2 run stretch_core stretch_lidar_check
```

Options: `--left`, `--right`, `--duration 30`, `--json`, `--verbose`.

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

## Head lidar PTC check

`stretch_lidar_check` verifies JT128 return mode (Last + Strongest), point-cloud filter (Strong), PTP lock offset (350 µs), locked PTP status, and jitter p95 over PTC TCP (port 9347):


## License

Please see the [LICENSE](./LICENSE.md) file.
