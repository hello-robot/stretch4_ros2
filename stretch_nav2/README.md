## Overview

The *stretch_nav2* package provides the standard ROS 2 navigation stack (Nav2) with its launch files. This package utilizes slam_toolbox and Nav2 to drive Stretch around a mapped space. Running this code will require the robot to be untethered. We recommend stowing the arm while running navigation on the robot.

## Quickstart

The first step is to map the space that the robot will navigate in. The `offline_mapping.launch.py` will enable you to do this. First, run:

```bash
ros2 launch stretch_nav2 offline_mapping.launch.py
```

Rviz will show the robot and the map that is being constructed. With the terminal open, use the joystick (see instructions below for using a keyboard) to teleoperate the robot around. Avoid sharp or fast turns and revisit previously visited spots to form loop closures. In Rviz, once you see a map that has reconstructed the space well enough, open a new terminal and run the following commands to save the map to the `stretch_user/` directory.

```bash
mkdir ${HELLO_FLEET_PATH}/maps
ros2 run nav2_map_server map_saver_cli -f ${HELLO_FLEET_PATH}/maps/<map_name>
```

**NOTE**: The `<map_name>` does not include an extension. The map_saver node will save two files as `<map_name>.pgm` and `<map_name>.yaml`.

**Tip**: For a quick sanity check, you can inspect the saved map using a pre-installed tool called Eye of Gnome (eog) by running the following command:

```bash
eog ${HELLO_FLEET_PATH}/maps/<map_name>.pgm
```

Next, with `<map_name>.yaml`, we can navigate the robot around the mapped space. Run:

```bash
ros2 launch stretch_nav2 navigation_mppi.launch.py  map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
```

A new RViz window should pop up with a `Startup` button in a menu at the bottom left of the window. Press the `Startup` button to kick-start all navigation related lifecycle nodes. Rviz will show the robot in the previously mapped space, however, it's likely that the robot's location on the map does not match the robot's location in the real space. To correct this, from the top bar of Rviz, use `2D Pose Estimate` to lay an arrow down roughly where the robot is located in the real space. This gives an initial estimate of the robot's location to AMCL, the localization package. AMCL will better localize the robot once we pass the robot a `2D Nav Goal`.

In the top bar of Rviz, use `2D Nav Goal` to lay down an arrow where you'd like the robot to navigate. In the terminal, you'll see Nav2 go through the planning phases and then navigate the robot to the goal. If planning fails, the robot will begin a recovery behavior - spinning around 180 degrees in place or backing up.

**Tip**: If navigation fails or the robot becomes unresponsive to subsequent goals through RViz, you can still teleoperate the robot using the Xbox controller.

---

---

## Launch structure + Nav2 params overlay

### Which launch files to use

Top-level launch files live directly under `launch/` — these are the ones you run:

| Launch file | Purpose |
|-------------|---------|
| `offline_mapping.launch.py` | Build a map with SLAM |
| `navigation_mppi.launch.py` | Navigate on a saved map (main entry point) |
| `navigation_mppi_nav2_filters.launch.py` | Navigate with the keepout and/or speed costmap filters |
| `navigation_mppi_binary_filter.launch.py` | Navigate with binary-filter adaptive params |
| `binary_filter_launch.py` | Start the binary costmap filter servers |
| `global_plan_demo.launch.py` | Standalone global planner demo |

Supporting launch files that are included by the above live under `launch/include/` (`nav_core`, `bringup`, `navigation_launch`, `nav2_filters`, `slam_toolbox`). You normally do not run these directly.

### Launch ordering

The usual entry point is `navigation_mppi.launch.py`, which includes launch files in this order:

`navigation_mppi.launch.py` → `include/nav_core.launch.py` → `include/bringup_launch.py` → `include/navigation_launch.py`

What each file contains:

- **`navigation_mppi.launch.py`**: top-level “run navigation on the robot” launcher. Starts the Stretch driver, starts the dual-lidar filter that publishes `/scan_filtered`, then launches `include/nav_core.launch.py` with the merged Nav2 params.
- **`include/nav_core.launch.py`**: Stretch wrapper around bringup. Validates the map file, includes `include/bringup_launch.py`, and optionally launches RViz.
- **`include/bringup_launch.py`**: Nav2 bringup orchestrator. Loads/rewrites the `params_file`, then includes localization/SLAM (from `nav2_bringup`) and navigation (from `stretch_nav2`).
- **`include/navigation_launch.py`**: Nav2 navigation servers (controller/planner/BT/etc). Runs either as composed components in a container or as separate ROS nodes depending on `use_composition`.

### Nav2 parameter overlay order (`MultiYaml`)

`navigation_mppi.launch.py` passes a `params_file` built with `MultiYaml([...])`. YAML files are merged **in order**; later files override earlier ones at nested keys.

Overlay order:

- `config/original_nav2_params.yaml`: upstream Nav2 baseline
- `config/nav2_params_core.yaml`: Stretch-specific changes (e.g., `/scan_filtered`, omni AMCL, costmap scan topics)
- `config/nav2_params_mppi.yaml`: MPPI controller selection + core controller/costmap changes
- `config/mppi_params.yaml`: MPPI tuning parameters

### Debugging Nav2 components: `use_composition:=False`

By default, Nav2 may run as composable components inside a single container node, which makes per-component logs harder to follow. For debugging, set:

`use_composition:=False`

This runs each Nav2 component as its own ROS node so its logs are visible directly. Note: the `use_composition` launch argument is declared in `include/bringup_launch.py` / `include/navigation_launch.py` and must be passed through from the top-level launch file to take effect.

## Navigation Launch Options:
Different environments often require different navigation strategies. There’s no single setup that works best everywhere. Below are options you can try to adapt navigation performance to your environment.  


**NOTE:** use can use the argument map:=<path to your map>/<map_name>.yaml with all the navigation launch commands
### 1) Handling Noisy Laser Scans
If your LaserScan data contains a lot of noise, use the denoise layer:

```bash
ros2 launch stretch_nav2 navigation_mppi.launch.py \
  params_file:=/home/hello-robot/ament_ws/src/stretch4_ros2/stretch_nav2/config/nav2_params_mppi_denoise.yaml
```

### 2) Preventing Unnecessary Replanning

In environments with many possible paths (e.g., highly connected spaces), Nav2 may constantly switch paths.
You can avoid this by using a behavior tree  (set via the `default_nav_to_pose_bt_xml` parameter)  that only replans when the current path becomes invalid:

```bash
ros2 launch stretch_nav2 navigation_mppi.launch.py params_file:=/home/hello-robot/ament_ws/src/stretch4_ros2/stretch_nav2/config/nav2_params_mppi_bt.yaml
```

### 3) Forcing the Robot to Always Face Forward
You can set the motion_model parameter of the MPPI to diffdrive.
⚠️ Not recommended unless you specifically want the robot to always face forward.
This disables omni-motion, which is usually helpful for obstacle avoidance (the robot can slide sideways without rotating making response faster).

### 4) Using a Binary Filter for Adaptive Navigation

The **binary filter** detects when the robot **enters or exits specific areas** of the map.  
It publishes to the `/binary_state` topic, which outputs `true` or `false` only when the robot transitions into or out of a marked region (not continuously).  

In our setup, the binary filter was used to **dynamically adjust navigation parameters** to improve doorway navigation:

- **Entering a doorway (narrow, cluttered area):** speed is reduced and costmap inflation is lowered.  
- **Exiting the doorway:** normal speed and inflation are restored.  

By lowering inflation in narrow areas, the robot could maneuver without being overly conservative. To maintain safety, speed was also reduced in these areas. In open spaces, higher inflation and normal speed were used to prevent collisions while enabling faster movement. This approach balances safety and efficiency by adjusting both inflation and speed according to the environment.

#### Launching the Binary Filter

1. Start the filter node:

```bash
ros2 launch stretch_nav2 binary_filter_launch.py
```
2. Launch navigation with the filter configuration:

```bash
ros2 launch stretch_nav2 navigation_mppi_binary_filter.launch.py
```

Alternative for running the navigation_mppi_binary_filter:

```bash
ros2 launch stretch_nav2 navigation_mppi.launch.py \
  params_file:=/home/hello-robot/ament_ws/src/stretch4_ros2/stretch_nav2/config/nav2_params_mppi_binary_filter.yaml

ros2 run stretch_nav2 binary_filter_switch.py
```

⚠️ Remember: **binary_filter_launch.py** must always be started first.

You can verify that the filter is active by subscribing to the map topic defined under mask_topic in binary_filter_param.yaml. Ensure the QoS settings match the publisher.

#### How the Filter Works

The binary_filter_switch.py node listens to /binary_state and updates navigation parameters whenever the robot enters or leaves a marked area.

You can modify this node (located in the stretch_nav2 subfolder) to adjust any parameter or trigger additional actions, such as disabling cameras in certain zones.

#### How Set Up Your Own Binary Filter

1. Annotate your map image with the areas where the filter should activate.
<!-- 
video on how to do that. In the video i used map enhancer https://github.com/ali-pahlevani/Map_Enhancer_Wizard to make the obstacles all solid color to make it easy to remove them.

The application used is Gimp https://www.gimp.org/downloads/.

<video src="video/gimp_tutorial.webm" controls width="600">
  Your browser does not support the video tag.
</video>
-->

2. Make sure your global or local costmap includes the filter under the filters: parameter.

### 5) Keepout and Speed Filters (`nav2_filters`)

Two Nav2 costmap filters, driven by mask images you paint over your map:

- **Keepout filter** — marks regions the planner must not route through. Applied to both the
  global and local costmaps.
- **Speed filter** — marks regions with a speed limit. Applied to the global costmap only.

Both live in one launch file and are selected with launch arguments, so you can run either
one or both together.

#### Launching

Both filters (the default):

```bash
ros2 launch stretch_nav2 navigation_mppi_nav2_filters.launch.py \
  map:=/path/to/map.yaml \
  keepout_mask:=/path/to/keepout_mask.yaml \
  speed_mask:=/path/to/speed_mask.yaml
```

Keepout only — no `speed_mask` needed:

```bash
ros2 launch stretch_nav2 navigation_mppi_nav2_filters.launch.py \
  map:=/path/to/map.yaml \
  keepout_mask:=/path/to/keepout_mask.yaml \
  enable_speed:=false
```

Speed only — no `keepout_mask` needed:

```bash
ros2 launch stretch_nav2 navigation_mppi_nav2_filters.launch.py \
  map:=/path/to/map.yaml \
  speed_mask:=/path/to/speed_mask.yaml \
  enable_keepout:=false
```

Setting both `enable_keepout:=false` and `enable_speed:=false` starts no filter servers and
skips the overlay entirely, leaving you with plain `navigation_mppi` behaviour.

#### Arguments

| Argument | Default | Purpose |
|----------|---------|---------|
| `map` | *(required)* | Occupancy map yaml |
| `enable_keepout` | `true` | Start the keepout mask/info servers and enable the plugin |
| `enable_speed` | `true` | Start the speed mask/info servers and enable the plugin |
| `keepout_mask` | `''` | Keepout mask yaml. **Required when `enable_keepout` is true** |
| `speed_mask` | `''` | Speed mask yaml. **Required when `enable_speed` is true** |
| `tool_preset` | `auto` | Mounted tool for the lidar self-filter: `auto`, `sg4`, `pg4`, `tablet`, `nil` |
| `use_rviz` | `true` | Start RViz |
| `use_composition` | `True` | Run Nav2 composed. Set `False` to debug individual nodes |

Enabling a filter without giving it a mask fails fast at launch:

```
RuntimeError: enable_keepout=true but keepout_mask is empty
```

Resources for [Keepout and Speed Filters]

https://docs.nav2.org/tutorials/docs/navigation2_with_speed_filter.html
https://docs.nav2.org/tutorials/docs/navigation2_with_keepout_filter.html
---

## ArUco Tag-Based Localization and Calibration

The `stretch_nav2` package supports seeding the robot's initial pose using a pre-calibrated ArUco tag on the map. This is useful for instantly localizing the robot without manually estimating its pose in RViz using the `2D Pose Estimate` tool.

The services for calibrating the tag location (`/calibrate_tag_pose` with `std_srv Trigger`) and seeding localization when the robot can see the tag (`/seed_localization` with `std_srv Trigger`) are provided by the `aruco_tag_localization.py` node.

The workflow consists of two phases:

1. **Calibration**: Measure and save the static transform between the `map` and the ArUco tag (default ID: `999`, 150mm, 6x6x1000 dictionary).
2. **Localization Seeding**: Whenever the robot is unlocalized, look at the tag and trigger initial pose estimation.

### 1. Calibration Setup & Launch

To run the calibration process, place the robot in a well-localized state on your map (using AMCL or RViz) facing your statically mounted localization ArUco tag.

#### Step 1: Launch the Tag Calibration Stack

Run the bringup launch file to start the driver, cameras, tag perception, Nav2, and the tag localization node:

```bash
ros2 launch stretch_nav2 tag_calibration_bringup.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
```

#### Step 2: Launch the Interactive Calibration GUI

In a separate terminal, start the interactive OpenCV calibration interface:

```bash
ros2 run stretch_nav2 calibrate_tag_cli.py
```

* Interactive Window Controls:
  * Adjust the robot's head or position until the target ArUco tag is highlighted with a green bounding box in the GUI.
  * Press c or SPACE in the GUI window, or press ENTER in your terminal to trigger the calibration.
  * Press ESC in the GUI or Ctrl+C in the terminal to exit.

Once triggered, the terminal will print a structured comparison table of the calibrated pose and automatically save it to `~/stretch_user/maps/tag_localization/<map_name>_tag_pose.yaml`.

### 2. Seeding Robot Localization

Once your tag is calibrated and saved, you can use it to instantly localize the robot.

Power on the robot and start navigation or tag-calibration stacks:

```bash
ros2 launch stretch_nav2 tag_calibration_bringup.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
```

Position the robot so that its camera can see the calibrated tag. Call the seed localization service to calculate and publish the robot's initial pose to Nav2 (/initialpose):

```bash
ros2 service call /seed_localization std_srvs/srv/Trigger
```

If the robot can see the calibration tag, AMCL will automatically ingest this initial pose estimate and align the robot's position on the map.

---

## Autodocking

The `docking_server` node implements the standard Nav2 `DockRobot` action on `/dock_robot`, so existing Nav2 docking clients work against it unchanged. The full definition of DockRobot is available at [this url](https://api.nav2.org/actions/jazzy/dockrobot.html).

### Discovering docks

After following the [quickstart](#quickstart) to create a map, run:
```
ros2 launch stretch_nav2 discover_dock.launch.py map:=${HELLO_FLEET_PATH}/maps/<map_name>.yaml
```

Similar to before, teleoperate the robot around the space. When the robot discovers a dock, it'll automatically add it to its database.

TODO:

```
ros2 launch stretch_nav2 autodocking_cpu.launch.py map:=${HELLO_FLEET_PATH}/maps/apt1.yaml autodocking_log_level:=debug
```

TODO
```
ros2 launch stretch_nav2 autodocking_cpu.launch.py
ros2 run stretch_nav2 dock_nav_cycle.py --ros-args -p dock_id:=<id in the dock database>
Then click Publish Point in RViz.
```

### Custom `error_code` values

Nav2 reserves the `900`–`999` block of `DockRobot.Result.error_code` for its own errors (`DOCK_NOT_IN_DB=901` through `UNKNOWN=999`). Stretch-specific outcomes that upstream has no code for are assigned codes in the `800` block instead, deliberately below that range so future Nav2 additions cannot collide with ours.

| Code | Name | Meaning |
|------|------|---------|
| `800` | `PREEMPTED_ERROR_CODE` | Goal was superseded by a newer dock request |
| `801` | `FAILED_TO_STOW_ERROR_CODE` | The arm could not be brought into its stowed configuration |
| `802` | `UNDOCK_BLOCKED_ERROR_CODE` | The space the robot would undock into is occupied |

`800` and `801` are defined at the top of `stretch_nav2/docking_server.py`, `802` in `stretch_nav2/undocking_server.py`.

The docking server accepts concurrent goals and always runs the most recent one, so `800` is a normal outcome rather than a fault. Treat it as "someone else took over," and only retry if the preemption was not intentional.

`801` means the robot never reached a configuration it can safely dock from. It is raised when the stow times out, when `/stow_the_robot` is unavailable or refused, or when the joints are still out of tolerance after the stow returns (custom servo EEs should tune tol). Unlike `800`, it is a genuine fault.

`802` is raised by `undocking_server` before it moves. Undocking drives the base 0.5 m to its right, so the server checks that corridor against the lidar costmap first - any obstacle or cliff cell within the servo's own 0.27 m stop radius of the swept path aborts the goal without moving, as does a missing or >1 s stale costmap. It replaces what used to be a blind timed sideways move.

Alongside these, the server emits the standard Nav2 codes:

- `DOCK_NOT_IN_DB` (901) - the requested dock ID isn't tracked by the [dock database](#dock-database)
- `FAILED_TO_STAGE` (903) - the robot should switch to servo-based docking as soon as it sights the dock, so arriving to the staging pose via Nav2 is an error and this err_code is returned. Additionally, if this step (STAGING) of the FSM takes longer than `goal.max_staging_time`, then this err_code is returned.
- `FAILED_TO_DETECT_DOCK` (904) - the dock was never identified after reaching the staging pose, or was lost partway through visual servoing.
- `FAILED_TO_CONTROL` (905) - servoing exceeded its timeout, or blind docking completed without the charger reporting a connection.
- `UNKNOWN` (999) - either 1) for preemption, a new goal gave up waiting for the previous one to release the robot, or 2) robot power state is unclear/unknown

A future improvement (TODO) is to implement `DOCK_NOT_VALID`, defind by Nav2 as "Error code indicating the dock pose or configuration is invalid". Currently, the autodocking software makes no attempt to determine whether the dock is in a configuration that permits docking. It'll try even if the dock is facing a wall. This isn't a big deal since the obstacle-aware docking routine won't permit collisions with the wall, and the robot will simply dither near the dock for 25s (or whatever timeout is configured) before erroring out.

### Dock Discovery

TODO

### Dock Database

Discovered docks are persisted to disk in `~/stretch_user/maps/docks/{map_name}_docks.yaml`. The format will be:

```yaml
version: '1.0'
docks:
  dock_1:
    timestamp: '2026-09-01T13:45:00'
    pose:
      position: [1.2, 0.5, 0.1]
      orientation: [0.0, 0.0, 0.0, 1.0]
```

Docks are auto-enumerated. The next discovered dock for this map would be "dock_2". You can edit the yaml directly give docks more interpretable names.

A Python class, `stretch_nav2.DockDatabase` makes working with this database easy:

```python
from hello_helpers.hello_misc import HelloNode
from stretch_nav2.dock_database import DockDatabase

temp_node = HelloNode.quick_create('temp_ipython_node')
db = DockDatabase(temp_node, default_map_name='apartment_map')
# If you have Nav2 running (specifically map_server), DockDatabase
# will automatically pick up the active map you're navigating with

# 1. Print summary (reports current active map, version, and loaded dock IDs)
print("--- Summary ---")
print(db)

# 2. List docks the database knows about
print("\n--- Available Dock IDs ---")
print(list(db.keys()))

# 3. Add a mock dock for prototyping (e.g., 'kitchen_dock')
print("\n--- Adding 'kitchen_dock' ---")
db['kitchen_dock'] = {
    'timestamp': '2026-09-01T14:40:00',
    'pose': {
        'position': [2.45, -1.12, 0.0],
        'orientation': [0.0, 0.0, 0.7071, 0.7071]
    }
}
db.save_database() # Save back to YAML
print(db)

# 4. Check if a dock exists by ID
print("\n--- Checking for 'kitchen_dock' ---")
if 'kitchen_dock' in db:
    print(f"Found '{search_id}'!")
else:
    print(f"'{search_id}' is not in the database.")

# 5. Retrieve a dock by name and convert its pose dict to PoseStamped
kitchen_dock = db['kitchen_dock']
pose_stamped_msg = DockDatabase.dict_to_pose_stamped(kitchen_dock['pose'])
print("\nConverted PoseStamped Msg:")
print(f"  Frame ID: {pose_stamped_msg.header.frame_id}")
print(f"  Position: [{pose_stamped_msg.pose.position.x:.2f}, {pose_stamped_msg.pose.position.y:.2f}]")
print(f"  Orientation: [{pose_stamped_msg.pose.orientation.z:.4f}, {pose_stamped_msg.pose.orientation.w:.4f}]")
```

## Type checking

Run `pip3 install pyright`. Pyright is the one worth running - it catches attribute typos (`self.future` for `self.seating_future`) that the `mypy/pyflake` miss entirely.

```bash
source /opt/ros/jazzy/setup.bash && source ~/ament_ws/install/setup.bash

python3 -m pyflakes stretch_nav2/*.py
python3 -m mypy --ignore-missing-imports --check-untyped-defs stretch_nav2/*.py
python3 -m pyright --pythonpath "$(which python3)" stretch_nav2/*.py
```

Pyright resolves ROS imports from the sourced environment via `--pythonpath`, so there is no
config file to keep in sync. Expect ~16 unavoidable errors from upstream, all of one of two kinds: `rclpy.time` / `rclpy.duration` / `tf2_ros.TransformException` reported as unknown attributes (submodules pyright cannot see without an explicit import), and `ActionServer` / `CancelResponse` / `GoalResponse` / `rclpy.ok` reported as private (rclpy re-exports them without `__all__`). Both are fine at runtime. Anything else is worth reading.

## Things to Note and Design Decisions

- **Filtering Methods:** We use two filters for point cloud processing:  
  1. **voxel_sor filter** – Applies a voxel grid followed by a StatisticalOutlierRemoval within a configurable radius (e.g., ~2 m works well) to clean the point cloud before converting it into a laser scan. For navigation, `voxel_sor` improves localization by reducing outliers, but it performs poorly for mapping because the map builder expects a consistent, full environment.  
  2. **region_filter** – Removes points based on position (e.g., base, ground, ceiling).  

  Mapping launch files use `region_filter`, while navigation uses `voxel_sor` (selected via the `filter_type` parameter). The reason for applying `voxel_sor` only within a certain distance is that farther points have a different distribution than closer ones so they tend to be more distant, and noise from closer points affects motion more significantly so one parameter doesn't fit close and far away points so we focus only on the closer points.  

  Dual-lidar filter nodes live in `stretch_core` (`region_dual_lidar_laserscan`, `voxel_dual_lidar_laserscan`, `voxel_dual_lidar_laserscan_RANSAC`), launched via `stretch_core/launch/dual_hesai.launch.py`.

- **LaserScan Topic:** The filtered laser scan is published to `/scan_filtered` with **Best Effort QoS** (not reliable).

- **Debugging Tips:** If a topic appears inactive in RViz2:
  1. Check that `ros2 topic echo` shows messages.  
  2. Verify that RViz is subscribed to the correct topic.  
  3. Run `ros2 topic info /topic_name -v` to inspect publishers, subscribers, and QoS. Ensure subscriber and publisher QoS match.

---


## License
For license information, please see the LICENSE files.
