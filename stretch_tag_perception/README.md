# Stretch Tag Perception

This package allows you to detect ArUco markers using Stretch 4's cameras.

## Usage
Start stretch_driver in a terminal:

```bash
ros2 launch stretch_core stretch_driver.launch.py
```

Start the cameras launch file in another terminal:

```bash
ros2 launch stretch_core luxonis.launch.py use_center:=true use_left:=true use_right:=true
```

Start the tag perception launch file in another terminal:

```bash
ros2 launch stretch_tag_perception stretch_aruco.launch.py cameras:=all publish_markers:=true
```

### Parameters

When you run `ros2 launch stretch_tag_perception stretch_aruco.launch.py`, you can specify:
- `aruco_config_filepath`: `""` by default. Optional filepath to a YAML file containing custom/additional ArUco marker configuration parameters.
- `cameras`: Camera(s) to use for detection (`center` by default). Options include `left`, `right`, `center`, comma-separated lists like `left,right`, or `all`.
- `show_debug_images`: `false` by default. Set to `true` to display OpenCV debug image windows during detection.
- `use_rviz`: `false` by default. Set to `true` to launch RViz2 automatically with pre-configured tag visualization.
- `publish_markers`: `false` by default. Set to `true` to publish marker visualization array to `/aruco/marker_array`.

## Known Aruco Tags

- [stretch_marker_dict.yaml](./config/stretch_marker_dict.yaml) contains all the markers on Stretch 4 or with official accessories, such as the Docking Station.
- [user_aruco_dict.yaml] contains markers that ship in Stretch 4's accessories box, and is a file where you could add your own marker id's and sizes.