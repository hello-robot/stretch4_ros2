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
- `cameras:=left` for only the left camera. 
- `cameras:=right` for only the right camera. 
- `cameras:=center` for only the center camera. 
- `cameras:=left,right` for only the left/right cameras.
- `cameras:=all` for all the cameras.

- `publish_markers` is `false` by default. You can view the ArUco markers as TF frames if `publish_markers` is `false`.

## Known Aruco Tags

- [stretch_marker_dict.yaml](./config/stretch_marker_dict.yaml) contains all the markers on Stretch 4 or with official accessories, such as the Docking Station.
- [user_aruco_dict.yaml] contains some markers that ship in Stretch 4's accessories box, and is a file where you could add your own marker id's and sizes.