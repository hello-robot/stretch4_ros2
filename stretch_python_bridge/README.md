# Stretch Python Bridge

The `stretch_python_bridge` package provides Python generators that allow simple, synchronous access to asynchronous ROS 2 topics like images, pointclouds, and TF transforms. This is particularly useful for quickly writing scripts or for integration into standard python loop structures without the need for explicitly setting up ROS2 subscriptions and callbacks for topics.

## Prerequisites

For the streams to work properly, ensure the primary robot nodes and Zenoh router are running in the background:

```bash
ros2 run rmw_zenoh_cpp rmw_zenohd
ros2 launch stretch_core stretch_driver.launch.py
ros2 launch stretch_core dual_hesai.launch.py
ros2 launch stretch_core luxonis.launch.py
ros2 launch stretch_core gripper_camera.launch.py
```

## Features

- **Generators for ROS 2 Topics**: Access the latest message from a topic directly without defining subscriptions or explicit callbacks.
- **Image Bridge**: Convert ROS 2 `sensor_msgs/Image` topis to NumPy arrays seamlessly. Yields a numpy array.
- **PointCloud2 Bridge**: Convert ROS 2 `sensor_msgs/PointCloud2` streams to structured NumPy arrays, making it easy to access fields like x, y, z, and intensity. Yields a numpy array.
- **Transforms Bridge**: Query TF2 transforms asynchronously and yield the 4x4 homogenous transformation matrix from a base frame to a target frame. Yields a 4x4 tracking numpy ndarray.
- **CameraInfo Bridge**: Convert `sensor_msgs/CameraInfo` streams to fetch identical timestamped intrinsic parameter arrays cleanly.
- **IMU Bridge**: Convert `sensor_msgs/Imu` streams to fetch identical timestamped IMU data arrays cleanly.

## Installation

This package is a standard `ament_python` package. Compile the ROS workspace with `colcon build`:

```bash
cd ~/ament_ws
colcon build --packages-select stretch_python_bridge
source install/setup.bash
```

## Typical usage

## Prepared Methods for Known Topics

To make scripts exceptionally concise, all primary robot cameras and lidars have prepared wrapper methods inside the module export that stream the payloads locally. They natively support `(timeout: float | None = 10.0, blocking: bool = True)`.

- `stream_camera_center()` → `/cameras_head/center/image_raw`
- `stream_camera_center_rotated()` → `/cameras_head/center/rotated_image`
- `stream_camera_center_info()` → `/cameras_head/center/camera_info`
- `stream_camera_left()` → `/cameras_head/left/image_raw`
- `stream_camera_left_rotated()` → `/cameras_head/left/rotated_image`
- `stream_camera_left_info()` → `/cameras_head/left/camera_info`
- `stream_camera_right()` → `/cameras_head/right/image_raw`
- `stream_camera_right_rotated()` → `/cameras_head/right/rotated_image`
- `stream_camera_right_info()` → `/cameras_head/right/camera_info`
- `stream_lidar_points_left()` → `/lidar_points_left`
- `stream_lidar_points_right()` → `/lidar_points_right`
- `stream_gripper_imu()` → `/cameras_gripper/imu/data`
- `stream_gripper_right_info()` → `/cameras_gripper/right/camera_info`
- `stream_gripper_right()` → `/cameras_gripper/right/image_raw`
- `stream_gripper_stereo_info()` → `/cameras_gripper/stereo/camera_info`
- `stream_gripper_stereo()` → `/cameras_gripper/stereo/image_raw`
- `stream_gripper_stereo_points()` → `/cameras_gripper/stereo_left_rgbd/points`

```
import cv2
from stretch_python_bridge import *

for rgb_frame in stream_camera_right_rotated():
    cv2.namedWindow("RGB Stream", cv2.WINDOW_NORMAL)
    if rgb_frame is not None:
        print(f"Got a frame with timestamp: {rgb_frame.timestamp}")
        cv2.imshow("RGB Stream", rgb_frame.image)
        cv2.waitKey(1)
```

```
import rerun as rr
import numpy as np
from stretch_python_bridge import *

rr.init("pointcloud_stream", spawn=True)
for pc_frame in stream_lidar_points_left():
    if pc_frame is not None:
        rr.log("pointcloud", rr.Points3D(pc_frame.points))
```

### Examples:
- [Lidar Pointcloud with intensity coloring](examples/lidar_pointcloud.py)
- [Gripper Colored Pointcloud](examples/gripper_pointcloud.py)
- [Right camera image with cv2](examples/right_camera_image.py)

### 1. Typical usage using Stream Manager 

If your script requires monitoring multiple ROS topics simultaneously (e.g., streaming images from 3 cameras and a TF listener at the same time), using the individual generator methods will spawn heavy background threads for each topic. 

Instead, use `StreamManager` to monitor an unlimited number of topics using only a single internal execution thread.

```python
from stretch_python_bridge import StreamManager
from stretch_python_bridge import *

def main():
    # 1. Initialize the manager
    manager = StreamManager()
    
    # 2. Pass the manager to the stream generators to use the manager.
    center_stream = stream_camera_center(stream_manager=manager)
    lidar_stream = stream_lidar_points_left(stream_manager=manager)

    # You can also add as many subscriptions as you want explicitly:
    manager.add_tf_stream(target_frame="grasp_center_link", base_frame="base_link")
    manager.add_image_topic("/cameras_head/left/image_raw")
    
    print("Waiting for any data to arrive...")
    
    try:
        # 3. Use the stream as an iterable generator, this will still use the manager under-the-hood
        for frame in center_stream:
            if frame is not None:
                print(f"Got camera image at {frame.timestamp}")
                break
        
        # 4. you could also use `next()` on the generator
        camera_frame = next(center_stream)        

        # 5. Get a specific topic from the manager, with options for blocking and timeout:
        camera_frame = manager.get(center_stream, block=True, timeout=5.0)
        lidar_frame = manager.get(lidar_stream, block=True, timeout=5.0)

        # 6. Stream from the unified generator
        # If block is True, this will wait until AT LEAST ONE new frame arrives across the registered streams.
        for frames_dict in manager.stream(block=True):
            
            # frames_dict contains the absolute *latest* cached data for ALL topics
            # If a topic hasn't received any data yet, its value will be None

            # Access the topic using .get()
            left_camera_frame = frames_dict.get("/cameras_head/left/image_raw")
            if left_camera_frame is not None:
                print(f"Got left camera image at {left_camera_frame.timestamp}")
            
            # Use the returned generators to cleanly retrieve the frames
            camera_frame = next(center_gen)
            if camera_frame is not None:
                print(f"Got camera image at {camera_frame.timestamp}")
                
            # Access the TF transform
            tf_data = frames_dict.get("grasp_center_link")
            if tf_data is not None:
                x = tf_data.transform_4x4[0, 3]
                print(f"Got Grasp Center X: {x:.3f}")
                
    except KeyboardInterrupt:
        pass
    finally:
        # Important: shut down the internal threads cleanly
        manager.close()

if __name__ == '__main__':
    main()
```

## For custom topics, you can use your own stream generators


### Blocking vs Non-blocking Generators

The bridging functions are split into two variants for each data type:
- **Blocking Generators** (e.g. `image_stream_blocking`, `tf_stream_blocking`): Guarantee a non-`None` return. Calling `next()` or using them in a `for` loop will block indefinitely until a *new* frame or message arrives.
- **Non-blocking Generators** (e.g. `image_stream`): Can be configured to be completely non-blocking, or blocking with a specific timeout window. They yield `Frame | None`.


### 1. Image Stream Example

Use the `image_stream_blocking` to effortlessly fetch frames from an active image topic.

```python
import cv2
from stretch_python_bridge import image_stream_blocking

def main():
    rgb_topic = "/cameras_head/center/image_raw"
    rgb_gen = image_stream_blocking(rgb_topic)

    print(f"Subscribed to {rgb_topic}. Press CTRL+C to stop.")

    try:
        # Continuously fetch the newest image frame 
        for rgb_frame in rgb_gen:
            cv2.imshow("RGB Stream", rgb_frame.image)
            key = cv2.waitKey(1)
            if key == 27:
                break
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
```

### 2. Pointcloud Stream Example

Use the `pointcloud_stream_blocking` generator to get the most recent pointcloud as a structured Numpy array. Given the structured output, you can query specific fields using keys.

```python
from stretch_python_bridge import pointcloud_stream_blocking

def main():
    pc_topic = '/lidar_points_left'
    pc_gen = pointcloud_stream_blocking(pc_topic)

    print(f"Waiting for pointclouds on {pc_topic}...")
    
    # Grab just the very first available pointcloud frame
    try:
        first_cloud = next(pc_gen) 
        
        print("\nSuccessfully received 1 point cloud!")
        print(f"Type: {type(first_cloud.points)}")
        print(f"Shape: {first_cloud.points.shape}")
        
        # Access structured fields
        x_vals = first_cloud.points['x']
        y_vals = first_cloud.points['y']
        z_vals = first_cloud.points['z']
        intensity_vals = first_cloud.points['intensity']
        
        print(f"First point: x={x_vals[0]:.3f}, y={y_vals[0]:.3f}, z={z_vals[0]:.3f}, intensity={intensity_vals[0]:.3f}")
        
    except StopIteration:
        print("Generator stopped unexpectedly.")

if __name__ == '__main__':
    main()
```

### 3. Transforms Stream Example

Use the `tf_stream` to track the geometric relationship and 6DoF pose of arbitrary frames. The matrix yielded is a 4x4 affine homogenous transformation containing rotation and translation data.

```python
import numpy as np
from stretch_python_bridge import tf_stream

def main():
    target_frame = "grasp_center_link"
    base_frame = "base_link"

    # Creates a generator that yields 4x4 matrices 5 times a second
    pose_gen = tf_stream(target_frame, base_frame=base_frame)

    print(f"Tracking {target_frame} with respect to {base_frame}")

    try:
        for pose_frame in pose_gen:
            if pose_frame is None:
                print("Transform lookup failed this tick...")
                continue
            
            # Extract out the translation components 
            x = pose_frame.transform_4x4[0, 3]
            y = pose_frame.transform_4x4[1, 3]
            z = pose_frame.transform_4x4[2, 3]
            
            print(f"[{target_frame} pose in {base_frame}]: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")

    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
```

## Frame Dataclasses

All generator wrappers from this bridge yield native Python dataclasses, providing clean and strongly-typed interfaces to underlying ROS 2 messages.

Here is the structure and available bindings for each underlying datum:

### `ImageFrame`
Returned by `image_stream`, `stream_camera_center`, etc.
- `image: np.ndarray`: A standard NumPy array representing the image data. Its dimensions and encoding match the original topic config.
- `timestamp: float`: The absolute floating-point timestamp acquired from the ROS 2 message header (`sec + nanosec * 1e-9`).

### `PointCloudFrame`
Returned by `pointcloud_stream`, `stream_lidar_points_left`, etc.
- `points: np.ndarray`: A structured NumPy array output from `ros2_numpy`. Common accessible channels include `.points['x']`, `.points['y']`, `.points['z']`, and `.points['intensity']`.
- `timestamp: float`: The absolute floating-point timestamp acquired from the ROS 2 message header.

### `TransformsFrame`
Returned by `tf_stream`.
- `transform_4x4: np.ndarray`: A 4x4 homogenous affine transformation matrix containing the resolved translation and rotation from the queried base frame to target frame. 
- `timestamp: float`: The absolute floating-point timestamp acquired from the ROS 2 TF buffer.

### `ImuFrame`
Returned by `imu_stream`, `stream_gripper_imu`.
- `orientation: np.ndarray`: 1D array natively ordered as `[x, y, z, w]`.
- `angular_velocity: np.ndarray`: 1D array ordered as `[x, y, z]`.
- `linear_acceleration: np.ndarray`: 1D array ordered as `[x, y, z]`.
- `timestamp: float`: The absolute floating-point timestamp acquired from the ROS 2 message header.

### `CameraInfoFrame`
Returned by `camera_info_stream`, `stream_camera_center_info`, etc.
- `camera_matrix: np.ndarray`: A 3x3 NumPy array of the intrinsic `K` parameters.
- `distortion_coefficients: np.ndarray`: A 1D NumPy array covering the `D` vector.
- `distortion_model: str`: The specific distortion model (e.g. `plumb_bob`) utilized by this camera logic.
- `timestamp: float`: The absolute floating-point timestamp acquired from the ROS 2 message header.
