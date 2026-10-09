from __future__ import annotations
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
import threading
import queue
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
import ros2_numpy
import numpy as np
from dataclasses import dataclass
from typing import Generator



@dataclass
class PointCloudFrame:
    """A container for point cloud data and ros2 message timestamp."""
    points: list[tuple[float, float, float]] # x, y, z
    timestamp: float

    @staticmethod
    def from_pointcloud2(pc_array: np.ndarray, timestamp:float) -> LidarPointCloudFrame|StereoPointCloudFrame|PointCloudFrame:

        if pc_array.dtype.names is not None:
            if 'intensity' in pc_array.dtype.names:
                return LidarPointCloudFrame.from_lidar_pointcloud2(pc_array, timestamp)
            elif 'rgb' in pc_array.dtype.names:
                return StereoPointCloudFrame.from_stereo_pointcloud2(pc_array, timestamp)
            
        return PointCloudFrame(
            points=pc_array,
            timestamp=timestamp
        )

@dataclass
class LidarPointCloudFrame(PointCloudFrame):
    intensity: list[float] # intensity

    @staticmethod
    def from_lidar_pointcloud2(pc_array: np.ndarray, timestamp: float) -> 'PointCloudFrame':
        """
        The logic for unpacking the lidar point cloud assumes the point cloud is from a Hesai lidar.
        """
        points = np.stack([
            pc_array['x'],
            pc_array['y'],
            pc_array['z']
        ], axis=-1).astype(np.float32).reshape(-1, 3)
        intensity = pc_array['intensity']
        return LidarPointCloudFrame(
            points=points,
            intensity=intensity,
            timestamp=timestamp
        )

@dataclass
class StereoPointCloudFrame(PointCloudFrame):
    colors: list[tuple[float, float, float]] # r, g, b

    @staticmethod
    def from_stereo_pointcloud2(pc_array: np.ndarray, timestamp: float) -> 'PointCloudFrame':
        """
        The logic for unpacking the stereo point cloud assumes the point cloud is from a Luxonis stereo camera.
        """
        
        # Stack and flatten positions
        points = np.stack([
            pc_array['x'],
            pc_array['y'],
            pc_array['z']
        ], axis=-1).astype(np.float32).reshape(-1, 3)

        # Unpack colors
        packed_rgb = pc_array['rgb'].flatten()
        rgb_int = packed_rgb.view(np.uint32)
        r = ((rgb_int >> 16) & 255).astype(np.uint8)
        g = ((rgb_int >> 8) & 255).astype(np.uint8)
        b = (rgb_int & 255).astype(np.uint8)
        colors = np.stack([r, g, b], axis=-1)
        
        valid_mask = np.isfinite(points[:, 2]) & (points[:, 2] > 0.0)

        clean_points = points[valid_mask]
        clean_colors = colors[valid_mask]

        return StereoPointCloudFrame(
            points=clean_points,
            colors=clean_colors,
            timestamp=timestamp
        )

@dataclass
class RGBDPointCloudFrame(PointCloudFrame):
    colors: list[tuple[float, float, float]] # r, g, b
    intensity: list[float] # intensity
    

class PointCloudBufferNode(Node):
    def __init__(self, topic_name, data_queue):
        super().__init__('pc_generator_node')
        self.data_queue = data_queue
        
        self.subscription = self.create_subscription(
            PointCloud2,
            topic_name,
            self.listener_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

    def listener_callback(self, msg):
        """
        There is some baked-in logic here where we try to infer the type of point cloud
        based on the fields in the point cloud message.
        For example, if the point cloud message has an 'intensity' field, we assume it is a lidar point cloud.
        If it has an 'rgb' field, we assume it is a stereo point cloud.
        The logic for unpacking those are hard-coded for the sensors we're using now (Luxonis stereo, Hesai lidar).
        """
        try:
            pc_array = ros2_numpy.numpify(msg)
            timestamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            
            if self.data_queue.full():
                try:
                    self.data_queue.get_nowait()
                except queue.Empty:
                    pass
                
            self.data_queue.put(PointCloudFrame.from_pointcloud2(pc_array, timestamp))
        except Exception as e:
            self.get_logger().error(f'PC Conversion error: {e}')

def pointcloud_stream_blocking(topic_name: str) -> Generator[PointCloudFrame, None, None]:
    """
    Generator that yields a PointCloudFrame object. Blocks indefinitely until a new message arrives.
    """
    for frame in pointcloud_stream(topic_name, block=True, timeout=None):
        if frame is not None:
            yield frame

def pointcloud_stream(topic_name: str, timeout: float | None, block: bool = False) -> Generator[PointCloudFrame | None, None, None]:
    """
    Generator that optionally yields a PointCloudFrame.

    Returned point tuples are in the format (x, y, z, intensity).
    """
    if not rclpy.ok():
        rclpy.init()

    pc_queue = queue.Queue(maxsize=1)
    
    node = PointCloudBufferNode(topic_name, pc_queue)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        while True:
            try:
                yield pc_queue.get(block=block, timeout=timeout)
            except queue.Empty:
                yield None
            
    except GeneratorExit:
        pass
        
    finally:
        node.destroy_node()
        executor.shutdown()
        spin_thread.join()
