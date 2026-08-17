import threading
import queue
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo
import numpy as np
from dataclasses import dataclass
from typing import Generator

@dataclass
class CameraInfoFrame:
    """A container for camera info data and ros2 message timestamp."""
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    distortion_model: str
    timestamp: float

class CameraInfoBufferNode(Node):
    def __init__(self, topic_name, data_queue):
        super().__init__('camera_info_generator_node')
        self.data_queue = data_queue
        
        self.subscription = self.create_subscription(
            CameraInfo,
            topic_name,
            self.listener_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

    def listener_callback(self, msg):
        try:
            k_matrix = np.array(msg.k).reshape((3, 3))
            d_coeffs = np.array(msg.d)
            
            if self.data_queue.full():
                try:
                    self.data_queue.get_nowait()
                except queue.Empty:
                    pass
            
            self.data_queue.put(
                CameraInfoFrame(
                    camera_matrix=k_matrix,
                    distortion_coefficients=d_coeffs,
                    distortion_model=msg.distortion_model,
                    timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                )
            )
        except Exception as e:
            self.get_logger().error(f'CameraInfo Conversion error: {e}')

def camera_info_stream_blocking(topic_name: str) -> Generator[CameraInfoFrame, None, None]:
    """
    Generator that yields CameraInfoFrame objects from a ROS 2 CameraInfo topic.
    Blocks indefinitely until a new message arrives.
    """
    for frame in camera_info_stream(topic_name, block=True, timeout=None):
        if frame is not None:
            yield frame

def camera_info_stream(topic_name: str, timeout: float | None = 10.0, block: bool = False) -> Generator[CameraInfoFrame | None, None, None]:
    """
    Generator that optionally yields CameraInfoFrame objects. Can be made non-blocking or having a timeout.
    """
    if not rclpy.ok():
        rclpy.init()

    # Maxsize 1 ensures we only keep the latest frame
    info_queue = queue.Queue(maxsize=1)
    
    node = CameraInfoBufferNode(topic_name, info_queue)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        while True:
            try:
                yield info_queue.get(block=block, timeout=timeout)
            except queue.Empty:
                yield None
            
    except GeneratorExit:
        pass
        
    finally:
        node.destroy_node()
        executor.shutdown()
        spin_thread.join()
