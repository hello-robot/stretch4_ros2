import threading
import queue
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
from dataclasses import dataclass
from typing import Generator

@dataclass
class ImuFrame:
    """A container for IMU data and ros2 message timestamp."""
    orientation: np.ndarray
    angular_velocity: np.ndarray
    linear_acceleration: np.ndarray
    timestamp: float

class ImuBufferNode(Node):
    def __init__(self, topic_name, data_queue):
        super().__init__('imu_generator_node')
        self.data_queue = data_queue
        
        self.subscription = self.create_subscription(
            Imu,
            topic_name,
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            orientation = np.array([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
            angular_velocity = np.array([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            linear_acceleration = np.array([msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z])
            
            if self.data_queue.full():
                try:
                    self.data_queue.get_nowait()
                except queue.Empty:
                    pass
            
            self.data_queue.put(
                ImuFrame(
                    orientation=orientation,
                    angular_velocity=angular_velocity,
                    linear_acceleration=linear_acceleration,
                    timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                )
            )
        except Exception as e:
            self.get_logger().error(f'Imu Conversion error: {e}')

def imu_stream_blocking(topic_name: str) -> Generator[ImuFrame, None, None]:
    """
    Generator that yields ImuFrame objects from a ROS 2 Imu topic.
    Blocks indefinitely until a new message arrives.
    """
    for frame in imu_stream(topic_name, block=True, timeout=None):
        if frame is not None:
            yield frame

def imu_stream(topic_name: str, timeout: float | None = 10.0, block: bool = False) -> Generator[ImuFrame | None, None, None]:
    """
    Generator that optionally yields ImuFrame objects. Can be made non-blocking or having a timeout.
    """
    if not rclpy.ok():
        rclpy.init()

    # Maxsize 1 ensures we only keep the latest frame
    info_queue = queue.Queue(maxsize=1)
    
    node = ImuBufferNode(topic_name, info_queue)
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
