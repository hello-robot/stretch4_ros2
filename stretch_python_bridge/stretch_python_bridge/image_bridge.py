import threading
import queue
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import ros2_numpy # Requires ros2_numpy package
import numpy as np
from dataclasses import dataclass
from typing import Generator

@dataclass
class ImageFrame:
    """A container for image data and ros2 message timestamp."""
    image: np.ndarray
    timestamp: float

class ImageBufferNode(Node):
    def __init__(self, topic_name, data_queue):
        super().__init__('image_generator_node')
        self.data_queue = data_queue
        
        self.subscription = self.create_subscription(
            Image,
            topic_name,
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            # ros2_numpy.numpify creates a numpy view of the message
            # We use .copy() to ensure we own the data if the msg gets garbage collected
            np_image = ros2_numpy.numpify(msg).copy()
            
            # If the queue is full, remove the old frame to make room for the new one
            # (Ensures next() always gets the freshest data)
            if self.data_queue.full():
                try:
                    self.data_queue.get_nowait()
                except queue.Empty:
                    pass
            
            self.data_queue.put(
                ImageFrame(
                    image=np_image,
                    timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
                )
            )
        except Exception as e:
            self.get_logger().error(f'Conversion error: {e}')

def image_stream_blocking(topic_name: str) -> Generator[ImageFrame, None, None]:
    """
    Generator that yields ImageFrame objects from a ROS 2 Image topic.
    Blocks indefinitely until a new message arrives.
    """
    for frame in image_stream(topic_name, block=True, timeout=None):
        if frame is not None:
            yield frame

def image_stream(topic_name: str, timeout: float | None, block: bool = False) -> Generator[ImageFrame | None, None, None]:
    """
    Generator that optionally yields ImageFrame objects. Can be made non-blocking or having a timeout.
    """
    if not rclpy.ok():
        rclpy.init()

    # Maxsize 1 ensures we only keep the latest frame
    img_queue = queue.Queue(maxsize=1)
    
    node = ImageBufferNode(topic_name, img_queue)
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    try:
        while True:
            try:
                yield img_queue.get(block=block, timeout=timeout)
            except queue.Empty:
                yield None
            
    except GeneratorExit:
        pass
        
    finally:
        node.destroy_node()
        executor.shutdown()
        spin_thread.join()

