
import threading
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
from scipy.spatial.transform import Rotation as R
from dataclasses import dataclass
from typing import Generator
import time

@dataclass
class TransformsFrame:
    """A container for transform data and ros2 message timestamp."""
    transform_4x4: np.ndarray
    timestamp: float

def transform_to_list(stamped_transform):
    """Helper to extract translation and rotation list from TransformStamped."""
    t = stamped_transform.transform.translation
    r = stamped_transform.transform.rotation
    return [t.x, t.y, t.z], [r.x, r.y, r.z, r.w]

def to_matrix(translation, rotation):
    """Helper to convert translation list and quaternion list to 4x4 matrix."""
    mat = np.eye(4)
    mat[:3, 3] = translation
    mat[:3, :3] = R.from_quat(rotation).as_matrix()
    return mat

class TfBufferNode(Node):
    """
    A Node specifically for holding the TF Buffer and Listener.
    """
    def __init__(self):
        super().__init__('tf_generator_node')
        self.tf2_buffer = Buffer()
        self.tf2_listener = TransformListener(self.tf2_buffer, self)

def tf_stream_blocking(target_frame: str, base_frame: str = "base_link") -> Generator[TransformsFrame, None, None]:
    """
    Generator that yields a TransformsFrame of target_frame in base_frame coordinates.
    Blocks indefinitely until a strictly newer timestamp is fetched.
    """
    for frame in tf_stream(target_frame, base_frame, block=True, timeout=None):
        if frame is not None:
            yield frame

def tf_stream(target_frame: str, base_frame: str = "base_link", timeout: float | None=10.0, block: bool = False) -> Generator[TransformsFrame | None, None, None]:
    """
    Generator that optionally yields a TransformsFrame of target_frame in base_frame coordinates.

    Args:
        target_frame (str): The frame you want to track.
        base_frame (str): The reference frame (default: "base_link").
        block (bool): If True, blocks until a new transform is available.
        timeout (float | None): Max seconds to block before yielding None.

    Yields:
        TransformsFrame | None
    """
    if not rclpy.ok():
        rclpy.init()

    # 1. Setup Node and TF Buffer
    node = TfBufferNode()
    
    # 2. Spin in background thread so the buffer fills up asynchronously
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()
    
    last_stamp = 0.0
    start_time = None

    try:
        while rclpy.ok():
            new_frame = None
            
            # Use native C++ conditional blocking if we want to wait for a *new* frame
            if block and last_stamp > 0.0:
                sec = int(last_stamp)
                nsec = int((last_stamp - sec) * 1e9) + 1
                if nsec >= 1_000_000_000:
                    sec += 1
                    nsec -= 1_000_000_000
                lookup_time = rclpy.time.Time(seconds=sec, nanoseconds=nsec)
                
                wait_duration = timeout if timeout is not None else 1000000.0 # Wait forever practically
                duration_obj = Duration(seconds=wait_duration)
            else:
                lookup_time = rclpy.time.Time() # get latest instantly
                duration_obj = Duration(seconds=0)
            
            try:
                stamped_transform = node.tf2_buffer.lookup_transform(
                    base_frame, 
                    target_frame, 
                    lookup_time, 
                    duration_obj
                )
                stamp = stamped_transform.header.stamp.sec + stamped_transform.header.stamp.nanosec * 1e-9
                
                if stamp > last_stamp:
                    trans, rot = transform_to_list(stamped_transform)
                    pose_mat = to_matrix(trans, rot)
                    new_frame = TransformsFrame(transform_4x4=pose_mat, timestamp=stamp)
                    last_stamp = stamp
            except (LookupException, ConnectivityException, ExtrapolationException):
                pass
                
            if new_frame is not None:
                yield new_frame
                continue
                
            if not block:
                yield None
                continue
                
            if timeout is not None:
                # The duration_obj natively blocked up to the timeout. If we are here, it means we timed out.
                yield None
                continue

    except GeneratorExit:
        pass
        
    finally:
        node.destroy_node()
        executor.shutdown()
        spin_thread.join()