import threading
import queue
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image
import ros2_numpy # Requires ros2_numpy package
import numpy as np
from dataclasses import dataclass
from typing import Generator

SEQUENCE_NUMBER_KEY = "seq"
"""Key used to carry a sensor sequence number inside CompressedImage.format. See compressed_format_with_sequence()."""


@dataclass
class ImageFrame:
    """A container for image data and ros2 message timestamp."""
    image: np.ndarray
    timestamp: float

    frame_number: int | None = None
    """The sensor's sequence number, when the publisher provided one. None on topics that cannot carry it."""

    compression_format: str = ""
    """Empty for raw images. Otherwise the codec of `image`, e.g. "jpeg", which the consumer has to decode."""

    def is_compressed(self) -> bool:
        return self.compression_format != ""


def compressed_format_with_sequence(compression_format: str, sequence_number: int | None) -> str:
    """Builds a CompressedImage.format string that also carries the sensor's sequence number.

    ROS 2 removed `header.seq` and sensor_msgs/CompressedImage has nowhere else to put a frame
    counter, so publishers append it to the format string, e.g. "jpeg; seq=1234". Consumers that only
    care about the codec (rviz, image_transport) read the leading token and ignore the remainder.
    """
    if sequence_number is None:
        return compression_format
    return f"{compression_format}; {SEQUENCE_NUMBER_KEY}={sequence_number}"


def parse_compressed_format(format_string: str) -> tuple[str, int | None]:
    """The inverse of compressed_format_with_sequence(): returns (compression_format, sequence_number)."""
    compression_format_parts = []
    sequence_number = None

    for part in format_string.split(";"):
        part = part.strip()
        if part.startswith(f"{SEQUENCE_NUMBER_KEY}="):
            try:
                sequence_number = int(part.split("=", 1)[1])
            except ValueError:
                sequence_number = None
        elif part:
            compression_format_parts.append(part)

    return "; ".join(compression_format_parts), sequence_number


def compressed_image_message_to_frame(msg) -> ImageFrame:
    """Wraps a CompressedImage as an ImageFrame *without* decoding it.

    `image` stays the 1-D encoded buffer, which is the whole point: decoding is the caller's choice
    (and its cost), and nothing has to move megabytes of raw pixels to get here.
    """
    compression_format, sequence_number = parse_compressed_format(msg.format)

    return ImageFrame(
        image=np.frombuffer(msg.data, dtype=np.uint8).copy(),
        timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
        frame_number=sequence_number,
        compression_format=compression_format or "jpeg",
    )


class ImageBufferNode(Node):
    def __init__(self, topic_name, data_queue, is_compressed: bool = False):
        super().__init__('image_generator_node')
        self.data_queue = data_queue
        self.is_compressed = is_compressed

        self.subscription = self.create_subscription(
            CompressedImage if is_compressed else Image,
            topic_name,
            self.listener_callback,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        )

    def listener_callback(self, msg):
        try:
            if self.is_compressed:
                frame = compressed_image_message_to_frame(msg)
            else:
                # ros2_numpy.numpify creates a numpy view of the message
                # We use .copy() to ensure we own the data if the msg gets garbage collected
                frame = ImageFrame(
                    image=ros2_numpy.numpify(msg).copy(),
                    timestamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
                )

            # If the queue is full, remove the old frame to make room for the new one
            # (Ensures next() always gets the freshest data)
            if self.data_queue.full():
                try:
                    self.data_queue.get_nowait()
                except queue.Empty:
                    pass

            self.data_queue.put(frame)
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

def compressed_image_stream_blocking(topic_name: str) -> Generator[ImageFrame, None, None]:
    """
    Generator that yields still-encoded ImageFrame objects from a ROS 2 CompressedImage topic.
    Blocks indefinitely until a new message arrives.
    """
    for frame in compressed_image_stream(topic_name, block=True, timeout=None):
        if frame is not None:
            yield frame

def compressed_image_stream(topic_name: str, timeout: float | None, block: bool = False) -> Generator[ImageFrame | None, None, None]:
    """
    Generator that optionally yields still-encoded ImageFrame objects from a CompressedImage topic.
    """
    return image_stream(topic_name, timeout=timeout, block=block, is_compressed=True)

def image_stream(topic_name: str, timeout: float | None, block: bool = False, is_compressed: bool = False) -> Generator[ImageFrame | None, None, None]:
    """
    Generator that optionally yields ImageFrame objects. Can be made non-blocking or having a timeout.
    """
    if not rclpy.ok():
        rclpy.init()

    # Maxsize 1 ensures we only keep the latest frame
    img_queue = queue.Queue(maxsize=1)

    node = ImageBufferNode(topic_name, img_queue, is_compressed=is_compressed)
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

