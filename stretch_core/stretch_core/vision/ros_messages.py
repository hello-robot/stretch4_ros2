import time
from collections import deque

import cv2
import numpy as np
import ros2_numpy
from sensor_msgs.msg import PointField
from std_msgs.msg import Header
from sensor_msgs_py import point_cloud2
from array import array
from sensor_msgs.msg._compressed_image import CompressedImage
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time


def rotate_img_msg(img_msg, rotate_rgb_image_number_of_times):
    cv_image = ros2_numpy.numpify(img_msg)

    if cv_image is None:
        raise Exception("Could not convert the image msg into a numpy array.")

    cv_image = np.rot90(cv_image, k=rotate_rgb_image_number_of_times)
    img_msg = ros2_numpy.msgify(Image, cv_image, encoding=img_msg.encoding)
    img_msg.header.stamp = img_msg.header.stamp
    img_msg.header.frame_id = img_msg.header.frame_id
    return img_msg


def create_pointcloud_rgb_msg(rgb_image: np.ndarray, points: np.ndarray):
    x, y, z = points.T
    r, g, b = rgb_image.T

    rgb = (r.astype(np.uint32) << 16) | (g.astype(np.uint32) << 8) | b.astype(np.uint32)

    cloud_dtype = np.dtype([("x", "f4"), ("y", "f4"), ("z", "f4"), ("rgb", "u4")])
    cloud_data = np.empty(z.size, dtype=cloud_dtype)

    # Fill the structured array by column
    cloud_data["x"] = x
    cloud_data["y"] = y
    cloud_data["z"] = z
    cloud_data["rgb"] = rgb

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.UINT32, count=1),
    ]

    header = Header()
    cloud_msg = point_cloud2.create_cloud(header, fields, cloud_data)

    return cloud_msg


def depth_image_msg_to_numpy(msg):
    dtype_class, channels = (np.uint16, 1)
    dtype = np.dtype(dtype_class)
    dtype = dtype.newbyteorder(">" if msg.is_bigendian else "<")
    shape = (msg.height, msg.width, channels)

    data = np.frombuffer(bytes(msg.data), dtype=dtype).reshape(shape)
    data.strides = (msg.step, dtype.itemsize * channels, dtype.itemsize)

    if channels == 1:
        data = data[..., 0]
    return data


__COMPRESSED_DEPTH_16UC1_HEADER = array("B", [0] * 12)
"""image_transport's compressedDepth ConfigHeader, which prefixes the PNG in every compressedDepth
message: `{compressionFormat format; float depthParam[2];}`. The quantization parameters are only
used for 32FC1 depth, so a 16-bit map leaves the whole header zeroed."""

COMPRESSED_DEPTH_FORMAT = "16UC1; compressedDepth png"
"""`CompressedImage.format` for a 16-bit depth map PNG encoded the way image_transport does it."""


def compressed_depth_payload(depth_millimeters: np.ndarray, png_compression_level: int = 1) -> array:
    """The bytes of a compressedDepth message: the 12-byte ConfigHeader followed by a PNG of the map.

    `depth_millimeters` has to be a 16-bit map. PNG is lossless, so the depth values survive exactly,
    at roughly half the bytes of the raw image. `png_compression_level` trades encode time for size;
    the default is the fast end, because this runs once per frame at the camera's frame rate.
    """
    if depth_millimeters.dtype != np.uint16:
        raise ValueError(f"compressedDepth carries a 16-bit depth map, got {depth_millimeters.dtype}.")

    is_encoded, encoded_image = cv2.imencode(
        ".png", depth_millimeters, [cv2.IMWRITE_PNG_COMPRESSION, png_compression_level]
    )
    if not is_encoded:
        raise RuntimeError("Failed to PNG encode a depth map.")

    return __COMPRESSED_DEPTH_16UC1_HEADER + array("B", encoded_image.tobytes())


def compress_depth_image(frame: np.ndarray):
    """
    Converts a F32 depth map in meters to a U16 map in millimeters
    """
    normalized_array = (frame * 1000).astype(np.uint16)

    ros_image_compressed = CompressedImage()
    ros_image_compressed.format = "16uc1; compressedDepth"
    ros_image_compressed.data = compressed_depth_payload(normalized_array)

    return ros_image_compressed


def create_timestamp(epoch_seconds:float):
    stamp = Time()
    stamp.sec = int(epoch_seconds)
    stamp.nanosec = int((epoch_seconds - stamp.sec) * 1e9)
    return stamp


class DeviceClockOffset:
    """Maps Luxonis frame timestamps onto the ROS system clock.

    Device timing is kept because it is far steadier than host arrival time; it is
    only shifted onto the system clock.
    """

    def __init__(self, window: int = 300):
        self._candidates = deque(maxlen=window)

    def to_ros(self, device_seconds: float, system_seconds: float | None = None) -> float:
        if system_seconds is None:
            system_seconds = time.time()
        candidate = system_seconds - device_seconds
        if abs(candidate) < 1.0:
            # Already on the system clock, leave it alone.
            return device_seconds
        self._candidates.append(candidate)
        return device_seconds + min(self._candidates)