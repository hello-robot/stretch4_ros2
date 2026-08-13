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


def compress_depth_image(frame: np.ndarray):
    """
    Converts a F32 depth map in meters to a U16 map in millimeters
    """
    normalized_array = (frame * 1000).astype(np.uint16)

    _, encoded_image = cv2.imencode(".png", normalized_array)

    ros_image_compressed = CompressedImage()
    ros_image_compressed.format = "16uc1; compressedDepth"
    ros_image_compressed.data = __COMPRESSED_DEPTH_16UC1_HEADER + array(
        "B", encoded_image.tobytes()
    )

    return ros_image_compressed


def create_timestamp(epoch_seconds:float):
    stamp = Time()
    stamp.sec = int(epoch_seconds)
    stamp.nanosec = int((epoch_seconds - stamp.sec) * 1e9)
    return stamp