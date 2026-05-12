from dataclasses import asdict, dataclass
import datetime
import numpy as np
from stretch_core.vision.vision_topics import (
    VisionFrames,
    VisionTopics,
    get_camera_calibration_file_path,
)
import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

from stretch_core.vision.ros_messages import rotate_img_msg


@dataclass
class CalibrateCameraResults:
    """Stores the results of a camera calibration."""

    camera_name: str
    calibration_date: datetime.datetime
    image_size: list[int]  # e.g., [1920, 1200]
    number_of_images_processed: int
    number_of_images_used: int
    number_of_corresponding_points_used: int
    projection_error: float
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    projection_matrix: np.ndarray
    fisheye: bool
    rectification_matrix: np.ndarray

    @staticmethod
    def _serialize(dictionary: dict):
        return {
            (k): (v.tolist() if "tolist" in dir(v) else v)
            for k, v in dictionary.items()
        }

    def get_serializable(self):
        return CalibrateCameraResults._serialize(asdict(self))


class CameraInfoPublisher(Node):
    """
    This node publishes a topic with camera calibration information from a YAML file, and a rotated image topic with the same frame as defined in vision_topics.py's VisionFrames.
    """

    def __init__(self):
        super().__init__("camera_info_publisher")

        self.declare_parameter("camera_name", "left")

        self.camera_name = (
            self.get_parameter("camera_name").get_parameter_value().string_value
        )

        self.rotate_rgb_image_number_of_times = (
            VisionFrames.camera_frame_number_of_rotations(self.camera_name)
        )

        valid_sides = ["left", "right", "center"]
        if self.camera_name not in valid_sides:
            self.get_logger().warn(f"Expected one of {valid_sides}")

        self.yaml_filename = get_camera_calibration_file_path(self.camera_name)

        self.input_topic = VisionTopics.image_raw(self.camera_name)
        self.output_rotated_topic = VisionTopics.rotated_image(self.camera_name)
        self.output_info_topic = VisionTopics.camera_info(self.camera_name)

        self.sub_img = self.create_subscription(
            Image, self.input_topic, self.img_cb, 10
        )

        # Parse the YAML file
        self.get_logger().info(f"Loading {self.yaml_filename}")
        self.camera_info_msg = self.load_yaml_to_msg(self.yaml_filename)

        if self.camera_info_msg is None:
            raise Exception(f"Failed to load {self.yaml_filename}.")

        self.get_logger().info(f"Loaded {self.yaml_filename}")

        self.camera_info_msg.header.frame_id = VisionFrames.camera_frame(
            self.camera_name
        )

        if self.camera_info_msg.distortion_model == "fisheye":
            # fisheye needs to be converted to equidistant for image_proc rectify_node to work
            # Not sure if this is the best place to do this, but "fisheye" is more intuitive for the user in a calibration file and different apps expect different names for the same thing.
            self.camera_info_msg.distortion_model = "equidistant"

        latching_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

        self.info_publisher = self.create_publisher(
            CameraInfo, self.output_info_topic, latching_qos
        )
        self.rotated_img_publisher = self.create_publisher(
            Image, self.output_rotated_topic, 10
        )

    def img_cb(self, img_msg):
        # COPY the timestamp from Image to Info
        self.camera_info_msg.header.stamp = img_msg.header.stamp
        # Publish the synced info
        self.info_publisher.publish(self.camera_info_msg)

        if self.rotated_img_publisher.get_subscription_count() > 0:
            self.rotated_img_msg = rotate_img_msg(
                img_msg, self.rotate_rgb_image_number_of_times
            )
            self.rotated_img_msg.header.frame_id = self.camera_info_msg.header.frame_id
            self.rotated_img_msg.header.stamp = img_msg.header.stamp
            self.rotated_img_publisher.publish(self.rotated_img_msg)

    def load_yaml_to_msg(self, filename):
        """Reads the custom YAML format and converts it to a ROS 2 CameraInfo message."""
        try:
            with open(filename, "r") as f:
                calib_data = yaml.safe_load(f)
        except FileNotFoundError:
            self.get_logger().error(
                f"FATAL: Could not find file '{filename}' in current directory."
            )
            return None
        except Exception as e:
            self.get_logger().error(f"FATAL: Error parsing YAML: {e}")
            return None

        msg = CameraInfo()

        # Basic dimensions
        msg.width = calib_data.get("image_width", 0)
        msg.height = calib_data.get("image_height", 0)
        msg.distortion_model = calib_data.get("distortion_model", "plumb_bob")

        # Matrix mapping (YAML 'data' lists -> Message lists)
        # Distortion coefficients (D)
        if "distortion_coefficients" in calib_data:
            msg.d = calib_data["distortion_coefficients"]["data"]

        # Intrinsic Camera Matrix (K) - 3x3
        if "camera_matrix" in calib_data:
            msg.k = calib_data["camera_matrix"]["data"]

        # Rectification Matrix (R) - 3x3
        if "rectification_matrix" in calib_data:
            msg.r = calib_data["rectification_matrix"]["data"]

        # Projection Matrix (P) - 3x4
        if "projection_matrix" in calib_data:
            msg.p = calib_data["projection_matrix"]["data"]

        return msg


def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()

    # Spin is required to keep the process alive so late subscribers
    # can connect and receive the latched message.
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()