#!/usr/bin/env python3

"""
Interactive ROS 2 node and OpenCV GUI utility to trigger tag pose calibration.
"""

from enum import Enum, auto
import os
import sys
import threading
import time
from typing import List, Optional, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from cv_bridge import CvBridge, CvBridgeError

from tf2_ros import Buffer, TransformListener
from tf2_ros import TransformException


class CalibrationState(Enum):
    """Lifecycle states for tag calibration workflow."""
    IDLE = auto()
    CALIBRATING = auto()
    SUCCESS = auto()
    FAILED = auto()


def quaternion_to_yaw(q: List[float]) -> float:
    """
    Convert a quaternion [x, y, z, w] to a yaw angle in radians.

    Args:
        q: Quaternion list [x, y, z, w].

    Returns:
        Yaw angle in radians.
    """
    x, y, z, w = q
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    return float(np.arctan2(siny_cosp, cosy_cosp))


class CalibrateTagCli(Node):
    """Interactive ROS 2 node for ArUco tag visual feedback and pose calibration."""

    def __init__(self) -> None:
        super().__init__('calibrate_tag_cli')

        # ----------------------------------------------------------------------
        # Parameters
        # ----------------------------------------------------------------------
        self.declare_parameter('camera_name', 'center')
        self.declare_parameter('tag_id', 999)
        self.declare_parameter('target_frame', 'aruco_perception_localization_marker_center')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('rotate_90_cw', True)

        self.camera_name: str = self.get_parameter('camera_name').value
        self.tag_id: int = self.get_parameter('tag_id').value
        self.target_frame: str = self.get_parameter('target_frame').value
        self.base_frame: str = self.get_parameter('base_frame').value
        self.map_frame: str = self.get_parameter('map_frame').value
        self.rotate_90_cw: bool = self.get_parameter('rotate_90_cw').value

        self.get_logger().info("Initializing Tag Calibration Node...")

        # ----------------------------------------------------------------------
        # State Variables & Thread Locking
        # ----------------------------------------------------------------------
        self._lock = threading.Lock()
        self._running = True
        self._camera_connected = False
        self._tag_seen = False
        self._last_seen_time = 0.0
        self._latest_frame: Optional[np.ndarray] = None

        self._calibration_state = CalibrationState.IDLE
        self._calibration_error = ""
        self._success_start_time = 0.0
        self._fail_start_time = 0.0
        self._lookup_needed = False
        self._lookup_start_time = 0.0

        # ----------------------------------------------------------------------
        # ROS 2 Interfaces & Tools
        # ----------------------------------------------------------------------
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # ArUco Detector Setup
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_1000)
        self.aruco_params = aruco.DetectorParameters()
        self.aruco_params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.aruco_detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Non-blocking service client
        self.srv_client = self.create_client(Trigger, '/calibrate_tag_pose')

        # Subscriber (using SensorDataQoS for optimal video stream handling)
        img_topic = f"/cameras_head/{self.camera_name}/image_raw"
        self.img_sub = self.create_subscription(
            Image,
            img_topic,
            self._image_callback,
            qos_profile=qos_profile_sensor_data
        )

        self._print_instructions(img_topic)

    @property
    def is_running(self) -> bool:
        """Thread-safe getter for running flag."""
        with self._lock:
            return self._running

    @is_running.setter
    def is_running(self, value: bool) -> None:
        """Thread-safe setter for running flag."""
        with self._lock:
            self._running = value

    @property
    def lookup_needed(self) -> bool:
        """Thread-safe getter for lookup flag."""
        with self._lock:
            return self._lookup_needed

    def _print_instructions(self, img_topic: str) -> None:
        """Print startup instructions to standard output."""
        self.get_logger().info(f"Subscribed to camera topic: {img_topic}")
        self.get_logger().info("=" * 60)
        self.get_logger().info("INSTRUCTIONS:")
        self.get_logger().info("  * Adjust robot/tag until the tag is outlined in GREEN.")
        self.get_logger().info("  * Press 'c' or SPACE in the window, or ENTER in terminal to calibrate.")
        self.get_logger().info("  * Press ESC in window or Ctrl+C in terminal to exit.")
        self.get_logger().info("=" * 60)

    # --------------------------------------------------------------------------
    # Image Callback & Processing
    # --------------------------------------------------------------------------
    def _image_callback(self, msg: Image) -> None:
        """Process incoming camera frames and detect ArUco markers."""
        if not self._camera_connected:
            self._camera_connected = True
            self.get_logger().info("Camera feed connected! Display window active.")

        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.rotate_90_cw:
                cv_image = cv2.rotate(cv_image, cv2.ROTATE_90_CLOCKWISE)
        except CvBridgeError as e:
            self.get_logger().error(f"CvBridge Error: {e}")
            return

        # Perform ArUco Detection
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.aruco_detector.detectMarkers(gray)

        tag_detected = False
        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == self.tag_id:
                    tag_detected = True
                    self._draw_tag_overlay(cv_image, corners[i][0])

        # State updates (thread-safe)
        now = time.time()
        with self._lock:
            if tag_detected:
                self._tag_seen = True
                self._last_seen_time = now
            elif now - self._last_seen_time > 0.5:
                self._tag_seen = False

            tag_visible = self._tag_seen
            current_state = self._calibration_state

        # Scale down display image if needed
        cv_image = self._scale_image(cv_image, max_size=800)

        # Draw HUD overlay
        cv_image = self._draw_hud(cv_image, tag_visible, current_state)

        with self._lock:
            self._latest_frame = cv_image

    def _draw_tag_overlay(self, image: np.ndarray, marker_corners: np.ndarray) -> None:
        """Draw bounding box and floating text over detected marker."""
        pts = marker_corners.astype(np.int32)
        cv2.polylines(image, [pts], isClosed=True, color=(0, 255, 0), thickness=4)

        for pt in pts:
            cv2.circle(image, (pt[0], pt[1]), 6, (0, 255, 0), -1)

        min_x = int(np.min(marker_corners[:, 0]))
        min_y = int(np.min(marker_corners[:, 1]))
        label_pos = (min_x, max(30, min_y - 15))

        label_text = f"Localization Tag (ID {self.tag_id})"
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale, thickness = 0.8, 2

        (text_w, text_h), baseline = cv2.getTextSize(label_text, font, scale, thickness)
        bg_pt1 = (label_pos[0] - 5, label_pos[1] - text_h - 5)
        bg_pt2 = (label_pos[0] + text_w + 5, label_pos[1] + baseline + 5)

        overlay = image.copy()
        cv2.rectangle(overlay, bg_pt1, bg_pt2, (0, 0, 0), -1)
        cv2.addWeighted(overlay, 0.6, image, 0.4, 0, image)
        cv2.putText(image, label_text, label_pos, font, scale, (0, 255, 0), thickness, cv2.LINE_AA)

    def _scale_image(self, img: np.ndarray, max_size: int = 800) -> np.ndarray:
        """Rescale image maintaining aspect ratio if dimensions exceed max_size."""
        h, w = img.shape[:2]
        scale_factor = min(max_size / w, max_size / h, 1.0)
        if scale_factor < 1.0:
            new_size = (int(w * scale_factor), int(h * scale_factor))
            return cv2.resize(img, new_size, interpolation=cv2.INTER_AREA)
        return img

    def _draw_hud(self, img: np.ndarray, tag_visible: bool, state: CalibrationState) -> np.ndarray:
        """Append HUD bar below image showing current node state."""
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale, thickness = 0.7, 2

        (_, text_h), baseline = cv2.getTextSize("TEST", font, font_scale, thickness)
        line_height = text_h + baseline + 8
        bar_height = (line_height * 3) + 12
        h, w = img.shape[:2]

        black_bar = np.zeros((bar_height, w, 3), dtype=np.uint8)
        extended_img = np.vstack([img, black_bar])

        y1 = h + 12 + text_h
        y2 = y1 + line_height
        y3 = y2 + line_height

        # Camera & Tag Status
        cv2.putText(extended_img, f"CAMERA: {self.camera_name.upper()}", (15, y1), font, font_scale, (255, 255, 255), thickness, cv2.LINE_AA)

        if tag_visible:
            cv2.putText(extended_img, f"TAG {self.tag_id}: VISIBLE", (15, y2), font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)
        else:
            cv2.putText(extended_img, f"TAG {self.tag_id}: NOT VISIBLE", (15, y2), font, font_scale, (0, 0, 255), thickness, cv2.LINE_AA)

        # State Banners
        now = time.time()
        if state == CalibrationState.IDLE:
            text = "Press 'c' or SPACE to calibrate" if tag_visible else f"Position robot to see Tag {self.tag_id}"
            color = (0, 255, 0) if tag_visible else (0, 165, 255)
            cv2.putText(extended_img, text, (15, y3), font, font_scale, color, thickness, cv2.LINE_AA)

        elif state == CalibrationState.CALIBRATING:
            cv2.putText(extended_img, "CALIBRATING... HOLD STILL!", (15, y3), font, font_scale, (0, 255, 255), thickness, cv2.LINE_AA)
            cv2.rectangle(extended_img, (w // 4, h // 2 - 40), (3 * w // 4, h // 2 + 40), (0, 165, 255), -1)
            cv2.putText(extended_img, "CALIBRATING POSE", (w // 4 + 30, h // 2 + 10), font, 1.0, (0, 0, 0), 3, cv2.LINE_AA)

        elif state == CalibrationState.SUCCESS:
            cv2.putText(extended_img, "CALIBRATION SUCCESSFUL!", (15, y3), font, font_scale, (0, 255, 0), thickness, cv2.LINE_AA)
            cv2.rectangle(extended_img, (w // 4, h // 2 - 40), (3 * w // 4, h // 2 + 40), (0, 255, 0), -1)
            cv2.putText(extended_img, "SUCCESS!", (w // 2 - 80, h // 2 + 10), font, 1.2, (0, 0, 0), 3, cv2.LINE_AA)

            if now - self._success_start_time > 3.0:
                self.is_running = False

        elif state == CalibrationState.FAILED:
            cv2.putText(extended_img, "CALIBRATION FAILED", (15, y3), font, font_scale, (0, 0, 255), thickness, cv2.LINE_AA)
            cv2.rectangle(extended_img, (w // 6, h // 2 - 50), (5 * w // 6, h // 2 + 50), (0, 0, 255), -1)
            cv2.putText(extended_img, "FAILED", (w // 2 - 60, h // 2 - 10), font, 1.0, (255, 255, 255), 3, cv2.LINE_AA)
            cv2.putText(extended_img, self._calibration_error[:50], (w // 6 + 10, h // 2 + 30), font, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

            if now - self._fail_start_time > 5.0:
                with self._lock:
                    self._calibration_state = CalibrationState.IDLE

        return extended_img

    # --------------------------------------------------------------------------
    # Trigger & Service Handling
    # --------------------------------------------------------------------------
    def trigger_calibration(self) -> None:
        """Trigger calibration asynchronously if conditions are met."""
        with self._lock:
            if self._calibration_state == CalibrationState.CALIBRATING:
                return
            if not self._tag_seen:
                self.get_logger().warn(f"Cannot trigger calibration: Tag {self.tag_id} is not visible!")
                return

        if not self.srv_client.service_is_ready():
            self.get_logger().error("Service '/calibrate_tag_pose' is not available!")
            return

        self.get_logger().info("Calling '/calibrate_tag_pose' service...")
        with self._lock:
            self._calibration_state = CalibrationState.CALIBRATING

        req = Trigger.Request()
        future = self.srv_client.call_async(req)
        future.add_done_callback(self._calibration_service_callback)

    def _calibration_service_callback(self, future) -> None:
        """Handle response from calibration service."""
        now = time.time()
        try:
            response = future.result()
            with self._lock:
                if response.success:
                    self.get_logger().info(
                        f"Service succeeded: {response.message}. Verifying TF buffer for transforms..."
                    )
                    # Defer SUCCESS state setting until TF verification completes
                    self._lookup_needed = True
                    self._lookup_start_time = now
                else:
                    self.get_logger().error(f"Service failed: {response.message}")
                    self._calibration_state = CalibrationState.FAILED
                    self._calibration_error = response.message
                    self._fail_start_time = now
        except Exception as e:
            self.get_logger().error(f"Service call exception: {e}")
            with self._lock:
                self._calibration_state = CalibrationState.FAILED
                self._calibration_error = str(e)
                self._fail_start_time = now

    # --------------------------------------------------------------------------
    # TF Lookup & Diagnostic Reporting
    # --------------------------------------------------------------------------
    def check_and_perform_lookup(self) -> None:
        """Check availability of required TF transformations and update calibration state."""
        now = time.time()
        with self._lock:
            elapsed = now - self._lookup_start_time

        has_robot = self.tf_buffer.can_transform(
            self.base_frame, self.target_frame, rclpy.time.Time()
        )
        has_map = self.tf_buffer.can_transform(
            self.map_frame, self.target_frame, rclpy.time.Time()
        )

        # Case 1: Ideal scenario – both robot and map transforms are ready
        if has_robot and has_map:
            with self._lock:
                self._lookup_needed = False
                self._calibration_state = CalibrationState.SUCCESS
                self._success_start_time = now
            self._print_pose_diagnostics(has_robot, has_map)

        # Case 2: Timeout reached (3.0 seconds)
        elif elapsed > 3.0:
            with self._lock:
                self._lookup_needed = False
                if has_robot:
                    # Robot transform exists, but map transform is missing (e.g. unlocalized)
                    self.get_logger().warn(
                        f"Map frame '{self.map_frame}' not found, but base transform is valid."
                    )
                    self._calibration_state = CalibrationState.SUCCESS
                    self._success_start_time = now
                else:
                    # Critical Failure: Missing required base frame transform
                    err_msg = f"TF transform missing for frame '{self.target_frame}'"
                    self.get_logger().error(f"Calibration Failure: {err_msg}")
                    self._calibration_state = CalibrationState.FAILED
                    self._calibration_error = err_msg
                    self._fail_start_time = now

            self._print_pose_diagnostics(has_robot, has_map)

    def _print_pose_diagnostics(self, has_robot_tf: bool, has_map_tf: bool) -> None:
        """Query TF2 transforms and print structured pose comparison table."""
        t_robot, q_robot = [0.0] * 3, [0.0, 0.0, 0.0, 1.0]
        t_map, q_map = [0.0] * 3, [0.0, 0.0, 0.0, 1.0]

        if has_robot_tf:
            try:
                tf_robot = self.tf_buffer.lookup_transform(self.base_frame, self.target_frame, rclpy.time.Time())
                t_robot = [tf_robot.transform.translation.x, tf_robot.transform.translation.y, tf_robot.transform.translation.z]
                q_robot = [tf_robot.transform.rotation.x, tf_robot.transform.rotation.y, tf_robot.transform.rotation.z, tf_robot.transform.rotation.w]
            except TransformException as e:
                self.get_logger().warn(f"Could not retrieve TF {self.base_frame} -> {self.target_frame}: {e}")
                has_robot_tf = False

        if has_map_tf:
            try:
                tf_map = self.tf_buffer.lookup_transform(self.map_frame, self.target_frame, rclpy.time.Time())
                t_map = [tf_map.transform.translation.x, tf_map.transform.translation.y, tf_map.transform.translation.z]
                q_map = [tf_map.transform.rotation.x, tf_map.transform.rotation.y, tf_map.transform.rotation.z, tf_map.transform.rotation.w]
            except TransformException as e:
                self.get_logger().warn(f"Could not retrieve TF {self.map_frame} -> {self.target_frame}: {e}")
                has_map_tf = False

        yaw_robot = np.degrees(quaternion_to_yaw(q_robot)) if has_robot_tf else 0.0
        yaw_map = np.degrees(quaternion_to_yaw(q_map)) if has_map_tf else 0.0

        print("\n" + "=" * 60)
        print("                  CALIBRATED TAG POSES")
        print("=" * 60)
        print("Frame Reference    |  X (m)  |  Y (m)  |  Z (m)  | Yaw (deg)")
        print("-" * 60)

        if has_robot_tf:
            print(f"w.r.t. Robot ({self.base_frame:4s}) | {t_robot[0]:7.3f} | {t_robot[1]:7.3f} | {t_robot[2]:7.3f} | {yaw_robot:9.1f}")
        else:
            print(f"w.r.t. Robot ({self.base_frame:4s}) | [No Transform Available in TF Buffer]")

        if has_map_tf:
            print(f"w.r.t. Map          | {t_map[0]:7.3f} | {t_map[1]:7.3f} | {t_map[2]:7.3f} | {yaw_map:9.1f}")
        else:
            print("w.r.t. Map          | [No Transform - is the robot localized in map?]")

        print("=" * 60)
        print("Calibration complete. Closing window in 3 seconds...\n")

    # --------------------------------------------------------------------------
    # GUI Event Loop Handling
    # --------------------------------------------------------------------------
    def show_frame(self) -> None:
        """Render current image frame and process UI hotkeys (Main thread execution)."""
        with self._lock:
            frame = self._latest_frame.copy() if self._latest_frame is not None else None

        if frame is None:
            return

        window_name = 'Tag Calibration View'
        cv2.imshow(window_name, frame)
        key = cv2.waitKey(1) & 0xFF

        if key in (ord('c'), ord(' ')):
            self.trigger_calibration()
        elif key == 27:  # ESC Key
            self.is_running = False


def terminal_input_thread(node: CalibrateTagCli) -> None:
    """Worker thread listening to terminal input without blocking ROS execution."""
    while rclpy.ok() and node.is_running:
        try:
            line = sys.stdin.readline()
            if not line:
                break
            node.trigger_calibration()
        except Exception:
            break


def main(args: Optional[List[str]] = None) -> None:
    """Main execution loop for the calibration node."""
    rclpy.init(args=args)
    node = CalibrateTagCli()

    # Start non-blocking terminal listener thread
    t_input = threading.Thread(target=terminal_input_thread, args=(node,), daemon=True)
    t_input.start()

    try:
        while rclpy.ok() and node.is_running:
            rclpy.spin_once(node, timeout_sec=0.01)

            if node.lookup_needed:
                node.check_and_perform_lookup()

            node.show_frame()
    except KeyboardInterrupt:
        pass
    finally:
        node.is_running = False
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()