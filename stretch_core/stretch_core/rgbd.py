#!/usr/bin/env python3

import os
from pathlib import Path
import threading
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from sensor_msgs.msg import Image, CameraInfo, PointCloud2

import ros2_numpy

# Import stretch4_body camera stream functions
from stretch4_body.subsystem.cameras import (
    stream_left_rgbd,
    stream_right_rgbd,
    stream_center_rgbd,
    stream_left_right_rgbd,
    stream_left_right_center_rgbd,
    stream_gripper_rgbd,
)

from stretch_core.vision.vision_topics import (
    VisionTopics,
    VisionFrames,
    get_camera_calibration_file_path,
)

from stretch_core.vision.ros_messages import create_pointcloud_rgb_msg


class RGBDCameraNode(Node):

    def __init__(self):
        super().__init__('rgbd_camera_node')
        
        # Declare parameters
        self.declare_parameter('use_left', True)
        self.declare_parameter('use_right', True)
        self.declare_parameter('use_center', False)
        self.declare_parameter('use_gripper', False)
        self.declare_parameter('publish_rotated', False)
        
        self.declare_parameter('left_calibration_file', '')
        self.declare_parameter('right_calibration_file', '')
        self.declare_parameter('center_calibration_file', '')
        
        # Read parameters
        self.use_left = self.get_parameter('use_left').value
        self.use_right = self.get_parameter('use_right').value
        self.use_center = self.get_parameter('use_center').value
        self.use_gripper = self.get_parameter('use_gripper').value
        self.publish_rotated = self.get_parameter('publish_rotated').value
        
        # Publishers and camera info caches
        self.publishers_topics = {}
        self.rotated_publishers = {}
        self.info_publishers = {}
        self.depth_publishers = {}
        self.depth_info_publishers = {}
        self.points_publishers = {}
        self.camera_info = {}
        
        self.get_logger().info("Configuring RGBD Node...")
        
        # Configure enabled head cameras
        for cam in ['left', 'right', 'center']:
            if getattr(self, f"use_{cam}"):
                self.publishers_topics[cam] = self.create_publisher(
                    Image, VisionTopics.image_raw(cam), 10)
                
                if self.publish_rotated:
                    self.rotated_publishers[cam] = self.create_publisher(
                        Image, VisionTopics.rotated_image(cam), 10)
                        
                self.info_publishers[cam] = self.create_publisher(
                    CameraInfo, VisionTopics.camera_info(cam), 10)
                    
                self.depth_publishers[cam] = self.create_publisher(
                    Image, VisionTopics.depth(cam), 10)
                    
                self.depth_info_publishers[cam] = self.create_publisher(
                    CameraInfo, VisionTopics.depth_camera_info(cam), 10)
                    
                self.points_publishers[cam] = self.create_publisher(
                    PointCloud2, VisionTopics.points(cam), 10)
                    
                # Load calibration
                calib_file = self.get_parameter(f'{cam}_calibration_file').value
                if not calib_file:
                    calib_file = get_camera_calibration_file_path(cam)
                if calib_file and os.path.exists(calib_file):
                    self.camera_info[cam] = self.load_camera_info(calib_file)
                    
        # Configure gripper camera if enabled
        if self.use_gripper:
            self.publishers_topics['gripper'] = self.create_publisher(
                Image, "/cameras_gripper/right/image_raw", 10)
            self.info_publishers['gripper'] = self.create_publisher(
                CameraInfo, "/cameras_gripper/right/camera_info", 10)
            self.depth_publishers['gripper'] = self.create_publisher(
                Image, "/cameras_gripper/stereo/image_raw", 10)
            self.depth_info_publishers['gripper'] = self.create_publisher(
                CameraInfo, "/cameras_gripper/stereo/camera_info", 10)
            self.points_publishers['gripper'] = self.create_publisher(
                PointCloud2, "/cameras_gripper/stereo_left_rgbd/points", 10)

        # Initialize thread states
        self.running = True
        
        self.publish_thread = threading.Thread(target=self.publish_loop, daemon=True)
        self.publish_thread.start()
        
        if self.use_gripper:
            self.gripper_publish_thread = threading.Thread(target=self.gripper_publish_loop, daemon=True)
            self.gripper_publish_thread.start()
        
        self.get_logger().info('RGBD camera node initialized successfully')

    def load_camera_info(self, calib_file):
        """Load camera calibration from YAML file"""
        try:
            # Remove file:// prefix if present
            if calib_file.startswith('file://'):
                calib_file = calib_file[7:]

            if not os.path.exists(calib_file):
                self.get_logger().warn(f'Calibration file not found: {calib_file}')
                return None

            with open(calib_file, 'r') as f:
                calib = yaml.safe_load(f)

            if calib is None:
                return None

            # Create CameraInfo message
            info = CameraInfo()
            info.width = calib.get('image_width', 0)
            info.height = calib.get('image_height', 0)
            info.distortion_model = calib.get('distortion_model', '')

            # Load distortion coefficients
            if 'distortion_coefficients' in calib:
                info.d = calib['distortion_coefficients'].get('data', [])

            # Load camera matrix
            if 'camera_matrix' in calib:
                info.k = calib['camera_matrix'].get('data', [0.0] * 9)

            # Load rectification matrix
            if 'rectification_matrix' in calib:
                info.r = calib['rectification_matrix'].get('data', [0.0] * 9)

            # Load projection matrix
            if 'projection_matrix' in calib:
                info.p = calib['projection_matrix'].get('data', [0.0] * 12)

            if info.distortion_model == "fisheye":
                info.distortion_model = "equidistant"

            self.get_logger().info(f'Loaded calibration from {calib_file}')
            return info

        except Exception as e:
            self.get_logger().error(f'Failed to load calibration file {calib_file}: {e}')
            return None

    def get_default_camera_info(self, width, height, frame_id):
        """Return a default/uncalibrated CameraInfo message based on frame dimensions"""
        info = CameraInfo()
        info.width = width
        info.height = height
        info.distortion_model = "plumb_bob"
        info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        fx = float(width)
        fy = float(width)
        cx = width / 2.0
        cy = height / 2.0
        info.k = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.p = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]
        info.header.frame_id = frame_id
        return info

    def publish_rgbd_frame(self, rgbd_frame, camera_name, stamp):
        if rgbd_frame is None:
            return
        
        # Extract components
        img_frame = rgbd_frame.image_frame
        if img_frame is None or img_frame.image is None:
            return
            
        img = img_frame.image
        h, w = img.shape[:2]
        
        # Get frame id
        if camera_name == 'gripper':
            frame_id = VisionFrames.gripper_camera_frame('right')
        else:
            frame_id = VisionFrames.camera_frame(camera_name)
        
        # 1. Publish Color Image message
        img_msg = ros2_numpy.msgify(Image, img, encoding='bgr8')
        img_msg.header.stamp = stamp
        img_msg.header.frame_id = frame_id
        self.publishers_topics[camera_name].publish(img_msg)
        
        # 2. Publish Rotated Color Image message (Head only)
        if camera_name != 'gripper' and self.publish_rotated:
            try:
                pub = self.rotated_publishers.get(camera_name)
                if pub is not None and pub.get_subscription_count() > 0:
                    rotate_k = VisionFrames.camera_frame_number_of_rotations(camera_name)
                    rotated_img = np.rot90(img, k=rotate_k)
                    rotated_msg = ros2_numpy.msgify(Image, rotated_img, encoding='bgr8')
                    rotated_msg.header.stamp = stamp
                    rotated_msg.header.frame_id = frame_id
                    pub.publish(rotated_msg)
            except Exception as ex:
                self.get_logger().error(f"Failed to rotate and publish image for {camera_name}: {ex}")
        
        # 3. Publish CameraInfo message (Color)
        if camera_name not in self.camera_info or self.camera_info[camera_name] is None:
            self.camera_info[camera_name] = self.get_default_camera_info(w, h, frame_id)
        info_msg = self.camera_info[camera_name]
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = frame_id
        self.info_publishers[camera_name].publish(info_msg)
        
        # 4. Publish Depth Image message
        depth_img = rgbd_frame.depth_image
        if depth_img is not None and depth_img.size > 0:
            try:
                # depth_image is float32 (meters).
                # Let's publish it as 32FC1 (meters).
                depth_msg = ros2_numpy.msgify(Image, depth_img, encoding='32FC1')
                depth_msg.header.stamp = stamp
                depth_msg.header.frame_id = frame_id
                self.depth_publishers[camera_name].publish(depth_msg)
            except Exception as ex:
                self.get_logger().error(f"Failed to serialize depth image for {camera_name}: {ex}")
        
        # 5. Publish Depth CameraInfo message
        self.depth_info_publishers[camera_name].publish(info_msg)
        
        # 6. Publish Point Cloud message (PointCloud2)
        points = rgbd_frame.pointcloud
        colors = rgbd_frame.pointcloud_colors
        if points is not None and points.size > 0:
            try:
                # colors has shape (N, 3) and is RGB.
                colors_uint8 = colors.astype(np.uint8)
                # Create pointcloud msg using create_pointcloud_rgb_msg
                cloud_msg = create_pointcloud_rgb_msg(colors_uint8, points)
                cloud_msg.header.stamp = stamp
                cloud_msg.header.frame_id = frame_id
                self.points_publishers[camera_name].publish(cloud_msg)
            except Exception as ex:
                self.get_logger().error(f"Failed to serialize pointcloud for {camera_name}: {ex}")

    def publish_loop(self):
        """Main publishing loop running in separate thread"""
        self.get_logger().info("Starting Head RGBD Cameras Stream...")
        
        # Decide the correct generator depending on enabled head cameras
        if self.use_left and self.use_right and self.use_center:
            generator = stream_left_right_center_rgbd(is_rotate=False)
        elif self.use_left and self.use_right:
            generator = stream_left_right_rgbd(is_rotate=False)
        elif self.use_left:
            generator = stream_left_rgbd(is_rotate=False)
        elif self.use_right:
            generator = stream_right_rgbd(is_rotate=False)
        elif self.use_center:
            generator = stream_center_rgbd(is_rotate=False)
        else:
            self.get_logger().warn("No head cameras selected!")
            return

        for frame in generator:
            if not self.running or not rclpy.ok():
                break
            if frame is None:
                continue
                
            stamp = self.get_clock().now().to_msg()

            # Check if synced frame or single RGBDFrame
            if hasattr(frame, 'left') or hasattr(frame, 'right') or hasattr(frame, 'center'):
                # SyncedRGBDFrame
                if self.use_left and hasattr(frame, 'left') and frame.left is not None:
                    self.publish_rgbd_frame(frame.left, 'left', stamp)
                if self.use_right and hasattr(frame, 'right') and frame.right is not None:
                    self.publish_rgbd_frame(frame.right, 'right', stamp)
                if self.use_center and hasattr(frame, 'center') and frame.center is not None:
                    self.publish_rgbd_frame(frame.center, 'center', stamp)
            else:
                # Single RGBDFrame
                if self.use_left:
                    self.publish_rgbd_frame(frame, 'left', stamp)
                elif self.use_right:
                    self.publish_rgbd_frame(frame, 'right', stamp)
                elif self.use_center:
                    self.publish_rgbd_frame(frame, 'center', stamp)

    def gripper_publish_loop(self):
        """Gripper publishing loop running in separate thread"""
        self.get_logger().info("Starting Gripper RGBD Camera Stream...")
        generator = stream_gripper_rgbd(is_rotate=False)
        for frame in generator:
            if not self.running or not rclpy.ok():
                break
            if frame is None:
                continue
                
            stamp = self.get_clock().now().to_msg()
            self.publish_rgbd_frame(frame, 'gripper', stamp)

    def destroy_node(self):
        self.running = False
        if self.publish_thread.is_alive():
            self.publish_thread.join(timeout=1.0)
        if self.use_gripper and hasattr(self, 'gripper_publish_thread') and self.gripper_publish_thread.is_alive():
            self.gripper_publish_thread.join(timeout=1.0)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RGBDCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
