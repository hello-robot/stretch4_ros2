#!/usr/bin/env python3

import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
import yaml

from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.srv import GetParameters
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.task import Future
from std_srvs.srv import Trigger
from tf2_ros import (
    Buffer,
    TransformException,
    TransformListener,
)

MAX_TRANSFORM_AGE_SEC = 2.0  # s

class TagLocalizationNode(Node):
    """
    A ROS 2 node that calibrates and seeds robot localization using ArUco tags.

    This node provides services to calibrate the static transform from the map frame
    to an ArUco tag, save it to a configuration file, and subsequently estimate the
    robot's initial pose (map to base_link transform) when the cameras see the tag,
    publishing the estimate to Nav2's /initialpose topic.
    """

    def __init__(self) -> None:
        """
        Initialize the TagLocalizationNode, configure TF2, and set up ROS 2 communication.
        """
        super().__init__('tag_localization_node')

        # Parameters
        self.declare_parameter('map_name', 'map')
        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value

        # Callback group for concurrent execution to allow TF listener to run during service calls
        self.cb_group = ReentrantCallbackGroup()

        # Configuration
        self.map_frame = 'map'
        self.base_frame = 'base_link'
        self.tag_frames = [
            'aruco_perception_localization_marker_center',
            'aruco_perception_localization_marker_left',
            'aruco_perception_localization_marker_right',
        ]

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Publisher for AMCL initial pose
        self.initial_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10
        )

        # Services using the reentrant callback group
        self.srv_calibrate = self.create_service(
            Trigger,
            '/calibrate_tag_pose',
            self.calibrate_tag_pose_callback,
            callback_group=self.cb_group,
        )
        self.srv_seed = self.create_service(
            Trigger,
            '/seed_localization',
            self.seed_localization_callback,
            callback_group=self.cb_group,
        )

        # Service client for map_server parameters to dynamically retrieve map name
        self.param_client = self.create_client(GetParameters, '/map_server/get_parameters')
        self._query_pending = False

        # Timer to query map_server's parameter
        self.query_timer = self.create_timer(1.0, self._query_map_server_parameter)

        # Config File Path Resolution (Initial log with fallback)
        initial_yaml_path = self._get_config_path(self.map_name)
        self.get_logger().info(
            f'Tag Localization Node initialized. Fallback Storage Path: {initial_yaml_path}'
        )

    def _query_map_server_parameter(self) -> None:
        """Query the map_server node for the yaml_filename parameter if the service is ready."""
        if not self.param_client.service_is_ready():
            return

        if self._query_pending:
            return

        self._query_pending = True
        req = GetParameters.Request()
        req.names = ['yaml_filename']

        future = self.param_client.call_async(req)
        future.add_done_callback(self._parameter_callback)

    def _parameter_callback(self, future: Future) -> None:
        """
        Process the parameter service response from map_server.

        Args:
            future: The finished ROS 2 service future.
        """
        self._query_pending = False
        try:
            response = future.result()
            if response and response.values:
                yaml_path = response.values[0].string_value
                if yaml_path:
                    filename = os.path.basename(yaml_path)
                    map_name, _ = os.path.splitext(filename)
                    if map_name:
                        self.map_name = map_name
                        self.get_logger().info(
                            f"Successfully queried map name '{self.map_name}' from map_server. "
                            f"Calibration Storage Path: {self._get_config_path(self.map_name)}"
                        )
                        # Cancel the timer since we successfully fetched the map name
                        self.query_timer.cancel()
        except Exception as e:
            self.get_logger().error(f"Failed to get parameter from map_server: {str(e)}")

    def _get_config_path(self, map_name: str) -> str:
        """
        Resolve and ensure the directory exists for the calibration configuration file.

        Args:
            map_name: The name of the map to prepend to the filename.

        Returns:
            The absolute file path to the tag pose YAML configuration file.
        """
        config_dir = os.path.expanduser('~/stretch_user/maps/tag_localization')

        os.makedirs(config_dir, exist_ok=True)
        return os.path.join(config_dir, f'{map_name}_tag_pose.yaml')

    # --------------------------------------------------------------------------
    # Matrix Helpers
    # --------------------------------------------------------------------------
    def _transform_to_matrix(
        self,
        translation: list[float] | np.ndarray,
        rotation_q: list[float] | np.ndarray,
    ) -> np.ndarray:
        """
        Convert a translation vector and a quaternion into a 4x4 homogeneous transformation matrix.

        Args:
            translation: A 3-element list or array representing [x, y, z] coordinates.
            rotation_q: A 4-element list or array representing a quaternion [qx, qy, qz, qw].

        Returns:
            A 4x4 numpy array representing the homogeneous transformation matrix.
        """
        mat = np.eye(4)
        mat[:3, :3] = R.from_quat(rotation_q).as_matrix()
        mat[:3, 3] = translation
        return mat

    def _matrix_to_transform(self, matrix: np.ndarray) -> tuple[list[float], list[float]]:
        """
        Convert a 4x4 homogeneous transformation matrix into translation and quaternion.

        Args:
            matrix: A 4x4 homogeneous transformation matrix.

        Returns:
            A tuple containing:
                - A list of 3 floats representing the [x, y, z] translation.
                - A list of 4 floats representing the [qx, qy, qz, qw] quaternion.
        """
        translation = matrix[:3, 3].tolist()
        rotation_q = R.from_matrix(matrix[:3, :3]).as_quat().tolist()
        return translation, rotation_q

    def _average_transforms(
        self,
        transform_list: list[tuple[list[float], list[float]]],
        outlier_threshold: float = 0.10,
    ) -> tuple[list[float], list[float], int]:
        """
        Compute the averaged translation and sign-aligned quaternion from a list of transforms.

        Outliers are rejected based on their Euclidean distance to the median position
        using a specified threshold. The orientation is averaged by resolving antipodal
        sign ambiguity before computing the mean.

        Args:
            transform_list: A list of tuples, where each tuple contains:
                - A list of 3 floats representing the translation [x, y, z].
                - A list of 4 floats representing the quaternion [qx, qy, qz, qw].
            outlier_threshold: The maximum allowed distance in meters from the median
                position for a transform to be considered an inlier.

        Returns:
            A tuple containing:
                - A list of 3 floats representing the averaged translation [x, y, z].
                - A list of 4 floats representing the averaged quaternion [qx, qy, qz, qw].
                - An integer representing the count of valid transforms retained after filtering.
        """
        positions = np.array([t[0] for t in transform_list])
        quats = np.array([t[1] for t in transform_list])

        # Outlier Rejection based on distance to median position
        median_pos = np.median(positions, axis=0)
        distances = np.linalg.norm(positions - median_pos, axis=1)
        valid_indices = distances < outlier_threshold

        if not np.any(valid_indices):
            valid_indices = np.ones(len(positions), dtype=bool)

        filtered_pos = positions[valid_indices]
        filtered_quats = quats[valid_indices]

        # 1. Unweighted Translation Mean
        avg_pos = np.mean(filtered_pos, axis=0).tolist()

        # 2. Unweighted Quaternion Mean (Handling sign ambiguity q == -q)
        q0 = filtered_quats[0]
        aligned_quats = []
        for q in filtered_quats:
            if np.dot(q, q0) < 0:
                aligned_quats.append(-q)
            else:
                aligned_quats.append(q)

        avg_q = np.mean(aligned_quats, axis=0)
        avg_q = (avg_q / np.linalg.norm(avg_q)).tolist()

        return avg_pos, avg_q, int(np.sum(valid_indices))

    # --------------------------------------------------------------------------
    # Service 1: Calibrate Tag Pose (map -> tag)
    # --------------------------------------------------------------------------
    def calibrate_tag_pose_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """
        Handle requests to calibrate the static transform from the map frame to the ArUco tag.

        This service polls and collects ArUco tag detections relative to the map frame
        for 2.0 seconds (if ROS is ok), filters out outliers, computes the averaged
        transform, and saves the resulting calibration data to a YAML configuration file.

        Args:
            request: The service trigger request.
            response: The service trigger response to be populated.

        Returns:
            The populated service trigger response with success status and descriptive message.
        """
        map_name = self.map_name
        yaml_path = self._get_config_path(map_name)

        self.get_logger().info(f'Starting Tag Calibration (2.0s window for map "{map_name}")...')

        seen_stamps = set()
        collected_transforms = []
        start_time = time.time()

        # Sampling Loop constrained strictly to 2.0 seconds and rclpy.ok()
        while rclpy.ok() and (time.time() - start_time) < 2.0:
            now = self.get_clock().now()
            for frame in self.tag_frames:
                try:
                    tf_msg = self.tf_buffer.lookup_transform(
                        self.map_frame, frame, rclpy.time.Time()
                    )
                    
                    # Calculate age of the transform and reject stale
                    tf_time = rclpy.time.Time.from_msg(tf_msg.header.stamp)
                    age_sec = (now - tf_time).nanoseconds / 1e9

                    if age_sec > MAX_TRANSFORM_AGE_SEC:
                        continue

                    stamp_key = (frame, tf_msg.header.stamp.sec, tf_msg.header.stamp.nanosec)
                    if stamp_key not in seen_stamps:
                        seen_stamps.add(stamp_key)

                        pos = [
                            tf_msg.transform.translation.x,
                            tf_msg.transform.translation.y,
                            tf_msg.transform.translation.z,
                        ]
                        quat = [
                            tf_msg.transform.rotation.x,
                            tf_msg.transform.rotation.y,
                            tf_msg.transform.rotation.z,
                            tf_msg.transform.rotation.w,
                        ]
                        collected_transforms.append((pos, quat))

                except TransformException:
                    pass

            time.sleep(0.02)  # Poll at 50Hz

        num_frames = len(collected_transforms)

        if num_frames == 0:
            response.success = False
            response.message = f'Calibration Failed: No camera tag transforms detected within 2.0s window for map "{map_name}".'
            self.get_logger().error(response.message)
            return response

        # Average frames
        avg_pos, avg_quat, valid_count = self._average_transforms(collected_transforms)

        # Write to YAML
        calibration_data = {
            'map_to_tag': {
                'translation': {'x': avg_pos[0], 'y': avg_pos[1], 'z': avg_pos[2]},
                'rotation': {
                    'x': avg_quat[0],
                    'y': avg_quat[1],
                    'z': avg_quat[2],
                    'w': avg_quat[3],
                },
                'total_raw_frames': num_frames,
                'used_filtered_frames': valid_count,
            }
        }

        try:
            with open(yaml_path, 'w') as f:
                yaml.dump(calibration_data, f, default_flow_style=False)
        except Exception as e:
            response.success = False
            response.message = f'Failed to write YAML config to {yaml_path}: {str(e)}'
            return response

        response.success = True
        response.message = (
            f'Calibration successful for map "{map_name}"! Acquired {num_frames} total frames '
            f'across cameras ({valid_count} retained after outlier filtering). Saved to: {yaml_path}'
        )
        self.get_logger().info(response.message)
        return response

    # --------------------------------------------------------------------------
    # Service 2: Seed Localization (map -> base_link)
    # --------------------------------------------------------------------------
    def seed_localization_callback(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        """
        Handle requests to seed the robot's initial localization using the calibrated ArUco tag.

        This service loads the calibrated map-to-tag transform, reads the live
        base_link-to-tag transform from the cameras over a 0.5 second window, averages
        them, calculates the map-to-base_link transform, and publishes it as a
        PoseWithCovarianceStamped message to the Nav2 /initialpose topic.

        Args:
            request: The service trigger request.
            response: The service trigger response to be populated.

        Returns:
            The populated service trigger response with success status and descriptive message.
        """
        self.get_logger().info('Seeding localization...')

        map_name = self.map_name
        yaml_path = self._get_config_path(map_name)

        # 1. Load Map -> Tag transform
        if not os.path.exists(yaml_path):
            response.success = False
            response.message = f'YAML configuration file not found at {yaml_path}. Run /calibrate_tag_pose first.'
            self.get_logger().error(response.message)
            return response

        try:
            with open(yaml_path, 'r') as f:
                config = yaml.safe_load(f)

            map_tag_data = config['map_to_tag']
            t_map_tag = [
                map_tag_data['translation']['x'],
                map_tag_data['translation']['y'],
                map_tag_data['translation']['z'],
            ]
            q_map_tag = [
                map_tag_data['rotation']['x'],
                map_tag_data['rotation']['y'],
                map_tag_data['rotation']['z'],
                map_tag_data['rotation']['w'],
            ]
        except Exception as e:
            response.success = False
            response.message = f'Failed reading YAML config from {yaml_path}: {str(e)}'
            return response

        # 2. Read live Base_link -> Tag transform from cameras (0.5s window)
        seen_stamps = set()
        collected_transforms = []
        start_time = time.time()

        while rclpy.ok() and (time.time() - start_time) < 0.5:
            now = self.get_clock().now()
            for frame in self.tag_frames:
                try:
                    tf_msg = self.tf_buffer.lookup_transform(
                        self.base_frame, frame, rclpy.time.Time()
                    )
                    
                    # Calculate age of the transform and reject stale
                    tf_time = rclpy.time.Time.from_msg(tf_msg.header.stamp)
                    age_sec = (now - tf_time).nanoseconds / 1e9

                    if age_sec > MAX_TRANSFORM_AGE_SEC:
                        continue
                    
                    stamp_key = (frame, tf_msg.header.stamp.sec, tf_msg.header.stamp.nanosec)

                    if stamp_key not in seen_stamps:
                        seen_stamps.add(stamp_key)
                        pos = [
                            tf_msg.transform.translation.x,
                            tf_msg.transform.translation.y,
                            tf_msg.transform.translation.z,
                        ]
                        quat = [
                            tf_msg.transform.rotation.x,
                            tf_msg.transform.rotation.y,
                            tf_msg.transform.rotation.z,
                            tf_msg.transform.rotation.w,
                        ]
                        collected_transforms.append((pos, quat))
                except TransformException:
                    pass

            time.sleep(0.02)

        if len(collected_transforms) == 0:
            response.success = False
            response.message = 'Seeding Failed: Robot cameras cannot currently see the ArUco tag.'
            self.get_logger().error(response.message)
            return response

        # Average current base -> tag transform
        avg_base_tag_pos, avg_base_tag_quat, _ = self._average_transforms(collected_transforms)

        # 3. Perform Transform Matrix Calculations:
        # T_(map->base) = T_(map->tag) * (T_(base->tag))^-1
        M_map_tag = self._transform_to_matrix(t_map_tag, q_map_tag)
        M_base_tag = self._transform_to_matrix(avg_base_tag_pos, avg_base_tag_quat)
        M_tag_base = np.linalg.inv(M_base_tag)

        M_map_base = np.matmul(M_map_tag, M_tag_base)
        robot_pos, _ = self._matrix_to_transform(M_map_base)

        # Extract 2D planar yaw-only orientation from the 3D transformation matrix
        # to avoid projection instability and AMCL initialization failures.
        yaw = np.arctan2(M_map_base[1, 0], M_map_base[0, 0])
        robot_quat = [0.0, 0.0, np.sin(yaw / 2.0), np.cos(yaw / 2.0)]

        # 4. Publish PoseWithCovarianceStamped to /initialpose
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = self.map_frame
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        initial_pose.pose.pose.position.x = float(robot_pos[0])
        initial_pose.pose.pose.position.y = float(robot_pos[1])
        initial_pose.pose.pose.position.z = 0.0  # Planar constraint for Nav2 AMCL

        initial_pose.pose.pose.orientation.x = float(robot_quat[0])
        initial_pose.pose.pose.orientation.y = float(robot_quat[1])
        initial_pose.pose.pose.orientation.z = float(robot_quat[2])
        initial_pose.pose.pose.orientation.w = float(robot_quat[3])

        # Covariance Matrix (6x6 flat list): Non-zero diagonal to let AMCL scan match
        cov = [0.0] * 36
        cov[0] = 0.05   # Var(X)
        cov[7] = 0.05   # Var(Y)
        cov[35] = 0.03  # Var(Yaw)
        initial_pose.pose.covariance = cov

        self.initial_pose_pub.publish(initial_pose)

        response.success = True
        response.message = (
            f'Seeded AMCL pose at x={robot_pos[0]:.3f}, y={robot_pos[1]:.3f} '
            f'using {len(collected_transforms)} live camera frames.'
        )
        self.get_logger().info(response.message)
        return response


def main(args: list[str] | None = None) -> None:
    """
    Initialize the ROS 2 Python client library, spin the tag localization node, and shutdown cleanly.

    Args:
        args: Optional command-line arguments to pass to rclpy initialization.
    """
    rclpy.init(args=args)
    node = TagLocalizationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()