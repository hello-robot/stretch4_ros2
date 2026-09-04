#!/usr/bin/env python3

import os
import time
import math
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

import tf2_ros
from tf2_ros import Buffer, TransformListener

from stretch4_docking.trackers.dock_tracker import DockTracker
from stretch4_docking.utils import cloud_reader
from stretch4_docking.utils.cloud_reader import transform_to_rt
from message_filters import SimpleFilter, ApproximateTimeSynchronizer
from stretch_nav2.dock_database import DockDatabase


EXPECTED_FRAMES = [
    "aruco_perception_docking_station_right_center",
    "aruco_perception_docking_station_right_left",
    "aruco_perception_docking_station_right_right",
    "aruco_perception_docking_station_left_center",
    "aruco_perception_docking_station_left_left",
    "aruco_perception_docking_station_left_right",
    "aruco_perception_docking_station_apex_center",
    "aruco_perception_docking_station_apex_left",
    "aruco_perception_docking_station_apex_right",
]


def pose_to_matrix(pos, quat):
    """Convert position and quaternion into a 4x4 SE(3) transform matrix."""
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat(quat).as_matrix()
    T[:3, 3] = pos
    return T


def matrix_to_pose(T):
    """Convert a 4x4 SE(3) transform matrix into position and quaternion."""
    pos = T[:3, 3]
    quat = Rotation.from_matrix(T[:3, :3]).as_quat()
    return pos.tolist(), quat.tolist()


class DiscoverDockNode(Node):
    """
    A ROS 2 node to discover, validate, and record docking stations.
    It uses dual lidar scans to find geometry candidates, validates them
    using ArUco tags seen by the cameras, and saves them to a map-specific database.
    """

    def __init__(self) -> None:
        super().__init__('discover_dock_node')

        self.get_logger().info("Initializing Discover Dock Node...")

        # Parameters
        self.declare_parameter('map_name', 'map')
        self.map_name = self.get_parameter('map_name').get_parameter_value().string_value
        
        self.declare_parameter('duplicate_distance_threshold', 0.5)
        self.duplicate_distance_threshold = self.get_parameter('duplicate_distance_threshold').get_parameter_value().double_value

        self.declare_parameter('validation_consecutive_frames', 5)
        self.validation_consecutive_frames = self.get_parameter('validation_consecutive_frames').get_parameter_value().integer_value

        # Callback groups
        self.cloud_group = MutuallyExclusiveCallbackGroup()
        self.timer_group = MutuallyExclusiveCallbackGroup()

        # Dock Tracker
        self.tracker = DockTracker()
        self.tracker.warm_start()

        # Point cloud processing warm start
        cloud_reader.warm_start()

        # TF2 Setup
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        self._right_R, self._right_t = None, None
        self._left_R, self._left_t = None, None
        self.get_lidars2baselinktfs()

        # Subscriptions
        cloud_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=2,
        )
        self.right_cloud_sub = SimpleFilter()
        self.left_cloud_sub = SimpleFilter()
        
        self.create_subscription(PointCloud2, '/lidar_points_right',
                                 self.right_cloud_sub.signalMessage, cloud_qos,
                                 callback_group=self.cloud_group)
        self.create_subscription(PointCloud2, '/lidar_points_left',
                                 self.left_cloud_sub.signalMessage, cloud_qos,
                                 callback_group=self.cloud_group)
        
        self.ts = ApproximateTimeSynchronizer(
            [self.right_cloud_sub, self.left_cloud_sub],
            queue_size=3,
            slop=0.06,
            allow_headerless=False
        )
        self.ts.registerCallback(self.cloud_cb)

        # Publishers
        self.marker_pub = self.create_publisher(MarkerArray, '/discovered_docks_markers', 10)

        # Dock database manager
        self.db = DockDatabase(self, default_map_name=self.map_name)

        # Validation consecutive tracker. The counter is a leaky bucket -- a frame that sees no
        # tags walks it back down rather than resetting it -- so the tags accumulate alongside it
        # and are only forgotten once the streak has fully decayed.
        self.validation_counter = 0
        self.validation_markers = set()

        # Marker timer
        self.create_timer(0.5, self.publish_markers, callback_group=self.timer_group)

        self.get_logger().info("Discover Dock Ready!")

    def get_lidars2baselinktfs(self, timeout_s=10.0) -> None:
        """Wait for lidar->base_link transforms to arrive."""
        deadline = time.time() + timeout_s
        while True:
            try:
                right_tf = self.tf_buffer.lookup_transform(
                    'base_link', 'lidar_right_link', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                left_tf = self.tf_buffer.lookup_transform(
                    'base_link', 'lidar_left_link', rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.5))
                break
            except tf2_ros.TransformException as e:
                if time.time() > deadline:
                    raise RuntimeError(f"lidar->base_link TF never arrived: {e}") from e
                self.get_logger().info("Waiting on lidar->base_link TF...")

        self._right_R, self._right_t = transform_to_rt(right_tf)
        self._left_R, self._left_t = transform_to_rt(left_tf)

    def cloud_cb(self, right_cloud: PointCloud2, left_cloud: PointCloud2) -> None:
        """Process dual lidar point clouds, search for docks, validate with ArUco tags, and record."""
        # Fusing and transforming both point clouds to base_link
        n_r_raw = right_cloud.width * right_cloud.height
        n_l_raw = left_cloud.width * left_cloud.height
        cloud_buf = np.empty((n_r_raw + n_l_raw, 5), dtype=np.float32)
        n_right = cloud_reader.fused_read_transform_points(
            right_cloud, self._right_R, self._right_t, cloud_buf, 0)
        n_total = cloud_reader.fused_read_transform_points(
            left_cloud, self._left_R, self._left_t, cloud_buf, n_right)
        cloud_mat = cloud_buf[:n_total]

        # Search for dock candidate
        self.tracker.identify(cloud_mat[:, :4], allow_ambiguity=True)

        if not self.tracker.is_tracking():
            self.decay_validation()
            return

        dock_pose_baselink = self.tracker.get_pose()
        cx, cy, cz = dock_pose_baselink[0], dock_pose_baselink[1], dock_pose_baselink[2]

        # 2-step verification using ArUco tags
        seen_markers = []
        now_time = self.get_clock().now()

        for frame in EXPECTED_FRAMES:
            try:
                transform = self.tf_buffer.lookup_transform('base_link', frame, rclpy.time.Time())
                
                # Verify age
                stamp = rclpy.time.Time.from_msg(transform.header.stamp)
                age = (now_time - stamp).nanoseconds / 1e9
                if age > 2.0:
                    continue

                # Verify distance
                tx = transform.transform.translation.x
                ty = transform.transform.translation.y
                tz = transform.transform.translation.z
                dist = math.sqrt((tx - cx)**2 + (ty - cy)**2 + (tz - cz)**2)

                if dist < 0.5:
                    marker_id = 43 if "right" in frame else (44 if "left" in frame else 45)
                    if marker_id not in seen_markers:
                        seen_markers.append(marker_id)
            except tf2_ros.TransformException:
                continue

        if len(seen_markers) > 0:
            # Candidate matches at least one seen ArUco tag
            self.validation_counter += 1
            self.validation_markers.update(seen_markers)
            self.get_logger().info(
                f"Tracking validated dock candidate! Frame count: {self.validation_counter}/{self.validation_consecutive_frames}. "
                f"Seen tags: {seen_markers}, accumulated: {sorted(self.validation_markers)}",
                throttle_duration_sec=1.0
            )

            if self.validation_counter >= self.validation_consecutive_frames:
                # Save/Add validated dock to database. Record every tag seen across the window,
                # not just the ones the confirming frame happened to catch -- a tag at a glancing
                # angle drops in and out between frames, so a single frame under-reports what the
                # robot actually saw.
                self.process_validated_dock(dock_pose_baselink, sorted(self.validation_markers))
                # Keep counter at threshold + 1 to avoid re-triggering while looking at it
                self.validation_counter = self.validation_consecutive_frames + 1
        else:
            self.decay_validation()

    def decay_validation(self) -> None:
        """Walk the validation counter down a frame, forgetting the tags once the streak ends."""
        if self.validation_counter > 0:
            self.validation_counter -= 1
            if self.validation_counter == 0:
                self.validation_markers.clear()

    def process_validated_dock(self, dock_pose_baselink: tuple, seen_markers: list) -> None:
        """Transform dock pose to map frame, check for duplicates, and save to YAML database."""
        try:
            map_to_base = self.tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
        except tf2_ros.TransformException as e:
            self.get_logger().warn(f"Cannot transform dock to map frame. map->base_link TF unavailable: {e}", throttle_duration_sec=3.0)
            return

        # SE(3) Transformations
        q_mb = map_to_base.transform.rotation
        t_mb = map_to_base.transform.translation
        T_map_to_base = pose_to_matrix([t_mb.x, t_mb.y, t_mb.z], [q_mb.x, q_mb.y, q_mb.z, q_mb.w])

        cx, cy, cz, qx, qy, qz, qw = dock_pose_baselink
        T_base_to_dock = pose_to_matrix([cx, cy, cz], [qx, qy, qz, qw])

        T_map_to_dock = T_map_to_base @ T_base_to_dock
        dock_pos_map, dock_quat_map = matrix_to_pose(T_map_to_dock)

        # Check for duplicates
        is_duplicate = False
        for _, dock in self.db.items():
            pos = dock['pose']['position']
            dist = math.sqrt(
                (dock_pos_map[0] - pos[0])**2 +
                (dock_pos_map[1] - pos[1])**2 +
                (dock_pos_map[2] - pos[2])**2
            )
            if dist < self.duplicate_distance_threshold:
                is_duplicate = True
                self.get_logger().info(f"Dock near location {pos} already exists in database. Skipping duplicate entry.", throttle_duration_sec=10.0)
                break

        if not is_duplicate:
            # Generate clean ID and timestamp
            dock_id = f"dock_{len(self.db) + 1}"
            timestamp = time.strftime('%Y-%m-%dT%H:%M:%S')

            dock_entry = {
                'timestamp': timestamp,
                'pose': {
                    'position': dock_pos_map,
                    'orientation': dock_quat_map
                },
                'validation': {
                    'lidar_candidate_found': True,
                    'aruco_markers_seen': seen_markers
                }
            }

            self.db[dock_id] = dock_entry
            self.get_logger().info(f"SUCCESS! NEW VALIDATED DOCK DISCOVERED: '{dock_id}' at {dock_pos_map}")
            self.db.save_database()

    def publish_markers(self) -> None:
        """Publish RViz visualization markers for all discovered/saved docks."""
        if not self.db:
            return

        msg = MarkerArray()
        current_time = self.get_clock().now().to_msg()

        for i, (dock_id, dock) in enumerate(self.db.items()):
            # Cube visualization representing the docking station
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = current_time
            marker.ns = 'discovered_docks_cubes'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = dock['pose']['position'][0]
            marker.pose.position.y = dock['pose']['position'][1]
            marker.pose.position.z = dock['pose']['position'][2]
            marker.pose.orientation.x = dock['pose']['orientation'][0]
            marker.pose.orientation.y = dock['pose']['orientation'][1]
            marker.pose.orientation.z = dock['pose']['orientation'][2]
            marker.pose.orientation.w = dock['pose']['orientation'][3]
            
            marker.scale.x = 0.4
            marker.scale.y = 0.6
            marker.scale.z = 0.2
            
            # Glowing transparent green
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = 0.7
            msg.markers.append(marker)
            
            # Text visualization showing the Dock ID
            text_marker = Marker()
            text_marker.header.frame_id = 'map'
            text_marker.header.stamp = current_time
            text_marker.ns = 'discovered_docks_text'
            text_marker.id = i + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.pose.position.x = dock['pose']['position'][0]
            text_marker.pose.position.y = dock['pose']['position'][1]
            text_marker.pose.position.z = dock['pose']['position'][2] + 0.35
            
            text_marker.scale.z = 0.12 # Font height
            
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0
            text_marker.text = f"Validated: {dock_id}"
            msg.markers.append(text_marker)

        self.marker_pub.publish(msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        node = DiscoverDockNode()
        executor = MultiThreadedExecutor(num_threads=4)
        executor.add_node(node)
        try:
            executor.spin()
        finally:
            executor.shutdown()
            node.destroy_node()
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
