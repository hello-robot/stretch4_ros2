#!/usr/bin/env python3
"""ROS 2 topic adapter for direct-style base hazard mapping."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
from threading import Lock
import traceback

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import numpy as np
from rcl_interfaces.msg import SetParametersResult
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Header

from stretch_base_hazard.layer_tags import (
    LAYER_LIDAR_CLIFF,
    LAYER_LIDAR_OBSTACLE,
    LAYER_LIDAR_OCCLUSION,
    LAYER_LINE_DEEP_DROP,
    LAYER_LINE_DEGRADED,
    LAYER_LINE_OBSTACLE,
    LAYER_LINE_PROBABLE_CLIFF,
    LAYER_LINE_SMALL_DROP,
)
from stretch_base_hazard.lidar_detector import LidarDetector, LidarDetectorConfig, LidarHits
from stretch_base_hazard.morphology import HazardExtractor, HazardOutput, MorphologyConfig
from stretch_base_hazard.pointcloud_io import numpy_to_pointcloud2, read_xyz_from_pointcloud2
from stretch_base_hazard.rolling_grid import GridConfig, RollingHazardGrid
from stretch_base_hazard.transform_utils import transform_points, translation_rotation_to_matrix
from tf2_ros import Buffer, TransformListener


@dataclass
class LineTopicHits:
    """Already-filtered line hazard points from ROS topics."""

    obstacle_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    small_drop_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    deep_drop_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    probable_cliff_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    degraded_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))


class HazardMapNode(Node):

    def __init__(self):
        super().__init__('hazard_map_node')

        self._declare_params()
        self._load_params()

        self._grid = RollingHazardGrid(GridConfig(
            map_radius_m=self._map_radius_m,
            resolution_m=self._resolution_m,
            decay_factor=self._decay_factor,
        ))
        self._lidar_detector = LidarDetector(LidarDetectorConfig(
            map_radius_m=self._map_radius_m,
            crop_to_map_radius=self._lidar_crop_to_map_radius,
            resolution_m=self._resolution_m,
            base_radius=self._base_radius,
            voxel_leaf_size=self._voxel_leaf_size,
            floor_detect_z_min=self._floor_detect_z_min,
            floor_detect_z_max=self._floor_detect_z_max,
            floor_fit_threshold_m=self._floor_fit_threshold_m,
            floor_observed_threshold_m=self._floor_observed_threshold_m,
            floor_max_tilt_deg=self._floor_max_tilt_deg,
            floor_ransac_iterations=self._floor_ransac_iterations,
            lidar_obstacle_min_height_m=self._lidar_obstacle_min_height_m,
            thresh_cliff_m=self._thresh_cliff_m,
            min_cliff_cells=self._min_cliff_cells,
            min_edge_floor_cells=self._min_edge_floor_cells,
            overhead_z_min_m=self._overhead_z_min_m,
            base_collision_z_max_m=self._base_collision_z_max_m,
        ))
        self._extractor = HazardExtractor(MorphologyConfig(
            hazard_threshold=self._hazard_threshold,
            dilation_cells=self._dilation_cells,
            erosion_cells=self._erosion_cells,
            cliff_z=self._cliff_z,
            obstacle_z=self._obstacle_z,
            occlusion_z=self._occlusion_z,
        ))

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pointcloud_lock = Lock()
        self._line_topic_lock = Lock()
        self._latest_lidar_msg: PointCloud2 | None = None
        self._latest_line_obstacle_msg: PointCloud2 | None = None
        self._latest_line_small_drop_msg: PointCloud2 | None = None
        self._latest_line_deep_drop_msg: PointCloud2 | None = None
        self._latest_line_probable_cliff_msg: PointCloud2 | None = None
        self._latest_line_degraded_msg: PointCloud2 | None = None
        self._line_obstacle_stamp_ns: int | None = None
        self._line_small_drop_stamp_ns: int | None = None
        self._line_deep_drop_stamp_ns: int | None = None
        self._line_probable_cliff_stamp_ns: int | None = None
        self._line_degraded_stamp_ns: int | None = None
        self._odom_pose: tuple[float, float, float] | None = None

        self._hazard_pub = self.create_publisher(PointCloud2, '/under_base_hazard/points', 10)
        self._cliff_pub = self.create_publisher(PointCloud2, '/under_base_hazard/cliff_points', 10)
        self._obstacle_pub = self.create_publisher(
            PointCloud2, '/under_base_hazard/obstacle_points', 10,
        )
        self._occluded_pub = self.create_publisher(
            PointCloud2, '/under_base_hazard/occluded_points', 10,
        )
        self._line_cliff_pub = self.create_publisher(
            PointCloud2, '/under_base_hazard/line_cliff_points', 10,
        )
        self._degraded_pub = self.create_publisher(
            PointCloud2, '/under_base_hazard/degraded_points', 10,
        )
        self._debug_ransac_floor_pub = None
        self._debug_observed_floor_pub = None
        self._debug_lidar_obstacle_pub = None
        self._debug_lidar_cliff_pub = None
        self._debug_lidar_occlusion_pub = None
        self._debug_line_obstacle_pub = None
        self._debug_line_small_drop_pub = None
        if self._publish_debug:
            self._debug_ransac_floor_pub = self.create_publisher(
                PointCloud2, '/under_base_hazard/debug/ransac_floor_points', 10,
            )
            self._debug_observed_floor_pub = self.create_publisher(
                PointCloud2, '/under_base_hazard/debug/observed_floor_cells', 10,
            )
            self._debug_lidar_obstacle_pub = self.create_publisher(
                PointCloud2, '/under_base_hazard/debug/lidar_obstacle_points', 10,
            )
            self._debug_lidar_cliff_pub = self.create_publisher(
                PointCloud2, '/under_base_hazard/debug/lidar_cliff_points', 10,
            )
            self._debug_lidar_occlusion_pub = self.create_publisher(
                PointCloud2, '/under_base_hazard/debug/lidar_occlusion_points', 10,
            )
            self._debug_line_obstacle_pub = self.create_publisher(
                PointCloud2, '/under_base_hazard/debug/line_obstacle_points', 10,
            )
            self._debug_line_small_drop_pub = self.create_publisher(
                PointCloud2, '/under_base_hazard/debug/line_small_drop_points', 10,
            )

        self._sub_group = MutuallyExclusiveCallbackGroup()
        self._update_group = MutuallyExclusiveCallbackGroup()

        self.create_subscription(
            PointCloud2,
            self._lidar_topic,
            self._on_lidar_pc,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            Odometry,
            self._odom_topic,
            self._on_odom,
            10,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            PointCloud2,
            self._line_obstacle_topic,
            self._on_line_obstacle_pc,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            PointCloud2,
            self._line_small_drop_topic,
            self._on_line_small_drop_pc,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            PointCloud2,
            self._line_deep_drop_topic,
            self._on_line_deep_drop_pc,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            PointCloud2,
            self._line_probable_cliff_topic,
            self._on_line_probable_cliff_pc,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )
        self.create_subscription(
            PointCloud2,
            self._line_degraded_topic,
            self._on_line_degraded_pc,
            qos_profile_sensor_data,
            callback_group=self._sub_group,
        )

        self.add_on_set_parameters_callback(self._validate_parameters)
        self.add_post_set_parameters_callback(self._apply_parameters)

        self.create_timer(
            1.0 / max(self._detector_rate_hz, 0.1),
            self._update_timer_callback,
            callback_group=self._update_group,
        )

        self.get_logger().info(
            f'hazard_map_node started (detector={self._detector_rate_hz} Hz, '
            f'lidar_topic={self._lidar_topic}, radius={self._map_radius_m} m)',
        )

    def _declare_params(self) -> None:
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('odom_topic', 'wheel_odom')
        self.declare_parameter('lidar_topic', '/lidar_pointcloud')
        self.declare_parameter('lidar_frame', '')
        self.declare_parameter('transform_timeout_s', 0.05)
        self.declare_parameter('line_obstacle_topic', '/line_sensor/obstacle_points')
        self.declare_parameter('line_small_drop_topic', '/line_sensor/small_drop_points')
        self.declare_parameter('line_deep_drop_topic', '/line_sensor/deep_drop_points')
        self.declare_parameter(
            'line_probable_cliff_topic', '/line_sensor/probable_cliff_points',
        )
        self.declare_parameter('line_degraded_topic', '/line_sensor/degraded_points')
        self.declare_parameter('degraded_weight', 5.0)
        self.declare_parameter('use_lidar', True)
        self.declare_parameter('degraded_deferred_to_lidar', True)
        self.declare_parameter('line_frame', '')
        self.declare_parameter('line_topic_timeout_s', 0.5)

        self.declare_parameter('detector_rate_hz', 10.0)
        self.declare_parameter('map_radius_m', 2.0)
        self.declare_parameter('lidar_crop_to_map_radius', True)
        self.declare_parameter('resolution_m', 0.05)
        self.declare_parameter('decay_factor', 0.98)
        self.declare_parameter('lidar_weight', 1.0)
        self.declare_parameter('occlusion_weight', 1.0)
        self.declare_parameter('line_weight', 5.0)
        self.declare_parameter('hazard_threshold', 5.0)

        self.declare_parameter('base_radius', 0.33)
        self.declare_parameter('voxel_leaf_size', 0.05)
        self.declare_parameter('floor_detect_z_min', -0.4)
        self.declare_parameter('floor_detect_z_max', 0.1)
        self.declare_parameter('floor_fit_threshold_m', 0.015)
        self.declare_parameter('floor_observed_threshold_m', 0.04)
        self.declare_parameter('floor_max_tilt_deg', 10.0)
        self.declare_parameter('floor_ransac_iterations', 200)
        self.declare_parameter('lidar_obstacle_min_height_m', 0.05)
        self.declare_parameter('thresh_cliff_m', 0.04)
        self.declare_parameter('min_cliff_cells', 4)
        self.declare_parameter('min_edge_floor_cells', 2)
        self.declare_parameter('overhead_z_min_m', 0.30)
        self.declare_parameter('base_collision_z_max_m', 0.35)

        self.declare_parameter('dilation_cells', 1)
        self.declare_parameter('erosion_cells', 1)
        self.declare_parameter('cliff_z', -0.05)
        self.declare_parameter('obstacle_z', 0.02)
        self.declare_parameter('occlusion_z', 0.15)
        self.declare_parameter('publish_debug', False)

    def _load_params(self) -> None:
        g = self.get_parameter
        self._base_frame = g('base_frame').value
        self._odom_topic = g('odom_topic').value
        self._lidar_topic = g('lidar_topic').value
        self._lidar_frame = g('lidar_frame').value
        self._transform_timeout_s = float(g('transform_timeout_s').value)
        self._line_obstacle_topic = g('line_obstacle_topic').value
        self._line_small_drop_topic = g('line_small_drop_topic').value
        self._line_deep_drop_topic = g('line_deep_drop_topic').value
        self._line_probable_cliff_topic = g('line_probable_cliff_topic').value
        self._line_degraded_topic = g('line_degraded_topic').value
        self._degraded_weight = float(g('degraded_weight').value)
        self._use_lidar = bool(g('use_lidar').value)
        self._degraded_deferred_to_lidar = bool(g('degraded_deferred_to_lidar').value)
        self._line_frame = g('line_frame').value
        self._line_topic_timeout_s = float(g('line_topic_timeout_s').value)

        self._detector_rate_hz = float(g('detector_rate_hz').value)
        self._map_radius_m = float(g('map_radius_m').value)
        self._lidar_crop_to_map_radius = bool(g('lidar_crop_to_map_radius').value)
        self._resolution_m = float(g('resolution_m').value)
        self._decay_factor = float(g('decay_factor').value)
        self._lidar_weight = float(g('lidar_weight').value)
        self._occlusion_weight = float(g('occlusion_weight').value)
        self._line_weight = float(g('line_weight').value)
        self._hazard_threshold = float(g('hazard_threshold').value)

        self._base_radius = float(g('base_radius').value)
        self._voxel_leaf_size = float(g('voxel_leaf_size').value)
        self._floor_detect_z_min = float(g('floor_detect_z_min').value)
        self._floor_detect_z_max = float(g('floor_detect_z_max').value)
        self._floor_fit_threshold_m = float(g('floor_fit_threshold_m').value)
        self._floor_observed_threshold_m = float(g('floor_observed_threshold_m').value)
        self._floor_max_tilt_deg = float(g('floor_max_tilt_deg').value)
        self._floor_ransac_iterations = int(g('floor_ransac_iterations').value)
        self._lidar_obstacle_min_height_m = float(g('lidar_obstacle_min_height_m').value)
        self._thresh_cliff_m = float(g('thresh_cliff_m').value)
        self._min_cliff_cells = int(g('min_cliff_cells').value)
        self._min_edge_floor_cells = int(g('min_edge_floor_cells').value)
        self._overhead_z_min_m = float(g('overhead_z_min_m').value)
        self._base_collision_z_max_m = float(g('base_collision_z_max_m').value)

        self._dilation_cells = int(g('dilation_cells').value)
        self._erosion_cells = int(g('erosion_cells').value)
        self._cliff_z = float(g('cliff_z').value)
        self._obstacle_z = float(g('obstacle_z').value)
        self._occlusion_z = float(g('occlusion_z').value)
        self._publish_debug = bool(g('publish_debug').value)

    def _on_lidar_pc(self, msg: PointCloud2) -> None:
        with self._pointcloud_lock:
            self._latest_lidar_msg = msg

    def _on_line_obstacle_pc(self, msg: PointCloud2) -> None:
        now_ns = self.get_clock().now().nanoseconds
        with self._line_topic_lock:
            self._latest_line_obstacle_msg = msg
            self._line_obstacle_stamp_ns = now_ns

    def _on_line_small_drop_pc(self, msg: PointCloud2) -> None:
        now_ns = self.get_clock().now().nanoseconds
        with self._line_topic_lock:
            self._latest_line_small_drop_msg = msg
            self._line_small_drop_stamp_ns = now_ns

    def _on_line_deep_drop_pc(self, msg: PointCloud2) -> None:
        now_ns = self.get_clock().now().nanoseconds
        with self._line_topic_lock:
            self._latest_line_deep_drop_msg = msg
            self._line_deep_drop_stamp_ns = now_ns

    def _on_line_probable_cliff_pc(self, msg: PointCloud2) -> None:
        now_ns = self.get_clock().now().nanoseconds
        with self._line_topic_lock:
            self._latest_line_probable_cliff_msg = msg
            self._line_probable_cliff_stamp_ns = now_ns

    def _on_line_degraded_pc(self, msg: PointCloud2) -> None:
        now_ns = self.get_clock().now().nanoseconds
        with self._line_topic_lock:
            self._latest_line_degraded_msg = msg
            self._line_degraded_stamp_ns = now_ns

    def _validate_parameters(self, params) -> SetParametersResult:
        """Validate a proposed parameter set. Must not mutate node state.
        """
        return SetParametersResult(successful=True)

    def _apply_parameters(self, params) -> None:
        """Apply runtime parameter changes, after the set has been committed."""
        for param in params:
            if param.name == 'use_lidar':
                self._use_lidar = bool(param.value)
                self.get_logger().info(
                    f'use_lidar set to {self._use_lidar}'
                    + ('' if self._use_lidar else ' (lidar layers cleared)'),
                )
            elif param.name == 'degraded_deferred_to_lidar':
                self._degraded_deferred_to_lidar = bool(param.value)

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
        self._odom_pose = (p.x, p.y, yaw)

    @staticmethod
    def _quat_to_yaw(x: float, y: float, z: float, w: float) -> float:
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)

    def _lookup_transform(self, source_frame: str) -> np.ndarray | None:
        try:
            tf_msg: TransformStamped = self._tf_buffer.lookup_transform(
                self._base_frame,
                source_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=self._transform_timeout_s),
            )
        except Exception as exc:
            self.get_logger().warning(
                f'TF {source_frame}->{self._base_frame} failed: {exc}',
                throttle_duration_sec=5.0,
            )
            return None
        t = tf_msg.transform.translation
        r = tf_msg.transform.rotation
        return translation_rotation_to_matrix(t.x, t.y, t.z, r.x, r.y, r.z, r.w)

    def _pointcloud_xyz_in_base(
        self,
        msg: PointCloud2 | None,
        frame_override: str = '',
    ) -> np.ndarray | None:
        if msg is None:
            return np.zeros((0, 3), dtype=np.float64)
        points = read_xyz_from_pointcloud2(msg)
        source_frame = str(frame_override or msg.header.frame_id or '').strip()
        if source_frame and source_frame != self._base_frame:
            tf = self._lookup_transform(source_frame)
            if tf is None:
                return None
            points = transform_points(tf, points)
        return points

    def _process_lidar_snapshot(self) -> LidarHits:
        if not self._use_lidar:
            return LidarHits()
        with self._pointcloud_lock:
            msg = self._latest_lidar_msg
        points = self._pointcloud_xyz_in_base(msg, self._lidar_frame)
        if points is None:
            return LidarHits()
        return self._lidar_detector.process(points)

    def _line_hits(self) -> LineTopicHits:
        now_ns = self.get_clock().now().nanoseconds
        with self._line_topic_lock:
            obstacle_msg = self._latest_line_obstacle_msg
            small_drop_msg = self._latest_line_small_drop_msg
            deep_drop_msg = self._latest_line_deep_drop_msg
            probable_cliff_msg = self._latest_line_probable_cliff_msg
            degraded_msg = self._latest_line_degraded_msg
            obstacle_stamp_ns = self._line_obstacle_stamp_ns
            small_drop_stamp_ns = self._line_small_drop_stamp_ns
            deep_drop_stamp_ns = self._line_deep_drop_stamp_ns
            probable_cliff_stamp_ns = self._line_probable_cliff_stamp_ns
            degraded_stamp_ns = self._line_degraded_stamp_ns

        return LineTopicHits(
            obstacle_xy=self._line_topic_xy(obstacle_msg, obstacle_stamp_ns, now_ns),
            small_drop_xy=self._line_topic_xy(small_drop_msg, small_drop_stamp_ns, now_ns),
            deep_drop_xy=self._line_topic_xy(deep_drop_msg, deep_drop_stamp_ns, now_ns),
            probable_cliff_xy=self._line_topic_xy(
                probable_cliff_msg, probable_cliff_stamp_ns, now_ns,
            ),
            degraded_xy=self._line_topic_xy(degraded_msg, degraded_stamp_ns, now_ns),
        )

    def _line_topic_xy(
        self,
        msg: PointCloud2 | None,
        stamp_ns: int | None,
        now_ns: int,
    ) -> np.ndarray:
        if msg is None or stamp_ns is None:
            return np.zeros((0, 2), dtype=np.float64)
        if self._line_topic_timeout_s > 0.0:
            timeout_ns = int(self._line_topic_timeout_s * 1e9)
            if now_ns - stamp_ns > timeout_ns:
                return np.zeros((0, 2), dtype=np.float64)
        points = self._pointcloud_xyz_in_base(msg, self._line_frame)
        if points is None or len(points) == 0:
            return np.zeros((0, 2), dtype=np.float64)
        return np.asarray(points[:, :2], dtype=np.float64).reshape(-1, 2)

    def _update_timer_callback(self) -> None:
        try:
            lidar_hits = self._process_lidar_snapshot()
            line_hits = self._line_hits()
            if self._publish_debug:
                self._publish_lidar_debug(lidar_hits)
                self._publish_line_debug(line_hits)
            self._update_grid(lidar_hits, line_hits)
            self._publish_hazards(lidar_hits.clear_floor_xy)
        except Exception:
            self.get_logger().error(traceback.format_exc())

    def _update_grid(self, lidar_hits: LidarHits, line_hits: LineTopicHits) -> None:
        pose = self._odom_pose if self._odom_pose is not None else (0.0, 0.0, 0.0)
        self._grid.update_pose(*pose)
        self._grid.decay()
        self._grid.clear(
            lidar_hits.clear_floor_xy,
            layers=(LAYER_LIDAR_CLIFF, LAYER_LIDAR_OBSTACLE, LAYER_LIDAR_OCCLUSION),
        )
        self._grid.accumulate(
            lidar_hits.obstacle_xy,
            self._lidar_weight,
            LAYER_LIDAR_OBSTACLE,
        )
        self._grid.accumulate(lidar_hits.cliff_xy, self._lidar_weight, LAYER_LIDAR_CLIFF)
        self._grid.accumulate(
            lidar_hits.occlusion_xy,
            self._occlusion_weight,
            LAYER_LIDAR_OCCLUSION,
        )
        if not self._use_lidar:
            self._grid.clear_layers(
                (LAYER_LIDAR_CLIFF, LAYER_LIDAR_OBSTACLE, LAYER_LIDAR_OCCLUSION),
            )

        self._grid.clear_layers((
            LAYER_LINE_OBSTACLE,
            LAYER_LINE_SMALL_DROP,
            LAYER_LINE_DEEP_DROP,
            LAYER_LINE_PROBABLE_CLIFF,
            LAYER_LINE_DEGRADED,
        ))
        self._grid.accumulate(line_hits.degraded_xy, self._degraded_weight, LAYER_LINE_DEGRADED)
        self._grid.accumulate(line_hits.obstacle_xy, self._line_weight, LAYER_LINE_OBSTACLE)
        self._grid.accumulate(line_hits.small_drop_xy, self._line_weight, LAYER_LINE_SMALL_DROP)
        self._grid.accumulate(line_hits.deep_drop_xy, self._line_weight, LAYER_LINE_DEEP_DROP)
        self._grid.accumulate(
            line_hits.probable_cliff_xy, self._line_weight, LAYER_LINE_PROBABLE_CLIFF,
        )
        if self._degraded_deferred_to_lidar and self._use_lidar:
            self._grid.clear(lidar_hits.clear_floor_xy, layers=(LAYER_LINE_DEGRADED,))

    def _publish_lidar_debug(self, hits: LidarHits) -> None:
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self._base_frame)
        self._debug_ransac_floor_pub.publish(
            numpy_to_pointcloud2(hits.ransac_floor_xyz, header),
        )
        self._debug_lidar_obstacle_pub.publish(
            numpy_to_pointcloud2(self._xy_to_xyz(hits.obstacle_xy, self._obstacle_z), header),
        )
        self._debug_lidar_cliff_pub.publish(
            numpy_to_pointcloud2(self._xy_to_xyz(hits.cliff_xy, self._cliff_z), header),
        )
        self._debug_lidar_occlusion_pub.publish(
            numpy_to_pointcloud2(self._xy_to_xyz(hits.occlusion_xy, self._occlusion_z), header),
        )

    def _publish_line_debug(self, hits: LineTopicHits) -> None:
        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self._base_frame)
        self._debug_line_obstacle_pub.publish(
            numpy_to_pointcloud2(self._xy_to_xyz(hits.obstacle_xy, self._obstacle_z), header),
        )
        self._debug_line_small_drop_pub.publish(
            numpy_to_pointcloud2(self._xy_to_xyz(hits.small_drop_xy, self._cliff_z), header),
        )

    def _publish_hazards(self, observed_floor_xy: np.ndarray) -> None:
        out = self._extractor.extract(
            self._grid.evidence,
            self._grid.layers,
            self._grid.min_x_odom,
            self._grid.min_y_odom,
            self._grid.resolution,
        )
        out = self._hazard_output_odom_to_base(out)

        stamp = self.get_clock().now().to_msg()
        header = Header(stamp=stamp, frame_id=self._base_frame)
        self._hazard_pub.publish(numpy_to_pointcloud2(out.all_points, header))
        self._cliff_pub.publish(numpy_to_pointcloud2(out.cliff_points, header))
        self._obstacle_pub.publish(numpy_to_pointcloud2(out.obstacle_points, header))
        self._occluded_pub.publish(numpy_to_pointcloud2(out.occluded_points, header))
        self._line_cliff_pub.publish(numpy_to_pointcloud2(out.line_cliff_points, header))
        self._degraded_pub.publish(numpy_to_pointcloud2(out.degraded_points, header))
        if self._publish_debug and self._debug_observed_floor_pub is not None:
            self._debug_observed_floor_pub.publish(
                numpy_to_pointcloud2(self._xy_to_xyz(observed_floor_xy, 0.0), header),
            )

    def _hazard_output_odom_to_base(self, out: HazardOutput) -> HazardOutput:
        out.cliff_points = self._odom_xyz_to_base(out.cliff_points)
        out.obstacle_points = self._odom_xyz_to_base(out.obstacle_points)
        out.occluded_points = self._odom_xyz_to_base(out.occluded_points)
        out.line_cliff_points = self._odom_xyz_to_base(out.line_cliff_points)
        out.degraded_points = self._odom_xyz_to_base(out.degraded_points)
        out.all_points = self._odom_xyz_to_base(out.all_points)
        return out

    def _odom_xyz_to_base(self, points: np.ndarray) -> np.ndarray:
        points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
        if len(points) == 0:
            return points
        out = points.copy()
        out[:, :2] = self._grid._odom_to_base(out[:, :2])
        return out

    @staticmethod
    def _xy_to_xyz(xy: np.ndarray, z: float) -> np.ndarray:
        xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
        if len(xy) == 0:
            return np.zeros((0, 3), dtype=np.float64)
        return np.column_stack([
            xy[:, 0],
            xy[:, 1],
            np.full(len(xy), z, dtype=np.float64),
        ])


def main(args=None):
    rclpy.init(args=args)
    node = HazardMapNode()
    executor = MultiThreadedExecutor(num_threads=4)
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
