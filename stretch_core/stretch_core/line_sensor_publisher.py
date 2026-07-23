#!/usr/bin/env python3
"""Publish filtered line-sensor as ROS PointCloud2 topics."""

from __future__ import annotations

import json
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Float32MultiArray, Header, String

from stretch4_body.robot.robot_client import RobotClient
from stretch4_body.subsystem.line_sensor.line_sensor_utils import (
    LineSensorCalibration,
    LineSensorGeometry,
)

from stretch_core.line_sensor_filter import (
    LineSensorConfig,
    LineSensorHits,
    LineSensorSource,
    as_range_array,
)
from stretch_core.line_sensor_raw_ranges import build_raw_ranges_multiarray


STALE_RECONNECT_AFTER_S = 5.0
STALE_RECONNECT_PERIOD_S = 5.0


def numpy_to_pointcloud2(points: np.ndarray, header: Header) -> PointCloud2:
    points = np.atleast_2d(np.asarray(points, dtype=np.float32))
    if points.size == 0:
        points = np.zeros((0, 3), dtype=np.float32)
    else:
        points = np.ascontiguousarray(points.reshape(-1, 3), dtype=np.float32)

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg


def xy_to_xyz(xy: np.ndarray, z: float) -> np.ndarray:
    xy = np.asarray(xy, dtype=np.float64).reshape(-1, 2)
    if len(xy) == 0:
        return np.zeros((0, 3))
    return np.column_stack([xy[:, 0], xy[:, 1], np.full(len(xy), z, dtype=np.float64)])


class LineSensorPublisher(Node):
    def __init__(self):
        super().__init__('line_sensor_publisher')
        self._declare_params()
        self._load_params()

        self._robot = None
        self._line_source = None
        self._calibration = None
        self._sensor_names: list[str] = []
        self._last_stale_warn_time = 0.0
        self._stale_since_time = None
        self._last_reconnect_attempt_time = 0.0
        self._connect_robot_or_raise()

        self._points_pub = self.create_publisher(PointCloud2, self._points_topic, qos_profile_sensor_data)
        self._obstacle_pub = self.create_publisher(PointCloud2, self._obstacle_topic, qos_profile_sensor_data)
        self._small_drop_pub = self.create_publisher(PointCloud2, self._small_drop_topic, qos_profile_sensor_data)
        self._counts_pub = self.create_publisher(String, self._counts_topic, 10)

        self._debug_pubs = {}
        if self._publish_debug:
            self._debug_pubs = {
                'raw_obstacle': self.create_publisher(PointCloud2, self._raw_obstacle_topic, qos_profile_sensor_data),
                'spatial_obstacle': self.create_publisher(PointCloud2, self._spatial_obstacle_topic, qos_profile_sensor_data),
                'raw_small_drop': self.create_publisher(PointCloud2, self._raw_small_drop_topic, qos_profile_sensor_data),
                'spatial_small_drop': self.create_publisher(PointCloud2, self._spatial_small_drop_topic, qos_profile_sensor_data),
                'spray': self.create_publisher(PointCloud2, self._spray_topic, qos_profile_sensor_data),
            }

        self._raw_range_pubs: dict[str, object] = {}
        if self._publish_raw_scans:
            for sensor_name in self._sensor_names:
                topic = f'{self._raw_scan_topic_prefix}/{sensor_name}/ranges'
                self._raw_range_pubs[sensor_name] = self.create_publisher(
                    Float32MultiArray, topic, qos_profile_sensor_data,
                )

        self.create_timer(1.0 / max(self._publish_rate_hz, 0.1), self._timer_callback)
        self.get_logger().info(
            'line_sensor_publisher started '
            f'points={self._points_topic} obstacle={self._obstacle_topic} '
            f'small_drop={self._small_drop_topic} raw_scans={self._publish_raw_scans}',
        )

    def _connect_robot_or_raise(self) -> None:
        robot = RobotClient(client_id='ros2_line_sensor_publisher')
        if not robot.startup():
            raise RuntimeError('RobotClient startup failed')
        if not hasattr(robot, 'line_sensor_loop'):
            robot.stop()
            raise RuntimeError('line_sensor_loop not available on robot_server')

        if self._line_source is None:
            self._initialize_line_source(robot.line_sensor_loop)
        old_robot = self._robot
        self._robot = robot
        if old_robot is not None:
            try:
                old_robot.stop()
            except Exception as exc:  # pragma: no cover - defensive cleanup path
                self.get_logger().warn(f'failed to stop previous RobotClient after reconnect: {exc}')

    def _initialize_line_source(self, line_loop) -> None:
        sensor_names = list(line_loop.params['sensor_names'])
        geometry = LineSensorGeometry(line_loop.params.get('line_sensor_geometry', {}))
        self._sensor_names = sensor_names

        self._calibration = None
        if self._use_tare:
            self._calibration = LineSensorCalibration(line_loop)
            self._calibration.load_latest_tare()

        self._line_source = LineSensorSource(
            geometry=geometry,
            sensor_names=sensor_names,
            config=LineSensorConfig(
                line_obstacle_min_height_m=self._line_obstacle_min_height_m,
                floor_band_m=self._floor_band_m,
                cliff_min_drop_m=self._cliff_min_drop_m,
                cliff_max_drop_m=self._cliff_max_drop_m,
                line_min_run_bins=self._line_min_run_bins,
                line_max_run_radial_span_m=self._line_max_run_radial_span_m,
                line_point_noise_max_run_bins=self._line_point_noise_max_run_bins,
                line_point_noise_xy_span_max_m=self._line_point_noise_xy_span_max_m,
                line_point_noise_radial_span_max_m=self._line_point_noise_radial_span_max_m,
                line_confirm_frames=self._line_confirm_frames,
                line_fast_confirm_frames=self._line_fast_confirm_frames,
                line_fast_confirm_range_m=self._line_fast_confirm_range_m,
                line_window_frames=self._line_window_frames,
                line_require_consecutive=self._line_require_consecutive,
                line_spray_merge_gap_bins=self._line_spray_merge_gap_bins,
                line_radial_streak_head_radius_max_m=self._line_radial_streak_head_radius_max_m,
                line_radial_streak_span_min_m=self._line_radial_streak_span_min_m,
                line_radial_streak_angular_spread_max_deg=self._line_radial_streak_angular_spread_max_deg,
                line_radial_streak_aspect_ratio_min=self._line_radial_streak_aspect_ratio_min,
                spray_min_run_bins=self._spray_min_run_bins,
                spray_roughness_thresh_m=self._spray_roughness_thresh_m,
                spray_max_run_bins=self._spray_max_run_bins,
                spray_head_radius_max_m=self._spray_head_radius_max_m,
                spray_radial_span_min_m=self._spray_radial_span_min_m,
                spray_angular_spread_max_deg=self._spray_angular_spread_max_deg,
                spray_aspect_ratio_min=self._spray_aspect_ratio_min,
                spray_direction_cluster_gap_deg=self._spray_direction_cluster_gap_deg,
                spray_monotonic_score_min=self._spray_monotonic_score_min,
                spray_monotonic_tolerance_m=self._spray_monotonic_tolerance_m,
                spray_short_run_bonus_max_bins=self._spray_short_run_bonus_max_bins,
                spray_temporal_window_frames=self._spray_temporal_window_frames,
                spray_temporal_stable_min_frames=self._spray_temporal_stable_min_frames,
                spray_temporal_stable_fraction=self._spray_temporal_stable_fraction,
            ),
            apply_tare=None if self._calibration is None else self._calibration.apply_tare,
        )

    def _maybe_reconnect_stale_robot(self) -> bool:
        now = time.time()
        if self._stale_since_time is None:
            self._stale_since_time = now
            return False
        if now - self._stale_since_time < STALE_RECONNECT_AFTER_S:
            return False
        if now - self._last_reconnect_attempt_time < STALE_RECONNECT_PERIOD_S:
            return False

        self._last_reconnect_attempt_time = now
        self.get_logger().warn('line_sensor status stayed stale; reconnecting RobotClient')
        try:
            self._connect_robot_or_raise()
        except Exception as exc:
            self.get_logger().warn(f'RobotClient reconnect failed: {exc}')
            return False
        self.get_logger().info('RobotClient reconnected for line_sensor_publisher')
        self._stale_since_time = None
        return True

    def _declare_params(self) -> None:
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('stale_timeout_s', 0.5)
        self.declare_parameter('use_tare', True)
        self.declare_parameter('publish_debug', True)
        self.declare_parameter('publish_raw_scans', False)
        self.declare_parameter('raw_scan_topic_prefix', '/line_sensor')

        self.declare_parameter('points_topic', '/line_sensor/points')
        self.declare_parameter('obstacle_topic', '/line_sensor/obstacle_points')
        self.declare_parameter('small_drop_topic', '/line_sensor/small_drop_points')
        self.declare_parameter('counts_topic', '/line_sensor/debug/source_counts')
        self.declare_parameter('raw_obstacle_topic', '/line_sensor/debug/raw_obstacle_points')
        self.declare_parameter('spatial_obstacle_topic', '/line_sensor/debug/spatial_obstacle_points')
        self.declare_parameter('raw_small_drop_topic', '/line_sensor/debug/raw_small_drop_points')
        self.declare_parameter('spatial_small_drop_topic', '/line_sensor/debug/spatial_small_drop_points')
        self.declare_parameter('spray_topic', '/line_sensor/debug/spray_points')

        self.declare_parameter('obstacle_z', 0.02)
        self.declare_parameter('small_drop_z', -0.05)
        self.declare_parameter('line_obstacle_min_height_m', 0.025)
        self.declare_parameter('floor_band_m', 0.015)
        self.declare_parameter('cliff_min_drop_m', 0.02)
        self.declare_parameter('cliff_max_drop_m', 0.10)
        self.declare_parameter('line_min_run_bins', 3)
        self.declare_parameter('line_max_run_radial_span_m', 0.25)
        self.declare_parameter('line_point_noise_max_run_bins', 12)
        self.declare_parameter('line_point_noise_xy_span_max_m', 0.010)
        self.declare_parameter('line_point_noise_radial_span_max_m', 0.010)
        self.declare_parameter('line_confirm_frames', 3)
        self.declare_parameter('line_fast_confirm_frames', 2)
        self.declare_parameter('line_fast_confirm_range_m', 0.55)
        self.declare_parameter('line_window_frames', 4)
        self.declare_parameter('line_require_consecutive', True)
        self.declare_parameter('line_spray_merge_gap_bins', 6)
        self.declare_parameter('line_radial_streak_head_radius_max_m', 0.35)
        self.declare_parameter('line_radial_streak_span_min_m', 0.04)
        self.declare_parameter('line_radial_streak_angular_spread_max_deg', 20.0)
        self.declare_parameter('line_radial_streak_aspect_ratio_min', 3.0)
        self.declare_parameter('spray_min_run_bins', 3)
        self.declare_parameter('spray_roughness_thresh_m', 0.03)
        self.declare_parameter('spray_max_run_bins', 0)
        self.declare_parameter('spray_head_radius_max_m', 0.30)
        self.declare_parameter('spray_radial_span_min_m', 0.05)
        self.declare_parameter('spray_angular_spread_max_deg', 15.0)
        self.declare_parameter('spray_aspect_ratio_min', 5.0)
        self.declare_parameter('spray_direction_cluster_gap_deg', 5.0)
        self.declare_parameter('spray_monotonic_score_min', 0.70)
        self.declare_parameter('spray_monotonic_tolerance_m', 0.005)
        self.declare_parameter('spray_short_run_bonus_max_bins', 15)
        self.declare_parameter('spray_temporal_window_frames', 5)
        self.declare_parameter('spray_temporal_stable_min_frames', 2)
        self.declare_parameter('spray_temporal_stable_fraction', 0.50)

    def _load_params(self) -> None:
        g = self.get_parameter
        self._base_frame = str(g('base_frame').value)
        self._publish_rate_hz = float(g('publish_rate_hz').value)
        self._stale_timeout_s = float(g('stale_timeout_s').value)
        self._use_tare = bool(g('use_tare').value)
        self._publish_debug = bool(g('publish_debug').value)
        self._publish_raw_scans = bool(g('publish_raw_scans').value)
        self._raw_scan_topic_prefix = str(g('raw_scan_topic_prefix').value).rstrip('/')

        self._points_topic = str(g('points_topic').value)
        self._obstacle_topic = str(g('obstacle_topic').value)
        self._small_drop_topic = str(g('small_drop_topic').value)
        self._counts_topic = str(g('counts_topic').value)
        self._raw_obstacle_topic = str(g('raw_obstacle_topic').value)
        self._spatial_obstacle_topic = str(g('spatial_obstacle_topic').value)
        self._raw_small_drop_topic = str(g('raw_small_drop_topic').value)
        self._spatial_small_drop_topic = str(g('spatial_small_drop_topic').value)
        self._spray_topic = str(g('spray_topic').value)

        self._obstacle_z = float(g('obstacle_z').value)
        self._small_drop_z = float(g('small_drop_z').value)
        self._line_obstacle_min_height_m = float(g('line_obstacle_min_height_m').value)
        self._floor_band_m = float(g('floor_band_m').value)
        self._cliff_min_drop_m = float(g('cliff_min_drop_m').value)
        self._cliff_max_drop_m = float(g('cliff_max_drop_m').value)
        self._line_min_run_bins = int(g('line_min_run_bins').value)
        self._line_max_run_radial_span_m = float(g('line_max_run_radial_span_m').value)
        self._line_point_noise_max_run_bins = int(g('line_point_noise_max_run_bins').value)
        self._line_point_noise_xy_span_max_m = float(g('line_point_noise_xy_span_max_m').value)
        self._line_point_noise_radial_span_max_m = float(g('line_point_noise_radial_span_max_m').value)
        self._line_confirm_frames = int(g('line_confirm_frames').value)
        self._line_fast_confirm_frames = int(g('line_fast_confirm_frames').value)
        self._line_fast_confirm_range_m = float(g('line_fast_confirm_range_m').value)
        self._line_window_frames = int(g('line_window_frames').value)
        self._line_require_consecutive = bool(g('line_require_consecutive').value)
        self._line_spray_merge_gap_bins = int(g('line_spray_merge_gap_bins').value)
        self._line_radial_streak_head_radius_max_m = float(g('line_radial_streak_head_radius_max_m').value)
        self._line_radial_streak_span_min_m = float(g('line_radial_streak_span_min_m').value)
        self._line_radial_streak_angular_spread_max_deg = float(g('line_radial_streak_angular_spread_max_deg').value)
        self._line_radial_streak_aspect_ratio_min = float(g('line_radial_streak_aspect_ratio_min').value)
        self._spray_min_run_bins = int(g('spray_min_run_bins').value)
        self._spray_roughness_thresh_m = float(g('spray_roughness_thresh_m').value)
        self._spray_max_run_bins = int(g('spray_max_run_bins').value)
        self._spray_head_radius_max_m = float(g('spray_head_radius_max_m').value)
        self._spray_radial_span_min_m = float(g('spray_radial_span_min_m').value)
        self._spray_angular_spread_max_deg = float(g('spray_angular_spread_max_deg').value)
        self._spray_aspect_ratio_min = float(g('spray_aspect_ratio_min').value)
        self._spray_direction_cluster_gap_deg = float(g('spray_direction_cluster_gap_deg').value)
        self._spray_monotonic_score_min = float(g('spray_monotonic_score_min').value)
        self._spray_monotonic_tolerance_m = float(g('spray_monotonic_tolerance_m').value)
        self._spray_short_run_bonus_max_bins = int(g('spray_short_run_bonus_max_bins').value)
        self._spray_temporal_window_frames = int(g('spray_temporal_window_frames').value)
        self._spray_temporal_stable_min_frames = int(g('spray_temporal_stable_min_frames').value)
        self._spray_temporal_stable_fraction = float(g('spray_temporal_stable_fraction').value)

    def _timer_callback(self) -> None:
        self._robot.pull_status()
        status = self._robot.line_sensor_loop.status
        line_age_s = self._line_status_age_s(status)
        stale = self._stale_timeout_s > 0.0 and line_age_s > self._stale_timeout_s

        if stale:
            self._warn_stale_status(line_age_s)
            if self._maybe_reconnect_stale_robot():
                self._robot.pull_status()
                status = self._robot.line_sensor_loop.status
                line_age_s = self._line_status_age_s(status)
                stale = self._stale_timeout_s > 0.0 and line_age_s > self._stale_timeout_s

        stamp = self.get_clock().now().to_msg()
        if self._publish_raw_scans:
            self._publish_raw_ranges_msg(status)

        if stale:
            raw_points = np.zeros((0, 3))
            hits = LineSensorHits()
        else:
            self._stale_since_time = None
            raw_points = self._line_source.project_all(status)
            hits = self._line_source.process(status)

        header = Header(stamp=stamp, frame_id=self._base_frame)
        self._points_pub.publish(numpy_to_pointcloud2(raw_points, header))
        self._obstacle_pub.publish(numpy_to_pointcloud2(xy_to_xyz(hits.obstacle_xy, self._obstacle_z), header))
        self._small_drop_pub.publish(numpy_to_pointcloud2(xy_to_xyz(hits.small_drop_xy, self._small_drop_z), header))

        if self._publish_debug:
            self._debug_pubs['raw_obstacle'].publish(
                numpy_to_pointcloud2(xy_to_xyz(hits.raw_obstacle_xy, self._obstacle_z), header),
            )
            self._debug_pubs['spatial_obstacle'].publish(
                numpy_to_pointcloud2(xy_to_xyz(hits.spatial_obstacle_xy, self._obstacle_z), header),
            )
            self._debug_pubs['raw_small_drop'].publish(
                numpy_to_pointcloud2(xy_to_xyz(hits.raw_small_drop_xy, self._small_drop_z), header),
            )
            self._debug_pubs['spatial_small_drop'].publish(
                numpy_to_pointcloud2(xy_to_xyz(hits.spatial_small_drop_xy, self._small_drop_z), header),
            )
            self._debug_pubs['spray'].publish(
                numpy_to_pointcloud2(xy_to_xyz(hits.raw_spray_xy, self._obstacle_z), header),
            )

        self._publish_counts(status, hits, raw_points, line_age_s, stale)

    def _publish_raw_ranges_msg(self, status: dict) -> None:
        for sensor_name in self._sensor_names:
            pub = self._raw_range_pubs.get(sensor_name)
            if pub is None:
                continue
            sensor_status = status.get(sensor_name, {})
            if not isinstance(sensor_status, dict):
                continue
            ranges_arr = as_range_array(sensor_status.get('ranges'))
            if ranges_arr.size == 0:
                continue
            pub.publish(build_raw_ranges_multiarray(ranges_arr))

    def _warn_stale_status(self, line_age_s: float) -> None:
        now = time.time()
        if now - self._last_stale_warn_time < 5.0:
            return
        self._last_stale_warn_time = now
        age_text = "unknown" if not np.isfinite(line_age_s) else f"{line_age_s:.2f}s"
        self.get_logger().warn(
            "line_sensor status is stale; publishing empty line sensor point clouds "
            f"(age={age_text}, stale_timeout_s={self._stale_timeout_s:.2f})",
        )

    @staticmethod
    def _line_status_age_s(status: dict) -> float:
        try:
            last_frame_time = float(status.get('last_frame_time', 0.0))
        except (TypeError, ValueError):
            last_frame_time = 0.0
        if last_frame_time <= 0.0:
            return float('inf')
        return time.time() - last_frame_time

    def _publish_counts(
        self,
        status: dict,
        hits: LineSensorHits,
        raw_points: np.ndarray,
        line_age_s: float,
        stale: bool,
    ) -> None:
        frame_ids = {}
        for sensor_name in self._robot.line_sensor_loop.params.get('sensor_names', []):
            sensor_status = status.get(sensor_name, {})
            if isinstance(sensor_status, dict):
                frame_ids[sensor_name] = sensor_status.get('frame_id')
        msg = String()
        msg.data = json.dumps({
            'stale': int(stale),
            'line_status_age_ms': -1 if not np.isfinite(line_age_s) else int(line_age_s * 1000.0),
            'raw_points': int(len(raw_points)),
            'raw_obstacle': int(len(hits.raw_obstacle_xy)),
            'spatial_obstacle': int(len(hits.spatial_obstacle_xy)),
            'obstacle': int(len(hits.obstacle_xy)),
            'raw_small_drop': int(len(hits.raw_small_drop_xy)),
            'spatial_small_drop': int(len(hits.spatial_small_drop_xy)),
            'small_drop': int(len(hits.small_drop_xy)),
            'spray': int(len(hits.raw_spray_xy)),
            'frame_ids': frame_ids,
        }, sort_keys=True)
        self._counts_pub.publish(msg)

    def destroy_node(self) -> bool:
        if getattr(self, '_robot', None) is not None:
            self._robot.stop()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LineSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
