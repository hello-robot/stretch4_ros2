#!/usr/bin/env python3
"""Publish filtered line-sensor as ROS PointCloud2 topics."""

from __future__ import annotations

import json
import time
from dataclasses import fields as dataclass_fields

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
        self._probable_cliff_pub = self.create_publisher(
            PointCloud2, self._probable_cliff_topic, qos_profile_sensor_data,
        )
        self._deep_drop_pub = self.create_publisher(PointCloud2, self._deep_drop_topic, qos_profile_sensor_data)
        self._degraded_pub = self.create_publisher(PointCloud2, self._degraded_topic, qos_profile_sensor_data)
        self._counts_pub = self.create_publisher(String, self._counts_topic, 10)

        self._debug_pubs = {}
        if self._publish_debug:
            self._debug_pubs = {
                'raw_obstacle': self.create_publisher(PointCloud2, self._raw_obstacle_topic, qos_profile_sensor_data),
                'spatial_obstacle': self.create_publisher(PointCloud2, self._spatial_obstacle_topic, qos_profile_sensor_data),
                'raw_small_drop': self.create_publisher(PointCloud2, self._raw_small_drop_topic, qos_profile_sensor_data),
                'spatial_small_drop': self.create_publisher(PointCloud2, self._spatial_small_drop_topic, qos_profile_sensor_data),
                'spray': self.create_publisher(PointCloud2, self._spray_topic, qos_profile_sensor_data),
                'benign_null': self.create_publisher(PointCloud2, self._benign_null_topic, qos_profile_sensor_data),
                'raw_marginal_obstacle': self.create_publisher(
                    PointCloud2, self._raw_marginal_obstacle_topic, qos_profile_sensor_data,
                ),
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
            config=self._filter_config,
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
        self.declare_parameter('probable_cliff_topic', '/line_sensor/probable_cliff_points')
        self.declare_parameter('deep_drop_topic', '/line_sensor/deep_drop_points')
        self.declare_parameter('degraded_topic', '/line_sensor/degraded_points')
        self.declare_parameter('benign_null_topic', '/line_sensor/debug/benign_null_points')
        self.declare_parameter(
            'raw_marginal_obstacle_topic', '/line_sensor/debug/raw_marginal_obstacle_points',
        )

        self.declare_parameter('obstacle_z', 0.02)
        self.declare_parameter('small_drop_z', -0.05)
        self.declare_parameter('degraded_z', 0.0)

        for f in dataclass_fields(LineSensorConfig):
            self.declare_parameter(f.name, f.default)

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
        self._probable_cliff_topic = str(g('probable_cliff_topic').value)
        self._deep_drop_topic = str(g('deep_drop_topic').value)
        self._degraded_topic = str(g('degraded_topic').value)
        self._benign_null_topic = str(g('benign_null_topic').value)
        self._raw_marginal_obstacle_topic = str(g('raw_marginal_obstacle_topic').value)

        self._obstacle_z = float(g('obstacle_z').value)
        self._small_drop_z = float(g('small_drop_z').value)
        self._degraded_z = float(g('degraded_z').value)

        self._filter_config = self._load_filter_config()

    def _load_filter_config(self) -> LineSensorConfig:
        """Read the LineSensorConfig-shaped parameters back into a config.

        Each value is cast to the type of its dataclass default, so a YAML
        override lands as the type the pipeline expects.
        """
        values = {}
        for f in dataclass_fields(LineSensorConfig):
            caster = type(f.default)
            values[f.name] = caster(self.get_parameter(f.name).value)
        return LineSensorConfig(**values)

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
        self._probable_cliff_pub.publish(
            numpy_to_pointcloud2(xy_to_xyz(hits.probable_cliff_xy, self._small_drop_z), header),
        )
        self._deep_drop_pub.publish(
            numpy_to_pointcloud2(xy_to_xyz(hits.deep_drop_xy, self._small_drop_z), header),
        )
        self._degraded_pub.publish(
            numpy_to_pointcloud2(xy_to_xyz(hits.degraded_xy, self._degraded_z), header),
        )

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
            self._debug_pubs['benign_null'].publish(
                numpy_to_pointcloud2(xy_to_xyz(hits.benign_null_xy, self._small_drop_z), header),
            )
            self._debug_pubs['raw_marginal_obstacle'].publish(
                numpy_to_pointcloud2(xy_to_xyz(hits.raw_marginal_obstacle_xy, self._obstacle_z), header),
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
            'raw_marginal_obstacle': int(len(hits.raw_marginal_obstacle_xy)),
            'probable_cliff': int(len(hits.probable_cliff_xy)),
            'benign_null': int(len(hits.benign_null_xy)),
            'deep_drop': int(len(hits.deep_drop_xy)),
            'degraded': int(len(hits.degraded_xy)),
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
