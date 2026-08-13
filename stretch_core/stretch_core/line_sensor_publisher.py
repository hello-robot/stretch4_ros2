#!/usr/bin/env python3
"""Publish line-sensor hazards as ROS PointCloud2 topics. """

from __future__ import annotations

import dataclasses
import json
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String

from stretch4_body.robot.robot_client import RobotClient
from stretch4_body.subsystem.line_sensor.filter import (
    LineSensorConfig,
    LineSensorSource,
)
from stretch4_body.subsystem.line_sensor.line_sensor_utils import LineSensorGeometry

STALE_RECONNECT_AFTER_S = 5.0
STALE_RECONNECT_PERIOD_S = 5.0

_POINT_STEP = 20
_DTYPE = np.dtype({
    'names': ['x', 'y', 'z', 'sensor', 'bin', 't'],
    'formats': ['<f4', '<f4', '<f4', 'u1', '<u2', '<f4'],
    'offsets': [0, 4, 8, 12, 14, 16],
    'itemsize': _POINT_STEP,
})
_FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    PointField(name='sensor', offset=12, datatype=PointField.UINT8, count=1),
    PointField(name='bin', offset=14, datatype=PointField.UINT16, count=1),
    # Seconds from the header stamp to when THIS point was actually measured,
    PointField(name='t', offset=16, datatype=PointField.FLOAT32, count=1),
]


def point_times(ids, offsets_by_sensor) -> np.ndarray:
    """Per-point `t`, row-aligned with `ids`, from a per-sensor offset table."""
    if offsets_by_sensor is None or not np.size(ids):
        return None
    ids = np.asarray(ids, dtype=np.int32).reshape(-1, 2)
    idx = np.clip(ids[:, 0], 0, len(offsets_by_sensor) - 1)
    return np.asarray(offsets_by_sensor)[idx]


def hazard_cloud(xy, ids, z, header: Header, t=None) -> PointCloud2:
    """(N,2) xy + (N,2) (sensor, bin) + per-point z -> an identified cloud.

    `z` is the measured height from the filter, not a display constant, so it
    means different things per topic -- see LineSensorHits. `t` is the
    per-point acquisition offset; without ids there is no sensor to attribute
    a time to, so it stays 0.
    """
    xy = np.atleast_2d(np.asarray(xy, dtype=np.float64)).reshape(-1, 2)
    n = len(xy)
    ids = np.asarray(ids, dtype=np.int32).reshape(-1, 2) if np.size(ids) else None

    rec = np.zeros(n, dtype=_DTYPE)
    if n:
        rec['x'] = xy[:, 0]
        rec['y'] = xy[:, 1]
        z = np.asarray(z, dtype=np.float64).reshape(-1)
        rec['z'] = z if z.size == n else 0.0
        if ids is not None and len(ids) == n:
            rec['sensor'] = ids[:, 0].astype(np.uint8)
            rec['bin'] = ids[:, 1].astype(np.uint16)
        if t is not None:
            t = np.asarray(t, dtype=np.float64).reshape(-1)
            rec['t'] = t if t.size == n else 0.0

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = n
    msg.fields = _FIELDS
    msg.is_bigendian = False
    msg.point_step = _POINT_STEP
    msg.row_step = _POINT_STEP * n
    msg.is_dense = True
    msg.data = rec.tobytes()
    return msg


def xyz_cloud(points, header: Header) -> PointCloud2:
    """The raw (N,3) projected cloud. No identity: this is the full floor scan,
    used for visualisation, not for a hazard decision."""
    pts = np.atleast_2d(np.asarray(points, dtype=np.float32)).reshape(-1, 3)
    rec = np.zeros(len(pts), dtype=_DTYPE)
    if len(pts):
        rec['x'], rec['y'], rec['z'] = pts[:, 0], pts[:, 1], pts[:, 2]
    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = len(pts)
    msg.fields = _FIELDS
    msg.is_bigendian = False
    msg.point_step = _POINT_STEP
    msg.row_step = _POINT_STEP * len(pts)
    msg.is_dense = True
    msg.data = rec.tobytes()
    return msg


class LineSensorPublisher(Node):
    def __init__(self):
        super().__init__('line_sensor_publisher')
        self._declare_params()
        self._load_params()

        self._robot = None
        self._source = None
        self._sensor_names = []
        self._last_stale_warn_time = 0.0
        self._stale_since_time = None
        self._last_reconnect_attempt_time = 0.0
        self._connect_robot_or_raise()

        q = qos_profile_sensor_data
        self._pub = {
            'points': self.create_publisher(PointCloud2, self._topic['points'], q),
            'obstacle': self.create_publisher(PointCloud2, self._topic['obstacle'], q),
            'small_drop': self.create_publisher(PointCloud2, self._topic['small_drop'], q),
            'deep_drop': self.create_publisher(PointCloud2, self._topic['deep_drop'], q),
            'probable_cliff': self.create_publisher(PointCloud2, self._topic['probable_cliff'], q),
            'degraded': self.create_publisher(PointCloud2, self._topic['degraded'], q),
        }
       
        self._coverage_pub = self.create_publisher(String, self._topic['coverage'], q)

        self._debug_pub = {}
        if self._publish_debug:
            for key in ('raw_obstacle', 'spatial_obstacle', 'raw_small_drop',
                        'spatial_small_drop', 'spray', 'benign_null'):
                self._debug_pub[key] = self.create_publisher(
                    PointCloud2, self._topic[key], q)

        self.create_timer(1.0 / max(self._publish_rate_hz, 0.1), self._timer_callback)
        self.get_logger().info(
            f'line_sensor_publisher started; sensors={self._sensor_names} '
            f'hazards -> {self._topic["obstacle"]}, {self._topic["probable_cliff"]}, ...')

    # -- setup -------------------------------------------------------------

    def _connect_robot_or_raise(self) -> None:
        robot = RobotClient(client_id='ros2_line_sensor_publisher')
        if not robot.startup():
            raise RuntimeError('RobotClient startup failed')
        if not hasattr(robot, 'line_sensor_loop'):
            robot.stop()
            raise RuntimeError('line_sensor_loop is not enabled on the robot server')

        if self._source is None:
            self._build_source(robot.line_sensor_loop)
        old_robot = self._robot
        self._robot = robot
        if old_robot is not None:
            try:
                old_robot.stop()
            except Exception as exc:      # pragma: no cover - cleanup path
                self.get_logger().warn(f'failed to stop previous RobotClient: {exc}')

    def _build_source(self, loop) -> None:
        self._sensor_names = list(loop.params['sensor_names'])
        geometry = LineSensorGeometry(loop.params.get('line_sensor_geometry', {}))
        loop.pull_status()
        tared = loop.calibrated_sensors()
        for name, why in sorted(loop.uncalibrated_sensors().items()):
            self.get_logger().warn(f'{name}: NO TARE ({str(why).split(":")[0]}) '
                                   f'-- classified on coarse absolute heights')
        self.get_logger().info(
            f'calibration served by the body: {len(tared)}/{len(self._sensor_names)} tared')

        self._source = LineSensorSource(
            geometry=geometry,
            sensor_names=self._sensor_names,
            config=self._filter_config,
            apply_tare=loop.apply_tare if self._use_tare else None,
            bin_reliable=loop.bin_reliable(),
            bin_null_rate=loop.bin_null_rate(),
        )

    # -- parameters --------------------------------------------------------

    _TOPIC_DEFAULTS = {
        'points': '/line_sensor/points',
        'obstacle': '/line_sensor/obstacle_points',
        'small_drop': '/line_sensor/small_drop_points',
        'deep_drop': '/line_sensor/deep_drop_points',
        'probable_cliff': '/line_sensor/probable_cliff_points',
        'degraded': '/line_sensor/degraded_points',
        'coverage': '/line_sensor/coverage',
        'raw_obstacle': '/line_sensor/debug/raw_obstacle_points',
        'spatial_obstacle': '/line_sensor/debug/spatial_obstacle_points',
        'raw_small_drop': '/line_sensor/debug/raw_small_drop_points',
        'spatial_small_drop': '/line_sensor/debug/spatial_small_drop_points',
        'spray': '/line_sensor/debug/spray_points',
        'benign_null': '/line_sensor/debug/benign_null_points',
    }

    def _declare_params(self) -> None:
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_rate_hz', 30.0)
        self.declare_parameter('stale_timeout_s', 0.5)
        self.declare_parameter('use_tare', True)
        self.declare_parameter('publish_debug', True)

        for key, default in self._TOPIC_DEFAULTS.items():
            self.declare_parameter(f'{key}_topic', default)

        for f in dataclasses.fields(LineSensorConfig):
            self.declare_parameter(f'filter.{f.name}', getattr(LineSensorConfig(), f.name))

    def _load_params(self) -> None:
        g = self.get_parameter
        self._base_frame = str(g('base_frame').value)
        self._publish_rate_hz = float(g('publish_rate_hz').value)
        self._stale_timeout_s = float(g('stale_timeout_s').value)
        self._use_tare = bool(g('use_tare').value)
        self._publish_debug = bool(g('publish_debug').value)

        self._topic = {k: str(g(f'{k}_topic').value) for k in self._TOPIC_DEFAULTS}

        overrides = {}
        for f in dataclasses.fields(LineSensorConfig):
            value = g(f'filter.{f.name}').value
            if value is not None:
                overrides[f.name] = value
        self._filter_config = LineSensorConfig(**overrides)

    # -- the frame ---------------------------------------------------------

    def _timer_callback(self) -> None:
        self._robot.pull_status()
        status = self._robot.line_sensor_loop.status
        age_s = self._status_age_s(status)
        stale = self._stale_timeout_s > 0.0 and age_s > self._stale_timeout_s

        if stale:
            self._warn_stale(age_s)
            if self._maybe_reconnect():
                self._robot.pull_status()
                status = self._robot.line_sensor_loop.status
                age_s = self._status_age_s(status)
                stale = self._stale_timeout_s > 0.0 and age_s > self._stale_timeout_s

        if stale:
            hits = None
            raw = np.zeros((0, 3))
            stamp = self.get_clock().now().to_msg()
            offsets = None
        else:
            self._stale_since_time = None
            hits = self._source.process(status)
            raw = self._source.project_all(status)
            acq_s = self._acquisition_time_s(status, hits.observed_sensors)
            stamp = self._stamp_from(acq_s)
            offsets = self._sensor_time_offsets(status, acq_s)

        header = Header(stamp=stamp, frame_id=self._base_frame)
        empty_xy, empty_id, empty_z = (
            np.zeros((0, 2)), np.zeros((0, 2), np.int32), np.zeros(0))

        for key in ('obstacle', 'small_drop', 'deep_drop', 'probable_cliff', 'degraded'):
            xy = getattr(hits, f'{key}_xy') if hits else empty_xy
            ids = getattr(hits, f'{key}_id') if hits else empty_id
            z = getattr(hits, f'{key}_z') if hits else empty_z
            self._pub[key].publish(
                hazard_cloud(xy, ids, z, header, point_times(ids, offsets)))
        self._pub['points'].publish(xyz_cloud(raw, header))

        if self._publish_debug and hits is not None:
            # Debug stages carry no measured height of their own
            for key, attr in (
                ('raw_obstacle', 'raw_obstacle_xy'),
                ('spatial_obstacle', 'spatial_obstacle_xy'),
                ('raw_small_drop', 'raw_small_drop_xy'),
                ('spatial_small_drop', 'spatial_small_drop_xy'),
                ('spray', 'raw_spray_xy'),
                ('benign_null', 'benign_null_xy'),
            ):
                self._debug_pub[key].publish(
                    hazard_cloud(getattr(hits, attr), empty_id, empty_z, header))

        self._publish_coverage(status, hits, age_s, stale)

    # -- time --------------------------------------------------------------

    @staticmethod
    def _status_age_s(status: dict) -> float:
        """Seconds since the last COMPLETE frame closed.

        `last_frame_time` is the one field that keeps telling the truth when
        the board stops talking with the port still open: no exception fires,
        no counter increments, and every other value freezes at its last
        healthy reading.
        """
        try:
            last = float(status.get('last_frame_time', 0.0))
        except (TypeError, ValueError):
            last = 0.0
        if last <= 0.0:
            return float('inf')
        return time.time() - last

    @staticmethod
    def _sensor_read_times(status: dict, observed) -> dict:
        """name -> its own last read time, for the sensors that contributed."""
        times = {}
        for name in observed or ():
            block = status.get(name)
            if isinstance(block, dict):
                ts = float(block.get('ts_last_read', 0.0) or 0.0)
                if ts > 0.0:
                    times[name] = ts
        return times

    def _acquisition_time_s(self, status: dict, observed):
        """The OLDEST contributing sensor's own read time, or None."""
        times = self._sensor_read_times(status, observed)
        return min(times.values()) if times else None

    def _stamp_from(self, acq_s):
        if acq_s is None:
            return self.get_clock().now().to_msg()
        return Time(sec=int(acq_s), nanosec=int((acq_s - int(acq_s)) * 1e9))

    def _sensor_time_offsets(self, status: dict, acq_s):
        """Per-sensor-INDEX seconds from the header stamp to its own read.
        """
        if acq_s is None:
            return None
        times = self._sensor_read_times(status, self._sensor_names)
        offsets = np.zeros(len(self._sensor_names), dtype=np.float64)
        for idx, name in enumerate(self._sensor_names):
            if name in times:
                offsets[idx] = times[name] - acq_s
        return offsets

    # -- reconnection ------------------------------------------------------

    def _maybe_reconnect(self) -> bool:
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
        self.get_logger().info('RobotClient reconnected')
        self._stale_since_time = None
        return True

    def _warn_stale(self, age_s: float) -> None:
        now = time.time()
        if now - self._last_stale_warn_time < 5.0:
            return
        self._last_stale_warn_time = now
        age = 'unknown' if not np.isfinite(age_s) else f'{age_s:.2f}s'
        self.get_logger().warn(
            'line_sensor status is stale; publishing EMPTY clouds and empty '
            f'coverage (age={age}, stale_timeout_s={self._stale_timeout_s:.2f})')

    # -- coverage ----------------------------------------------------------

    def _publish_coverage(self, status, hits, age_s, stale) -> None:
        """Which sensors were actually watched. The safety-relevant output.

        A consumer must build swept free space from `observed`, never from an
        absence of hazard points -- a dead sensor and a clean floor produce
        identical hazard topics.
        """
        health = (status.get('health') or {}) if not stale else {}
        msg = String()
        msg.data = json.dumps({
            'stale': int(bool(stale)),
            'status_age_ms': -1 if not np.isfinite(age_s) else int(age_s * 1000.0),
            'sensors_total': len(self._sensor_names),
            'observed': list(hits.observed_sensors) if hits else [],
            'skipped': dict(hits.skipped_sensors) if hits else {
                name: 'stale' for name in self._sensor_names},
            'port_open': bool(health.get('port_open', False)),
            'streaming': bool(health.get('streaming', False)),
            'sensors_dead': list(health.get('sensors_dead', [])),
            'disabled_sensors': list(health.get('disabled_sensors', [])),
            'reader_restarts': int(health.get('reader_restarts', 0)),
        }, sort_keys=True)
        self._coverage_pub.publish(msg)

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
