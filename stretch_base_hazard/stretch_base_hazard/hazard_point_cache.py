"""ROS helpers for filtering base velocity with hazard PointCloud2 topics."""

from __future__ import annotations

from dataclasses import dataclass
import math
from threading import Lock

from geometry_msgs.msg import Point
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2

from stretch_base_hazard.hazard_vel_filter import (
    classify_hazard_heading,
    filter_hazard_velocity,
    HazardVelFilterConfig,
    HazardVelFilterResult,
)
from stretch_base_hazard.pointcloud_io import read_xyz_from_pointcloud2
from visualization_msgs.msg import Marker, MarkerArray


@dataclass(frozen=True)
class HazardPointCacheConfig:
    obstacle_points_topic: str = '/under_base_hazard/obstacle_points'
    cliff_points_topic: str = '/under_base_hazard/cliff_points'
    hazard_timeout_s: float = 0.5
    stop_linear_on_stale: bool = True
    filter_config: HazardVelFilterConfig = HazardVelFilterConfig()
    blocked_directions_topic: str = '/under_base_hazard/blocked_directions'
    blocked_directions_frame: str = 'base_link'
    blocked_directions_publish_rate_hz: float = 5.0
    blocked_directions_bins: int = 72
    blocked_directions_inner_radius_m: float = 0.35
    blocked_directions_outer_radius_m: float = 0.95
    blocked_directions_z_m: float = 0.03
    blocked_directions_alpha: float = 0.55


@dataclass(frozen=True)
class HazardVelocityDecision:
    result: HazardVelFilterResult
    stale: bool = False


class HazardPointCache:
    """Keep the latest hazard clouds and filter base-frame velocity commands."""

    def __init__(self, node: Node, config: HazardPointCacheConfig):
        self._node = node
        self._config = config
        self._lock = Lock()
        self._obstacle_xy = np.zeros((0, 2), dtype=np.float64)
        self._cliff_xy = np.zeros((0, 2), dtype=np.float64)
        self._obstacle_stamp_ns: int | None = None
        self._cliff_stamp_ns: int | None = None
        self._obstacle_required = bool(config.obstacle_points_topic)
        self._cliff_required = bool(config.cliff_points_topic)
        self._blocked_directions_pub = None

        if self._obstacle_required:
            node.create_subscription(
                PointCloud2,
                config.obstacle_points_topic,
                self._on_obstacle_points,
                qos_profile_sensor_data,
            )

        if self._cliff_required:
            node.create_subscription(
                PointCloud2,
                config.cliff_points_topic,
                self._on_cliff_points,
                qos_profile_sensor_data,
            )

        if config.blocked_directions_topic and config.blocked_directions_publish_rate_hz > 0.0:
            self._blocked_directions_pub = node.create_publisher(
                MarkerArray,
                config.blocked_directions_topic,
                1,
            )
            node.create_timer(
                1.0 / max(config.blocked_directions_publish_rate_hz, 0.1),
                self.publish_blocked_directions,
            )

    @property
    def config(self) -> HazardPointCacheConfig:
        return self._config

    def filter_velocity(self, vx: float, vy: float, wz: float) -> HazardVelocityDecision:
        obstacle_xy, cliff_xy, stale = self.snapshot()
        if stale and self._config.stop_linear_on_stale:
            return HazardVelocityDecision(
                HazardVelFilterResult(0.0, 0.0, float(wz)),
                stale=True,
            )
        return HazardVelocityDecision(
            filter_hazard_velocity(
                vx,
                vy,
                wz,
                obstacle_xy,
                cliff_xy,
                self._config.filter_config,
            ),
            stale=stale,
        )

    def snapshot(self) -> tuple[np.ndarray, np.ndarray, bool]:
        now_ns = self._node.get_clock().now().nanoseconds
        with self._lock:
            obstacle_xy = self._obstacle_xy.copy()
            cliff_xy = self._cliff_xy.copy()
            obstacle_stamp_ns = self._obstacle_stamp_ns
            cliff_stamp_ns = self._cliff_stamp_ns
        return obstacle_xy, cliff_xy, self._is_stale(
            now_ns,
            obstacle_stamp_ns,
            cliff_stamp_ns,
        )

    def publish_blocked_directions(self) -> None:
        if self._blocked_directions_pub is None:
            return

        obstacle_xy, cliff_xy, stale = self.snapshot()
        marker_array = MarkerArray()
        stamp = self._node.get_clock().now().to_msg()
        marker_array.markers.append(self._delete_all_marker(stamp))
        marker_array.markers.append(self._circle_marker(stamp, 1, self._outer_radius()))
        marker_array.markers.append(self._circle_marker(stamp, 2, self._inner_radius()))

        if stale and self._config.stop_linear_on_stale:
            marker_array.markers.append(self._sector_marker(
                stamp,
                marker_id=5,
                ns='stale_blocked_directions',
                color=(1.0, 0.85, 0.0, 0.45),
                sectors=self._all_direction_sectors(),
            ))
        else:
            obstacle_sectors, cliff_sectors = self._blocked_direction_sectors(
                obstacle_xy, cliff_xy)
            marker_array.markers.append(self._sector_marker(
                stamp,
                marker_id=3,
                ns='obstacle_blocked_directions',
                color=(1.0, 0.22, 0.0, self._alpha()),
                sectors=obstacle_sectors,
            ))
            marker_array.markers.append(self._sector_marker(
                stamp,
                marker_id=4,
                ns='cliff_blocked_directions',
                color=(0.55, 0.0, 1.0, self._alpha()),
                sectors=cliff_sectors,
            ))

        self._blocked_directions_pub.publish(marker_array)

    def _blocked_direction_sectors(
        self,
        obstacle_xy: np.ndarray,
        cliff_xy: np.ndarray,
    ) -> tuple[list[tuple[float, float]], list[tuple[float, float]]]:
        bins = max(int(self._config.blocked_directions_bins), 8)
        step = 2.0 * math.pi / bins
        obstacle_sectors: list[tuple[float, float]] = []
        cliff_sectors: list[tuple[float, float]] = []
        for idx in range(bins):
            start = -math.pi + idx * step
            end = start + step
            center = 0.5 * (start + end)
            heading = np.asarray([math.cos(center), math.sin(center)], dtype=np.float64)
            blockage = classify_hazard_heading(
                heading,
                obstacle_xy,
                cliff_xy,
                self._config.filter_config,
            )
            if blockage.blocked_by_cliff:
                cliff_sectors.append((start, end))
            elif blockage.blocked_by_obstacle:
                obstacle_sectors.append((start, end))
        return obstacle_sectors, cliff_sectors

    def _all_direction_sectors(self) -> list[tuple[float, float]]:
        bins = max(int(self._config.blocked_directions_bins), 8)
        step = 2.0 * math.pi / bins
        return [
            (-math.pi + idx * step, -math.pi + (idx + 1) * step)
            for idx in range(bins)
        ]

    def _is_stale(
        self,
        now_ns: int,
        obstacle_stamp_ns: int | None,
        cliff_stamp_ns: int | None,
    ) -> bool:
        timeout_s = float(self._config.hazard_timeout_s)
        if timeout_s <= 0.0:
            return (
                (self._obstacle_required and obstacle_stamp_ns is None)
                or (self._cliff_required and cliff_stamp_ns is None)
            )
        if self._obstacle_required and obstacle_stamp_ns is None:
            return True
        if self._cliff_required and cliff_stamp_ns is None:
            return True
        timeout_ns = int(timeout_s * 1e9)
        return (
            (
                self._obstacle_required
                and obstacle_stamp_ns is not None
                and now_ns - obstacle_stamp_ns > timeout_ns
            )
            or (
                self._cliff_required
                and cliff_stamp_ns is not None
                and now_ns - cliff_stamp_ns > timeout_ns
            )
        )

    def _on_obstacle_points(self, msg: PointCloud2) -> None:
        self._set_obstacle_xy(_xy_from_pointcloud2(msg))

    def _on_cliff_points(self, msg: PointCloud2) -> None:
        self._set_cliff_xy(_xy_from_pointcloud2(msg))

    def _set_obstacle_xy(self, xy: np.ndarray) -> None:
        now_ns = self._node.get_clock().now().nanoseconds
        with self._lock:
            self._obstacle_xy = xy
            self._obstacle_stamp_ns = now_ns

    def _set_cliff_xy(self, xy: np.ndarray) -> None:
        now_ns = self._node.get_clock().now().nanoseconds
        with self._lock:
            self._cliff_xy = xy
            self._cliff_stamp_ns = now_ns

    def _delete_all_marker(self, stamp) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._config.blocked_directions_frame
        marker.header.stamp = stamp
        marker.action = Marker.DELETEALL
        return marker

    def _circle_marker(self, stamp, marker_id: int, radius_m: float) -> Marker:
        marker = self._base_marker(stamp, marker_id, 'blocked_directions_outline')
        marker.type = Marker.LINE_STRIP
        marker.scale.x = 0.015
        marker.color.r = 0.75
        marker.color.g = 0.75
        marker.color.b = 0.75
        marker.color.a = 0.65
        bins = max(int(self._config.blocked_directions_bins), 24)
        for idx in range(bins + 1):
            angle = 2.0 * math.pi * idx / bins
            marker.points.append(self._point(
                radius_m * math.cos(angle), radius_m * math.sin(angle)))
        return marker

    def _sector_marker(
        self,
        stamp,
        *,
        marker_id: int,
        ns: str,
        color: tuple[float, float, float, float],
        sectors: list[tuple[float, float]],
    ) -> Marker:
        marker = self._base_marker(stamp, marker_id, ns)
        marker.type = Marker.TRIANGLE_LIST
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        inner = self._inner_radius()
        outer = self._outer_radius()
        for start, end in sectors:
            p0 = self._point(inner * math.cos(start), inner * math.sin(start))
            p1 = self._point(outer * math.cos(start), outer * math.sin(start))
            p2 = self._point(outer * math.cos(end), outer * math.sin(end))
            p3 = self._point(inner * math.cos(end), inner * math.sin(end))
            marker.points.extend([p0, p1, p2, p0, p2, p3])
        return marker

    def _base_marker(self, stamp, marker_id: int, ns: str) -> Marker:
        marker = Marker()
        marker.header.frame_id = self._config.blocked_directions_frame
        marker.header.stamp = stamp
        marker.ns = ns
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        return marker

    def _point(self, x: float, y: float) -> Point:
        point = Point()
        point.x = float(x)
        point.y = float(y)
        point.z = float(self._config.blocked_directions_z_m)
        return point

    def _inner_radius(self) -> float:
        return max(float(self._config.blocked_directions_inner_radius_m), 0.0)

    def _outer_radius(self) -> float:
        inner = self._inner_radius()
        return max(float(self._config.blocked_directions_outer_radius_m), inner + 0.05)

    def _alpha(self) -> float:
        return min(1.0, max(0.0, float(self._config.blocked_directions_alpha)))


def _xy_from_pointcloud2(msg: PointCloud2) -> np.ndarray:
    xyz = read_xyz_from_pointcloud2(msg)
    if len(xyz) == 0:
        return np.zeros((0, 2), dtype=np.float64)
    return np.ascontiguousarray(xyz[:, :2], dtype=np.float64)


def declare_hazard_filter_parameters(node: Node) -> None:
    node.declare_parameter('obstacle_points_topic', '/under_base_hazard/obstacle_points')
    node.declare_parameter('cliff_points_topic', '/under_base_hazard/cliff_points')
    node.declare_parameter('hazard_timeout_s', 0.5)
    node.declare_parameter('stop_linear_on_stale', True)
    node.declare_parameter('lookahead_m', 0.50)
    node.declare_parameter('cliff_lookahead_m', 0.80)
    node.declare_parameter('footprint_m', 0.35)
    node.declare_parameter('obstacle_buffer_m', 0.02)
    node.declare_parameter('hard_obstacle_buffer_m', 0.01)
    node.declare_parameter('soft_obstacle_buffer_m', 0.10)
    node.declare_parameter('min_clearance_speed_scale', 0.20)
    node.declare_parameter('creep_linear_speed_mps', 0.05)
    node.declare_parameter('cliff_buffer_m', 0.20)
    node.declare_parameter('min_linear_speed_mps', 1e-4)
    node.declare_parameter('blocked_directions_topic', '/under_base_hazard/blocked_directions')
    node.declare_parameter('blocked_directions_frame', 'base_link')
    node.declare_parameter('blocked_directions_publish_rate_hz', 5.0)
    node.declare_parameter('blocked_directions_bins', 72)
    node.declare_parameter('blocked_directions_inner_radius_m', 0.35)
    node.declare_parameter('blocked_directions_outer_radius_m', 0.95)
    node.declare_parameter('blocked_directions_z_m', 0.03)
    node.declare_parameter('blocked_directions_alpha', 0.55)


def hazard_point_cache_config_from_params(node: Node) -> HazardPointCacheConfig:
    get = node.get_parameter
    return HazardPointCacheConfig(
        obstacle_points_topic=str(get('obstacle_points_topic').value),
        cliff_points_topic=str(get('cliff_points_topic').value),
        hazard_timeout_s=float(get('hazard_timeout_s').value),
        stop_linear_on_stale=bool(get('stop_linear_on_stale').value),
        filter_config=HazardVelFilterConfig(
            lookahead_m=float(get('lookahead_m').value),
            cliff_lookahead_m=float(get('cliff_lookahead_m').value),
            footprint_m=float(get('footprint_m').value),
            obstacle_buffer_m=float(get('obstacle_buffer_m').value),
            hard_obstacle_buffer_m=float(get('hard_obstacle_buffer_m').value),
            soft_obstacle_buffer_m=float(get('soft_obstacle_buffer_m').value),
            min_clearance_speed_scale=float(get('min_clearance_speed_scale').value),
            creep_linear_speed_mps=float(get('creep_linear_speed_mps').value),
            cliff_buffer_m=float(get('cliff_buffer_m').value),
            min_linear_speed_mps=float(get('min_linear_speed_mps').value),
        ),
        blocked_directions_topic=str(get('blocked_directions_topic').value),
        blocked_directions_frame=str(get('blocked_directions_frame').value),
        blocked_directions_publish_rate_hz=float(get('blocked_directions_publish_rate_hz').value),
        blocked_directions_bins=int(get('blocked_directions_bins').value),
        blocked_directions_inner_radius_m=float(get('blocked_directions_inner_radius_m').value),
        blocked_directions_outer_radius_m=float(get('blocked_directions_outer_radius_m').value),
        blocked_directions_z_m=float(get('blocked_directions_z_m').value),
        blocked_directions_alpha=float(get('blocked_directions_alpha').value),
    )
