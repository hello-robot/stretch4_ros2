"""The foundation layer: everything that turns a slant range into a place.

`Projector` wraps the hardware `geometry` object and owns all the per-bin math
the stages lean on — projecting a bin to (x, y, z), where each ray would meet a
flat floor, each bin's world bearing, the rolling-median contrast, and the
shape metrics of a run. It is built once and caches the geometry-only results,
so calling it per frame adds no work beyond the arithmetic. Nothing here makes
a hazard decision — it only measures. See README.md → "From range to a point".
"""

from __future__ import annotations

import numpy as np

from .config import LineSensorConfig
from .hits import max_line_sensor_range


class Projector:
    def __init__(self, geometry, config: LineSensorConfig):
        self.geometry = geometry
        self.config = config
        self.angles = self.geometry.get_angles()
        self.tan_angles = np.tan(self.angles)
        # Geometry-only results (independent of the live ranges) cached per
        # sensor: the floor-intersection xy and the per-bin world bearings.
        self._floor_xy_cache: dict[int, np.ndarray] = {}
        self._bearing_cache: dict[int, np.ndarray] = {}

    def project(self, sensor_idx: int, ranges: np.ndarray) -> np.ndarray:
        """Slant ranges -> (N, 3) points in the robot floor frame."""
        slant = np.asarray(ranges, dtype=np.float64)
        x_s = np.where(self.tan_angles != 0.0, slant / self.tan_angles, 0.0)
        y_s = slant
        x_b, y_b, z_b = self.geometry.to_floor_coordinate_system(x_s, y_s)

        rot_angle_rad = -np.deg2rad(self.geometry.sensor_normals[sensor_idx])
        cos_r = np.cos(rot_angle_rad)
        sin_r = np.sin(rot_angle_rad)
        x_rot = x_b * cos_r - y_b * sin_r
        y_rot = x_b * sin_r + y_b * cos_r

        pos_angle_deg = sum(self.geometry.sensor_angles[: sensor_idx + 1])
        pos_angle_rad = -np.deg2rad(pos_angle_deg)
        r_m = (self.geometry.param_diameter_cm / 2.0) / 100.0
        sx = r_m * np.cos(pos_angle_rad)
        sy = r_m * np.sin(pos_angle_rad)

        return np.column_stack([x_rot + sx, y_rot + sy, z_b])

    def sensor_origin(self, sensor_idx: int) -> np.ndarray:
        """The (x, y) mount point of a sensor on the base rim."""
        pos_angle_deg = sum(self.geometry.sensor_angles[: sensor_idx + 1])
        pos_angle_rad = -np.deg2rad(pos_angle_deg)
        radius_m = (self.geometry.param_diameter_cm / 2.0) / 100.0
        return np.array([
            radius_m * np.cos(pos_angle_rad),
            radius_m * np.sin(pos_angle_rad),
        ], dtype=np.float64)

    def floor_intersections(self, sensor_idx: int) -> np.ndarray:
        """Per-bin xy where each ray meets a flat floor (Nx2, cached)."""
        cached = self._floor_xy_cache.get(sensor_idx)
        if cached is not None:
            return cached
        height_m = self.geometry.param_height_cm / 100.0
        angle_down_deg = getattr(self.geometry, 'angle_down_deg', 26.0)
        r_ideal = height_m / np.sin(np.deg2rad(angle_down_deg))
        xy = self.project(
            sensor_idx,
            np.full(len(self.angles), r_ideal, dtype=np.float64),
        )[:, :2]
        self._floor_xy_cache[sensor_idx] = xy
        return xy

    def bin_bearings(self, sensor_idx: int) -> np.ndarray:
        """Per-bin world bearing, degrees clockwise from robot forward.

        Higher bin index = clockwise (verified empirically 2026-07-21)."""
        cached = self._bearing_cache.get(sensor_idx)
        if cached is not None:
            return cached
        n = len(self.angles)
        fov_deg = np.rad2deg(abs(self.angles[0] - self.angles[-1]))
        offsets = (np.arange(n) - (n - 1) / 2.0) * fov_deg / max(n - 1, 1)
        bearings = float(self.geometry.sensor_normals[sensor_idx]) + offsets
        self._bearing_cache[sensor_idx] = bearings
        return bearings

    def local_contrast(self, z: np.ndarray, ranges: np.ndarray) -> np.ndarray:
        """z minus the rolling median of surrounding valid bins."""
        window = max(int(self.config.contrast_window_bins), 3)
        half = window // 2
        zv = np.where(
            np.isfinite(ranges) & (ranges > 0.0) & (ranges < max_line_sensor_range),
            z,
            np.nan,
        )
        padded = np.concatenate([np.full(half, np.nan), zv, np.full(half, np.nan)])
        windows = np.lib.stride_tricks.sliding_window_view(padded, window)
        with np.errstate(all='ignore'):
            baseline = np.nanmedian(windows, axis=1)
        baseline = np.where(np.isfinite(baseline), baseline, 0.0)
        return z - baseline

    def radial_metrics(self, run) -> dict[str, float] | None:
        """Shape descriptors of a run of bins as seen from the sensor origin:
        head radius, radial span, angular spread, extents, aspect ratio, and a
        monotonic-profile score. Returns None if the run is degenerate."""
        if not run:
            return None

        sensor_idx = run[0][0]
        xy = np.vstack([item[3][:2] for item in run]).astype(np.float64)
        origin = self.sensor_origin(sensor_idx)
        vectors = xy - origin
        ranges = np.linalg.norm(vectors, axis=1)
        if not np.all(np.isfinite(ranges)) or np.any(ranges <= 1e-6):
            return None

        directions = vectors / ranges[:, np.newaxis]
        dots = np.clip(directions @ directions.T, -1.0, 1.0)
        angular_spread_deg = float(np.rad2deg(np.arccos(np.min(dots))))

        mean_direction = np.mean(directions, axis=0)
        mean_norm = float(np.linalg.norm(mean_direction))
        if mean_norm <= 1e-6:
            return None
        mean_direction /= mean_norm

        along = vectors @ mean_direction
        perp_direction = np.array([-mean_direction[1], mean_direction[0]], dtype=np.float64)
        across = vectors @ perp_direction
        extent_para = float(np.ptp(along))
        extent_perp = float(np.ptp(across))
        aspect_ratio = extent_para / max(extent_perp, 1e-6)

        radial_diffs = np.diff(ranges)
        if radial_diffs.size == 0:
            monotonic_score = 0.0
        else:
            tol = max(float(self.config.spray_monotonic_tolerance_m), 0.0)
            increasing = float(np.mean(radial_diffs >= -tol))
            decreasing = float(np.mean(radial_diffs <= tol))
            monotonic_score = max(increasing, decreasing)

        return {
            'head_radius': float(np.min(ranges)),
            'radial_span': float(np.ptp(ranges)),
            'angular_spread_deg': angular_spread_deg,
            'extent_para': extent_para,
            'extent_perp': extent_perp,
            'aspect_ratio': aspect_ratio,
            'monotonic_score': monotonic_score,
        }
