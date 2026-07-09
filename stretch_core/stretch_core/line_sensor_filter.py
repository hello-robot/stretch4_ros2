"""Line-sensor projection and pre-grid hazard filtering."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np

DEFAULT_LINE_SENSOR_RADIUS_M = 0.4


class BinClass(IntEnum):
    UNKNOWN = 0
    FREE = 1
    OBSTACLE = 2
    SMALL_DROP = 3
    SPRAY = 4


@dataclass
class LineSensorConfig:
    max_range: float = 4.0
    line_obstacle_min_height_m: float = 0.025
    floor_band_m: float = 0.015
    cliff_min_drop_m: float = 0.02
    cliff_max_drop_m: float = 0.10
    line_sensor_radius_m: float = DEFAULT_LINE_SENSOR_RADIUS_M
    line_min_run_bins: int = 3
    line_max_run_radial_span_m: float = 0.25
    line_point_noise_max_run_bins: int = 12
    line_point_noise_xy_span_max_m: float = 0.010
    line_point_noise_radial_span_max_m: float = 0.010
    line_confirm_frames: int = 3
    line_fast_confirm_frames: int = 2
    line_fast_confirm_range_m: float = 0.55
    line_window_frames: int = 4
    line_require_consecutive: bool = True
    spray_min_run_bins: int = 16
    spray_max_run_bins: int = 96
    spray_depth_p2p_min_m: float = 0.07
    spray_residual_p2p_min_m: float = 0.06
    spray_jump_p90_min_m: float = 0.010
    spray_large_jump_min_count: int = 4
    spray_large_jump_m: float = 0.015
    spray_turn_min_count: int = 2
    spray_turn_jump_m: float = 0.010
    spray_path_ratio_min: float = 1.75


@dataclass
class LineSensorHits:
    obstacle_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    small_drop_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    raw_obstacle_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    raw_small_drop_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    spatial_obstacle_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    spatial_small_drop_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    raw_spray_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))


def as_range_array(ranges_raw) -> np.ndarray:
    if ranges_raw is None:
        return np.array([], dtype=np.float64)
    return np.asarray(ranges_raw, dtype=np.float64)


class LineSensorSource:
    """Classify projected line-sensor bins into obstacle and small-drop evidence."""

    def __init__(
        self,
        geometry,
        sensor_names: list[str],
        config: LineSensorConfig,
        apply_tare=None,
    ):
        self.geometry = geometry
        self.sensor_names = sensor_names
        self.config = config
        self.apply_tare = apply_tare
        self._angles = self.geometry.get_angles()
        self._tan_angles = np.tan(self._angles)
        self._history: deque[dict[tuple[int, int], BinClass]] = deque(
            maxlen=max(
                config.line_window_frames,
                config.line_confirm_frames,
                config.line_fast_confirm_frames,
                1,
            ),
        )

    def project_all(self, status: dict) -> np.ndarray:
        points: list[np.ndarray] = []
        cfg = self.config
        for sensor_idx, sensor_name in enumerate(self.sensor_names):
            sensor_status = status.get(sensor_name, {})
            if not isinstance(sensor_status, dict):
                continue
            ranges = self._ranges_for_sensor(sensor_status, sensor_name)
            if ranges.size == 0:
                continue
            projected = self._project_sensor_bins(sensor_idx, ranges)
            valid = (
                np.isfinite(ranges)
                & (ranges > 0.0)
                & (ranges < cfg.max_range)
                & ((projected[:, 0] ** 2 + projected[:, 1] ** 2) <= cfg.line_sensor_radius_m ** 2)
            )
            if np.any(valid):
                points.append(projected[valid])
        return np.vstack(points) if points else np.zeros((0, 3))

    def process(self, status: dict) -> LineSensorHits:
        cfg = self.config
        candidates: list[tuple[int, int, BinClass, np.ndarray]] = []

        for sensor_idx, sensor_name in enumerate(self.sensor_names):
            sensor_status = status.get(sensor_name, {})
            if not isinstance(sensor_status, dict):
                continue
            ranges = self._ranges_for_sensor(sensor_status, sensor_name)
            if ranges.size == 0:
                continue

            projected = self._project_sensor_bins(sensor_idx, ranges)
            for bin_idx in range(len(ranges)):
                if (
                    not np.isfinite(ranges[bin_idx])
                    or ranges[bin_idx] <= 0.0
                    or ranges[bin_idx] >= cfg.max_range
                ):
                    continue
                pt = projected[bin_idx]
                r2 = pt[0] * pt[0] + pt[1] * pt[1]
                if r2 > cfg.line_sensor_radius_m * cfg.line_sensor_radius_m:
                    continue
                cls = self._classify_bin(pt[2])
                if cls in (BinClass.OBSTACLE, BinClass.SMALL_DROP):
                    candidates.append((sensor_idx, bin_idx, cls, pt))

        gated = self._spatial_gate(candidates)
        self._history.append({
            (sensor_idx, bin_idx): cls
            for sensor_idx, bin_idx, cls, _pt in gated
        })

        promoted: list[tuple[int, int, BinClass, np.ndarray]] = []
        for sensor_idx, bin_idx, cls, pt in gated:
            confirm_frames = self._confirm_frames_for_bin(pt)
            if self._bin_confirmed(sensor_idx, bin_idx, cls, confirm_frames):
                promoted.append((sensor_idx, bin_idx, cls, pt))

        return LineSensorHits(
            obstacle_xy=_items_to_xy(promoted, BinClass.OBSTACLE),
            small_drop_xy=_items_to_xy(promoted, BinClass.SMALL_DROP),
            raw_obstacle_xy=_items_to_xy(candidates, BinClass.OBSTACLE),
            raw_small_drop_xy=_items_to_xy(candidates, BinClass.SMALL_DROP),
            spatial_obstacle_xy=_items_to_xy(gated, BinClass.OBSTACLE),
            spatial_small_drop_xy=_items_to_xy(gated, BinClass.SMALL_DROP),
            raw_spray_xy=_items_to_xy(gated, BinClass.SPRAY),
        )

    def _ranges_for_sensor(self, sensor_status: dict, sensor_name: str) -> np.ndarray:
        ranges = as_range_array(sensor_status.get('ranges'))
        if ranges.size == 0:
            return ranges
        if self.apply_tare is not None:
            ranges = self.apply_tare(ranges, sensor_name)
        return ranges

    def _project_sensor_bins(self, sensor_idx: int, ranges: np.ndarray) -> np.ndarray:
        slant = np.asarray(ranges, dtype=np.float64)
        x_s = np.where(self._tan_angles != 0.0, slant / self._tan_angles, 0.0)
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

    def _classify_bin(self, z: float) -> BinClass:
        cfg = self.config
        if not np.isfinite(z):
            return BinClass.UNKNOWN
        if abs(z) <= cfg.floor_band_m:
            return BinClass.FREE
        if z >= cfg.line_obstacle_min_height_m:
            return BinClass.OBSTACLE
        drop = -z
        if cfg.cliff_min_drop_m <= drop <= cfg.cliff_max_drop_m:
            return BinClass.SMALL_DROP
        return BinClass.UNKNOWN

    def _spatial_gate(
        self,
        classifications: list[tuple[int, int, BinClass, np.ndarray]],
    ) -> list[tuple[int, int, BinClass, np.ndarray]]:
        by_sensor: dict[int, list[tuple[int, int, BinClass, np.ndarray]]] = {}
        for item in classifications:
            by_sensor.setdefault(item[0], []).append(item)

        out: list[tuple[int, int, BinClass, np.ndarray]] = []
        for _sensor_idx, items in by_sensor.items():
            items.sort(key=lambda x: x[1])
            for hazard_cls in (BinClass.OBSTACLE, BinClass.SMALL_DROP):
                run: list[tuple[int, int, BinClass, np.ndarray]] = []
                last_bin: int | None = None
                for item in items:
                    bin_idx = item[1]
                    if item[2] != hazard_cls:
                        self._append_run(out, run)
                        run = []
                        last_bin = None
                        continue
                    if last_bin is None or bin_idx == last_bin + 1:
                        run.append(item)
                    else:
                        self._append_run(out, run)
                        run = [item]
                    last_bin = bin_idx
                self._append_run(out, run)
        return out

    def _append_run(
        self,
        out: list[tuple[int, int, BinClass, np.ndarray]],
        run: list[tuple[int, int, BinClass, np.ndarray]],
    ) -> None:
        if not run:
            return
        if self._is_spray(run):
            out.extend((s, b, BinClass.SPRAY, pt) for s, b, _cls, pt in run)
        elif self._is_point_noise(run):
            return
        elif self._valid_run(run):
            out.extend(run)

    def _is_spray(self, run: list[tuple[int, int, BinClass, np.ndarray]]) -> bool:
        cfg = self.config
        n = len(run)
        if n < cfg.spray_min_run_bins:
            return False
        if cfg.spray_max_run_bins > 0 and n > cfg.spray_max_run_bins:
            return False

        xy = np.vstack([item[3][:2] for item in run]).astype(np.float64)
        depths = np.linalg.norm(xy, axis=1)
        if not np.all(np.isfinite(depths)):
            return False

        depth_p2p = float(np.ptp(depths))
        if depth_p2p < cfg.spray_depth_p2p_min_m:
            return False

        bin_axis = np.arange(n, dtype=np.float64)
        slope, intercept = np.polyfit(bin_axis, depths, 1)
        residual = depths - (slope * bin_axis + intercept)
        residual_p2p = float(np.ptp(residual))
        if residual_p2p < cfg.spray_residual_p2p_min_m:
            return False

        abs_jumps = np.abs(np.diff(depths))
        if abs_jumps.size == 0:
            return False
        if float(np.percentile(abs_jumps, 90.0)) < cfg.spray_jump_p90_min_m:
            return False
        if int(np.count_nonzero(abs_jumps >= cfg.spray_large_jump_m)) < cfg.spray_large_jump_min_count:
            return False

        jumps = np.diff(depths)
        significant = jumps[np.abs(jumps) >= cfg.spray_turn_jump_m]
        if significant.size < 2:
            return False
        signs = np.sign(significant)
        turn_count = int(np.count_nonzero(signs[1:] * signs[:-1] < 0.0))
        if turn_count < cfg.spray_turn_min_count:
            return False

        path_len = float(np.sum(abs_jumps))
        return (path_len / max(depth_p2p, 1e-6)) >= cfg.spray_path_ratio_min

    def _is_point_noise(self, run: list[tuple[int, int, BinClass, np.ndarray]]) -> bool:
        cfg = self.config
        if run[0][2] != BinClass.OBSTACLE:
            return False
        if len(run) > cfg.line_point_noise_max_run_bins:
            return False

        xy = np.vstack([item[3][:2] for item in run]).astype(np.float64)
        centroid = np.mean(xy, axis=0)
        xy_span = float(np.max(np.linalg.norm(xy - centroid, axis=1)))
        if xy_span > cfg.line_point_noise_xy_span_max_m:
            return False

        radial_span = float(np.ptp(np.linalg.norm(xy, axis=1)))
        return radial_span <= cfg.line_point_noise_radial_span_max_m

    def _valid_run(self, run: list[tuple[int, int, BinClass, np.ndarray]]) -> bool:
        cfg = self.config
        if len(run) < cfg.line_min_run_bins:
            return False
        if cfg.line_max_run_radial_span_m > 0.0:
            xy = np.vstack([item[3][:2] for item in run])
            radial_span = float(np.ptp(np.linalg.norm(xy, axis=1)))
            if radial_span > cfg.line_max_run_radial_span_m:
                return False
        return True

    def _bin_confirmed(
        self,
        sensor_idx: int,
        bin_idx: int,
        hazard_cls: BinClass,
        confirm_frames: int,
    ) -> bool:
        key = (sensor_idx, bin_idx)
        if len(self._history) < confirm_frames:
            return False

        if self.config.line_require_consecutive:
            recent = list(self._history)[-confirm_frames:]
            return all(hist.get(key) == hazard_cls for hist in recent)

        return sum(1 for hist in self._history if hist.get(key) == hazard_cls) >= confirm_frames

    def _confirm_frames_for_bin(self, pt: np.ndarray) -> int:
        radius = float(np.linalg.norm(pt[:2]))
        if radius <= self.config.line_fast_confirm_range_m:
            return self.config.line_fast_confirm_frames
        return self.config.line_confirm_frames


def _items_to_xy(
    items: list[tuple[int, int, BinClass, np.ndarray]],
    cls: BinClass,
) -> np.ndarray:
    points = [pt[:2] for _sensor_idx, _bin_idx, item_cls, pt in items if item_cls == cls]
    return np.vstack(points) if points else np.zeros((0, 2))
