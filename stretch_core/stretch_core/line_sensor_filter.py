"""Line-sensor projection and pre-grid hazard filtering."""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np

max_line_sensor_range = 4.0


class BinClass(IntEnum):
    UNKNOWN = 0
    FREE = 1
    OBSTACLE = 2
    SMALL_DROP = 3
    SPRAY = 4


@dataclass
class LineSensorConfig:
    line_obstacle_min_height_m: float = 0.025
    floor_band_m: float = 0.015
    cliff_min_drop_m: float = 0.02
    cliff_max_drop_m: float = 0.10
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
    line_spray_merge_gap_bins: int = 6
    line_radial_streak_head_radius_max_m: float = 0.35
    line_radial_streak_span_min_m: float = 0.04
    line_radial_streak_angular_spread_max_deg: float = 20.0
    line_radial_streak_aspect_ratio_min: float = 3.0
    spray_min_run_bins: int = 3
    spray_roughness_thresh_m: float = 0.03
    spray_max_run_bins: int = 0
    spray_head_radius_max_m: float = 0.30
    spray_radial_span_min_m: float = 0.05
    spray_angular_spread_max_deg: float = 15.0
    spray_aspect_ratio_min: float = 5.0
    spray_direction_cluster_gap_deg: float = 5.0
    spray_monotonic_score_min: float = 0.70
    spray_monotonic_tolerance_m: float = 0.005
    spray_short_run_bonus_max_bins: int = 15
    spray_temporal_window_frames: int = 5
    spray_temporal_stable_min_frames: int = 2
    spray_temporal_stable_fraction: float = 0.50


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
        self._raw_history: deque[dict[tuple[int, int], BinClass]] = deque(
            maxlen=max(
                config.line_window_frames,
                config.spray_temporal_window_frames,
                1,
            ),
        )

    def project_all(self, status: dict) -> np.ndarray:
        points: list[np.ndarray] = []
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
                & (ranges < max_line_sensor_range)
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
                    or ranges[bin_idx] >= max_line_sensor_range
                ):
                    continue
                pt = projected[bin_idx]
                cls = self._classify_bin(pt[2])
                if cls in (BinClass.OBSTACLE, BinClass.SMALL_DROP):
                    candidates.append((sensor_idx, bin_idx, cls, pt))

        self._raw_history.append({
            (sensor_idx, bin_idx): cls
            for sensor_idx, bin_idx, cls, _pt in candidates
        })

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

    def _sensor_origin_xy(self, sensor_idx: int) -> np.ndarray:
        pos_angle_deg = sum(self.geometry.sensor_angles[: sensor_idx + 1])
        pos_angle_rad = -np.deg2rad(pos_angle_deg)
        radius_m = (self.geometry.param_diameter_cm / 2.0) / 100.0
        return np.array([
            radius_m * np.cos(pos_angle_rad),
            radius_m * np.sin(pos_angle_rad),
        ], dtype=np.float64)

    def _sensor_radial_metrics(
        self,
        run: list[tuple[int, int, BinClass, np.ndarray]],
    ) -> dict[str, float] | None:
        if not run:
            return None

        sensor_idx = run[0][0]
        xy = np.vstack([item[3][:2] for item in run]).astype(np.float64)
        origin = self._sensor_origin_xy(sensor_idx)
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

    def _run_temporally_unstable(
        self,
        run: list[tuple[int, int, BinClass, np.ndarray]],
    ) -> bool:
        cfg = self.config
        window_frames = cfg.spray_temporal_window_frames
        if window_frames <= 0 or len(self._raw_history) < window_frames:
            return False

        recent = list(self._raw_history)[-window_frames:]
        stable_bins = 0
        for sensor_idx, bin_idx, hazard_cls, _pt in run:
            key = (sensor_idx, bin_idx)
            hits = sum(1 for hist in recent if hist.get(key) == hazard_cls)
            if hits >= cfg.spray_temporal_stable_min_frames:
                stable_bins += 1

        stable_fraction = stable_bins / max(len(run), 1)
        return stable_fraction < cfg.spray_temporal_stable_fraction

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
            spray_keys = self._spray_keys_for_sensor(items)
            for sensor_idx, bin_idx, _cls, pt in items:
                if (sensor_idx, bin_idx) in spray_keys:
                    out.append((sensor_idx, bin_idx, BinClass.SPRAY, pt))

            for hazard_cls in (BinClass.OBSTACLE, BinClass.SMALL_DROP):
                runs: list[list[tuple[int, int, BinClass, np.ndarray]]] = []
                run: list[tuple[int, int, BinClass, np.ndarray]] = []
                last_bin: int | None = None
                for item in items:
                    if (item[0], item[1]) in spray_keys:
                        if run:
                            runs.append(run)
                        run = []
                        last_bin = None
                        continue
                    bin_idx = item[1]
                    if item[2] != hazard_cls:
                        if run:
                            runs.append(run)
                        run = []
                        last_bin = None
                        continue
                    if last_bin is None or bin_idx == last_bin + 1:
                        run.append(item)
                    else:
                        if run:
                            runs.append(run)
                        run = [item]
                    last_bin = bin_idx
                if run:
                    runs.append(run)
                self._append_class_runs(out, runs)
        return out

    def _spray_keys_for_sensor(
        self,
        items: list[tuple[int, int, BinClass, np.ndarray]],
    ) -> set[tuple[int, int]]:
        max_gap = max(int(self.config.line_spray_merge_gap_bins), 0)
        keys: set[tuple[int, int]] = set()
        group: list[tuple[int, int, BinClass, np.ndarray]] = []
        last_bin: int | None = None

        for item in items:
            bin_idx = item[1]
            if last_bin is None or bin_idx - last_bin <= max_gap + 1:
                group.append(item)
            else:
                keys.update(self._spray_keys_for_group(group))
                group = [item]
            last_bin = bin_idx

        keys.update(self._spray_keys_for_group(group))
        return keys

    def _spray_keys_for_group(
        self,
        group: list[tuple[int, int, BinClass, np.ndarray]],
    ) -> set[tuple[int, int]]:
        if self._is_spray(group):
            return {(sensor_idx, bin_idx) for sensor_idx, bin_idx, _cls, _pt in group}

        keys: set[tuple[int, int]] = set()
        for cluster in self._direction_clusters(group):
            if len(cluster) == len(group):
                continue
            if self._is_spray(cluster):
                keys.update(
                    (sensor_idx, bin_idx)
                    for sensor_idx, bin_idx, _cls, _pt in cluster
                )
        return keys

    def _direction_clusters(
        self,
        group: list[tuple[int, int, BinClass, np.ndarray]],
    ) -> list[list[tuple[int, int, BinClass, np.ndarray]]]:
        if len(group) < self.config.spray_min_run_bins:
            return []

        sensor_idx = group[0][0]
        origin = self._sensor_origin_xy(sensor_idx)
        decorated: list[tuple[float, tuple[int, int, BinClass, np.ndarray]]] = []
        for item in group:
            vector = item[3][:2].astype(np.float64) - origin
            radius = float(np.linalg.norm(vector))
            if not np.isfinite(radius) or radius <= 1e-6:
                continue
            decorated.append((float(np.arctan2(vector[1], vector[0])), item))

        if len(decorated) < self.config.spray_min_run_bins:
            return []

        decorated.sort(key=lambda x: x[0])
        angles = np.array([angle for angle, _item in decorated], dtype=np.float64)
        gaps = np.diff(angles)
        wrap_gap = float((angles[0] + 2.0 * np.pi) - angles[-1])
        if gaps.size > 0 and float(np.max(gaps)) > wrap_gap:
            split_idx = int(np.argmax(gaps)) + 1
            decorated = decorated[split_idx:] + decorated[:split_idx]
            angles = np.array([angle for angle, _item in decorated], dtype=np.float64)
            angles = np.unwrap(angles)

        max_gap_rad = np.deg2rad(max(float(self.config.spray_direction_cluster_gap_deg), 0.0))
        clusters: list[list[tuple[int, int, BinClass, np.ndarray]]] = []
        cluster = [decorated[0][1]]
        last_angle = float(angles[0])
        for angle, item in zip(angles[1:], [item for _angle, item in decorated[1:]]):
            if float(angle) - last_angle > max_gap_rad:
                if len(cluster) >= self.config.spray_min_run_bins:
                    clusters.append(cluster)
                cluster = [item]
            else:
                cluster.append(item)
            last_angle = float(angle)

        if len(cluster) >= self.config.spray_min_run_bins:
            clusters.append(cluster)
        return clusters

    def _append_class_runs(
        self,
        out: list[tuple[int, int, BinClass, np.ndarray]],
        runs: list[list[tuple[int, int, BinClass, np.ndarray]]],
    ) -> None:
        if not runs:
            return

        max_gap = max(int(self.config.line_spray_merge_gap_bins), 0)
        if max_gap == 0:
            for run in runs:
                self._append_run(out, run)
            return

        group: list[list[tuple[int, int, BinClass, np.ndarray]]] = []
        for run in runs:
            if not group:
                group = [run]
                continue

            gap = run[0][1] - group[-1][-1][1] - 1
            if gap <= max_gap:
                group.append(run)
            else:
                self._append_run_group(out, group)
                group = [run]

        self._append_run_group(out, group)

    def _append_run_group(
        self,
        out: list[tuple[int, int, BinClass, np.ndarray]],
        group: list[list[tuple[int, int, BinClass, np.ndarray]]],
    ) -> None:
        if len(group) == 1:
            self._append_run(out, group[0])
            return

        merged = [item for run in group for item in run]
        if self._is_spray(merged):
            out.extend((s, b, BinClass.SPRAY, pt) for s, b, _cls, pt in merged)
            return

        for run in group:
            self._append_run(out, run)

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

        metrics = self._sensor_radial_metrics(run)
        if metrics is None:
            return False

        if metrics['head_radius'] > cfg.spray_head_radius_max_m:
            return False
        if metrics['radial_span'] < cfg.spray_radial_span_min_m:
            return False

        narrow = metrics['angular_spread_deg'] <= cfg.spray_angular_spread_max_deg
        loose_narrow = metrics['angular_spread_deg'] <= cfg.line_radial_streak_angular_spread_max_deg
        thin = (
            metrics['aspect_ratio'] >= cfg.spray_aspect_ratio_min
            or metrics['extent_perp'] <= cfg.spray_roughness_thresh_m
        )
        monotonic = metrics['monotonic_score'] >= cfg.spray_monotonic_score_min
        unstable = self._run_temporally_unstable(run)
        short = n <= cfg.spray_short_run_bonus_max_bins

        if narrow and thin and monotonic:
            return True
        if narrow and thin and (unstable or short):
            return True
        return loose_narrow and thin and monotonic and unstable

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
        if self._is_radial_streak_noise(run):
            return False
        if cfg.line_max_run_radial_span_m > 0.0:
            xy = np.vstack([item[3][:2] for item in run])
            radial_span = float(np.ptp(np.linalg.norm(xy, axis=1)))
            if radial_span > cfg.line_max_run_radial_span_m:
                return False
        return True

    def _is_radial_streak_noise(
        self,
        run: list[tuple[int, int, BinClass, np.ndarray]],
    ) -> bool:
        cfg = self.config
        metrics = self._sensor_radial_metrics(run)
        if metrics is None:
            return False
        if metrics['head_radius'] > cfg.line_radial_streak_head_radius_max_m:
            return False
        if metrics['radial_span'] < cfg.line_radial_streak_span_min_m:
            return False
        narrow = metrics['angular_spread_deg'] <= cfg.line_radial_streak_angular_spread_max_deg
        thin = (
            metrics['aspect_ratio'] >= cfg.line_radial_streak_aspect_ratio_min
            or metrics['extent_perp'] <= cfg.spray_roughness_thresh_m
        )
        monotonic = metrics['monotonic_score'] >= cfg.spray_monotonic_score_min
        return narrow and thin and monotonic

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
