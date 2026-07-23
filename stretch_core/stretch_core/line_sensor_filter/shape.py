"""Shape defenses: the original spray/streak/point-noise rejection.

`ShapeGate` groups the surviving candidates into contiguous per-sensor runs and
runs each run through geometry tests — is it a thin monotonic streak (spray)? a
tiny isolated blob (point noise)? too long/spread to be real? Runs separated by
a small gap are merged and re-tested so a broken-up streak can't slip through
piecewise.

`gate()` is called once per frame with a snapshot of recent candidate history
(for the temporal-instability test); all shape math is delegated to the shared
`Projector`. See README.md → "Filtering by shape".
"""

from __future__ import annotations

import numpy as np

from .config import LineSensorConfig
from .geometry import Projector
from .hits import BinClass, DROP_FAMILY, OBSTACLE_FAMILY


class ShapeGate:
    def __init__(self, config: LineSensorConfig, projector: Projector):
        self.config = config
        self.projector = projector
        # Snapshot of recent per-frame candidate maps, set for the duration of
        # each gate() call so the spray tests can ask "was this bin stable?".
        self._recent_raw: list = []

    def gate(self, classifications, recent_raw):
        self._recent_raw = recent_raw
        return self._spatial_gate(classifications)

    def _spatial_gate(self, classifications):
        by_sensor: dict[int, list] = {}
        for item in classifications:
            by_sensor.setdefault(item[0], []).append(item)

        out: list = []
        for _sensor_idx, items in by_sensor.items():
            items.sort(key=lambda x: x[1])
            spray_keys = self._spray_keys_for_sensor(items)
            for sensor_idx, bin_idx, _cls, pt in items:
                if (sensor_idx, bin_idx) in spray_keys:
                    out.append((sensor_idx, bin_idx, BinClass.SPRAY, pt))

            for family in (OBSTACLE_FAMILY, DROP_FAMILY):
                runs: list = []
                run: list = []
                last_bin: int | None = None
                for item in items:
                    if (item[0], item[1]) in spray_keys:
                        if run:
                            runs.append(run)
                        run = []
                        last_bin = None
                        continue
                    bin_idx = item[1]
                    if item[2] not in family:
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

    def _spray_keys_for_sensor(self, items):
        max_gap = max(int(self.config.line_spray_merge_gap_bins), 0)
        keys: set = set()
        group: list = []
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

    def _spray_keys_for_group(self, group):
        if self._is_spray(group):
            return {(sensor_idx, bin_idx) for sensor_idx, bin_idx, _cls, _pt in group}

        keys: set = set()
        for cluster in self._direction_clusters(group):
            if len(cluster) == len(group):
                continue
            if self._is_spray(cluster):
                keys.update(
                    (sensor_idx, bin_idx)
                    for sensor_idx, bin_idx, _cls, _pt in cluster
                )
        return keys

    def _direction_clusters(self, group):
        if len(group) < self.config.spray_min_run_bins:
            return []

        sensor_idx = group[0][0]
        origin = self.projector.sensor_origin(sensor_idx)
        decorated: list = []
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
        clusters: list = []
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

    def _append_class_runs(self, out, runs):
        if not runs:
            return

        max_gap = max(int(self.config.line_spray_merge_gap_bins), 0)
        if max_gap == 0:
            for run in runs:
                self._append_run(out, run)
            return

        group: list = []
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

    def _append_run_group(self, out, group):
        if len(group) == 1:
            self._append_run(out, group[0])
            return

        merged = [item for run in group for item in run]
        if self._is_spray(merged):
            out.extend((s, b, BinClass.SPRAY, pt) for s, b, _cls, pt in merged)
            return

        for run in group:
            self._append_run(out, run)

    def _append_run(self, out, run):
        if not run:
            return
        if self._is_spray(run):
            out.extend((s, b, BinClass.SPRAY, pt) for s, b, _cls, pt in run)
        elif self._is_point_noise(run):
            return
        elif self._valid_run(run):
            out.extend(run)

    def _is_spray(self, run) -> bool:
        cfg = self.config
        n = len(run)
        if n < cfg.spray_min_run_bins:
            return False
        if cfg.spray_max_run_bins > 0 and n > cfg.spray_max_run_bins:
            return False

        metrics = self.projector.radial_metrics(run)
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

    def _is_point_noise(self, run) -> bool:
        cfg = self.config
        if run[0][2] not in OBSTACLE_FAMILY:
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

    def _valid_run(self, run) -> bool:
        cfg = self.config
        if len(run) < cfg.line_min_run_bins:
            return False
        strong_bins = sum(1 for item in run if item[2] != BinClass.OBSTACLE_MARGINAL)
        if strong_bins < cfg.line_min_run_bins and len(run) < cfg.marginal_min_run_bins:
            return False
        if self._is_radial_streak_noise(run):
            return False
        if cfg.line_max_run_radial_span_m > 0.0:
            xy = np.vstack([item[3][:2] for item in run])
            radial_span = float(np.ptp(np.linalg.norm(xy, axis=1)))
            if radial_span > cfg.line_max_run_radial_span_m:
                return False
        return True

    def _is_radial_streak_noise(self, run) -> bool:
        cfg = self.config
        metrics = self.projector.radial_metrics(run)
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

    def _run_temporally_unstable(self, run) -> bool:
        cfg = self.config
        window_frames = cfg.spray_temporal_window_frames
        if window_frames <= 0 or len(self._recent_raw) < window_frames:
            return False

        recent = list(self._recent_raw)[-window_frames:]
        stable_bins = 0
        for sensor_idx, bin_idx, hazard_cls, _pt in run:
            key = (sensor_idx, bin_idx)
            hits = sum(1 for hist in recent if hist.get(key) == hazard_cls)
            if hits >= cfg.spray_temporal_stable_min_frames:
                stable_bins += 1

        stable_fraction = stable_bins / max(len(run), 1)
        return stable_fraction < cfg.spray_temporal_stable_fraction
