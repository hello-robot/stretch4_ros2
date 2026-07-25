"""Reading the silences: no-return bins as evidence of cliffs and lost coverage.

Everything above works on bins that *return* a range. `NullEvidenceDetector`
works on the bins that *don't* — the no-return sentinels (5.11 m, and any
finite value past 4 m, the undocumented dark-stairs code). A trusted null run
is typed by its context: proven a void by the far sentinel, shadowed by an
obstacle, suppressed by a near bright return, next to a drop (a probable cliff
-> deep-drop output), or simply dark floor. Trusted nulls that stay unexplained
measure lost floor coverage and, once a smoothed/hysteretic fraction crosses
the threshold, publish as degraded.

Holds the small amount of frame-to-frame state this needs: the previous null
mask (persistence), and the degraded EMA + latch. See README.md → "Reading the
silences".
"""

from __future__ import annotations

import numpy as np

from .arrays import runs
from .config import LineSensorConfig
from .geometry import Projector
from .hits import BinClass, DROP_FAMILY, OBSTACLE_FAMILY, max_line_sensor_range


class NullEvidenceDetector:
    def __init__(
        self,
        config: LineSensorConfig,
        projector: Projector,
        bin_reliable: dict,
        bin_null_rate: dict,
    ):
        self.config = config
        self.projector = projector
        self.bin_reliable = bin_reliable
        self.bin_null_rate = bin_null_rate
        # Per-sensor last-frame null mask (persistence check).
        self._prev_null: dict[int, np.ndarray] = {}
        # Degraded hysteresis state: smoothed blind fraction and on/off latch.
        self._deg_frac_ema: dict[int, float] = {}
        self._deg_active: dict[int, bool] = {}
        self.last_void_runs: list = []

    def detect(self, sensor_ranges, gated):
        cfg = self.config
        empty = np.zeros((0, 2))
        self.last_void_runs = []
        if not cfg.use_null_evidence:
            return empty, empty, empty

        drop_bins: dict[int, list] = {}
        obstacle_bins: dict[int, list] = {}
        spray_bins: dict[int, list] = {}
        for sensor_idx, bin_idx, cls, _pt in gated:
            if cls in DROP_FAMILY:
                drop_bins.setdefault(sensor_idx, []).append(bin_idx)
            elif cls in OBSTACLE_FAMILY:
                obstacle_bins.setdefault(sensor_idx, []).append(bin_idx)
            elif cls == BinClass.SPRAY:
                spray_bins.setdefault(sensor_idx, []).append(bin_idx)

        drop_bearings: dict[int, np.ndarray] = {
            sensor_idx: self.projector.bin_bearings(sensor_idx)[np.array(bins, dtype=int)]
            for sensor_idx, bins in drop_bins.items()
        }

        cliff_pts: list = []
        benign_pts: list = []
        degraded_pts: list = []
        new_prev: dict[int, np.ndarray] = {}

        # --- Pass 1: gate the runs, and mark the ones the far sentinel proves
        # are voids. 
        prepared: list = []
        void_bearings: dict[int, list] = {}
        for sensor_idx, (sensor_name, ranges) in sensor_ranges.items():
            null = self._no_return_mask(ranges)
            prev = self._prev_null.get(sensor_idx)
            new_prev[sensor_idx] = null

            # A null is only evidence if the bin is known to return on clear
            # floor
            evidence = null.copy()
            trusted = self._null_trusted(sensor_name, len(evidence))
            evidence &= trusted
            reliable_count = max(int(trusted.sum()), 1)

            # Exposure suppression: one bright near return can blank the rest
            # of the array. Spray points should count.
            valid = np.isfinite(ranges) & (ranges > 0.0) & (ranges < max_line_sensor_range)
            suppressors = valid & (ranges < cfg.suppression_near_range_m)
            own_spray = spray_bins.get(sensor_idx)
            if own_spray:
                suppressors[np.array(own_spray, dtype=int)] = False
            near_suppressor = bool(np.any(suppressors))

            far = self._far_sentinel_mask(ranges) & evidence

            bearings = self.projector.bin_bearings(sensor_idx)
            gated_runs: list = []
            for start, end in runs(np.flatnonzero(evidence)):
                if end - start + 1 < cfg.null_min_run_bins:
                    continue
                # Persistence: the run must have been mostly null last frame
                # too (2-frame latency, matching marginal obstacle confirm).
                if prev is None or len(prev) <= end:
                    continue
                if float(np.mean(prev[start:end + 1])) < cfg.null_persist_min_fraction:
                    continue
                void = self._is_void_run(far, start, end)
                if void:
                    void_bearings.setdefault(sensor_idx, []).append(
                        bearings[start:end + 1])
                    self.last_void_runs.append((sensor_idx, start, end))
                gated_runs.append((start, end, void))

            prepared.append((
                sensor_idx, evidence, reliable_count, near_suppressor, gated_runs,
            ))

        # --- Pass 2: type each gated run and account for the leftovers.
        for sensor_idx, evidence, reliable_count, near_suppressor, gated_runs in prepared:
            # Null bins accounted for by a typed run (void/cliff/shadow/
            # suppression) are explained; the remainder measures lost floor
            # coverage.
            explained = np.zeros(len(evidence), dtype=bool)

            floor_xy = self.projector.floor_intersections(sensor_idx)
            bearings = self.projector.bin_bearings(sensor_idx)
            own_drops = np.array(sorted(drop_bins.get(sensor_idx, [])), dtype=int)
            own_obstacles = np.array(sorted(obstacle_bins.get(sensor_idx, [])), dtype=int)
            other_drop_bearings = self._other_bearings(drop_bearings, sensor_idx)
            other_void_bearings = self._other_bearings(void_bearings, sensor_idx)

            for start, end in ((run[0], run[1]) for run in gated_runs if run[2]):
                cliff_pts.append(floor_xy[start:end + 1])
                explained[start:end + 1] = True

            for start, end, void in gated_runs:
                if void:
                    continue
                run_pts = floor_xy[start:end + 1]
                if self._bearing_overlap(bearings, start, end, other_void_bearings,
                                         cfg.void_bearing_adjacency_deg):
                    cliff_pts.append(run_pts)  # same edge, seen by a neighbour
                    explained[start:end + 1] = True
                    continue
                if len(own_obstacles) and bool(np.any(
                    (own_obstacles >= start - cfg.shadow_adjacency_bins)
                    & (own_obstacles <= end + cfg.shadow_adjacency_bins)
                )):
                    benign_pts.append(run_pts)  # occlusion shadow
                    explained[start:end + 1] = True
                    continue
                if near_suppressor:
                    benign_pts.append(run_pts)  # exposure suppression
                    explained[start:end + 1] = True
                    continue

                cliff = bool(len(own_drops)) and bool(np.any(
                    (own_drops >= start - cfg.cliff_adjacent_drop_bins)
                    & (own_drops <= end + cfg.cliff_adjacent_drop_bins)
                ))
                if not cliff:
                    cliff = self._bearing_overlap(
                        bearings, start, end, other_drop_bearings,
                        cfg.cliff_bearing_adjacency_deg)

                if cliff:
                    cliff_pts.append(run_pts)
                    explained[start:end + 1] = True
                else:
                    benign_pts.append(run_pts)  # dark floor: benign, but unexplained

            unexplained = evidence & ~explained
            frac = float(unexplained.sum()) / reliable_count
            alpha = min(max(cfg.degraded_frac_alpha, 0.0), 1.0)
            ema = self._deg_frac_ema.get(sensor_idx, frac)
            ema = ema + alpha * (frac - ema)
            self._deg_frac_ema[sensor_idx] = ema
            active = self._deg_active.get(sensor_idx, False)
            if active:
                active = ema >= min(cfg.degraded_exit_fraction, cfg.degraded_min_fraction)
            else:
                active = ema >= cfg.degraded_min_fraction
            self._deg_active[sensor_idx] = active
            if active and unexplained.any():
                degraded_pts.append(floor_xy[unexplained])

        self._prev_null = new_prev
        cliff_xy = np.vstack(cliff_pts) if cliff_pts else empty
        benign_xy = np.vstack(benign_pts) if benign_pts else empty
        degraded_xy = np.vstack(degraded_pts) if degraded_pts else empty
        return cliff_xy, benign_xy, degraded_xy

    @staticmethod
    def _other_bearings(by_sensor: dict, sensor_idx: int) -> np.ndarray:
        """Every bearing recorded for a sensor other than this one, flattened.
        """
        parts: list = []
        for other_idx, value in by_sensor.items():
            if other_idx == sensor_idx:
                continue
            if isinstance(value, list):
                parts.extend(value)
            else:
                parts.append(value)
        return np.concatenate(parts) if parts else np.zeros(0)

    @staticmethod
    def _bearing_overlap(bearings, start, end, other, margin_deg) -> bool:
        """Checks whether another sensor is looking within `margin_deg` of this
        run's angular span"""
        
        if not np.size(other):
            return False
        mid = 0.5 * (bearings[start] + bearings[end])
        half_span = 0.5 * abs(bearings[end] - bearings[start])
        delta = np.abs((other - mid + 180.0) % 360.0 - 180.0)
        return bool(np.any(delta <= half_span + margin_deg))

    def _is_void_run(self, far: np.ndarray, start: int, end: int) -> bool:
        """Checks if the run carry enough far sentinels to call it a void
        """
        cfg = self.config
        if not cfg.use_far_sentinel_void:
            return False
        count = int(far[start:end + 1].sum())
        if count < cfg.void_far_sentinel_min_bins:
            return False
        return count / float(end - start + 1) >= cfg.void_far_sentinel_min_fraction

    def _far_sentinel_mask(self, ranges: np.ndarray) -> np.ndarray:
        """No-return bins whose value is 5.09.
        """
        cfg = self.config
        ranges = np.asarray(ranges, dtype=np.float64)
        return (
            np.isfinite(ranges)
            & (ranges > cfg.null_sentinel_min_m)
            & (np.abs(ranges - cfg.null_range_m) > cfg.null_tolerance_m)
        )

    def _no_return_mask(self, ranges: np.ndarray) -> np.ndarray:
        """All no-return sentinels: the documented 5.11 null plus any finite
        reading beyond null_sentinel_min_m (e.g. the undocumented 5.09)."""
        cfg = self.config
        return np.isfinite(ranges) & (
            np.isclose(ranges, cfg.null_range_m, atol=cfg.null_tolerance_m, rtol=0.0)
            | (ranges > cfg.null_sentinel_min_m)
        )

    def _null_trusted(self, sensor_name: str, bin_count: int) -> np.ndarray:
        """Bins whose nulls carry evidence: calibration null rate below the
        chronic threshold, falling back to the tare mask when no rates exist."""
        rates = self.bin_null_rate.get(sensor_name)
        if rates is not None and len(rates) == bin_count:
            return (
                np.asarray(rates, dtype=np.float64)
                <= self.config.chronic_null_rate_max
            )
        reliable = self.bin_reliable.get(sensor_name)
        if reliable is not None and len(reliable) == bin_count:
            return np.asarray(reliable, dtype=bool)
        return np.ones(bin_count, dtype=bool)
