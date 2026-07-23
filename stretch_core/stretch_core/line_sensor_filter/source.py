"""The orchestrator: one class, one `process()` per frame.

`LineSensorSource` owns the per-frame state (the confirmation histories and the
frame counter) and wires the stages together. It builds the stage helpers once,
then `process()` reads top-to-bottom as the pipeline: classify each returning
bin, quarantine glossy phantoms, shape-gate the runs, confirm across frames,
and — in parallel — read the silences for cliffs and lost coverage. Every stage
is one delegated call, so this file is the map; the how lives in each stage's
module. See README.md → "The frame, end to end".
"""

from __future__ import annotations

from collections import deque

import numpy as np

from .arrays import as_range_array, items_to_xy
from .classify import classify_bin
from .config import LineSensorConfig
from .confirm import bin_confirmed, confirm_frames_for_bin
from .geometry import Projector
from .gloss import FlipTracker, quarantine_spray_candidates
from .hits import (
    BinClass,
    DROP_FAMILY,
    LineSensorHits,
    OBSTACLE_FAMILY,
    family,
    max_line_sensor_range,
)
from .nulls import NullEvidenceDetector
from .shape import ShapeGate


class LineSensorSource:
    """Classify projected line-sensor bins into obstacle and small-drop evidence."""

    def __init__(
        self,
        geometry,
        sensor_names: list,
        config: LineSensorConfig,
        apply_tare=None,
        bin_reliable=None,
        bin_null_rate=None,
    ):
        self.geometry = geometry
        self.sensor_names = sensor_names
        self.config = config
        self.apply_tare = apply_tare
        # Per-sensor bool arrays: bins with a valid tare floor reference.
        # Fine-grained (marginal) deviation classification is only allowed on
        # reliable bins; untared bins keep the coarse legacy threshold.
        self.bin_reliable = bin_reliable or {}
        # Per-sensor float arrays: fraction of calibration frames each bin
        # returned null on clear floor (chronic-null prior, C5). Gates null
        # evidence and the degraded-coverage denominator; the tare mask above
        # remains the gate for deviation classification, which is a different
        # question (has a floor reference vs. is known to return).
        self.bin_null_rate = bin_null_rate or {}

        # Stage helpers, built once (never per frame — see README latency note).
        self.projector = Projector(geometry, config)
        self.flip_tracker = FlipTracker(config)
        self.shape_gate = ShapeGate(config, self.projector)
        self.null_detector = NullEvidenceDetector(
            config, self.projector, self.bin_reliable, self.bin_null_rate)

        self._frames_processed = 0
        self._history: deque = deque(
            maxlen=max(
                config.line_window_frames,
                config.line_confirm_frames,
                config.line_fast_confirm_frames,
                1,
            ),
        )
        self._raw_history: deque = deque(
            maxlen=max(
                config.line_window_frames,
                config.spray_temporal_window_frames,
                1,
            ),
        )

    def project_all(self, status: dict) -> np.ndarray:
        points: list = []
        for sensor_idx, sensor_name in enumerate(self.sensor_names):
            sensor_status = status.get(sensor_name, {})
            if not isinstance(sensor_status, dict):
                continue
            ranges = self._ranges_for_sensor(sensor_status, sensor_name)
            if ranges.size == 0:
                continue
            projected = self.projector.project(sensor_idx, ranges)
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
        candidates: list = []
        sensor_ranges: dict = {}
        suspect_bins: dict = {}
        near_sensor_count = 0

        # --- Stage 1: classify every returning bin --------------------------
        for sensor_idx, sensor_name in enumerate(self.sensor_names):
            sensor_status = status.get(sensor_name, {})
            if not isinstance(sensor_status, dict):
                continue
            ranges = self._ranges_for_sensor(sensor_status, sensor_name)
            if ranges.size == 0:
                continue
            sensor_ranges[sensor_idx] = (sensor_name, ranges)

            projected = self.projector.project(sensor_idx, ranges)
            contrast = self.projector.local_contrast(projected[:, 2], ranges)
            reliable = self.bin_reliable.get(sensor_name)

            if cfg.use_spray_bin_quarantine:
                valid = (
                    np.isfinite(ranges)
                    & (ranges > 0.0)
                    & (ranges < max_line_sensor_range)
                )
                hazard_band = valid & (projected[:, 2] > cfg.dev_floor_band_m)
                suspect_bins[sensor_idx] = self.flip_tracker.update(
                    sensor_idx, hazard_band)
                if int(np.count_nonzero(
                    hazard_band & (ranges < cfg.spray_cross_sensor_near_range_m)
                )) >= cfg.spray_cross_sensor_min_bins:
                    near_sensor_count += 1

            for bin_idx in range(len(ranges)):
                if (
                    not np.isfinite(ranges[bin_idx])
                    or ranges[bin_idx] <= 0.0
                    or ranges[bin_idx] >= max_line_sensor_range
                ):
                    continue
                pt = projected[bin_idx]
                bin_ok = bool(reliable[bin_idx]) if reliable is not None and bin_idx < len(reliable) else True
                cls = classify_bin(cfg, pt[2], contrast[bin_idx], bin_ok)
                if cls in OBSTACLE_FAMILY or cls in DROP_FAMILY:
                    candidates.append((sensor_idx, bin_idx, cls, pt))

        # --- Stage 2: glossy-floor quarantine -------------------------------
        candidates, quarantined = quarantine_spray_candidates(
            candidates, sensor_ranges, suspect_bins, near_sensor_count, cfg)
        # Introspection for diagnostic tooling (line_sensor_doctor.py).
        self.last_quarantined = quarantined
        self.last_suspect_bins = suspect_bins
        self.last_near_sensor_count = near_sensor_count

        # History stores the family class so a bin flapping between strong and
        # marginal keeps its confirmation streak.
        self._raw_history.append({
            (sensor_idx, bin_idx): family(cls)
            for sensor_idx, bin_idx, cls, _pt in candidates
        })

        # --- Stage 3: spatial (shape) gate ----------------------------------
        gated = self.shape_gate.gate(candidates, list(self._raw_history))
        gated.extend(quarantined)
        self.last_raw_candidates = candidates
        self.last_gated = gated
        self._history.append({
            (sensor_idx, bin_idx): family(cls)
            for sensor_idx, bin_idx, cls, _pt in gated
        })

        # --- Stage 4: temporal confirmation ---------------------------------
        self._frames_processed += 1
        warmup = self._frames_processed <= cfg.spray_warmup_frames
        promoted: list = []
        for sensor_idx, bin_idx, cls, pt in gated:
            if cls == BinClass.SPRAY:
                continue
            confirm_frames = confirm_frames_for_bin(cfg, pt, cls)
            if cfg.use_spray_bin_quarantine and (warmup or (
                sensor_idx in suspect_bins
                and int(np.count_nonzero(suspect_bins[sensor_idx]))
                >= cfg.spray_suspect_confirm_min_bins
            )):
                confirm_frames = max(confirm_frames, 2)
            if bin_confirmed(
                self._history, sensor_idx, bin_idx, family(cls),
                confirm_frames, cfg.line_require_consecutive,
            ):
                promoted.append((sensor_idx, bin_idx, cls, pt))

        self.last_promoted = promoted

        # --- Stage 5: null evidence (runs in parallel on full arrays) -------
        cliff_xy, benign_xy, degraded_xy = self.null_detector.detect(
            sensor_ranges, gated)

        # --- Stage 6: package outputs ---------------------------------------
        return LineSensorHits(
            probable_cliff_xy=cliff_xy,
            benign_null_xy=benign_xy,
            degraded_xy=degraded_xy,
            obstacle_xy=items_to_xy(promoted, OBSTACLE_FAMILY),
            small_drop_xy=items_to_xy(promoted, BinClass.SMALL_DROP),
            deep_drop_xy=items_to_xy(promoted, BinClass.DEEP_DROP),
            raw_obstacle_xy=items_to_xy(candidates, OBSTACLE_FAMILY),
            raw_small_drop_xy=items_to_xy(candidates, DROP_FAMILY),
            spatial_obstacle_xy=items_to_xy(gated, OBSTACLE_FAMILY),
            spatial_small_drop_xy=items_to_xy(gated, DROP_FAMILY),
            raw_spray_xy=items_to_xy(gated, BinClass.SPRAY),
            raw_marginal_obstacle_xy=items_to_xy(candidates, BinClass.OBSTACLE_MARGINAL),
        )

    def _ranges_for_sensor(self, sensor_status: dict, sensor_name: str) -> np.ndarray:
        ranges = as_range_array(sensor_status.get('ranges'))
        if ranges.size == 0:
            return ranges
        if self.apply_tare is not None:
            ranges = self.apply_tare(ranges, sensor_name)
        return ranges
