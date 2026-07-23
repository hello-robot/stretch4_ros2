"""Glossy-floor defense (C7): reject phantom near-field arcs off shiny floors
that are shape-identical to a real object pressed against the base.

Shape can't tell them apart, so this uses context instead:

* `FlipTracker` — a *time* signature. Gloss bins flip hazard<->quiet constantly;
  a real object flips once per approach. The tracker keeps a decayed per-bin
  flip count and flags bins that flicker too much.
* `quarantine_spray_candidates` — a *space* signature plus the flicker flag.
  When several sensors see near-field hazards at once (one object can't
  surround the robot), every near-field candidate is rerouted to SPRAY, except
  runs that look like a genuine pressed object.

Quarantined bins become SPRAY (a debug-only class), never a published hazard.
See README.md → "Rejecting glossy-floor phantoms".
"""

from __future__ import annotations

import numpy as np

from .arrays import runs
from .config import LineSensorConfig
from .hits import BinClass


class FlipTracker:
    """Per-sensor decayed count of hazard-band on/off flips."""

    def __init__(self, config: LineSensorConfig):
        self.config = config
        self._prev_hazard_band: dict[int, np.ndarray] = {}
        self._flip_ema: dict[int, np.ndarray] = {}

    def update(self, sensor_idx: int, hazard_band: np.ndarray) -> np.ndarray:
        """Fold this frame's hazard band in; return the suspect mask. Gloss
        flicker sustains a high count; a real object contributes one flip per
        approach and decays away."""
        cfg = self.config
        prev = self._prev_hazard_band.get(sensor_idx)
        ema = self._flip_ema.get(sensor_idx)
        if ema is None or len(ema) != len(hazard_band):
            ema = np.zeros(len(hazard_band), dtype=np.float64)
        if prev is not None and len(prev) == len(hazard_band):
            ema = ema * cfg.spray_flip_decay + (hazard_band != prev)
        self._prev_hazard_band[sensor_idx] = hazard_band
        self._flip_ema[sensor_idx] = ema
        return ema > cfg.spray_flip_suspect_threshold


def quarantine_spray_candidates(
    candidates,
    sensor_ranges,
    suspect_bins,
    near_sensor_count: int,
    cfg: LineSensorConfig,
):
    """Split candidates into (kept, quarantined-as-spray) using the flicker
    quarantine and the cross-sensor near-field gate."""
    if not cfg.use_spray_bin_quarantine or not candidates:
        return candidates, []
    cross_active = near_sensor_count >= cfg.spray_cross_sensor_min_sensors
    exempt: dict[int, np.ndarray] = {}
    if cross_active:
        for sensor_idx, (_name, ranges) in sensor_ranges.items():
            near = (
                np.isfinite(ranges)
                & (ranges > 0.0)
                & (ranges < cfg.spray_cross_sensor_near_range_m)
            )
            mask = np.zeros(len(ranges), dtype=bool)
            for start, end in runs(np.flatnonzero(near)):
                if end - start + 1 < cfg.spray_cross_sensor_exempt_run_bins:
                    continue
                ragged = float(np.std(np.diff(ranges[start:end + 1])))
                if ragged <= cfg.spray_cross_sensor_exempt_max_ragged_m:
                    mask[start:end + 1] = True
            exempt[sensor_idx] = mask
    kept = []
    quarantined = []
    for sensor_idx, bin_idx, cls, pt in candidates:
        suspect = suspect_bins.get(sensor_idx)
        flickering = (
            suspect is not None
            and bin_idx < len(suspect)
            and bool(suspect[bin_idx])
        )
        near_phantom = False
        if cross_active and sensor_idx in sensor_ranges:
            r = sensor_ranges[sensor_idx][1][bin_idx]
            wide_real = bool(exempt.get(sensor_idx, np.zeros(0, dtype=bool))[bin_idx]) \
                if bin_idx < len(exempt.get(sensor_idx, np.zeros(0, dtype=bool))) else False
            near_phantom = bool(r < cfg.spray_cross_sensor_near_range_m) and not wide_real
        if flickering or near_phantom:
            quarantined.append((sensor_idx, bin_idx, BinClass.SPRAY, pt))
        else:
            kept.append((sensor_idx, bin_idx, cls, pt))
    return kept, quarantined
