"""Temporal confirmation: a hazard must persist across enough frames before it
is published.

Two pure helpers. `confirm_frames_for_bin` says how many frames this bin needs;
`bin_confirmed` checks the frame history (a deque owned by `LineSensorSource`)
for that streak. See README.md → "Making it wait a beat".
"""

from __future__ import annotations

import numpy as np

from .config import LineSensorConfig
from .hits import BinClass


def confirm_frames_for_bin(
    cfg: LineSensorConfig,
    pt: np.ndarray,
    cls: BinClass = BinClass.OBSTACLE,
) -> int:
    if cfg.use_range_deviation:
        if cls == BinClass.OBSTACLE_MARGINAL:
            return cfg.marginal_confirm_frames
        return cfg.strong_confirm_frames
    radius = float(np.linalg.norm(pt[:2]))
    if radius <= cfg.line_fast_confirm_range_m:
        return cfg.line_fast_confirm_frames
    return cfg.line_confirm_frames


def bin_confirmed(
    history,
    sensor_idx: int,
    bin_idx: int,
    hazard_cls: BinClass,
    confirm_frames: int,
    require_consecutive: bool,
) -> bool:
    key = (sensor_idx, bin_idx)
    if len(history) < confirm_frames:
        return False

    if require_consecutive:
        recent = list(history)[-confirm_frames:]
        return all(hist.get(key) == hazard_cls for hist in recent)

    return sum(1 for hist in history if hist.get(key) == hazard_cls) >= confirm_frames
