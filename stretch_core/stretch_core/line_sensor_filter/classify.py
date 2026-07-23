"""Stage 1 of a returning bin's life: turn one bin's height `z` (and its local
contrast) into a `BinClass`.

This is the whole "deviation -> label" decision, and nothing here is stateful:
give it a `z`, a `contrast`, and whether the bin has a trustworthy tare
reference, and it answers with a class. See README.md → "Classifying one bin".
"""

from __future__ import annotations

import numpy as np

from .config import LineSensorConfig
from .hits import BinClass


def classify_bin(
    cfg: LineSensorConfig,
    z: float,
    contrast: float = None,
    bin_reliable: bool = True,
) -> BinClass:
    if not np.isfinite(z):
        return BinClass.UNKNOWN
    depth_scale = max(cfg.depth_underread_scale, 1e-6)
    if not cfg.use_range_deviation or not bin_reliable:
        # Legacy / untared path: coarse absolute-height bands.
        if abs(z) <= cfg.floor_band_m:
            return BinClass.FREE
        if z >= cfg.line_obstacle_min_height_m:
            return BinClass.OBSTACLE
        drop = -z / depth_scale
        if cfg.cliff_min_drop_m <= drop <= cfg.cliff_max_drop_m:
            return BinClass.SMALL_DROP
        return BinClass.UNKNOWN
    if contrast is None:
        contrast = z
    if z >= cfg.line_obstacle_min_height_m:
        return BinClass.OBSTACLE
    if (
        z >= cfg.dev_obstacle_strong_m
        and contrast >= cfg.dev_obstacle_strong_m
    ):
        return BinClass.OBSTACLE
    if (
        z > cfg.dev_floor_band_m
        and contrast >= cfg.marginal_contrast_min_m
    ):
        return BinClass.OBSTACLE_MARGINAL
    if abs(z) <= cfg.dev_floor_band_m:
        return BinClass.FREE
    drop = -z / depth_scale
    if cfg.cliff_min_drop_m <= drop <= cfg.cliff_max_drop_m:
        return BinClass.SMALL_DROP
    if cfg.use_deep_drop and drop > cfg.cliff_max_drop_m:
        return BinClass.DEEP_DROP
    return BinClass.UNKNOWN
