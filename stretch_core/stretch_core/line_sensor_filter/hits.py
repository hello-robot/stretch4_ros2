"""Vocabulary and output types — the words the rest of the package speaks.

`BinClass` is the label every returning bin gets. `LineSensorHits` is exactly
what one call to `LineSensorSource.process()` returns. See README.md → "The
words we use" and "What a frame produces".
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import IntEnum

import numpy as np

# Valid-range cutoff: a bin counts as "returning" when 0 < r < 4.0 m. This is a
# module constant, not a config knob 
max_line_sensor_range = 4.0


class BinClass(IntEnum):
    UNKNOWN = 0
    FREE = 1
    OBSTACLE = 2
    SMALL_DROP = 3
    SPRAY = 4
    OBSTACLE_MARGINAL = 5
    DEEP_DROP = 6


# Marginal obstacles group with strong obstacles for run-building and
# publication; they differ only in required run length and confirm frames.
OBSTACLE_FAMILY = (BinClass.OBSTACLE, BinClass.OBSTACLE_MARGINAL)
# Deep drops group with small drops for run-building; they publish on a
# separate output so the hazard layer can treat them as lethal cliffs.
DROP_FAMILY = (BinClass.SMALL_DROP, BinClass.DEEP_DROP)


def family(cls: BinClass) -> BinClass:
    """Collapse a bin class to its family representative.

    History keys use the family, so a bin flapping between strong and marginal
    (or small and deep drop) keeps a single confirmation streak.
    """
    if cls in OBSTACLE_FAMILY:
        return BinClass.OBSTACLE
    if cls in DROP_FAMILY:
        return BinClass.SMALL_DROP
    return cls


@dataclass
class LineSensorHits:
    obstacle_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    small_drop_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    raw_obstacle_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    raw_small_drop_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    spatial_obstacle_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    spatial_small_drop_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    raw_spray_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    raw_marginal_obstacle_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    # Cliff-typed null runs projected at each bin's expected floor
    # intersection (the nearest possible hazard location — conservative).
    probable_cliff_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    # Benign-typed null runs (suppression / shadow / dark floor), debug only.
    benign_null_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    # Confirmed returning drops deeper than cliff_max_drop_m.
    deep_drop_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    # Floor intersections of sectors that lost coverage without explanation.
    degraded_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
