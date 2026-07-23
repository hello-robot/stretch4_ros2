"""Line-sensor projection and pre-grid hazard filtering.

Public API :

    LineSensorSource   build once, call .process(status) each frame
    LineSensorConfig   the tunables (all defaults live here)
    LineSensorHits     what .process() returns (arrays of (x, y) points)
    BinClass           the per-bin labels, if you need to reason about them
    as_range_array     coerce a raw ranges payload to a float64 array

"""

from __future__ import annotations

from .config import LineSensorConfig
from .hits import BinClass, LineSensorHits
from .arrays import as_range_array
from .source import LineSensorSource

__all__ = [
    'LineSensorSource',
    'LineSensorConfig',
    'LineSensorHits',
    'BinClass',
    'as_range_array',
]
