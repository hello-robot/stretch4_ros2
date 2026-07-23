"""Array helpers shared across stages. """

from __future__ import annotations

import numpy as np


def as_range_array(ranges_raw) -> np.ndarray:
    """Coerce a raw ranges payload (list / None / ndarray) to a float64 array."""
    if ranges_raw is None:
        return np.array([], dtype=np.float64)
    return np.asarray(ranges_raw, dtype=np.float64)


def runs(indices: np.ndarray) -> list[tuple[int, int]]:
    """Contiguous (start, end) runs in a sorted index array."""
    if len(indices) == 0:
        return []
    breaks = np.flatnonzero(np.diff(indices) > 1)
    starts = np.concatenate([[0], breaks + 1])
    ends = np.concatenate([breaks, [len(indices) - 1]])
    return [(int(indices[a]), int(indices[b])) for a, b in zip(starts, ends)]


def items_to_xy(items, cls) -> np.ndarray:
    """Stack the (x, y) of every item whose class is in `cls` (a class or a
    tuple of classes). Empty -> (0, 2)."""
    wanted = cls if isinstance(cls, tuple) else (cls,)
    points = [pt[:2] for _sensor_idx, _bin_idx, item_cls, pt in items if item_cls in wanted]
    return np.vstack(points) if points else np.zeros((0, 2))
