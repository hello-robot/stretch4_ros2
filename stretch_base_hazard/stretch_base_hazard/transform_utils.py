"""Helpers for transforming points between frames."""

from __future__ import annotations

import numpy as np


def translation_rotation_to_matrix(
    tx: float,
    ty: float,
    tz: float,
    qx: float,
    qy: float,
    qz: float,
    qw: float,
) -> np.ndarray:
    """Build a 4x4 homogeneous matrix from translation and quaternion."""
    x, y, z, w = qx, qy, qz, qw
    rot = np.array([
        [
            1.0 - 2.0 * (y * y + z * z),
            2.0 * (x * y - z * w),
            2.0 * (x * z + y * w),
        ],
        [
            2.0 * (x * y + z * w),
            1.0 - 2.0 * (x * x + z * z),
            2.0 * (y * z - x * w),
        ],
        [
            2.0 * (x * z - y * w),
            2.0 * (y * z + x * w),
            1.0 - 2.0 * (x * x + y * y),
        ],
    ])

    mat = np.eye(4, dtype=np.float64)
    mat[:3, :3] = rot
    mat[:3, 3] = [tx, ty, tz]
    return mat


def transform_points(matrix: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply a 4x4 transform to an (N, 3) array."""
    points = np.atleast_2d(np.asarray(points, dtype=np.float64))
    if len(points) == 0:
        return points
    ones = np.ones((len(points), 1), dtype=np.float64)
    hom = np.hstack([points, ones])
    return (hom @ matrix.T)[:, :3]


def invert_transform(matrix: np.ndarray) -> np.ndarray:
    """Invert a 4x4 rigid transform."""
    rot = matrix[:3, :3]
    trans = matrix[:3, 3]
    inv = np.eye(4, dtype=np.float64)
    inv[:3, :3] = rot.T
    inv[:3, 3] = -rot.T @ trans
    return inv
