"""Perpendicular floor-plane RANSAC for LIDAR preprocessing."""

from __future__ import annotations

import numpy as np
from sklearn.linear_model import LinearRegression, RANSACRegressor

PlaneCoeffs = tuple[float, float, float, float]  # ax + by + cz + d = 0


def _coeffs_from_z_plane(mx: float, my: float, intercept: float) -> PlaneCoeffs:
    """Convert z = mx*x + my*y + intercept to unit-normal plane coefficients."""
    normal = np.array([-mx, -my, 1.0], dtype=np.float64)
    norm = np.linalg.norm(normal)
    if norm <= 0.0:
        return 0.0, 0.0, 1.0, -intercept
    normal /= norm
    if normal[2] < 0.0:
        normal = -normal
    d = -float(np.dot(normal, np.array([0.0, 0.0, intercept])))
    return float(normal[0]), float(normal[1]), float(normal[2]), d


def _normal_tilt_deg(normal: np.ndarray) -> float:
    z = abs(float(normal[2]))
    z = min(max(z, 0.0), 1.0)
    return float(np.degrees(np.arccos(z)))


def signed_height_above_plane(points: np.ndarray, coeffs: PlaneCoeffs) -> np.ndarray:
    """Signed distance above the plane (positive = above floor)."""
    a, b, c, d = coeffs
    norm = np.sqrt(a * a + b * b + c * c)
    if norm <= 0.0:
        return np.zeros(points.shape[0])
    return (a * points[:, 0] + b * points[:, 1] + c * points[:, 2] + d) / norm


def fit_perpendicular_floor_plane(
    points: np.ndarray,
    *,
    max_tilt_deg: float = 10.0,
    iterations: int = 200,
    threshold: float = 0.1,
    rng: np.random.Generator | None = None,
) -> tuple[PlaneCoeffs | None, np.ndarray]:
    """
    Fit a near-horizontal floor plane with scikit-learn RANSAC.

    Equivalent to stretch_core::FloorPlaneFilter's PCL
    SACMODEL_PERPENDICULAR_PLANE: model z = mx*x + my*y + intercept, which
    constrains the plane normal to stay near +Z.
    """
    points = np.atleast_2d(np.asarray(points, dtype=np.float64))
    n = points.shape[0]
    if n < 3:
        return None, np.zeros(n, dtype=bool)

    random_state = None
    if rng is not None:
        random_state = int(rng.integers(0, 2**31 - 1))

    ransac = RANSACRegressor(
        estimator=LinearRegression(),
        min_samples=3,
        residual_threshold=threshold,
        max_trials=max(iterations, 1),
        random_state=random_state,
    )
    try:
        ransac.fit(points[:, :2], points[:, 2])
    except ValueError:
        return None, np.zeros(n, dtype=bool)

    inliers = np.asarray(ransac.inlier_mask_, dtype=bool)
    if inliers.shape[0] != n or np.count_nonzero(inliers) < 3:
        return None, np.zeros(n, dtype=bool)

    mx, my = ransac.estimator_.coef_
    coeffs = _coeffs_from_z_plane(mx, my, float(ransac.estimator_.intercept_))
    if _normal_tilt_deg(np.array(coeffs[:3])) > max_tilt_deg:
        return None, np.zeros(n, dtype=bool)

    return coeffs, inliers
