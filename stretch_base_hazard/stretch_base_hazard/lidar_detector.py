"""Merged point-cloud inaccessible-space detector."""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from stretch_base_hazard.floor_ransac import (
    fit_perpendicular_floor_plane,
    signed_height_above_plane,
)
from stretch_base_hazard.layer_tags import (
    LAYER_LIDAR_CLIFF,
    LAYER_LIDAR_OBSTACLE,
    LAYER_LIDAR_OCCLUSION,
)


@dataclass
class LidarDetectorConfig:
    map_radius_m: float = 2.0
    crop_to_map_radius: bool = True
    resolution_m: float = 0.05
    base_radius: float = 0.25
    voxel_leaf_size: float = 0.05
    floor_detect_z_min: float = -0.4
    floor_detect_z_max: float = 0.1
    floor_fit_threshold_m: float = 0.015
    floor_observed_threshold_m: float = 0.04
    floor_max_tilt_deg: float = 10.0
    floor_ransac_iterations: int = 200
    lidar_obstacle_min_height_m: float = 0.05
    thresh_cliff_m: float = 0.04
    min_cliff_cells: int = 4
    min_edge_floor_cells: int = 2
    overhead_z_min_m: float = 0.30
    base_collision_z_max_m: float = 0.35


@dataclass
class LidarHits:
    """Compact detector output in base_link."""

    obstacle_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    cliff_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    occlusion_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    clear_floor_xy: np.ndarray = field(default_factory=lambda: np.zeros((0, 2)))
    ransac_floor_xyz: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))
    obstacle_layer: int = LAYER_LIDAR_OBSTACLE
    cliff_layer: int = LAYER_LIDAR_CLIFF
    occlusion_layer: int = LAYER_LIDAR_OCCLUSION


def _crop_cylinder(points: np.ndarray, radius_m: float) -> np.ndarray:
    if len(points) == 0:
        return points
    r2 = points[:, 0] * points[:, 0] + points[:, 1] * points[:, 1]
    return points[r2 <= radius_m * radius_m]


def _remove_robot_cylinder(points: np.ndarray, base_radius: float) -> np.ndarray:
    if len(points) == 0:
        return points
    r2 = points[:, 0] * points[:, 0] + points[:, 1] * points[:, 1]
    return points[r2 > base_radius * base_radius]


def _voxel_downsample(
    points: np.ndarray,
    leaf_size: float,
    xy_origin: float | None = None,
) -> np.ndarray:
    if len(points) == 0 or leaf_size <= 0.0:
        return points
    keys = np.floor(points / leaf_size).astype(np.int64)
    if xy_origin is not None:
        keys[:, 0] = np.floor((points[:, 0] - xy_origin) / leaf_size).astype(np.int64)
        keys[:, 1] = np.floor((points[:, 1] - xy_origin) / leaf_size).astype(np.int64)
    mins = keys.min(axis=0)
    shifted = keys - mins
    dims = shifted.max(axis=0) + 1
    flat = (
        shifted[:, 0]
        + shifted[:, 1] * dims[0]
        + shifted[:, 2] * dims[0] * dims[1]
    )
    _, inverse, counts = np.unique(flat, return_inverse=True, return_counts=True)
    out = np.zeros((len(counts), 3), dtype=np.float64)
    np.add.at(out, inverse, points)
    out /= counts[:, None]
    return out


def _connected_components(mask: np.ndarray) -> list[np.ndarray]:
    if not np.any(mask):
        return []
    h, w = mask.shape
    visited = np.zeros_like(mask, dtype=bool)
    components: list[np.ndarray] = []
    for r in range(h):
        for c in range(w):
            if not mask[r, c] or visited[r, c]:
                continue
            stack = [(r, c)]
            cells = []
            visited[r, c] = True
            while stack:
                cr, cc = stack.pop()
                cells.append((cr, cc))
                for dr, dc in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    nr, nc = cr + dr, cc + dc
                    if 0 <= nr < h and 0 <= nc < w and mask[nr, nc] and not visited[nr, nc]:
                        visited[nr, nc] = True
                        stack.append((nr, nc))
            components.append(np.asarray(cells, dtype=np.int64))
    return components


def _drop_small_components(mask: np.ndarray, min_cells: int) -> np.ndarray:
    if min_cells <= 1 or not np.any(mask):
        return mask
    out = np.zeros_like(mask, dtype=bool)
    for comp in _connected_components(mask):
        if comp.shape[0] >= min_cells:
            out[comp[:, 0], comp[:, 1]] = True
    return out


class LidarDetector:
    """Detect obstacle, cliff, and occlusion cells from a merged LIDAR cloud."""

    def __init__(self, config: LidarDetectorConfig):
        self.config = config
        self.resolution = config.resolution_m
        self.radius = config.map_radius_m
        self.size = int(np.ceil(2.0 * self.radius / self.resolution))
        self.origin = -self.radius + self.resolution / 2.0

    def _indices(self, points: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        cols = np.floor((points[:, 0] - self.origin) / self.resolution).astype(np.int64)
        rows = np.floor((points[:, 1] - self.origin) / self.resolution).astype(np.int64)
        valid = (cols >= 0) & (cols < self.size) & (rows >= 0) & (rows < self.size)
        return rows[valid], cols[valid], points[valid]

    def process(self, xyz: np.ndarray) -> LidarHits:
        cfg = self.config
        points = np.asarray(xyz, dtype=np.float64).reshape(-1, 3)
        cropped = _crop_cylinder(points, cfg.map_radius_m) if cfg.crop_to_map_radius else points
        cropped = _remove_robot_cylinder(cropped, cfg.base_radius)
        cropped = _voxel_downsample(cropped, cfg.voxel_leaf_size, xy_origin=self.origin)
        if len(cropped) == 0:
            return LidarHits()

        # Per-cell vertical summary before discarding high points.
        n = self.size
        has_any = np.zeros((n, n), dtype=bool)
        has_tall = np.zeros((n, n), dtype=bool)
        has_low_obstacle = np.zeros((n, n), dtype=bool)

        rows, cols, pts = self._indices(cropped)
        if len(rows) == 0:
            return LidarHits()

        has_any[rows, cols] = True

        floor_band = (
            (pts[:, 2] >= cfg.floor_detect_z_min)
            & (pts[:, 2] <= cfg.floor_detect_z_max)
        )

        tall = pts[:, 2] >= cfg.overhead_z_min_m
        has_tall[rows[tall], cols[tall]] = True

        # Fit floor plane on floor-band points only.
        floor_pts = cropped[
            (cropped[:, 2] >= cfg.floor_detect_z_min)
            & (cropped[:, 2] <= cfg.floor_detect_z_max)
        ]
        plane, floor_inliers = fit_perpendicular_floor_plane(
            floor_pts,
            max_tilt_deg=cfg.floor_max_tilt_deg,
            iterations=cfg.floor_ransac_iterations,
            threshold=cfg.floor_fit_threshold_m,
        )
        ransac_floor_xyz = floor_pts[floor_inliers] if plane is not None else np.zeros((0, 3))

        observed_floor = np.zeros((n, n), dtype=bool)
        lower_floor = np.zeros((n, n), dtype=bool)
        if plane is not None:
            heights = signed_height_above_plane(pts, plane)
            floor_mask = np.abs(heights) <= cfg.floor_observed_threshold_m
            observed_floor[rows[floor_mask], cols[floor_mask]] = True

            low_band = (
                (heights >= cfg.lidar_obstacle_min_height_m)
                & (heights <= cfg.base_collision_z_max_m)
            )
            has_low_obstacle[rows[low_band], cols[low_band]] = True

            below = floor_band & (heights < -cfg.thresh_cliff_m)
            lower_floor[rows[below], cols[below]] = True

        else:
            floor_mask = (
                (pts[:, 2] >= cfg.floor_detect_z_min)
                & (pts[:, 2] <= cfg.floor_detect_z_max)
            )
            observed_floor[rows[floor_mask], cols[floor_mask]] = True

        ray_hit = has_any.copy()
        non_floor_occupied = has_low_obstacle | has_tall
        missing_floor = ray_hit & (~observed_floor) & (~non_floor_occupied)
        # Any currently observed floor suppresses stale/overhead occlusion.
        # Low obstacle evidence is still accumulated separately as obstacle.
        clear_floor = observed_floor & (~lower_floor)

        # Cliff: connected missing-floor regions with nearby floor support.
        cliff_mask = np.zeros((n, n), dtype=bool)
        for comp in _connected_components(missing_floor):
            if comp.shape[0] < cfg.min_cliff_cells:
                continue
            floor_neighbors = 0
            for row, col in comp:
                for dr in (-1, 0, 1):
                    for dc in (-1, 0, 1):
                        nr, nc = row + dr, col + dc
                        if 0 <= nr < n and 0 <= nc < n and observed_floor[nr, nc]:
                            floor_neighbors += 1
            if floor_neighbors >= cfg.min_edge_floor_cells:
                for row, col in comp:
                    cliff_mask[row, col] = True

        # Also mark cells with measured lower floor as cliff.
        cliff_mask |= lower_floor & ray_hit & (~non_floor_occupied)
        cliff_mask = _drop_small_components(cliff_mask, cfg.min_cliff_cells)

        # Occlusion: tall returns that hide the floor, plus shadowed cells behind
        # them. If floor is visible in the same cell, it is not occluded.
        occlusion_mask = has_tall & (~observed_floor) & (~has_low_obstacle)
        tall_rows, tall_cols = np.where(has_tall)
        for row, col in zip(tall_rows, tall_cols):
            # Shadow cells beyond the tall return along the radial ray from the robot center.
            cr = self.size // 2
            cc = self.size // 2
            dr = row - cr
            dc = col - cc
            if dr == 0 and dc == 0:
                continue
            steps = max(abs(dr), abs(dc))
            for t in range(steps + 1, self.size + 1):
                sr = cr + int(round(dr * t / steps))
                sc = cc + int(round(dc * t / steps))
                if not (0 <= sr < n and 0 <= sc < n):
                    break
                if missing_floor[sr, sc]:
                    occlusion_mask[sr, sc] = True
        occlusion_mask &= ~clear_floor

        obstacle_mask = has_low_obstacle & (~cliff_mask) & (~occlusion_mask)

        return LidarHits(
            obstacle_xy=self._mask_to_xy(obstacle_mask),
            cliff_xy=self._mask_to_xy(cliff_mask),
            occlusion_xy=self._mask_to_xy(occlusion_mask),
            clear_floor_xy=self._mask_to_xy(clear_floor),
            ransac_floor_xyz=np.asarray(ransac_floor_xyz, dtype=np.float64).reshape(-1, 3),
        )

    def _mask_to_xy(self, mask: np.ndarray) -> np.ndarray:
        idx = np.argwhere(mask)
        if len(idx) == 0:
            return np.zeros((0, 2))
        rows = idx[:, 0].astype(np.float64)
        cols = idx[:, 1].astype(np.float64)
        x = self.origin + cols * self.resolution
        y = self.origin + rows * self.resolution
        return np.column_stack([x, y])
