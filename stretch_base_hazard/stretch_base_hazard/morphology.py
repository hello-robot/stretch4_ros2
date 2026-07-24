"""Morphological cleanup and hazard extraction from evidence grid."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy import ndimage

from stretch_base_hazard.layer_tags import (
    LAYER_LIDAR_CLIFF,
    LAYER_LIDAR_OBSTACLE,
    LAYER_LIDAR_OCCLUSION,
    LAYER_LINE_DEEP_DROP,
    LAYER_LINE_DEGRADED,
    LAYER_LINE_OBSTACLE,
    LAYER_LINE_PROBABLE_CLIFF,
    LAYER_LINE_SMALL_DROP,
    LAYER_UNKNOWN,
    LINE_CLIFF_LAYERS,
)

# Order used to label cells that morphology invented.
# Least severe first: a later match overwrites an earlier one, so the most
# severe neighbouring class wins the cell.
LAYER_FILL_PRIORITY = (
    LAYER_LINE_DEGRADED,
    LAYER_LIDAR_OCCLUSION,
    LAYER_LIDAR_OBSTACLE,
    LAYER_LINE_OBSTACLE,
    LAYER_LIDAR_CLIFF,
    LAYER_LINE_SMALL_DROP,
    LAYER_LINE_DEEP_DROP,
    LAYER_LINE_PROBABLE_CLIFF,
)


@dataclass
class MorphologyConfig:
    # Minimum evidence a cell needs before it's considered a hazard at all.
    hazard_threshold: float = 5.0
    # Expands the hazard mask by N cells and fills gaps so fragmented
    # detections merge into one blob.
    dilation_cells: int = 1
    # Shrinks the hazard mask by N cells to remove small artifacts.
    erosion_cells: int = 1
    # Visualization height (not physical detection)
    cliff_z: float = -0.05
    obstacle_z: float = 0.02
    occlusion_z: float = 0.15


@dataclass
class HazardOutput:
    all_points: np.ndarray
    cliff_points: np.ndarray
    obstacle_points: np.ndarray
    occluded_points: np.ndarray
    line_cliff_points: np.ndarray = None
    degraded_points: np.ndarray = None


def _connected_components(mask: np.ndarray) -> list[np.ndarray]:
    labeled, n = ndimage.label(mask)
    if n == 0:
        return []
    components = []
    for i in range(1, n + 1):
        idx = np.argwhere(labeled == i)
        components.append(idx)
    return components


class HazardExtractor:
    """Threshold, morphology, and layer split from rolling grid."""

    def __init__(self, config: MorphologyConfig):
        self.config = config

    def extract(
        self,
        evidence: np.ndarray,
        layers: np.ndarray,
        origin_x: float,
        origin_y: float,
        resolution: float,
    ) -> HazardOutput:
        cfg = self.config
        mask = evidence >= cfg.hazard_threshold
        if not np.any(mask):
            return HazardOutput(
                all_points=np.zeros((0, 3)),
                cliff_points=np.zeros((0, 3)),
                obstacle_points=np.zeros((0, 3)),
                occluded_points=np.zeros((0, 3)),
                line_cliff_points=np.zeros((0, 3)),
                degraded_points=np.zeros((0, 3)),
            )

        structure = ndimage.generate_binary_structure(2, 1)
        if cfg.dilation_cells > 0:
            mask = ndimage.binary_dilation(mask, structure=structure,
                                           iterations=cfg.dilation_cells)
        if cfg.erosion_cells > 0:
            mask = ndimage.binary_erosion(mask, structure=structure, iterations=cfg.erosion_cells)

        filtered = np.zeros_like(mask)
        for comp in _connected_components(mask):
            for row, col in comp:
                filtered[row, col] = True

        if not np.any(filtered):
            return HazardOutput(
                all_points=np.zeros((0, 3)),
                cliff_points=np.zeros((0, 3)),
                obstacle_points=np.zeros((0, 3)),
                occluded_points=np.zeros((0, 3)),
                line_cliff_points=np.zeros((0, 3)),
                degraded_points=np.zeros((0, 3)),
            )

        layers = _fill_invented_cells(layers, filtered, structure, cfg.dilation_cells)

        idx = np.argwhere(filtered)
        rows = idx[:, 0]
        cols = idx[:, 1]
        x = origin_x + cols.astype(np.float64) * resolution
        y = origin_y + rows.astype(np.float64) * resolution
        cell_layers = layers[rows, cols]

        cliff_mask = cell_layers == LAYER_LIDAR_CLIFF
        line_cliff_mask = np.isin(cell_layers, list(LINE_CLIFF_LAYERS))
        obstacle_mask = np.isin(cell_layers, [LAYER_LIDAR_OBSTACLE, LAYER_LINE_OBSTACLE])
        occluded_mask = cell_layers == LAYER_LIDAR_OCCLUSION
        degraded_mask = cell_layers == LAYER_LINE_DEGRADED
        other_mask = ~(
            cliff_mask | line_cliff_mask | obstacle_mask | occluded_mask | degraded_mask
        )
        obstacle_mask |= other_mask

        cliff_pts = _xy_to_xyz(x[cliff_mask], y[cliff_mask], cfg.cliff_z)
        line_cliff_pts = _xy_to_xyz(x[line_cliff_mask], y[line_cliff_mask], cfg.cliff_z)
        obstacle_pts = _xy_to_xyz(x[obstacle_mask], y[obstacle_mask], cfg.obstacle_z)
        occluded_pts = _xy_to_xyz(x[occluded_mask], y[occluded_mask], cfg.occlusion_z)
        degraded_pts = _xy_to_xyz(x[degraded_mask], y[degraded_mask], cfg.occlusion_z)

        parts = (cliff_pts, line_cliff_pts, obstacle_pts, occluded_pts, degraded_pts)
        non_empty = [p for p in parts if len(p) > 0]
        all_pts = np.vstack(non_empty) if non_empty else np.zeros((0, 3))

        return HazardOutput(
            all_points=all_pts,
            cliff_points=cliff_pts,
            obstacle_points=obstacle_pts,
            occluded_points=occluded_pts,
            line_cliff_points=line_cliff_pts,
            degraded_points=degraded_pts,
        )


def _fill_invented_cells(
    layers: np.ndarray,
    filtered: np.ndarray,
    structure: np.ndarray,
    dilation_cells: int,
) -> np.ndarray:
    """
    Label hazard cells that dilation invented. Each invented cell instead inherits the tag of the
    blob it grew from, most severe neighbour winning.
    """
    if dilation_cells <= 0:
        return layers
    unknown = filtered & (layers == LAYER_UNKNOWN)
    if not np.any(unknown):
        return layers

    resolved = np.full(layers.shape, LAYER_UNKNOWN, dtype=layers.dtype)
    for tag in LAYER_FILL_PRIORITY:
        source = layers == tag
        if not np.any(source):
            continue
        grown = ndimage.binary_dilation(
            source, structure=structure, iterations=dilation_cells,
        )
        resolved[unknown & grown] = tag

    out = layers.copy()
    take = unknown & (resolved != LAYER_UNKNOWN)
    out[take] = resolved[take]
    return out


def _xy_to_xyz(x: np.ndarray, y: np.ndarray, z: float) -> np.ndarray:
    if len(x) == 0:
        return np.zeros((0, 3))
    return np.column_stack([x, y, np.full(len(x), z, dtype=np.float64)])
