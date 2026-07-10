"""Odom-aligned integer-shifted rolling evidence grid."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from stretch_base_hazard.layer_tags import LAYER_UNKNOWN


@dataclass
class GridConfig:
    map_radius_m: float = 2.0
    resolution_m: float = 0.05
    decay_factor: float = 0.98
    max_evidence: float = 20.0


class RollingHazardGrid:
    """Odom-aligned integer-shifted rolling evidence grid."""

    def __init__(self, config: GridConfig):
        self.config = config
        self.resolution = config.resolution_m
        self.radius = config.map_radius_m
        self.size = int(np.ceil(2.0 * self.radius / self.resolution))

        # Grid bounds track the ODOM frame
        self.min_x_odom = 0.0
        self.min_y_odom = 0.0

        self.evidence = np.zeros((self.size, self.size), dtype=np.float32)
        self.layers = np.zeros((self.size, self.size), dtype=np.uint8)
        self._last_pose: tuple[float, float, float] | None = None

    def reset(self) -> None:
        self.evidence.fill(0.0)
        self.layers.fill(LAYER_UNKNOWN)
        self._last_pose = None
        self.min_x_odom = 0.0
        self.min_y_odom = 0.0

    def _base_to_odom(self, xy: np.ndarray) -> np.ndarray:
        if self._last_pose is None or len(xy) == 0:
            return xy
        rx, ry, ryaw = self._last_pose
        c, s = np.cos(ryaw), np.sin(ryaw)
        out = np.empty_like(xy)
        out[:, 0] = c * xy[:, 0] - s * xy[:, 1] + rx
        out[:, 1] = s * xy[:, 0] + c * xy[:, 1] + ry
        return out

    def _odom_to_base(self, xy: np.ndarray) -> np.ndarray:
        if self._last_pose is None or len(xy) == 0:
            return xy
        rx, ry, ryaw = self._last_pose
        c, s = np.cos(-ryaw), np.sin(-ryaw)
        tx = xy[:, 0] - rx
        ty = xy[:, 1] - ry
        out = np.empty_like(xy)
        out[:, 0] = c * tx - s * ty
        out[:, 1] = s * tx + c * ty
        return out

    def world_to_indices(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Accept base_link xy and return internal odom indices."""
        xy_odom = self._base_to_odom(xy)
        return self.odom_to_indices(xy_odom)

    def odom_to_indices(self, xy_odom: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Accept odom-frame XY and return internal row/col indices."""
        xy_odom = np.atleast_2d(np.asarray(xy_odom, dtype=np.float64))
        cols = np.floor((xy_odom[:, 0] - self.min_x_odom) / self.resolution).astype(np.int64)
        rows = np.floor((xy_odom[:, 1] - self.min_y_odom) / self.resolution).astype(np.int64)
        return rows, cols

    def indices_to_world(self, rows: np.ndarray, cols: np.ndarray) -> np.ndarray:
        """Take internal odom indices and return base_link xy."""
        rows = np.asarray(rows, dtype=np.float64)
        cols = np.asarray(cols, dtype=np.float64)
        x_odom = self.min_x_odom + (cols + 0.5) * self.resolution
        y_odom = self.min_y_odom + (rows + 0.5) * self.resolution
        xy_odom = np.column_stack([x_odom, y_odom])
        return self._odom_to_base(xy_odom)

    def in_bounds(self, rows: np.ndarray, cols: np.ndarray) -> np.ndarray:
        return (rows >= 0) & (rows < self.size) & (cols >= 0) & (cols < self.size)

    def update_pose(self, x: float, y: float, yaw: float) -> tuple[float, float, float]:
        """Shifts grid array purely by discrete integer cells."""
        if self._last_pose is None:
            self.min_x_odom = x - self.radius
            self.min_y_odom = y - self.radius
            self._last_pose = (x, y, yaw)
            return 0.0, 0.0, 0.0

        cx = self.min_x_odom + self.radius
        cy = self.min_y_odom + self.radius

        # How many full cells has the robot drifted from the physical center of the array?
        shift_cols = int(np.floor((x - cx) / self.resolution))
        shift_rows = int(np.floor((y - cy) / self.resolution))

        if shift_cols != 0 or shift_rows != 0:
            # Structurally shift the grid data in opposite direction to motion
            self.evidence = np.roll(self.evidence, shift=(-shift_rows, -shift_cols), axis=(0, 1))
            self.layers = np.roll(self.layers, shift=(-shift_rows, -shift_cols), axis=(0, 1))

            # Clear newly exposed edges
            if shift_rows > 0:
                self.evidence[-shift_rows:, :] = 0.0
                self.layers[-shift_rows:, :] = LAYER_UNKNOWN
            elif shift_rows < 0:
                self.evidence[:-shift_rows, :] = 0.0
                self.layers[:-shift_rows, :] = LAYER_UNKNOWN

            if shift_cols > 0:
                self.evidence[:, -shift_cols:] = 0.0
                self.layers[:, -shift_cols:] = LAYER_UNKNOWN
            elif shift_cols < 0:
                self.evidence[:, :-shift_cols] = 0.0
                self.layers[:, :-shift_cols] = LAYER_UNKNOWN

            self.min_x_odom += shift_cols * self.resolution
            self.min_y_odom += shift_rows * self.resolution

        lx, ly, lyaw = self._last_pose
        self._last_pose = (x, y, yaw)
        return x - lx, y - ly, yaw - lyaw

    def decay(self) -> None:
        self.evidence *= self.config.decay_factor

    def accumulate(self, xy: np.ndarray, weight: float, layer: int) -> None:
        if len(xy) == 0 or weight <= 0.0:
            return
        rows, cols = self.world_to_indices(xy[:, :2])
        self._accumulate_indices(rows, cols, weight, layer)

    def _accumulate_indices(
        self,
        rows: np.ndarray,
        cols: np.ndarray,
        weight: float,
        layer: int,
    ) -> None:
        valid = self.in_bounds(rows, cols)
        rows, cols = rows[valid], cols[valid]
        if len(rows) == 0:
            return

        # Optional: strictly enforce circular accumulation bound based on current pose
        cx = self.min_x_odom + self.radius
        cy = self.min_y_odom + self.radius
        x_odom = self.min_x_odom + cols * self.resolution
        y_odom = self.min_y_odom + rows * self.resolution
        in_radius = ((x_odom - cx)**2 + (y_odom - cy)**2) <= self.radius**2
        rows, cols = rows[in_radius], cols[in_radius]
        if len(rows) == 0:
            return

        for r, c in zip(rows, cols):
            self.evidence[r, c] = min(self.evidence[r, c] + weight, self.config.max_evidence)
            self.layers[r, c] = layer

    def clear(self, xy: np.ndarray, layers: tuple[int, ...] | None = None) -> None:
        if len(xy) == 0:
            return
        rows, cols = self.world_to_indices(xy[:, :2])
        self._clear_indices(rows, cols, layers)

    def _clear_indices(
        self,
        rows: np.ndarray,
        cols: np.ndarray,
        layers: tuple[int, ...] | None = None,
    ) -> None:
        valid = self.in_bounds(rows, cols)
        rows, cols = rows[valid], cols[valid]
        if len(rows) == 0:
            return

        if layers is not None:
            clearable = np.isin(self.layers[rows, cols], layers)
            rows, cols = rows[clearable], cols[clearable]
            if len(rows) == 0:
                return

        self.evidence[rows, cols] = 0.0
        self.layers[rows, cols] = LAYER_UNKNOWN

    def clear_layers(self, layers: tuple[int, ...]) -> None:
        if not layers:
            return
        mask = np.isin(self.layers, layers)
        self.evidence[mask] = 0.0
        self.layers[mask] = LAYER_UNKNOWN
