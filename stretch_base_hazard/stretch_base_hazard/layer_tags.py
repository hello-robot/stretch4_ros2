"""Layer tags for hazard grid cells."""

from __future__ import annotations

LAYER_UNKNOWN = 0
LAYER_LIDAR_CLIFF = 1
LAYER_LIDAR_OBSTACLE = 2
LAYER_LIDAR_OCCLUSION = 3
LAYER_LINE_SMALL_DROP = 4
LAYER_LINE_OBSTACLE = 5

LAYER_NAMES = {
    LAYER_UNKNOWN: 'unknown',
    LAYER_LIDAR_CLIFF: 'lidar_cliff',
    LAYER_LIDAR_OBSTACLE: 'lidar_obstacle',
    LAYER_LIDAR_OCCLUSION: 'lidar_occlusion',
    LAYER_LINE_SMALL_DROP: 'line_small_drop',
    LAYER_LINE_OBSTACLE: 'line_obstacle',
}
