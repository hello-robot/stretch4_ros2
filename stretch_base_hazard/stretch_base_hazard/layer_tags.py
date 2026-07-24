"""Layer tags for hazard grid cells."""

from __future__ import annotations

LAYER_UNKNOWN = 0
LAYER_LIDAR_CLIFF = 1
LAYER_LIDAR_OBSTACLE = 2
LAYER_LIDAR_OCCLUSION = 3
LAYER_LINE_SMALL_DROP = 4
LAYER_LINE_OBSTACLE = 5
LAYER_LINE_DEEP_DROP = 6
LAYER_LINE_DEGRADED = 7
LAYER_LINE_PROBABLE_CLIFF = 8

# Cliff evidence from the line sensors block motion toward them outright unlike lidar cliffs,
# which are seen far enough ahead for the usual lookahead logic to apply.
LINE_CLIFF_LAYERS = (
    LAYER_LINE_SMALL_DROP,
    LAYER_LINE_DEEP_DROP,
    LAYER_LINE_PROBABLE_CLIFF,
)

LAYER_NAMES = {
    LAYER_UNKNOWN: 'unknown',
    LAYER_LIDAR_CLIFF: 'lidar_cliff',
    LAYER_LIDAR_OBSTACLE: 'lidar_obstacle',
    LAYER_LIDAR_OCCLUSION: 'lidar_occlusion',
    LAYER_LINE_SMALL_DROP: 'line_small_drop',
    LAYER_LINE_OBSTACLE: 'line_obstacle',
    LAYER_LINE_DEEP_DROP: 'line_deep_drop',
    LAYER_LINE_DEGRADED: 'line_degraded',
    LAYER_LINE_PROBABLE_CLIFF: 'line_probable_cliff',
}
