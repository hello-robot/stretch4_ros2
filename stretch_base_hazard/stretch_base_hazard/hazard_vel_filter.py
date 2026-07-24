"""Velocity filtering against robot-centric hazard points."""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class HazardVelFilterConfig:
    """Geometry used to decide whether a base velocity points into a hazard."""

    lookahead_m: float = 0.50
    cliff_lookahead_m: float = 0.80
    footprint_m: float = 0.35
    obstacle_buffer_m: float = 0.02
    hard_obstacle_buffer_m: float = 0.01
    soft_obstacle_buffer_m: float = 0.10
    min_clearance_speed_scale: float = 0.20
    creep_linear_speed_mps: float = 0.05
    cliff_buffer_m: float = 0.20
    min_linear_speed_mps: float = 1e-4
    line_cliff_block_half_angle_deg: float = 60.0
    line_cliff_max_range_m: float = 1.0
    degraded_lookahead_m: float = 0.50
    degraded_buffer_m: float = 0.10
    degraded_speed_scale: float = 0.5


@dataclass(frozen=True)
class HazardVelFilterResult:
    """Filtered base velocity and the reason it was limited."""

    vx: float
    vy: float
    wz: float
    blocked_by_obstacle: bool = False
    slowed_by_obstacle: bool = False
    blocked_by_cliff: bool = False
    blocking_obstacle_count: int = 0
    soft_obstacle_count: int = 0
    blocking_cliff_count: int = 0
    obstacle_speed_scale: float = 1.0
    blocked_by_line_cliff: bool = False
    line_cliff_count: int = 0
    slowed_by_degraded: bool = False
    degraded_count: int = 0


@dataclass(frozen=True)
class HazardHeadingBlockage:
    """Hazard counts for one candidate base translation direction."""

    blocked_by_obstacle: bool = False
    slowed_by_obstacle: bool = False
    blocked_by_cliff: bool = False
    blocking_obstacle_count: int = 0
    soft_obstacle_count: int = 0
    blocking_cliff_count: int = 0
    obstacle_speed_scale: float = 1.0
    blocked_by_line_cliff: bool = False
    line_cliff_count: int = 0
    slowed_by_degraded: bool = False
    degraded_count: int = 0


def filter_hazard_velocity(
    vx: float,
    vy: float,
    wz: float,
    obstacle_xy: np.ndarray,
    cliff_xy: np.ndarray,
    config: HazardVelFilterConfig | None = None,
    line_cliff_xy: np.ndarray | None = None,
    degraded_xy: np.ndarray | None = None,
) -> HazardVelFilterResult:
    """
    Remove commanded linear motion that points into known hazards.

    Points are expected in the robot base frame, with +X forward and +Y left.
    Rotation is preserved because the under-base hazard map constrains
    translational motion rather than in-place yaw.
    """
    cfg = config or HazardVelFilterConfig()
    linear = np.asarray([float(vx), float(vy)], dtype=np.float64)
    speed = float(np.linalg.norm(linear))
    if speed <= cfg.min_linear_speed_mps:
        return HazardVelFilterResult(float(vx), float(vy), float(wz))

    heading = linear / speed
    blockage = classify_hazard_heading(
        heading, obstacle_xy, cliff_xy, cfg, line_cliff_xy, degraded_xy,
    )

    if (
        blockage.blocked_by_obstacle
        or blockage.blocked_by_cliff
        or blockage.blocked_by_line_cliff
    ):
        linear = linear - float(np.dot(linear, heading)) * heading
        linear[np.abs(linear) <= cfg.min_linear_speed_mps] = 0.0
    elif blockage.slowed_by_obstacle:
        scale = min(max(float(blockage.obstacle_speed_scale), 0.0), 1.0)
        linear = linear * scale
        if cfg.creep_linear_speed_mps > 0.0:
            slowed_speed = float(np.linalg.norm(linear))
            creep_speed = float(cfg.creep_linear_speed_mps)
            if slowed_speed > creep_speed:
                linear = linear * (creep_speed / slowed_speed)
        linear[np.abs(linear) <= cfg.min_linear_speed_mps] = 0.0
    elif blockage.slowed_by_degraded:
        # Reduced confidence, not a hazard: Scale speed down.
        scale = min(max(float(cfg.degraded_speed_scale), 0.0), 1.0)
        linear = linear * scale
        linear[np.abs(linear) <= cfg.min_linear_speed_mps] = 0.0

    return HazardVelFilterResult(
        vx=float(linear[0]),
        vy=float(linear[1]),
        wz=float(wz),
        blocked_by_obstacle=blockage.blocked_by_obstacle,
        slowed_by_obstacle=blockage.slowed_by_obstacle,
        blocked_by_cliff=blockage.blocked_by_cliff,
        blocking_obstacle_count=blockage.blocking_obstacle_count,
        soft_obstacle_count=blockage.soft_obstacle_count,
        blocking_cliff_count=blockage.blocking_cliff_count,
        obstacle_speed_scale=blockage.obstacle_speed_scale,
        blocked_by_line_cliff=blockage.blocked_by_line_cliff,
        line_cliff_count=blockage.line_cliff_count,
        slowed_by_degraded=blockage.slowed_by_degraded,
        degraded_count=blockage.degraded_count,
    )


def classify_hazard_heading(
    heading_xy: np.ndarray,
    obstacle_xy: np.ndarray,
    cliff_xy: np.ndarray,
    config: HazardVelFilterConfig | None = None,
    line_cliff_xy: np.ndarray | None = None,
    degraded_xy: np.ndarray | None = None,
) -> HazardHeadingBlockage:
    """Classify whether translation along a heading would be blocked."""
    cfg = config or HazardVelFilterConfig()
    heading = np.asarray(heading_xy, dtype=np.float64).reshape(2)
    norm = float(np.linalg.norm(heading))
    if norm <= cfg.min_linear_speed_mps:
        return HazardHeadingBlockage()
    heading = heading / norm

    hard_lateral_limit = cfg.footprint_m + cfg.hard_obstacle_buffer_m
    soft_lateral_limit = cfg.footprint_m + max(
        cfg.soft_obstacle_buffer_m,
        cfg.obstacle_buffer_m,
        cfg.hard_obstacle_buffer_m,
    )
    obstacle_clearance = _obstacle_clearance(
        obstacle_xy,
        heading,
        lookahead_m=cfg.lookahead_m,
        hard_lateral_limit_m=hard_lateral_limit,
        soft_lateral_limit_m=soft_lateral_limit,
        min_speed_scale=cfg.min_clearance_speed_scale,
    )
    cliff_count = _count_blockers(
        cliff_xy,
        heading,
        lookahead_m=cfg.cliff_lookahead_m,
        lateral_limit_m=cfg.footprint_m + cfg.cliff_buffer_m,
    )
    line_cliff_count = 0
    if line_cliff_xy is not None and len(line_cliff_xy):
        line_cliff_count = _count_bearing_blockers(
            line_cliff_xy,
            heading,
            half_angle_deg=cfg.line_cliff_block_half_angle_deg,
            max_range_m=cfg.line_cliff_max_range_m,
        )
    degraded_count = 0
    if degraded_xy is not None and len(degraded_xy):
        degraded_count = _count_blockers(
            degraded_xy,
            heading,
            lookahead_m=cfg.degraded_lookahead_m,
            lateral_limit_m=cfg.footprint_m + cfg.degraded_buffer_m,
        )
    return HazardHeadingBlockage(
        blocked_by_obstacle=obstacle_clearance['hard_count'] > 0,
        slowed_by_obstacle=(
            obstacle_clearance['hard_count'] == 0
            and obstacle_clearance['soft_count'] > 0
        ),
        blocked_by_cliff=cliff_count > 0,
        blocking_obstacle_count=obstacle_clearance['hard_count'],
        soft_obstacle_count=obstacle_clearance['soft_count'],
        blocking_cliff_count=cliff_count,
        obstacle_speed_scale=obstacle_clearance['speed_scale'],
        blocked_by_line_cliff=line_cliff_count > 0,
        line_cliff_count=line_cliff_count,
        slowed_by_degraded=degraded_count > 0,
        degraded_count=degraded_count,
    )


def _obstacle_clearance(
    points_xy: np.ndarray,
    heading: np.ndarray,
    *,
    lookahead_m: float,
    hard_lateral_limit_m: float,
    soft_lateral_limit_m: float,
    min_speed_scale: float,
) -> dict[str, float | int]:
    points = _as_xy(points_xy)
    if len(points) == 0:
        return {'hard_count': 0, 'soft_count': 0, 'speed_scale': 1.0}

    lateral_axis = np.asarray([-heading[1], heading[0]], dtype=np.float64)
    along = points @ heading
    lateral = np.abs(points @ lateral_axis)
    finite = np.isfinite(points).all(axis=1)
    lookahead = (
        finite
        & (along > 0.0)
        & (along <= max(float(lookahead_m), 0.0))
    )
    hard_limit = max(float(hard_lateral_limit_m), 0.0)
    soft_limit = max(float(soft_lateral_limit_m), hard_limit)

    hard = lookahead & (lateral <= hard_limit)
    soft = lookahead & (lateral <= soft_limit)
    hard_count = int(np.count_nonzero(hard))
    soft_count = int(np.count_nonzero(soft))
    if hard_count > 0 or soft_count == 0 or soft_limit <= hard_limit:
        return {'hard_count': hard_count, 'soft_count': soft_count, 'speed_scale': 1.0}

    soft_lateral = lateral[soft]
    min_lateral = float(np.min(soft_lateral))
    clearance = (min_lateral - hard_limit) / (soft_limit - hard_limit)
    clearance = min(max(clearance, 0.0), 1.0)
    min_scale = min(max(float(min_speed_scale), 0.0), 1.0)
    speed_scale = min_scale + (1.0 - min_scale) * clearance
    return {
        'hard_count': hard_count,
        'soft_count': soft_count,
        'speed_scale': speed_scale,
    }


def _count_blockers(
    points_xy: np.ndarray,
    heading: np.ndarray,
    *,
    lookahead_m: float,
    lateral_limit_m: float,
) -> int:
    points = _as_xy(points_xy)
    if len(points) == 0:
        return 0

    lateral_axis = np.asarray([-heading[1], heading[0]], dtype=np.float64)
    along = points @ heading
    lateral = np.abs(points @ lateral_axis)
    finite = np.isfinite(points).all(axis=1)
    blocking = (
        finite
        & (along > 0.0)
        & (along <= max(float(lookahead_m), 0.0))
        & (lateral <= max(float(lateral_limit_m), 0.0))
    )
    return int(np.count_nonzero(blocking))


def _count_bearing_blockers(
    points_xy: np.ndarray,
    heading: np.ndarray,
    *,
    half_angle_deg: float,
    max_range_m: float,
) -> int:
    """
    Count points whose bearing falls within half_angle_deg of the heading.
    heading is not part of the test. Used with line-sensor cliff evidence to stop directly without usign a lookahead distance.
    """
    points = _as_xy(points_xy)
    if len(points) == 0:
        return 0

    ranges = np.linalg.norm(points, axis=1)
    finite = np.isfinite(points).all(axis=1)
    # Points at the origin have no meaningful bearing.
    usable = finite & (ranges > 0.0)
    if max_range_m > 0.0:
        usable &= ranges <= float(max_range_m)
    if not np.any(usable):
        return 0

    safe_ranges = np.where(ranges > 0.0, ranges, 1.0)
    cos_to_heading = (points @ heading) / safe_ranges
    half_angle = math.radians(min(max(float(half_angle_deg), 0.0), 180.0))
    return int(np.count_nonzero(usable & (cos_to_heading >= math.cos(half_angle))))


def _as_xy(points_xy: np.ndarray) -> np.ndarray:
    points = np.asarray(points_xy, dtype=np.float64)
    if points.size == 0:
        return np.zeros((0, 2), dtype=np.float64)
    if points.ndim == 1:
        points = points.reshape(1, -1)
    if points.shape[1] < 2:
        raise ValueError('hazard points must have at least x and y columns')
    return np.ascontiguousarray(points[:, :2], dtype=np.float64)
