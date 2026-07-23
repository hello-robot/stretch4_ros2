"""Every tunable in one place.

`LineSensorConfig` is the single knob-board for the whole pipeline. Each field
is read by exactly one stage; the grouping below follows the stages so you can
find the knobs for the behaviour you are looking at. Defaults here are the
in-code defaults; a ROS param override where the node is built supersedes them.
See README.md → "Where each knob lives".
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass
class LineSensorConfig:
    # --- Stage: classify (classify.py) --------------------------------------
    line_obstacle_min_height_m: float = 0.025
    floor_band_m: float = 0.015

    use_range_deviation: bool = True
    dev_floor_band_m: float = 0.010
    dev_obstacle_strong_m: float = 0.020
    marginal_min_run_bins: int = 5
    strong_confirm_frames: int = 1
    marginal_confirm_frames: int = 2
    # Marginal evidence is judged on LOCAL contrast (z minus the rolling
    # median of surrounding bins)
    marginal_contrast_min_m: float = 0.010
    contrast_window_bins: int = 81
    # Null-as-evidence. 5.11 m no-returns are classified by context
    # instead of discarded.
    use_null_evidence: bool = True
    null_range_m: float = 5.11
    null_tolerance_m: float = 0.005
    # The chip emits more than one no-return code: 5.11 is the documented
    # null.
    null_sentinel_min_m: float = 4.0
    # Chronic-null prior : a bin only carries null evidence if it
    # demonstrably returned on clear floor during calibration. When
    # per-bin null rates are unavailable the tare mask is the fallback gate.
    chronic_null_rate_max: float = 0.10
    null_min_run_bins: int = 8
    null_persist_min_fraction: float = 0.6
    suppression_near_range_m: float = 0.15
    shadow_adjacency_bins: int = 3
    cliff_adjacent_drop_bins: int = 6
    cliff_bearing_adjacency_deg: float = 15.0
    # Deep drops: returning bins deeper than cliff_max_drop classify as
    # DEEP_DROP (by checking deviation) and publish on the deep-drop output alongside cliff-typed null runs.
    use_deep_drop: bool = True
    # Depth under-read correction: the sensor reads drops shallow by a
    # roughly depth-proportional factor. 1.0 disables.
    depth_underread_scale: float = 0.91
    # Degraded sectors: a sensor whose reliable bins are mostly nulls
    # with no benign explanation (not suppression/shadow) and no confirmed
    # cliff typing has lost floor coverage — published as degraded points so
    # the hazard layer can slow down through that sector instead of stopping.
    degraded_min_fraction: float = 0.35
    # Degraded anti-strobe: reflective floors have a 
    # blind fraction in the 0.25-0.45 band around the threshold
    # The fraction is smoothed with a per-frame EMA and the state has
    # hysteresis: enters at degraded_min_fraction, exits below
    # degraded_exit_fraction. 
    degraded_exit_fraction: float = 0.25
    # EMA weight of the new frame (0.1 at 30 Hz ~ 1 s memory). 1.0 disables
    # smoothing.
    degraded_frac_alpha: float = 0.1
    cliff_min_drop_m: float = 0.02
    cliff_max_drop_m: float = 0.10
    line_min_run_bins: int = 3
    line_max_run_radial_span_m: float = 0.25
    line_point_noise_max_run_bins: int = 12
    line_point_noise_xy_span_max_m: float = 0.010
    line_point_noise_radial_span_max_m: float = 0.010
    line_confirm_frames: int = 3
    line_fast_confirm_frames: int = 2
    line_fast_confirm_range_m: float = 0.55
    line_window_frames: int = 4
    line_require_consecutive: bool = True
    line_spray_merge_gap_bins: int = 6
    line_radial_streak_head_radius_max_m: float = 0.35
    line_radial_streak_span_min_m: float = 0.04
    line_radial_streak_angular_spread_max_deg: float = 20.0
    line_radial_streak_aspect_ratio_min: float = 3.0
    spray_min_run_bins: int = 3
    spray_roughness_thresh_m: float = 0.03
    spray_max_run_bins: int = 0
    spray_head_radius_max_m: float = 0.30
    spray_radial_span_min_m: float = 0.05
    spray_angular_spread_max_deg: float = 15.0
    spray_aspect_ratio_min: float = 5.0
    spray_direction_cluster_gap_deg: float = 5.0
    spray_monotonic_score_min: float = 0.70
    spray_monotonic_tolerance_m: float = 0.005
    spray_short_run_bonus_max_bins: int = 15
    spray_temporal_window_frames: int = 5
    spray_temporal_stable_min_frames: int = 2
    spray_temporal_stable_fraction: float = 0.50
    # Glossy-floor spray defense. Suppressed candidates are rerouted to SPRAY so
    # they stay visible on the spray debug topic.
    use_spray_bin_quarantine: bool = True
    # EMA decay per frame for the flip counter (~2 s memory at 30 Hz).
    spray_flip_decay: float = 0.967
    # Quarantine a bin when its decayed flip count exceeds this.
    spray_flip_suspect_threshold: float = 4.0
    # Cross-sensor gate: this many sensors each showing at least min_bins
    # hazard candidates nearer than near_range makes near-field arcs on ALL
    # sensors spray for that frame.
    spray_cross_sensor_min_sensors: int = 3
    spray_cross_sensor_min_bins: int = 8
    spray_cross_sensor_near_range_m: float = 0.20
    # A near-field run is exempt from the cross-sensor gate when it looks
    # like a solid object pressed against the base.
    spray_cross_sensor_exempt_run_bins: int = 60
    spray_cross_sensor_exempt_max_ragged_m: float = 0.006
    # When a sensor has at least this many quarantine-suspect bins, its
    # strong obstacles need 2-frame confirmation.
    spray_suspect_confirm_min_bins: int = 10
    # The flip counter has no history right after startup, so the stricter
    # confirmation also applies to every sensor for this many frames
    spray_warmup_frames: int = 30
