"""Parameter file lists for dual-lidar self-filter and Nav2 footprint."""

import os

TOOL_PRESETS = ('sg4', 'pg4', 'tablet', 'nil')

ROBOT_SELF_FILTER_YAML = 'robot_self_filter.yaml'
ROBOT_FOOTPRINT_YAML = 'robot_footprint.yaml'
DUAL_LIDAR_PIPELINE_YAML = 'dual_lidar_filter.yaml'


def validate_tool_preset(tool_preset: str) -> None:
    if tool_preset not in TOOL_PRESETS:
        raise RuntimeError(
            f"Invalid tool_preset '{tool_preset}'. "
            f"Expected one of: {', '.join(TOOL_PRESETS)}"
        )


def _config_path(stretch_core: str, name: str) -> str:
    path = os.path.join(stretch_core, 'config', name)
    if not os.path.isfile(path):
        raise RuntimeError(f'Missing stretch_core config: {path}')
    return path


def robot_self_filter_yaml(stretch_core: str) -> str:
    return _config_path(stretch_core, ROBOT_SELF_FILTER_YAML)


def tool_preset_yaml(stretch_core: str, tool_preset: str) -> str:
    validate_tool_preset(tool_preset)
    return _config_path(stretch_core, f'self_filter_{tool_preset}.yaml')


def dual_lidar_self_filter_parameters(stretch_core: str, tool_preset: str) -> list:
    """Pipeline + robot geometry + tool attachment (dual_lidar_laserscan node)."""
    return [
        _config_path(stretch_core, DUAL_LIDAR_PIPELINE_YAML),
        robot_self_filter_yaml(stretch_core),
        tool_preset_yaml(stretch_core, tool_preset),
    ]


def footprint_self_filter_parameters(stretch_core: str, tool_preset: str) -> list:
    """Nav2 footprint publisher wiring + same geometry as lidar self-filter."""
    return [
        _config_path(stretch_core, ROBOT_FOOTPRINT_YAML),
        robot_self_filter_yaml(stretch_core),
        tool_preset_yaml(stretch_core, tool_preset),
    ]
