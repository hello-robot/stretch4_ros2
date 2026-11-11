"""Parameter file lists for dual-lidar self-filter and Nav2 footprint."""

from __future__ import annotations

import math
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import Any

import yaml
from stretch4_urdf import get_robot_params, get_urdf

TOOL_PRESETS = ('auto', 'sg4', 'pg4', 'tablet', 'nil')

ROBOT_SELF_FILTER_YAML = 'robot_self_filter.yaml'
ROBOT_FOOTPRINT_YAML = 'robot_footprint.yaml'
DUAL_LIDAR_PIPELINE_YAML = 'dual_lidar_filter.yaml'
FUSED_LIDAR_PIPELINE_YAML = 'dual_lidar_fused.yaml'

TOOL_NAME_TO_PRESET = {
    'eoa_wrist_dw4_tool_sg4': 'sg4',
    'SE4_eoa_wrist_dw4_tool_sg4': 'sg4',
    'eoa_wrist_dw4_tool_pg4': 'pg4',
    'SE4_eoa_wrist_dw4_tool_pg4': 'pg4',
    'eoa_wrist_dw4_tool_tablet': 'tablet',
    'SE4_eoa_wrist_dw4_tool_tablet': 'tablet',
    'eoa_wrist_dw4_tool_nil': 'nil',
    'SE4_eoa_wrist_dw4_tool_nil': 'nil',
}

PRESET_TO_TOOL_DIR = {
    'sg4': 'eoa_wrist_dw4_tool_sg4',
    'pg4': 'eoa_wrist_dw4_tool_pg4',
    'tablet': 'eoa_wrist_dw4_tool_tablet',
}

# Links from the robot body URDF. The arm capsule remains enabled, while these
# collision-derived boxes catch local geometry such as the shoulder/wire cover,
# wrist housings, gripper camera, and flange details that a simple capsule misses.
MAIN_LINK_GROUPS = (
    ('arm', (
        'arm_l0_link', 'arm_l1_link', 'arm_l2_link', 'arm_l3_link', 'arm_l4_link',
        'lift_link',
    )),
    ('wrist', (
        'wrist_link', 'wrist_yaw_link', 'wrist_pitch_link', 'wrist_roll_link',
        'gripper_camera_link',
    )),
)

TOOL_LINK_GROUPS_BY_PRESET = {
    'sg4': ('quick_connect_interface_link',
            'gripper_finger_right_link', 'gripper_fingertip_right_link',
            'gripper_finger_left_link', 'gripper_fingertip_left_link'),
    'pg4': ('pjg_body_link', 'finger_right_link', 'finger_left_link'),
    'tablet': ('quick_connect_interface_link',),
}

# These buffers remove lidar artifact returns and, by default,
# expand the Nav2 footprint around the same URDF boxes for conservative planning.
FILTER_BUFFER_BY_GROUP = {
    'arm': 0.040,
    'wrist': 0.025,
    'gripper_camera': 0.025,
    'tool': 0.025,
}

# Footprint boxes intentionally use the same effective buffer as the self-filter
# by default. Set self_filter_box_footprint_buffers explicitly only when a deployment
# needs Nav2 geometry to differ from lidar self-filter geometry.

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


def _strip_ns(tag: str) -> str:
    return tag.rsplit('}', 1)[-1]


def _children(elem: ET.Element, name: str):
    return [child for child in elem if _strip_ns(child.tag) == name]


def _first_child(elem: ET.Element, name: str):
    for child in elem:
        if _strip_ns(child.tag) == name:
            return child
    return None


def _parse_vector(text: str | None, default: tuple[float, ...]) -> tuple[float, ...]:
    if not text:
        return default
    parts = [float(v) for v in text.split()]
    if len(parts) != len(default):
        return default
    return tuple(parts)


def _rpy_matrix(roll: float, pitch: float, yaw: float) -> list[list[float]]:
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def _mat_vec_mul(mat: list[list[float]], vec: tuple[float, float, float]) -> tuple[float, float, float]:
    return tuple(sum(mat[r][c] * vec[c] for c in range(3)) for r in range(3))


def _tool_name_for_preset(resolved: str) -> str:
    if resolved == 'nil':
        return 'eoa_wrist_dw4_tool_nil'
    return PRESET_TO_TOOL_DIR[resolved]


def _resolve_urdf_identity(tool_preset: str) -> tuple[str, str, str]:
    resolved = resolve_tool_preset(tool_preset)
    try:
        model_name, batch_name, robot_tool = get_robot_params()
    except Exception as ex:
        raise RuntimeError(
            'Failed to read robot model/batch/tool from stretch4_body RobotParams: '
            f'{ex}. URDF self-filter generation requires on-robot robot params.'
        ) from ex

    if not model_name or not batch_name:
        raise RuntimeError(
            'stretch4_body RobotParams did not provide model_name and batch_name '
            f'(got model_name={model_name!r}, batch_name={batch_name!r}). '
            'URDF self-filter generation requires on-robot robot params.'
        )

    if tool_preset == 'auto':
        tool_name = robot_tool or _tool_name_for_preset(resolved)
    else:
        tool_name = _tool_name_for_preset(resolved)

    if not tool_name:
        raise RuntimeError(
            f"Could not determine URDF tool name for tool_preset '{tool_preset}' "
            f"(resolved preset: {resolved!r})."
        )

    return model_name, batch_name, tool_name


def _resolve_mesh_path(filename: str) -> Path:
    return Path(filename.removeprefix('file://'))


def _mesh_bounds(mesh_path: Path, scale: tuple[float, float, float]) -> tuple[tuple[float, float, float], tuple[float, float, float]] | None:
    try:
        import numpy as np
        import trimesh
        loaded = trimesh.load_mesh(str(mesh_path), force='mesh', process=False)
        if isinstance(loaded, trimesh.Scene):
            meshes = [geom for geom in loaded.geometry.values()]
            if not meshes:
                return None
            loaded = trimesh.util.concatenate(meshes)
        vertices = np.asarray(loaded.vertices, dtype=float)
        if vertices.size == 0:
            return None
        vertices = vertices * np.asarray(scale, dtype=float)
        mins = vertices.min(axis=0)
        maxs = vertices.max(axis=0)
        center = tuple(((mins + maxs) * 0.5).tolist())
        half = tuple(((maxs - mins) * 0.5).tolist())
        return center, half
    except Exception as ex:
        print(f'Warning: failed to read collision mesh {mesh_path}: {ex}')
        return None


def _collision_box(link: ET.Element) -> list[dict[str, Any]]:
    boxes: list[dict[str, Any]] = []
    link_name = link.attrib.get('name', '')
    for collision in _children(link, 'collision'):
        origin = _first_child(collision, 'origin')
        xyz = _parse_vector(origin.attrib.get('xyz') if origin is not None else None, (0.0, 0.0, 0.0))
        rpy = _parse_vector(origin.attrib.get('rpy') if origin is not None else None, (0.0, 0.0, 0.0))
        geometry = _first_child(collision, 'geometry')
        if geometry is None:
            continue

        mesh = _first_child(geometry, 'mesh')
        box = _first_child(geometry, 'box')
        sphere = _first_child(geometry, 'sphere')
        cylinder = _first_child(geometry, 'cylinder')

        center = (0.0, 0.0, 0.0)
        half = None
        if mesh is not None:
            filename = mesh.attrib.get('filename')
            if not filename:
                continue
            scale = _parse_vector(mesh.attrib.get('scale'), (1.0, 1.0, 1.0))
            bounds = _mesh_bounds(_resolve_mesh_path(filename), scale)
            if bounds is None:
                continue
            center, half = bounds
        elif box is not None:
            size = _parse_vector(box.attrib.get('size'), (0.0, 0.0, 0.0))
            half = tuple(v * 0.5 for v in size)
        elif sphere is not None:
            radius = float(sphere.attrib.get('radius', 0.0))
            half = (radius, radius, radius)
        elif cylinder is not None:
            radius = float(cylinder.attrib.get('radius', 0.0))
            length = float(cylinder.attrib.get('length', 0.0))
            half = (radius, radius, length * 0.5)
        if half is None:
            continue

        rot = _rpy_matrix(*rpy)
        rotated_center = _mat_vec_mul(rot, center)
        local_origin = tuple(xyz[i] + rotated_center[i] for i in range(3))
        if max(half) <= 0.0:
            continue
        boxes.append({
            'frame': link_name,
            'name': collision.attrib.get('name') or link_name,
            'origin': local_origin,
            'rpy': rpy,
            'half': half,
        })
    return boxes


def _read_link_boxes(urdf_content: str, requested: dict[str, str]) -> list[dict[str, Any]]:
    root = ET.fromstring(urdf_content)
    links = {link.attrib.get('name'): link for link in root.iter() if _strip_ns(link.tag) == 'link'}
    boxes: list[dict[str, Any]] = []
    for link_name, group in requested.items():
        link = links.get(link_name)
        if link is None:
            print(f'Warning: URDF link {link_name} not found in generated URDF')
            continue
        link_group = 'gripper_camera' if link_name == 'gripper_camera_link' else group
        for box in _collision_box(link):
            box['group'] = link_group
            boxes.append(box)
    return boxes


def _tool_from_robot_params() -> tuple[str | None, str | None]:
    try:
        from stretch4_body.core.robot_params import RobotParams
        _, params = RobotParams.get_params()
        tool = params.get('robot', {}).get('tool')
        if not tool:
            return None, 'RobotParams has no robot.tool field'
        return tool, None
    except Exception as ex:
        return None, f'could not read Stretch RobotParams: {ex}'


def _find_tool_in_mapping(value: Any) -> str | None:
    if isinstance(value, str):
        for tool_name, preset in TOOL_NAME_TO_PRESET.items():
            if value == tool_name or tool_name in value:
                return preset
        for preset in ('sg4', 'pg4', 'tablet', 'nil'):
            if value == preset or value.endswith(f'_{preset}'):
                return preset
    if isinstance(value, dict):
        for key in ('tool', 'tool_name', 'end_of_arm_tool', 'end_effector', 'robot_tool'):
            if key in value:
                found = _find_tool_in_mapping(value[key])
                if found:
                    return found
        for child in value.values():
            found = _find_tool_in_mapping(child)
            if found:
                return found
    if isinstance(value, list):
        for child in value:
            found = _find_tool_in_mapping(child)
            if found:
                return found
    return None


def _tool_from_fleet_yaml() -> tuple[str | None, list[str]]:
    paths: list[Path] = []
    fleet_path = os.environ.get('HELLO_FLEET_PATH')
    fleet_id = os.environ.get('HELLO_FLEET_ID')
    if fleet_path and fleet_id:
        base = Path(fleet_path) / fleet_id
        paths.extend([
            base / 'stretch_user_params.yaml',
            base / 'stretch_configuration_params.yaml',
            base / 'exported_urdf/stretch.urdf',
        ])
    paths.extend([
        Path.home() / 'stretch_user_params.yaml',
        Path.home() / 'stretch_configuration_params.yaml',
    ])

    checked: list[str] = []
    for path in paths:
        checked.append(str(path))
        if not path.is_file():
            continue
        if path.suffix in ('.urdf', '.xml'):
            text = path.read_text(errors='ignore')
            found = _find_tool_in_mapping(text)
        else:
            try:
                found = _find_tool_in_mapping(yaml.safe_load(path.read_text()) or {})
            except Exception:
                found = None
        if found:
            return found, checked
    return None, checked


def resolve_tool_preset(tool_preset: str) -> str:
    validate_tool_preset(tool_preset)
    if tool_preset != 'auto':
        return tool_preset

    failures: list[str] = []

    robot_tool, robot_params_error = _tool_from_robot_params()
    if robot_tool is not None:
        found = _find_tool_in_mapping(robot_tool)
        if found:
            print(f"Resolved tool_preset=auto from RobotParams tool '{robot_tool}' -> {found}")
            return found
        failures.append(
            f"RobotParams tool '{robot_tool}' is not a recognized SG4/PG4/tablet/nil preset"
        )
    elif robot_params_error:
        failures.append(robot_params_error)

    found, checked_paths = _tool_from_fleet_yaml()
    if found:
        print(f"Resolved tool_preset=auto from fleet params -> {found}")
        return found
    failures.append(
        'fleet/user YAML did not contain a recognized tool preset '
        f'(checked: {", ".join(checked_paths)})'
    )

    explicit_presets = ', '.join(preset for preset in TOOL_PRESETS if preset != 'auto')
    raise RuntimeError(
        'tool_preset=auto could not detect the mounted tool. '
        + '; '.join(failures)
        + f'. Pass tool_preset explicitly ({explicit_presets}).'
    )


def _generate_urdf_box_params(tool_preset: str) -> dict[str, Any]:
    resolved = resolve_tool_preset(tool_preset)
    model_name, batch_name, tool_name = _resolve_urdf_identity(tool_preset)
    urdf_content = get_urdf(model_name, batch_name, tool_name)

    requested: dict[str, str] = {}
    for group, links in MAIN_LINK_GROUPS:
        for link in links:
            requested[link] = group

    if resolved != 'nil':
        for link in TOOL_LINK_GROUPS_BY_PRESET.get(resolved, ()):
            requested[link] = 'tool'

    boxes = _read_link_boxes(urdf_content, requested)

    # Avoid sending degenerate boxes to the C++ hot path.
    boxes = [box for box in boxes if min(box['half']) > 1e-5]
    params = {
        'resolved_tool_preset': resolved,
        'publish_raw_urdf_self_filter_markers': False,
        'publish_buffered_self_filter_markers': True,
        'self_filter_box_frames': [box['frame'] for box in boxes],
        'self_filter_box_names': [box['name'] for box in boxes],
        'self_filter_box_groups': [box['group'] for box in boxes],
        'self_filter_box_origin_x': [float(box['origin'][0]) for box in boxes],
        'self_filter_box_origin_y': [float(box['origin'][1]) for box in boxes],
        'self_filter_box_origin_z': [float(box['origin'][2]) for box in boxes],
        'self_filter_box_rpy_roll': [float(box['rpy'][0]) for box in boxes],
        'self_filter_box_rpy_pitch': [float(box['rpy'][1]) for box in boxes],
        'self_filter_box_rpy_yaw': [float(box['rpy'][2]) for box in boxes],
        'self_filter_box_half_extents_x': [float(box['half'][0]) for box in boxes],
        'self_filter_box_half_extents_y': [float(box['half'][1]) for box in boxes],
        'self_filter_box_half_extents_z': [float(box['half'][2]) for box in boxes],
        'self_filter_arm_buffer': FILTER_BUFFER_BY_GROUP['arm'],
        'self_filter_wrist_buffer': FILTER_BUFFER_BY_GROUP['wrist'],
        'self_filter_gripper_cam_buffer': FILTER_BUFFER_BY_GROUP['gripper_camera'],
        'self_filter_tool_buffer': FILTER_BUFFER_BY_GROUP['tool'],
    }
    print(f"Generated {len(boxes)} URDF self-filter boxes for tool preset '{resolved}'.")
    return params


def generated_urdf_self_filter_yaml(tool_preset: str) -> str:
    params = _generate_urdf_box_params(tool_preset)
    with tempfile.NamedTemporaryFile(mode='w', delete=False, suffix='.yaml', dir='/tmp') as tmp_file:
        yaml.safe_dump({'/**': {'ros__parameters': params}}, tmp_file, sort_keys=False)
        return tmp_file.name


def dual_lidar_self_filter_parameters(stretch_core: str, tool_preset: str) -> list:
    """Pipeline + shared policy + generated URDF geometry."""
    return [
        _config_path(stretch_core, DUAL_LIDAR_PIPELINE_YAML),
        robot_self_filter_yaml(stretch_core),
        generated_urdf_self_filter_yaml(tool_preset),
    ]


def dual_lidar_fused_parameters(stretch_core: str, tool_preset: str) -> list:
    """Fused-pipeline params + shared policy + generated URDF geometry."""
    return [
        _config_path(stretch_core, FUSED_LIDAR_PIPELINE_YAML),
        robot_self_filter_yaml(stretch_core),
        generated_urdf_self_filter_yaml(tool_preset),
    ]


def footprint_self_filter_parameters(stretch_core: str, tool_preset: str) -> list:
    """Nav2 footprint publisher wiring + same generated geometry as lidar self-filter."""
    return [
        _config_path(stretch_core, ROBOT_FOOTPRINT_YAML),
        robot_self_filter_yaml(stretch_core),
        generated_urdf_self_filter_yaml(tool_preset),
    ]
