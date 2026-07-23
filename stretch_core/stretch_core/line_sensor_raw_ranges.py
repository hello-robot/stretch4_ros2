"""Build per-sensor std_msgs/Float32MultiArray from raw line-sensor range arrays."""

from __future__ import annotations

import numpy as np
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout


def sensor_name_to_frame(sensor_name: str) -> str:
    """Map stretch4_body sensor_N to stretch4_urdf line_sensor_N_link."""
    if not sensor_name.startswith('sensor_'):
        raise ValueError(f'Unexpected sensor name: {sensor_name}')
    index = sensor_name.split('_', 1)[1]
    return f'line_sensor_{index}_link'


def build_raw_ranges_multiarray(ranges: np.ndarray) -> Float32MultiArray:
    """
    Pack device range bins into a Float32MultiArray with no filtering.

    Each element is the slant range in meters for one bin, in device order
    (index 0 .. N-1). Values are copied as-is from the line-sensor status
    (including no-return such as ~5.11 m).
    """
    ranges_f = np.asarray(ranges, dtype=np.float32).reshape(-1)
    n = int(ranges_f.size)

    msg = Float32MultiArray()
    msg.layout = MultiArrayLayout()
    msg.layout.data_offset = 0
    dim = MultiArrayDimension()
    dim.label = 'bin'
    dim.size = n
    dim.stride = n
    msg.layout.dim = [dim]
    msg.data = ranges_f.tolist()
    return msg
