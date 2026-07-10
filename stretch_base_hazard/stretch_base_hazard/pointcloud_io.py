"""PointCloud2 helpers."""

from __future__ import annotations

import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def read_xyz_from_pointcloud2(msg) -> np.ndarray:
    """Extract XYZ from a sensor_msgs/PointCloud2 message."""
    from sensor_msgs_py import point_cloud2

    try:
        cloud_np = point_cloud2.read_points_numpy(
            msg, field_names=('x', 'y', 'z'), skip_nans=True,
        )
        if cloud_np.size == 0:
            return np.zeros((0, 3))
        return np.asarray(cloud_np, dtype=np.float64).reshape(-1, 3)
    except Exception:
        pass

    cloud = point_cloud2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True)
    if cloud is None:
        return np.zeros((0, 3))

    cloud = np.asarray(cloud)
    if cloud.dtype.names is not None:
        if cloud.size == 0:
            return np.zeros((0, 3))
        return np.column_stack([
            np.asarray(cloud['x'], dtype=np.float64),
            np.asarray(cloud['y'], dtype=np.float64),
            np.asarray(cloud['z'], dtype=np.float64),
        ])

    cloud = cloud.astype(np.float64).reshape(-1, 3) if cloud.size else cloud
    if cloud.size == 0:
        return np.zeros((0, 3))
    return cloud


def numpy_to_pointcloud2(
    points: np.ndarray,
    header: Header,
) -> PointCloud2:
    """Convert (N, 3) float array to PointCloud2."""
    points = np.atleast_2d(np.asarray(points, dtype=np.float32))
    if points.size == 0:
        points = np.zeros((0, 3), dtype=np.float32)
    else:
        points = np.ascontiguousarray(points.reshape(-1, 3), dtype=np.float32)

    msg = PointCloud2()
    msg.header = header
    msg.height = 1
    msg.width = points.shape[0]
    msg.fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = msg.point_step * msg.width
    msg.is_dense = True
    msg.data = points.tobytes()
    return msg
