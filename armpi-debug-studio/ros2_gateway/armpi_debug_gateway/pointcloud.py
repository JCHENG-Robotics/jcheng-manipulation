"""PointCloud2 to compact XYZ conversion for the G8 locate endpoint.

Pure NumPy: no ROS import, so the gateway tests can exercise it directly. The
Aurora cloud labels its frame ``depth_camera_link`` but the layout is the
OpenCV optical frame; the backend applies the validated ``T_link4_camera``
hand-eye, exactly as ``tools/phase2/object_segment_rgbd.py`` does.
"""

from __future__ import annotations

from typing import Any

import numpy as np

FLOAT32 = 7
FLOAT64 = 8


def xyz_bytes(
    data: Any,
    fields: dict[str, dict[str, int]],
    point_step: int,
) -> bytes:
    """Return the cloud as little-endian float32 ``x y z`` triples."""
    # rclpy returns ``uint8[]`` as array.array('B'), not bytes.
    if isinstance(data, (bytes, bytearray, memoryview)):
        raw_bytes = bytes(data)
    else:
        try:
            raw_bytes = bytes(memoryview(data))
        except TypeError as exc:
            raise ValueError("point cloud data is not a byte buffer") from exc
    if point_step <= 0:
        raise ValueError("point cloud point_step must be positive")
    for name in ("x", "y", "z"):
        item = fields.get(name)
        if not isinstance(item, dict) or item.get("datatype") not in (
            FLOAT32,
            FLOAT64,
        ):
            raise ValueError(f"point cloud has no usable {name} field")
    raw = np.frombuffer(raw_bytes, dtype=np.uint8)
    count = len(raw) // point_step
    if count == 0:
        raise ValueError("point cloud is empty")
    rows = raw[: count * point_step].reshape(count, point_step)
    columns = []
    for name in ("x", "y", "z"):
        item = fields[name]
        offset = int(item["offset"])
        dtype = np.dtype("<f4" if item["datatype"] == FLOAT32 else "<f8")
        size = dtype.itemsize
        if offset + size > point_step:
            raise ValueError(f"point cloud {name} field exceeds point_step")
        columns.append(
            rows[:, offset : offset + size]
            .copy()
            .view(dtype)
            .ravel()
            .astype(np.float32)
        )
    return np.column_stack(columns).astype("<f4").tobytes()
