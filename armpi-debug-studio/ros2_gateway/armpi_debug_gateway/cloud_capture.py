"""One-shot registered PointCloud2 capture (runs as a subprocess).

Invoked by ``armpi_debug_gateway.pointcloud_cache`` from the gateway's
``/pointcloud/meta`` handler:

    python3 -m armpi_debug_gateway.cloud_capture \
        --topic /depth_cam/depth_registered/points \
        --out /tmp/cloud_xyz.bin --meta-out /tmp/cloud_meta.json --timeout 8

It creates its own rclpy node and single-message subscription, waits for one
cloud, converts the x/y/z fields to little-endian float32 triples with
``pointcloud.xyz_bytes`` and exits. Running in a separate process keeps the
multi-megabyte deserialization out of the gateway's motion/HTTP process.
"""

from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2

from .pointcloud import FLOAT32, FLOAT64, xyz_bytes


def _field_map(message: PointCloud2) -> dict[str, dict[str, int]]:
    return {
        str(field.name): {
            "offset": int(field.offset),
            "datatype": int(field.datatype),
        }
        for field in message.fields
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/depth_cam/depth_registered/points")
    parser.add_argument("--out", required=True)
    parser.add_argument("--meta-out", required=True)
    parser.add_argument("--timeout", type=float, default=8.0)
    args = parser.parse_args(argv)

    state: dict[str, object] = {}
    rclpy.init()
    node = Node("armpi_debug_gateway_cloud_capture")
    qos = QoSProfile(
        depth=1,
        history=HistoryPolicy.KEEP_LAST,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
    )

    def callback(message: PointCloud2) -> None:
        if state.get("done"):
            return
        try:
            fields = _field_map(message)
            if not all(
                name in fields
                and fields[name]["datatype"] in (FLOAT32, FLOAT64)
                for name in ("x", "y", "z")
            ):
                return
            body = xyz_bytes(
                message.data, fields, int(message.point_step)
            )
        except (ValueError, TypeError) as exc:  # pragma: no cover - ROS types
            state["error"] = str(exc)
            return
        state["done"] = True
        state["body"] = body
        state["meta"] = {
            "available": True,
            "frame_id": str(message.header.frame_id),
            "width": int(message.width),
            "height": int(message.height),
            "point_step": int(message.point_step),
            "count": len(body) // 12,
            "is_bigendian": bool(message.is_bigendian),
            "stamp_ns": int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec),
            "fields": fields,
            "layout": "opencv_optical_xyz",
            "captured_at": time.time(),
        }

    node.create_subscription(PointCloud2, args.topic, callback, qos)
    deadline = time.monotonic() + max(1.0, float(args.timeout))
    try:
        while not state.get("done") and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
        if not state.get("done"):
            message = state.get("error") or (
                f"no cloud received on {args.topic} within {args.timeout:.1f} s"
            )
            print(f"cloud capture failed: {message}", file=sys.stderr)
            return 1
        with open(args.out, "wb") as handle:
            handle.write(state["body"])  # type: ignore[arg-type]
        with open(args.meta_out, "w", encoding="utf-8") as handle:
            json.dump(state["meta"], handle, indent=2)
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
