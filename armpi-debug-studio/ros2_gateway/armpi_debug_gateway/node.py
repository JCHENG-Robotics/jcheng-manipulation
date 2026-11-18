"""ROS 2 node that caches battery, camera, and joint-command state.

Read-only by construction: it only creates subscriptions. It never publishes a
motion command, never opens the serial port, and never calls the vendor SDK.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from datetime import datetime, timezone
from typing import Any

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import UInt16

from .imaging import cv2, image_to_bgr, np

try:  # tf2_msgs is optional; /tf reports unavailable if it is missing.
    from tf2_msgs.msg import TFMessage
except ImportError:  # pragma: no cover - depends on the robot image
    TFMessage = None

try:  # cv_bridge is optional; imaging.py provides the numpy fallback.
    from cv_bridge import CvBridge
except ImportError:  # pragma: no cover - depends on the robot image
    CvBridge = None


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class GatewayNode(Node):
    def __init__(
        self,
        *,
        battery_topic: str = "/ros_robot_controller/battery",
        camera_topic: str = "/depth_cam/rgb/image_raw",
        joint_state_topic: str = "/controller_manager/joint_states",
        tf_topic: str = "/tf",
        tf_static_topic: str = "/tf_static",
        jpeg_quality: int = 80,
    ) -> None:
        super().__init__("armpi_debug_gateway")
        self._lock = threading.Lock()
        self._battery_topic = battery_topic
        self._camera_topic = camera_topic
        self._joint_state_topic = joint_state_topic
        self._tf_topic = tf_topic
        self._tf_static_topic = tf_static_topic
        self._jpeg_quality = max(30, min(95, int(jpeg_quality)))

        self._battery_mv: int | None = None
        self._battery_at: float | None = None
        self._battery_wall: str | None = None

        self._frame_jpeg: bytes | None = None
        self._frame_at: float | None = None
        self._frame_wall: str | None = None
        self._frame_width: int | None = None
        self._frame_height: int | None = None
        self._frame_encoding: str | None = None
        self._frame_times: deque[float] = deque(maxlen=30)
        self._frame_count = 0

        self._joints: dict[str, dict[str, Any]] = {}
        self._joints_at: float | None = None
        self._joints_wall: str | None = None

        self._tf_edges: dict[tuple[str, str], dict[str, Any]] = {}
        self._tf_last_at: float | None = None

        self._bridge = CvBridge() if CvBridge is not None else None
        if cv2 is None:
            self.get_logger().error(
                "opencv is not available; camera will report unavailable"
            )
        elif self._bridge is None:
            self.get_logger().warning(
                "cv_bridge is not installed; using the numpy fallback for "
                "bgr8/rgb8/mono8 camera frames"
            )

        self.create_subscription(UInt16, battery_topic, self._on_battery, 10)
        self.create_subscription(Image, camera_topic, self._on_image, qos_profile_sensor_data)
        self.create_subscription(JointState, joint_state_topic, self._on_joint_state, 10)

        if TFMessage is None:
            self.get_logger().warning(
                "tf2_msgs is not installed; /tf will report unavailable"
            )
        else:
            tf_qos = QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            static_qos = QoSProfile(
                depth=100,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            tf_group = ReentrantCallbackGroup()
            self.create_subscription(
                TFMessage,
                tf_topic,
                lambda message: self._on_tf(message, static=False),
                tf_qos,
                callback_group=tf_group,
            )
            self.create_subscription(
                TFMessage,
                tf_static_topic,
                lambda message: self._on_tf(message, static=True),
                static_qos,
                callback_group=tf_group,
            )

        self.get_logger().info(
            f"read-only gateway topics: battery={battery_topic} "
            f"camera={camera_topic} joint_state={joint_state_topic} "
            f"tf={tf_topic} tf_static={tf_static_topic}"
        )

    # ------------------------------------------------------------- callbacks
    def _on_battery(self, message: UInt16) -> None:
        with self._lock:
            self._battery_mv = int(message.data)
            self._battery_at = time.monotonic()
            self._battery_wall = utc_now_iso()

    def _on_joint_state(self, message: JointState) -> None:
        now = time.monotonic()
        joints: dict[str, dict[str, Any]] = {}
        for index, name in enumerate(message.name):
            position = message.position[index] if index < len(message.position) else None
            velocity = message.velocity[index] if index < len(message.velocity) else None
            if position is None:
                continue
            joints[str(name)] = {
                "position_rad": float(position),
                "target_rad": float(position),
                "raw_position": None,
                "velocity_rad_s": float(velocity) if velocity is not None else None,
                "truth": "commanded",
                "validity": "valid",
                "sample_age_ms": 0,
                "source": self._joint_state_topic,
            }
        with self._lock:
            self._joints = joints
            self._joints_at = now
            self._joints_wall = utc_now_iso()

    @staticmethod
    def _stamp_ns(stamp: Any) -> int:
        try:
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        except (AttributeError, TypeError, ValueError):
            return 0

    def _on_tf(self, message: Any, *, static: bool) -> None:
        now = time.monotonic()
        wall = utc_now_iso()
        with self._lock:
            for transform in getattr(message, "transforms", []):
                parent = str(transform.header.frame_id).lstrip("/")
                child = str(transform.child_frame_id).lstrip("/")
                if not parent or not child:
                    continue
                key = (parent, child)
                existing = self._tf_edges.get(key)
                if existing is not None and existing["static"] and not static:
                    pass  # dynamic replaces static for the same edge
                elif existing is not None and static and not existing["static"]:
                    continue  # keep the dynamic edge
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self._tf_edges[key] = {
                    "parent": parent,
                    "child": child,
                    "translation_m": [
                        float(translation.x),
                        float(translation.y),
                        float(translation.z),
                    ],
                    "rotation_xyzw": [
                        float(rotation.x),
                        float(rotation.y),
                        float(rotation.z),
                        float(rotation.w),
                    ],
                    "stamp_ns": self._stamp_ns(transform.header.stamp),
                    "received_at": wall,
                    "received_monotonic": now,
                    "static": static,
                }
            if len(self._tf_edges) > 5000:
                oldest = min(
                    self._tf_edges.items(),
                    key=lambda item: item[1]["received_monotonic"],
                )[0]
                self._tf_edges.pop(oldest, None)
            self._tf_last_at = now

    def _on_image(self, message: Image) -> None:
        if cv2 is None:
            return
        try:
            if self._bridge is not None:
                frame = self._bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
            else:
                frame = image_to_bgr(message)
            ok, encoded = cv2.imencode(
                ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality]
            )
        except Exception as exc:  # noqa: BLE001 - a bad frame must not kill the node
            self.get_logger().warning(f"camera frame conversion failed: {exc}")
            return
        if not ok:
            return
        now = time.monotonic()
        with self._lock:
            self._frame_jpeg = encoded.tobytes()
            self._frame_at = now
            self._frame_wall = utc_now_iso()
            self._frame_width = int(message.width)
            self._frame_height = int(message.height)
            self._frame_encoding = str(message.encoding)
            self._frame_times.append(now)
            self._frame_count += 1

    # --------------------------------------------------------------- helpers
    @staticmethod
    def _age_ms(at: float | None) -> int | None:
        if at is None:
            return None
        return max(0, int((time.monotonic() - at) * 1000.0))

    def _fps_locked(self) -> float:
        if len(self._frame_times) < 2:
            return 0.0
        span = self._frame_times[-1] - self._frame_times[0]
        if span <= 0:
            return 0.0
        return (len(self._frame_times) - 1) / span

    # ---------------------------------------------------------------- payload
    def battery_payload(self) -> dict[str, Any]:
        with self._lock:
            voltage_mv = self._battery_mv
            at = self._battery_at
            wall = self._battery_wall
        if voltage_mv is None:
            return {
                "available": False,
                "voltage_mv": None,
                "voltage_v": None,
                "percent_estimate": None,
                "cell_count": 2,
                "status": "unavailable",
                "truth": "unknown",
                "validity": "unavailable",
                "source": "armpi_debug_gateway",
                "topic": self._battery_topic,
                "received_at": None,
                "sample_age_ms": None,
                "notes": ["No battery message received yet."],
            }

        percent = max(0.0, min(100.0, (voltage_mv - 6400.0) / (8400.0 - 6400.0) * 100.0))
        status = "critical" if percent < 10.0 else "low" if percent < 25.0 else "ok"
        age_ms = self._age_ms(at)
        validity = "valid" if age_ms is not None and age_ms < 5000 else "stale"
        return {
            "available": True,
            "voltage_mv": voltage_mv,
            "voltage_v": round(voltage_mv / 1000.0, 3),
            "percent_estimate": round(percent, 1),
            "cell_count": 2,
            "status": status,
            "truth": "measured",
            "validity": validity,
            "source": "armpi_debug_gateway",
            "topic": self._battery_topic,
            "received_at": wall,
            "sample_age_ms": age_ms,
            "notes": [],
        }

    def camera_status_payload(self) -> dict[str, Any]:
        with self._lock:
            has_frame = self._frame_jpeg is not None
            at = self._frame_at
            wall = self._frame_wall
            width = self._frame_width
            height = self._frame_height
            encoding = self._frame_encoding
            fps = self._fps_locked()
            count = self._frame_count
        age_ms = self._age_ms(at)
        available = has_frame and age_ms is not None and age_ms < 3000
        if available:
            validity = "valid"
            truth = "measured"
        elif has_frame:
            validity = "stale"
            truth = "measured"
        else:
            validity = "unavailable"
            truth = "unknown"
        notes = []
        if cv2 is None:
            notes.append("opencv is not available in this Python environment.")
        elif self._bridge is None:
            notes.append(
                "cv_bridge is not installed; frames were decoded with the numpy fallback."
            )
        return {
            "available": available,
            "truth": truth,
            "validity": validity,
            "source": "armpi_debug_gateway",
            "topic": self._camera_topic,
            "width": width,
            "height": height,
            "encoding": encoding,
            "fps": round(fps, 1),
            "last_frame_at": wall,
            "sample_age_ms": age_ms,
            "frame_count": count,
            "notes": notes,
        }

    def camera_snapshot(self) -> bytes | None:
        with self._lock:
            return self._frame_jpeg

    def state_payload(self) -> dict[str, Any]:
        with self._lock:
            joints = {name: dict(item) for name, item in self._joints.items()}
            at = self._joints_at
            wall = self._joints_wall
        age_ms = self._age_ms(at)
        for item in joints.values():
            item["sample_age_ms"] = age_ms
            item["validity"] = "valid" if age_ms is not None and age_ms < 500 else "stale"
        return {
            "joints": joints,
            "ros_time_ns": int(self.get_clock().now().nanoseconds),
            "wall_time": wall or utc_now_iso(),
            "source": self._joint_state_topic,
            "truth": "commanded",
            "notes": [
                "joint_states is the last commanded pulse, not encoder feedback.",
            ],
        }

    def tf_payload(self) -> dict[str, Any]:
        with self._lock:
            edges = [dict(item) for item in self._tf_edges.values()]
            last_at = self._tf_last_at
        now = time.monotonic()
        for edge in edges:
            edge["age_ms"] = max(
                0,
                int((now - float(edge["received_monotonic"])) * 1000.0),
            )
            edge.pop("received_monotonic", None)
        dynamic = [edge for edge in edges if not edge["static"]]
        static = [edge for edge in edges if edge["static"]]
        dynamic_ages = [int(edge["age_ms"]) for edge in dynamic]
        all_ages = [int(edge["age_ms"]) for edge in edges]
        freshest = min(dynamic_ages) if dynamic_ages else None
        if not edges:
            status = "unavailable"
        elif not dynamic:
            status = "static_only"
        elif freshest is not None and freshest <= 500:
            status = "ok"
        else:
            status = "stale"
        notes = [
            "TF is published by robot nodes. Dynamic transforms may be derived "
            "from commanded joint states, not encoder feedback.",
        ]
        if status == "static_only":
            notes.append("Only /tf_static has been received; dynamic links are not live.")
        elif status == "stale":
            notes.append(f"Newest dynamic TF edge is {freshest} ms old.")
        elif status == "unavailable":
            notes.append("No /tf or /tf_static message has been received.")
        return {
            "available": bool(edges),
            "status": status,
            "truth": "measured_tf" if edges else "unknown",
            "source": "tf2_topics",
            "topics": {"tf": self._tf_topic, "tf_static": self._tf_static_topic},
            "generated_at": utc_now_iso(),
            "last_at_monotonic": last_at,
            "dynamic_edges": len(dynamic),
            "static_edges": len(static),
            "freshest_age_ms": freshest,
            "max_age_ms": max(all_ages) if all_ages else None,
            "transforms": edges,
            "notes": notes,
        }

    def health_payload(self) -> dict[str, Any]:
        battery = self.battery_payload()
        camera = self.camera_status_payload()
        state = self.state_payload()
        tf = self.tf_payload()
        joints_ready = any(
            item.get("validity") == "valid" for item in state["joints"].values()
        )
        return {
            "status": "ok" if rclpy.ok() else "degraded",
            "ros_ready": bool(rclpy.ok()),
            "battery_ready": bool(battery["available"]),
            "camera_ready": bool(camera["available"]),
            "joints_ready": joints_ready,
            "tf_ready": bool(tf["available"]),
            "tf_status": tf["status"],
            "motion_enabled": False,
            "node": self.get_name(),
            "wall_time": utc_now_iso(),
            "topics": {
                "battery": self._battery_topic,
                "camera": self._camera_topic,
                "joint_state": self._joint_state_topic,
                "tf": self._tf_topic,
                "tf_static": self._tf_static_topic,
            },
        }
