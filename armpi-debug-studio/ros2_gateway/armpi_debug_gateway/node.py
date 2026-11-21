"""ROS 2 node that caches battery, camera, joint and measured servo state.

Read-only by default: it only creates subscriptions, never opens the serial
port and never calls the vendor SDK. When motion is explicitly enabled
(``--enable-motion`` plus a validated limits file), it creates exactly one
publisher for the vendor ``servo_controller`` topic. Every command is checked
by :class:`armpi_debug_gateway.motion.MotionGuard` first and completion is
derived from measured ``servo_states`` pulses, not from the command echo.
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

from .aux_camera import UvcCamera
from .imaging import cv2, image_to_bgr, np
from .motion import (
    GRIPPER_INIT_PULSE,
    GRIPPER_JOINT,
    MOTION_SERVO_IDS,
    SERVO_IDS,
    MotionConfig,
    MotionGuard,
    MotionRejected,
)
from .pointcloud_cache import PointCloudRunner
from .trajectory_capture import RADIANS_PER_TICK, pulse_to_rad

try:  # motion messages are only present in the vendor image
    from servo_controller_msgs.msg import ServoPosition, ServosPosition, ServoStateList
except ImportError:  # pragma: no cover - depends on the robot image
    ServoPosition = None
    ServosPosition = None
    ServoStateList = None

try:
    # The real bus-servo registers. ``/controller_manager/servo_states`` is the
    # vendor command cache; this service is the only actual read.
    from ros_robot_controller_msgs.msg import GetBusServoCmd
    from ros_robot_controller_msgs.srv import GetBusServoState
except ImportError:  # pragma: no cover - depends on the robot image
    GetBusServoCmd = None
    GetBusServoState = None

BUS_SERVO_SERVICE = "/ros_robot_controller/bus_servo/get_state"

SERVO_JOINTS = {
    servo_id: joint for joint, servo_id in MOTION_SERVO_IDS.items()
}
MOTION_TOPIC = "servo_controller"

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
        servo_state_topic: str = "/controller_manager/servo_states",
        pointcloud_topic: str = "",
        tf_topic: str = "/tf",
        tf_static_topic: str = "/tf_static",
        jpeg_quality: int = 80,
        aux_camera: UvcCamera | None = None,
        motion_enabled: bool = False,
        motion_config: MotionConfig | None = None,
    ) -> None:
        super().__init__("armpi_debug_gateway")
        self._lock = threading.Lock()
        self._battery_topic = battery_topic
        self._camera_topic = camera_topic
        self._aux_camera = aux_camera
        self._joint_state_topic = joint_state_topic
        self._servo_state_topic = servo_state_topic
        self._pointcloud_topic = pointcloud_topic
        self._tf_topic = tf_topic
        self._tf_static_topic = tf_static_topic
        self._jpeg_quality = max(30, min(95, int(jpeg_quality)))
        self._cloud_runner: PointCloudRunner | None = None

        self._motion_config = motion_config if motion_enabled else None
        self._motion_pub = None
        self._guard = MotionGuard(motion_config) if self._motion_config else None
        self._bus_client = None
        if motion_enabled:
            if ServosPosition is None or ServoPosition is None:
                raise RuntimeError(
                    "servo_controller_msgs is not available; cannot enable motion"
                )
            if self._motion_config is None:
                raise RuntimeError("motion enabled without a validated limits config")
            self._motion_pub = self.create_publisher(
                ServosPosition, MOTION_TOPIC, 1
            )
            if self._motion_config.safety.verification:
                if GetBusServoState is None or GetBusServoCmd is None:
                    raise RuntimeError(
                        "bus verification requires ros_robot_controller_msgs"
                    )
                self._bus_client = self.create_client(
                    GetBusServoState, BUS_SERVO_SERVICE
                )
                if not self._bus_client.wait_for_service(timeout_sec=5.0):
                    raise RuntimeError(
                        "bus verification requires " + BUS_SERVO_SERVICE +
                        "; start the gateway with the vendor CycloneDDS environment"
                    )

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

        self._measured: dict[str, dict[str, Any]] = {}
        self._measured_at: float | None = None
        self._measured_wall: str | None = None

        self._actual: dict[str, dict[str, Any]] = {}
        self._actual_at: float | None = None
        self._actual_wall: str | None = None
        self._verification_timer: threading.Timer | None = None

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
        # Single-owner mode: the aux camera may follow a ROS topic instead of
        # opening the UVC device itself (the robot-side ext_cam_node owns it).
        aux_topic = getattr(self._aux_camera, "ros_topic", None) if self._aux_camera else None
        if aux_topic:
            self.create_subscription(
                Image, aux_topic, self._on_aux_image, qos_profile_sensor_data
            )
        if ServoStateList is None:
            self.get_logger().warning(
                "servo_controller_msgs is not installed; measured servo state "
                "will be unavailable and motion cannot be enabled"
            )
        else:
            self.create_subscription(
                ServoStateList,
                servo_state_topic,
                self._on_servo_states,
                50,
            )

        if pointcloud_topic:
            # A subprocess per locate: a 2.5 MB cloud decoded in-process would
            # compete with the 50 Hz servo callbacks and the HTTP handlers.
            self._cloud_runner = PointCloudRunner(pointcloud_topic)
            self.get_logger().info(
                f"point cloud: {pointcloud_topic} (G8 locate, on-demand subprocess)"
            )

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

        if self._motion_pub is not None:
            self.get_logger().warning(
                f"MOTION ENABLED ({self._motion_config.safety.mode}): "
                f"publisher={MOTION_TOPIC} "
                f"measured={servo_state_topic} (command cache) "
                f"verification={self._motion_config.safety.verification} "
                f"bus={BUS_SERVO_SERVICE if self._bus_client else 'off'} "
                f"step<={self._motion_config.safety.max_step_rad} rad "
                f"velocity<={self._motion_config.safety.max_velocity_rad_s} rad/s "
                f"single_joint={self._motion_config.safety.single_joint_only}"
            )
        else:
            self.get_logger().info(
                f"read-only gateway topics: battery={battery_topic} "
                f"camera={camera_topic} joint_state={joint_state_topic} "
                f"measured={servo_state_topic} "
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

    def _servo_pulse(self, state: Any) -> tuple[int | None, int | None]:
        """Return ``(servo_id, pulse)`` from one message item.

        The vendor message declares int32 scalars; hand-written fixtures and
        some recordings use one-element lists, so both are accepted.
        """
        servo_id = getattr(state, "id", None)
        raw = getattr(state, "position", None)
        if isinstance(servo_id, (list, tuple)):
            servo_id = servo_id[0] if servo_id else None
        if isinstance(raw, (list, tuple)):
            raw = raw[0] if raw else None
        try:
            return int(servo_id), int(raw)
        except (TypeError, ValueError):
            return None, None

    def _pulse_to_radians(self, joint: str, pulse: int) -> float:
        if joint == GRIPPER_JOINT:
            if self._motion_config is not None and self._motion_config.gripper:
                return self._motion_config.gripper.pulse_to_rad(pulse)
            return (int(pulse) - GRIPPER_INIT_PULSE) * RADIANS_PER_TICK
        if self._motion_config is not None:
            return self._motion_config.joints[joint].pulse_to_rad(pulse)
        return pulse_to_rad(joint, pulse)

    def _on_servo_states(self, message: Any) -> None:
        now = time.monotonic()
        measured: dict[str, dict[str, Any]] = {}
        for state in getattr(message, "servo_state", []):
            servo_id, pulse = self._servo_pulse(state)
            if servo_id is None or pulse is None:
                continue
            joint = SERVO_JOINTS.get(servo_id)
            if joint is None:
                continue
            if not 0 <= pulse <= 1000:
                # An out-of-range sample preceded the 2026-11-18 vendor node
                # crash; do not convert it. Motion stays blocked by staleness.
                continue
            measured[joint] = {
                "position_rad": self._pulse_to_radians(joint, pulse),
                "raw_position": pulse,
                # The vendor controller_manager publishes the cache written by
                # ServoManager.set_position, not a bus read.
                "truth": "commanded",
                "validity": "valid",
                "sample_age_ms": 0,
                "source": self._servo_state_topic + " (command cache)",
            }
        if not measured:
            return
        with self._lock:
            self._measured = measured
            self._measured_at = now
            self._measured_wall = utc_now_iso()
            guard = self._guard
        if guard is not None:
            guard.observe(self._measured_with_age(now), now)

    def _measured_with_age(self, now: float | None = None) -> dict[str, dict[str, Any]]:
        now = time.monotonic() if now is None else now
        with self._lock:
            at = self._measured_at
            wall = self._measured_wall
            measured = {name: dict(item) for name, item in self._measured.items()}
        age_ms = (
            max(0.0, (now - at) * 1000.0) if at is not None else None
        )
        for item in measured.values():
            item["sample_age_ms"] = (
                int(round(age_ms)) if age_ms is not None else None
            )
            item["received_at"] = wall
        return measured

    def _actual_with_age(self, now: float | None = None) -> dict[str, dict[str, Any]]:
        now = time.monotonic() if now is None else now
        with self._lock:
            at = self._actual_at
            wall = self._actual_wall
            actual = {name: dict(item) for name, item in self._actual.items()}
        age_ms = max(0.0, (now - at) * 1000.0) if at is not None else None
        for item in actual.values():
            item["sample_age_ms"] = (
                int(round(age_ms)) if age_ms is not None else None
            )
            item["received_at"] = wall
        return actual

    def _measured_ready(self) -> bool:
        now = time.monotonic()
        measured = self._measured_with_age(now)
        if not measured or not set(SERVO_IDS).issubset(measured):
            return False
        age_ms = max(
            (item.get("sample_age_ms") or 0) for item in measured.values()
        )
        limit = (
            self._motion_config.safety.feedback_max_age_ms
            if self._motion_config is not None
            else 500
        )
        return age_ms <= limit

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

    def pointcloud_meta_payload(self) -> dict[str, Any]:
        """Capture one fresh cloud and return its metadata (G8 locate)."""
        runner = self._cloud_runner
        if runner is None:
            return {
                "available": False,
                "topic": self._pointcloud_topic or None,
                "frame_id": None,
                "count": 0,
                "sample_age_ms": None,
                "wall_time": None,
            }
        try:
            return runner.meta()
        except RuntimeError as exc:
            return {
                "available": False,
                "topic": self._pointcloud_topic,
                "frame_id": None,
                "count": 0,
                "sample_age_ms": None,
                "wall_time": None,
                "error": str(exc),
            }

    def pointcloud_status_payload(self) -> dict[str, Any]:
        runner = self._cloud_runner
        if runner is None:
            return {"available": False, "topic": None, "count": 0}
        return runner.status()

    def pointcloud_payload(self) -> tuple[bytes, str]:
        """Return the cloud captured by the preceding metadata request."""
        runner = self._cloud_runner
        if runner is None:
            raise RuntimeError("point cloud endpoint is disabled")
        return runner.payload()

    def close(self) -> None:
        """Release the point-cloud runner (the subprocess exits per capture)."""
        self._cloud_runner = None

    def _on_aux_image(self, message: Image) -> None:
        camera = self._aux_camera
        offer = getattr(camera, "offer", None)
        if callable(offer):
            offer(message)

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

    def aux_camera_status_payload(self) -> dict[str, Any]:
        """UVC color stream of the off-arm D435i (not a ROS topic)."""
        if self._aux_camera is None:
            return {
                "available": False,
                "truth": "unknown",
                "validity": "unavailable",
                "source": "armpi_debug_gateway_uvc",
                "topic": "uvc:disabled",
                "device": None,
                "width": None,
                "height": None,
                "encoding": None,
                "fps": 0.0,
                "last_frame_at": None,
                "sample_age_ms": None,
                "frame_count": 0,
                "notes": ["aux camera is disabled (--no-aux-camera)."],
            }
        return self._aux_camera.status_payload()

    def aux_camera_snapshot(self) -> bytes | None:
        if self._aux_camera is None:
            return None
        return self._aux_camera.snapshot()

    def state_payload(self) -> dict[str, Any]:
        with self._lock:
            joints = {name: dict(item) for name, item in self._joints.items()}
            at = self._joints_at
            wall = self._joints_wall
        age_ms = self._age_ms(at)
        for item in joints.values():
            item["sample_age_ms"] = age_ms
            item["validity"] = "valid" if age_ms is not None and age_ms < 500 else "stale"
        measured = self._measured_with_age()
        measured_ready = self._measured_ready()
        for item in measured.values():
            if not measured_ready:
                item["validity"] = "stale"
        actual = self._actual_with_age()
        with self._lock:
            measured_wall = self._measured_wall
            actual_wall = self._actual_wall
        return {
            "joints": joints,
            "measured": measured,
            "measured_ready": measured_ready,
            "measured_wall_time": measured_wall,
            "measured_truth": "commanded",
            "actual": actual,
            "actual_wall_time": actual_wall,
            "ros_time_ns": int(self.get_clock().now().nanoseconds),
            "wall_time": wall or utc_now_iso(),
            "source": self._joint_state_topic,
            "measured_source": self._servo_state_topic,
            "actual_source": BUS_SERVO_SERVICE,
            "truth": "commanded",
            "notes": [
                "joints is the last commanded pulse (command echo).",
                "measured is the vendor command cache, not a bus read.",
                "actual is the last GetBusServoState read (measured_on_demand).",
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
        static_ages = [int(edge["age_ms"]) for edge in static]
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
            # Static edges are latched and legitimately old after startup;
            # report dynamic and static maxima separately. ``max_age_ms`` is
            # kept for compatibility and is the all-edge maximum.
            "max_age_ms": max(all_ages) if all_ages else None,
            "max_dynamic_age_ms": max(dynamic_ages) if dynamic_ages else None,
            "max_static_age_ms": max(static_ages) if static_ages else None,
            "transforms": edges,
            "notes": notes,
        }

    def health_payload(self) -> dict[str, Any]:
        battery = self.battery_payload()
        camera = self.camera_status_payload()
        aux_camera = self.aux_camera_status_payload()
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
            "aux_camera_ready": bool(aux_camera["available"]),
            "joints_ready": joints_ready,
            "measured_ready": bool(state.get("measured_ready")),
            "tf_ready": bool(tf["available"]),
            "tf_status": tf["status"],
            "pointcloud_configured": self._cloud_runner is not None,
            "pointcloud_topic": self._pointcloud_topic or None,
            "pointcloud_last_capture": self.pointcloud_status_payload(),
            "motion_enabled": self.motion_enabled,
            "motion_state": self._motion_state(),
            "motion_mode": (
                self._motion_config.safety.mode
                if self.motion_enabled and self._motion_config is not None
                else "disabled"
            ),
            "node": self.get_name(),
            "wall_time": utc_now_iso(),
            "topics": {
                "battery": self._battery_topic,
                "camera": self._camera_topic,
                "aux_camera": aux_camera["topic"],
                "joint_state": self._joint_state_topic,
                "measured": self._servo_state_topic,
                "pointcloud": self._pointcloud_topic or None,
                "motion": MOTION_TOPIC if self.motion_enabled else None,
                "tf": self._tf_topic,
                "tf_static": self._tf_static_topic,
            },
        }

    # ------------------------------------------------------------------ motion
    @property
    def motion_enabled(self) -> bool:
        return self._motion_pub is not None and self._guard is not None

    def _motion_state(self) -> str:
        if self._guard is None:
            return "disabled"
        return str(self._guard.status().get("state") or "unknown")

    def _publish_positions(
        self,
        duration_s: float,
        positions: list[tuple[str, int]],
    ) -> None:
        if self._motion_pub is None or ServosPosition is None or ServoPosition is None:
            raise MotionRejected(
                "MOTION_DISABLED",
                "motion publisher is not configured",
                http_status=403,
            )
        message = ServosPosition()
        message.duration = float(duration_s)
        message.position_unit = "pulse"
        for joint, pulse in positions:
            item = ServoPosition()
            item.id = int(MOTION_SERVO_IDS[joint])
            item.position = float(int(pulse))
            message.position.append(item)
        self._motion_pub.publish(message)

    def dispatch_motion(self, payload: dict[str, Any]) -> dict[str, Any]:
        if not self.motion_enabled:
            raise MotionRejected(
                "MOTION_DISABLED",
                "hardware motion is disabled on this gateway instance",
                http_status=403,
            )
        assert self._guard is not None
        now = time.monotonic()
        measured = self._measured_with_age(now)
        plan = self._guard.validate(payload, measured, now)
        self._publish_positions(
            plan.duration_s,
            [(target.joint, target.target_pulse) for target in plan.targets],
        )
        self._guard.start(plan, measured)
        if self._motion_config is not None and self._motion_config.safety.verification:
            settle = self._motion_config.safety.verification_settle_s
            if any(target.completion == "timed" for target in plan.targets):
                # The linkage relaxes after the commanded close; wait longer
                # so the recorded register is the settled one, not the
                # transient at the end of the interpolation.
                settle = max(settle, 1.2)
            delay = plan.duration_s + settle
            if self._verification_timer is not None:
                self._verification_timer.cancel()
            timer = threading.Timer(delay, self._verify_bus, args=(plan,))
            timer.daemon = True
            timer.start()
            self._verification_timer = timer
        return {
            "status": "dispatched",
            "command_id": plan.command_id,
            "mode": plan.mode,
            "duration_s": plan.duration_s,
            "position_unit": "pulse",
            "topic": MOTION_TOPIC,
            "verification": (
                "pending"
                if self._motion_config is not None
                and self._motion_config.safety.verification
                else "cache"
            ),
            "joints": [
                {
                    "joint": target.joint,
                    "servo_id": target.servo_id,
                    "target_pulse": target.target_pulse,
                    "target_rad": target.target_rad,
                    "completion": target.completion,
                }
                for target in plan.targets
            ],
            "measured": measured,
            "published_at": utc_now_iso(),
        }

    # ------------------------------------------------------- bus verification
    def _verify_bus(self, plan: Any) -> None:
        """Read the actual registers after the interpolation and settle.

        The service is the only real position source. It is read once per
        command; a target whose last commanded pulse is below the configured
        margin is not read (a negative register crashes the vendor node), and
        the command is marked unverified instead.
        """
        guard = self._guard
        config = self._motion_config
        if guard is None or config is None or self._bus_client is None:
            if guard is not None:
                guard.mark_verified(
                    {},
                    time.monotonic(),
                    read_ok=False,
                    reason="bus verification is not configured",
                )
            return
        margin = config.safety.verification_margin_pulses
        with self._lock:
            measured = {
                joint: item.get("raw_position")
                for joint, item in self._measured.items()
            }
        unsafe = []
        request = GetBusServoState.Request()
        requested: dict[int, str] = {}
        for target in plan.targets:
            last = measured.get(target.joint)
            low = min(
                int(target.target_pulse),
                int(last) if last is not None else int(target.target_pulse),
            )
            if low < margin:
                unsafe.append(target.joint)
                continue
            command = GetBusServoCmd()
            command.id = int(target.servo_id)
            command.get_position = 1
            request.cmd.append(command)
            requested[int(target.servo_id)] = target.joint
        if unsafe or not requested:
            guard.mark_verified(
                {},
                time.monotonic(),
                read_ok=False,
                reason=(
                    "bus read skipped to protect the vendor node; pulse below "
                    f"the {margin}-count margin for " + ", ".join(unsafe)
                    if unsafe
                    else "no safe target to read"
                ),
            )
            return
        try:
            if not self._bus_client.wait_for_service(timeout_sec=1.0):
                raise RuntimeError("bus servo service unavailable")
            future = self._bus_client.call_async(request)
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            if not future.done():
                raise RuntimeError("bus servo service timed out")
            response = future.result()
            if response is None or not getattr(response, "success", False):
                raise RuntimeError("bus servo read failed")
            actual: dict[str, dict[str, Any]] = {}
            for command, state in zip(request.cmd, response.state):
                positions = getattr(state, "position", None)
                if not positions:
                    continue
                # rclpy returns uint16[] as array.array('H'); be explicit
                # instead of handing the whole array to int().
                if isinstance(positions, (bytes, bytearray, memoryview)):
                    raw = bytes(positions)
                    pulse = (
                        int.from_bytes(raw, "little") if len(raw) == 2 else raw[0]
                    )
                else:
                    pulse = int(positions[0])
                joint = requested.get(int(command.id))
                if joint is None:
                    continue
                actual[joint] = {
                    "position_rad": self._pulse_to_radians(joint, pulse),
                    "raw_position": pulse,
                    "truth": "measured_on_demand",
                    "validity": "valid",
                    "sample_age_ms": 0,
                    "source": BUS_SERVO_SERVICE,
                }
            with self._lock:
                self._actual = actual
                self._actual_at = time.monotonic()
                self._actual_wall = utc_now_iso()
            guard.mark_verified(
                actual,
                time.monotonic(),
                read_ok=True,
            )
        except Exception as error:  # noqa: BLE001 - report, never crash the node
            guard.mark_verified(
                {},
                time.monotonic(),
                read_ok=False,
                reason=f"bus read failed: {error}",
            )

    def stop_motion(self, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        if not self.motion_enabled:
            raise MotionRejected(
                "MOTION_DISABLED",
                "hardware motion is disabled on this gateway instance",
                http_status=403,
            )
        assert self._guard is not None
        now = time.monotonic()
        measured = self._measured_with_age(now)
        fresh_limit = self._motion_config.safety.feedback_max_age_ms
        positions = [
            (joint, int(item["raw_position"]))
            for joint, item in measured.items()
            if item.get("raw_position") is not None
            and item.get("sample_age_ms") is not None
            and int(item["sample_age_ms"]) <= fresh_limit
        ]
        if not positions:
            raise MotionRejected(
                "MOTION_STALE_FEEDBACK",
                "no fresh measured servo positions; cannot build a hold command",
                http_status=409,
            )
        # 0.1 s is above the vendor's 0.02 s floor and short enough to act as
        # a hold. This is not a brake and not an emergency stop.
        self._publish_positions(0.1, positions)
        status = self._guard.mark_stopped(measured, now)
        return {
            "status": "stop_dispatched",
            "command_id": (payload or {}).get("command_id"),
            "state": status.get("state"),
            "held_at": status.get("measured"),
            "evidence": (
                "hold command published at measured pulses; physical stop is "
                "not confirmed"
            ),
            "measured": measured,
            "dispatched_at": utc_now_iso(),
        }

    def motion_status_payload(self) -> dict[str, Any]:
        now = time.monotonic()
        measured = self._measured_with_age(now)
        payload: dict[str, Any] = {
            "motion_enabled": self.motion_enabled,
            "measured_ready": self._measured_ready(),
            "measured": measured,
            "measured_source": self._servo_state_topic,
            "motion_topic": MOTION_TOPIC if self.motion_enabled else None,
            "truth": "measured",
            "wall_time": utc_now_iso(),
        }
        if self._guard is None or self._motion_config is None:
            payload.update(
                {
                    "state": "disabled",
                    "command_id": None,
                    "targets": [],
                    "duration_s": None,
                    "last_stop": None,
                }
            )
            return payload
        payload.update(self._guard.status())
        payload["safety"] = {
            "mode": self._motion_config.safety.mode,
            "max_step_rad": self._motion_config.safety.max_step_rad,
            "max_velocity_rad_s": self._motion_config.safety.max_velocity_rad_s,
            "min_duration_s": self._motion_config.safety.min_duration_s,
            "max_duration_s": self._motion_config.safety.max_duration_s,
            "feedback_max_age_ms": self._motion_config.safety.feedback_max_age_ms,
            "tolerance_pulses": self._motion_config.safety.tolerance_pulses,
            "settle_samples": self._motion_config.safety.settle_samples,
            "single_joint_only": self._motion_config.safety.single_joint_only,
            "pose_requires_all_arm_joints": (
                self._motion_config.safety.pose_requires_all_arm_joints
            ),
            "verification": self._motion_config.safety.verification,
            "verification_tolerance_pulses": (
                self._motion_config.safety.verification_tolerance_pulses
            ),
            "verification_settle_s": (
                self._motion_config.safety.verification_settle_s
            ),
            "verification_margin_pulses": (
                self._motion_config.safety.verification_margin_pulses
            ),
        }
        payload["limits"] = {
            joint: {
                "servo_id": limit.servo_id,
                "min_pulse": limit.min_pulse,
                "max_pulse": limit.max_pulse,
                "min_rad": limit.min_rad,
                "max_rad": limit.max_rad,
            }
            for joint, limit in self._motion_config.joints.items()
        }
        return payload
