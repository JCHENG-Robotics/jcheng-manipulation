"""Thread-safe telemetry cache for the gateway node.

Owns the battery, commanded joint, measured servo, on-demand bus and TF state.
The node keeps the ROS subscriptions and the motion guard; this module has no
ROS imports and is unit-testable with message-like objects.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable


class TelemetryCache:
    """Last samples for battery, joints, servos, actual bus reads and TF."""

    def __init__(
        self,
        *,
        joint_state_topic: str,
        servo_state_topic: str,
        servo_joints: dict[int, str],
        pulse_to_radians: Callable[[str, int], float],
        feedback_max_age_ms: int,
        required_joints: tuple[str, ...],
        now_iso: Callable[[], str],
        on_measured: Callable[[dict[str, Any], float], None] | None = None,
    ) -> None:
        self._joint_state_topic = joint_state_topic
        self._servo_state_topic = servo_state_topic
        self._servo_joints = dict(servo_joints)
        self._pulse_to_radians = pulse_to_radians
        self._feedback_max_age_ms = int(feedback_max_age_ms)
        self._required_joints = tuple(required_joints)
        self._now_iso = now_iso
        self._on_measured = on_measured
        self._lock = threading.Lock()

        self._battery_mv: int | None = None
        self._battery_at: float | None = None
        self._battery_wall: str | None = None

        self._joints: dict[str, dict[str, Any]] = {}
        self._joints_at: float | None = None
        self._joints_wall: str | None = None

        self._measured: dict[str, dict[str, Any]] = {}
        self._measured_at: float | None = None
        self._measured_wall: str | None = None

        self._actual: dict[str, dict[str, Any]] = {}
        self._actual_at: float | None = None
        self._actual_wall: str | None = None

        self._tf_edges: dict[tuple[str, str], dict[str, Any]] = {}
        self._tf_last_at: float | None = None

    def on_battery(self, message: Any) -> None:
        with self._lock:
            self._battery_mv = int(message.data)
            self._battery_at = time.monotonic()
            self._battery_wall = self._now_iso()

    def on_joint_state(self, message: Any) -> None:
        now = time.monotonic()
        joints: dict[str, dict[str, Any]] = {}
        for index, name in enumerate(message.name):
            position = (
                message.position[index]
                if index < len(message.position)
                else None
            )
            velocity = (
                message.velocity[index]
                if index < len(message.velocity)
                else None
            )
            if position is None:
                continue
            joints[str(name)] = {
                "position_rad": float(position),
                "target_rad": float(position),
                "raw_position": None,
                "velocity_rad_s": (
                    float(velocity) if velocity is not None else None
                ),
                "truth": "commanded",
                "validity": "valid",
                "sample_age_ms": 0,
                "source": self._joint_state_topic,
            }
        with self._lock:
            self._joints = joints
            self._joints_at = now
            self._joints_wall = self._now_iso()

    @staticmethod
    def servo_pulse(state: Any) -> tuple[int | None, int | None]:
        """Return ``(servo_id, pulse)`` from one vendor message item."""
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

    def on_servo_states(self, message: Any) -> None:
        now = time.monotonic()
        measured: dict[str, dict[str, Any]] = {}
        for state in getattr(message, "servo_state", []):
            servo_id, pulse = self.servo_pulse(state)
            if servo_id is None or pulse is None:
                continue
            joint = self._servo_joints.get(servo_id)
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
            self._measured_wall = self._now_iso()
        if self._on_measured is not None:
            self._on_measured(self.measured_with_age(now), now)

    def measured_with_age(
        self, now: float | None = None
    ) -> dict[str, dict[str, Any]]:
        now = time.monotonic() if now is None else now
        with self._lock:
            at = self._measured_at
            wall = self._measured_wall
            measured = {
                name: dict(item) for name, item in self._measured.items()
            }
        age_ms = (
            max(0.0, (now - at) * 1000.0) if at is not None else None
        )
        for item in measured.values():
            item["sample_age_ms"] = (
                int(round(age_ms)) if age_ms is not None else None
            )
            item["received_at"] = wall
        return measured

    def actual_with_age(
        self, now: float | None = None
    ) -> dict[str, dict[str, Any]]:
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

    def measured_ready(self) -> bool:
        now = time.monotonic()
        measured = self.measured_with_age(now)
        if not measured or not set(self._required_joints).issubset(measured):
            return False
        age_ms = max(
            (item.get("sample_age_ms") or 0) for item in measured.values()
        )
        return age_ms <= self._feedback_max_age_ms

    def measured_raw(self) -> dict[str, int | None]:
        with self._lock:
            return {
                joint: item.get("raw_position")
                for joint, item in self._measured.items()
            }

    def record_actual(self, actual: dict[str, dict[str, Any]]) -> None:
        with self._lock:
            self._actual = actual
            self._actual_at = time.monotonic()
            self._actual_wall = self._now_iso()

    @staticmethod
    def stamp_ns(stamp: Any) -> int:
        try:
            return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        except (AttributeError, TypeError, ValueError):
            return 0

    def on_tf(self, message: Any, *, static: bool) -> None:
        now = time.monotonic()
        wall = self._now_iso()
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
                    "stamp_ns": self.stamp_ns(transform.header.stamp),
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

    def battery(self) -> tuple[int | None, float | None, str | None]:
        with self._lock:
            return self._battery_mv, self._battery_at, self._battery_wall

    def joints(
        self,
    ) -> tuple[dict[str, dict[str, Any]], float | None, str | None]:
        with self._lock:
            joints = {
                name: dict(item) for name, item in self._joints.items()
            }
            return joints, self._joints_at, self._joints_wall

    def walls(self) -> tuple[str | None, str | None]:
        with self._lock:
            return self._measured_wall, self._actual_wall

    def tf_edges(
        self,
    ) -> tuple[list[dict[str, Any]], float | None]:
        with self._lock:
            return (
                [dict(item) for item in self._tf_edges.values()],
                self._tf_last_at,
            )
