"""Pure-Python trajectory capture buffer and G2 dataset assembly.

This module is used by the read-only Studio recorder. It has no ROS import so
it can be unit-tested on a laptop, and it never publishes a command or calls a
service.

The recorder captures two live streams:

* ``commanded`` - the vendor controller's joint-state command echo;
* ``feedback``  - bus-servo positions published by ``servo_states``,
  converted from pulses to radians with the vendor mapping.

The ``planned`` stream is supplied as a JSON file produced by the MoveIt
plan/execute tool. Assembly writes a dataset in the G2 Trajectory Inspector
contract (``schema_version: 1``). Importing that dataset into the Studio is a
separate, explicit step.
"""

from __future__ import annotations

import json
import math
from datetime import datetime, timezone
from typing import Any

CAPTURE_SCHEMA_VERSION = 1
CAPTURE_KIND = "armpi_studio_trajectory_capture"
PLANNED_KIND = "moveit_planned_trajectory"
DATASET_SCHEMA_VERSION = 1
TIMEBASE = "shared_trajectory_start"
MIN_POINTS = 2
MAX_POINTS = 20000
FEEDBACK_TRUTHS = {
    "measured",
    "measured_on_demand",
    "simulated",
    "commanded",
    "unknown",
}
ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5")
# Vendor servo_controller.yaml: joint1..joint5 = IDs 6,5,4,3,2.
SERVO_IDS = {
    "joint1": 6,
    "joint2": 5,
    "joint3": 4,
    "joint4": 3,
    "joint5": 2,
}
SERVO_INIT_PULSE = 500
# 240 degrees over 1000 pulses, converted to radians per pulse.
RADIANS_PER_TICK = 240.0 / 360.0 * (2.0 * math.pi) / 1000.0
PULSE_MIN = 0
PULSE_MAX = 1000


class CaptureError(ValueError):
    """A capture or planned trajectory failed validation."""

    def __init__(self, issues: list[dict[str, str]]) -> None:
        super().__init__("trajectory capture is invalid")
        self.issues = issues


def _issue(path: str, message: str) -> dict[str, str]:
    return {"path": path, "message": message}


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _is_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _finite_positions(value: Any, joint_count: int) -> list[float] | None:
    if (
        not isinstance(value, (list, tuple))
        or len(value) != joint_count
        or not all(_is_number(item) for item in value)
    ):
        return None
    return [float(item) for item in value]


def pulse_to_rad(joint: str, pulse: int) -> float:
    """Convert a bus-servo pulse to radians with the vendor convention.

    ``joint3`` is inverted relative to the other four joints because of the
    vendor's mechanical assembly convention.
    """
    if joint not in SERVO_IDS:
        raise KeyError(f"unknown arm joint: {joint!r}")
    if joint == "joint3":
        return (SERVO_INIT_PULSE - int(pulse)) * RADIANS_PER_TICK
    return (int(pulse) - SERVO_INIT_PULSE) * RADIANS_PER_TICK


def rad_to_pulse(joint: str, radians: float) -> float:
    """Inverse of :func:`pulse_to_rad` (used by tests and diagnostics)."""
    if joint not in SERVO_IDS:
        raise KeyError(f"unknown arm joint: {joint!r}")
    ticks = float(radians) / RADIANS_PER_TICK
    if joint == "joint3":
        return SERVO_INIT_PULSE - ticks
    return SERVO_INIT_PULSE + ticks


def servo_states_to_positions(
    states: Any,
    joint_names: tuple[str, ...] = ARM_JOINTS,
) -> dict[str, float]:
    """Convert a ``ServoStateList``-like message to joint radians.

    ``states`` is the message's ``servo_state`` list. Each item must expose
    ``id`` and a non-empty ``position`` sequence. A pulse outside 0..1000 is
    rejected instead of being converted: the vendor node crashed after an
    out-of-range sample on 2026-11-18, so the recorder stops reading when it
    sees one.
    """
    by_id: dict[int, int] = {}
    for index, state in enumerate(states):
        servo_id = getattr(state, "id", None)
        raw_position = getattr(state, "position", None)
        if servo_id is None or raw_position is None:
            continue
        # The vendor message declares ``id``/``position`` as int32 scalars,
        # while hand-written fixtures sometimes use one-element lists.
        if isinstance(raw_position, (list, tuple)):
            if not raw_position:
                continue
            raw_position = raw_position[0]
        if isinstance(servo_id, (list, tuple)):
            if not servo_id:
                continue
            servo_id = servo_id[0]
        try:
            servo_id = int(servo_id)
            pulse = int(raw_position)
        except (TypeError, ValueError):
            raise CaptureError(
                [_issue(f"servo_state[{index}]", "id/position must be integers")]
            ) from None
        by_id[servo_id] = pulse

    positions: dict[str, float] = {}
    issues: list[dict[str, str]] = []
    for joint in joint_names:
        servo_id = SERVO_IDS[joint]
        if servo_id not in by_id:
            issues.append(_issue(joint, f"servo {servo_id} missing from state list"))
            continue
        pulse = by_id[servo_id]
        if pulse < PULSE_MIN or pulse > PULSE_MAX:
            issues.append(
                _issue(
                    joint,
                    f"servo {servo_id} pulse {pulse} is outside "
                    f"{PULSE_MIN}..{PULSE_MAX}; stop reading",
                )
            )
            continue
        positions[joint] = pulse_to_rad(joint, pulse)
    if issues:
        raise CaptureError(issues)
    return positions


def load_planned_trajectory(raw: Any) -> dict[str, Any]:
    """Validate a planned-trajectory JSON document.

    Accepted shapes:

    * the recorder tool format with ``kind: moveit_planned_trajectory`` and
      ``points: [{t_s, positions, velocities?}]``;
    * a dataset-like object with ``joint_names`` and ``points``.

    The function returns a normalized ``joint_names``/``points``/``source``
    object. It does not require the ``kind`` field so hand-written fixtures are
    usable, but it rejects negative or non-increasing times and non-finite
    positions.
    """
    issues: list[dict[str, str]] = []
    if not isinstance(raw, dict):
        raise CaptureError([_issue("$", "planned trajectory must be a JSON object")])

    joint_names = raw.get("joint_names")
    if (
        not isinstance(joint_names, list)
        or not 1 <= len(joint_names) <= 8
        or not all(isinstance(item, str) and 0 < len(item.strip()) <= 64 for item in joint_names)
        or len(set(joint_names)) != len(joint_names)
    ):
        issues.append(_issue("joint_names", "must be 1..8 unique non-empty names"))
        joint_names = []

    source = raw.get("source", "")
    if not isinstance(source, str) or len(source) > 200:
        issues.append(_issue("source", "must be at most 200 chars"))
        source = ""

    points = raw.get("points")
    if not isinstance(points, list):
        issues.append(_issue("points", "must be a list"))
        points = []
    if len(points) < MIN_POINTS:
        issues.append(_issue("points", f"must contain at least {MIN_POINTS} samples"))
    if len(points) > MAX_POINTS:
        issues.append(_issue("points", f"must contain at most {MAX_POINTS} samples"))

    normalized: list[dict[str, Any]] = []
    previous = None
    joint_count = len(joint_names)
    for index, point in enumerate(points[:MAX_POINTS]):
        path = f"points[{index}]"
        if not isinstance(point, dict):
            issues.append(_issue(path, "must be an object"))
            continue
        t_s = point.get("t_s", point.get("time_from_start_s"))
        if not _is_number(t_s) or float(t_s) < 0:
            issues.append(_issue(f"{path}.t_s", "must be a finite number >= 0"))
            continue
        t_value = float(t_s)
        if previous is not None and t_value <= previous:
            issues.append(_issue(f"{path}.t_s", "must be strictly increasing"))
        previous = t_value
        positions = _finite_positions(point.get("positions"), joint_count)
        if positions is None:
            issues.append(
                _issue(f"{path}.positions", f"must be {joint_count} finite numbers")
            )
            continue
        item: dict[str, Any] = {"t_s": t_value, "positions": positions}
        velocities = point.get("velocities")
        if velocities is not None:
            normalized_velocities = _finite_positions(velocities, joint_count)
            if normalized_velocities is None:
                issues.append(
                    _issue(
                        f"{path}.velocities",
                        f"must be {joint_count} finite numbers when present",
                    )
                )
            else:
                item["velocities"] = normalized_velocities
        normalized.append(item)

    if issues:
        raise CaptureError(issues)
    return {
        "joint_names": [str(item) for item in joint_names],
        "points": normalized,
        "source": source,
        "metadata": raw.get("metadata") if isinstance(raw.get("metadata"), dict) else {},
    }


def select_planned_joints(
    planned: dict[str, Any],
    joint_names: list[str],
) -> list[dict[str, Any]]:
    """Reorder planned points into the recorder's joint order."""
    source_names = planned["joint_names"]
    indices: list[int] = []
    issues: list[dict[str, str]] = []
    for joint in joint_names:
        if joint not in source_names:
            issues.append(_issue("planned.joint_names", f"missing joint {joint!r}"))
            continue
        indices.append(source_names.index(joint))
    if issues:
        raise CaptureError(issues)

    points: list[dict[str, Any]] = []
    for index, point in enumerate(planned["points"]):
        item: dict[str, Any] = {
            "t_s": point["t_s"],
            "positions": [point["positions"][source_index] for source_index in indices],
        }
        if "velocities" in point:
            item["velocities"] = [
                point["velocities"][source_index] for source_index in indices
            ]
        points.append(item)
    return points


def build_static_planned(
    joint_names: list[str],
    positions: list[float],
    duration_s: float = 1.0,
    *,
    source: str = "static hold fixture (no motion)",
) -> dict[str, Any]:
    """Return a two-point hold plan for recorder validation without motion."""
    if not _is_number(duration_s) or float(duration_s) <= 0:
        raise CaptureError([_issue("duration_s", "must be a finite positive number")])
    normalized = _finite_positions(positions, len(joint_names))
    if normalized is None:
        raise CaptureError(
            [_issue("positions", f"must be {len(joint_names)} finite numbers")]
        )
    return {
        "joint_names": list(joint_names),
        "points": [
            {"t_s": 0.0, "positions": normalized},
            {"t_s": float(duration_s), "positions": normalized},
        ],
        "source": source,
        "metadata": {"fixture": "static_hold", "motion": False},
    }


class CaptureBuffer:
    """Collect strictly increasing, finite samples for one stream."""

    def __init__(
        self,
        joint_names: list[str] | tuple[str, ...] = ARM_JOINTS,
        *,
        max_points: int = MAX_POINTS,
    ) -> None:
        names = [str(item) for item in joint_names]
        if not 1 <= len(names) <= 8 or len(set(names)) != len(names):
            raise CaptureError([_issue("joint_names", "must be 1..8 unique names")])
        self.joint_names = names
        self.max_points = max(MIN_POINTS, min(int(max_points), MAX_POINTS))
        self.samples: dict[str, list[dict[str, Any]]] = {
            "commanded": [],
            "feedback": [],
        }
        self._last_t: dict[str, float | None] = {"commanded": None, "feedback": None}
        self.dropped: dict[str, int] = {"commanded": 0, "feedback": 0}
        self.out_of_order: dict[str, int] = {"commanded": 0, "feedback": 0}

    def add(self, stream: str, t_s: float, positions: list[float]) -> bool:
        """Append one sample; return False when it was dropped."""
        if stream not in self.samples:
            raise CaptureError([_issue("stream", f"unknown stream {stream!r}")])
        if not _is_number(t_s) or float(t_s) < 0:
            raise CaptureError([_issue(f"{stream}.t_s", "must be finite and >= 0")])
        normalized = _finite_positions(positions, len(self.joint_names))
        if normalized is None:
            raise CaptureError(
                [
                    _issue(
                        f"{stream}.positions",
                        f"must be {len(self.joint_names)} finite numbers",
                    )
                ]
            )
        t_value = float(t_s)
        last = self._last_t[stream]
        if last is not None and t_value <= last:
            self.out_of_order[stream] += 1
            return False
        if len(self.samples[stream]) >= self.max_points:
            self.dropped[stream] += 1
            return False
        self.samples[stream].append({"t_s": t_value, "positions": normalized})
        self._last_t[stream] = t_value
        return True

    def count(self, stream: str) -> int:
        return len(self.samples[stream])

    def last(self, stream: str) -> dict[str, Any] | None:
        samples = self.samples[stream]
        return samples[-1] if samples else None

    def capture_document(
        self,
        *,
        started_at: str | None = None,
        ended_at: str | None = None,
        commanded_source: str = "",
        feedback_source: str = "",
        feedback_truth: str = "measured",
        topics: dict[str, str] | None = None,
        time_source: str = "monotonic_receive",
        motion_start_s: float | None = None,
        notes: list[str] | None = None,
        aborted: bool = False,
    ) -> dict[str, Any]:
        """Return the raw capture document (before planned data is attached)."""
        if feedback_truth not in FEEDBACK_TRUTHS:
            raise CaptureError(
                [_issue("feedback_truth", "must be one of " + ", ".join(sorted(FEEDBACK_TRUTHS)))]
            )
        document: dict[str, Any] = {
            "schema_version": CAPTURE_SCHEMA_VERSION,
            "kind": CAPTURE_KIND,
            "created_at": utc_now_iso(),
            "started_at": started_at or utc_now_iso(),
            "ended_at": ended_at or utc_now_iso(),
            "timebase": TIMEBASE,
            "time_source": time_source,
            "joint_names": list(self.joint_names),
            "topics": dict(topics or {}),
            "motion_start_s": motion_start_s,
            "aborted": bool(aborted),
            "notes": list(notes or []),
            "streams": {
                "commanded": {
                    "truth": "commanded",
                    "source": commanded_source,
                    "points": list(self.samples["commanded"]),
                    "dropped": self.dropped["commanded"],
                    "out_of_order": self.out_of_order["commanded"],
                },
                "feedback": {
                    "truth": feedback_truth,
                    "source": feedback_source,
                    "points": list(self.samples["feedback"]),
                    "dropped": self.dropped["feedback"],
                    "out_of_order": self.out_of_order["feedback"],
                },
            },
        }
        return document


def _validate_stream(
    points: Any,
    *,
    name: str,
    joint_count: int,
    truth: str,
    source: str,
    offset_s: float = 0.0,
) -> dict[str, Any]:
    issues: list[dict[str, str]] = []
    if not isinstance(points, list):
        raise CaptureError([_issue(name, "points must be a list")])
    if not MIN_POINTS <= len(points) <= MAX_POINTS:
        raise CaptureError(
            [_issue(name, f"must contain {MIN_POINTS}..{MAX_POINTS} samples")]
        )
    normalized: list[dict[str, Any]] = []
    previous: float | None = None
    for index, point in enumerate(points):
        path = f"{name}[{index}]"
        if not isinstance(point, dict):
            issues.append(_issue(path, "must be an object"))
            continue
        t_s = point.get("t_s")
        if not _is_number(t_s):
            issues.append(_issue(f"{path}.t_s", "must be a finite number"))
            continue
        value = float(t_s) + float(offset_s)
        if value < 0:
            issues.append(_issue(f"{path}.t_s", "must be >= 0 after offset"))
            continue
        if previous is not None and value <= previous:
            issues.append(_issue(f"{path}.t_s", "must be strictly increasing"))
        previous = value
        positions = _finite_positions(point.get("positions"), joint_count)
        if positions is None:
            issues.append(
                _issue(f"{path}.positions", f"must be {joint_count} finite numbers")
            )
            continue
        item: dict[str, Any] = {"t_s": value, "positions": positions}
        velocities = point.get("velocities")
        if velocities is not None:
            normalized_velocities = _finite_positions(velocities, joint_count)
            if normalized_velocities is None:
                issues.append(
                    _issue(
                        f"{path}.velocities",
                        f"must be {joint_count} finite numbers when present",
                    )
                )
            else:
                item["velocities"] = normalized_velocities
        normalized.append(item)
    if issues:
        raise CaptureError(issues)
    return {
        "truth": truth,
        "time_offset_s": 0.0,
        "source": source[:200],
        "points": normalized,
    }


def build_dataset(
    *,
    name: str,
    joint_names: list[str],
    planned: dict[str, Any] | list[dict[str, Any]],
    commanded: list[dict[str, Any]],
    feedback: list[dict[str, Any]],
    description: str = "",
    planned_source: str = "",
    planned_offset_s: float = 0.0,
    commanded_source: str = "",
    feedback_source: str = "",
    feedback_truth: str = "measured",
    executor: dict[str, Any] | None = None,
    metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Build a G2 ``schema_version: 1`` dataset.

    Raises:
        CaptureError: with a list of ``{path, message}`` issues.
    """
    if not isinstance(name, str) or not name.strip() or len(name) > 200:
        raise CaptureError([_issue("name", "must be a non-empty string of at most 200 chars")])
    if not isinstance(joint_names, list) or not 1 <= len(joint_names) <= 8:
        raise CaptureError([_issue("joint_names", "must be a list of 1..8 names")])
    if len(set(joint_names)) != len(joint_names):
        raise CaptureError([_issue("joint_names", "must be unique")])
    if feedback_truth not in FEEDBACK_TRUTHS:
        raise CaptureError(
            [_issue("feedback_truth", "must be one of " + ", ".join(sorted(FEEDBACK_TRUTHS)))]
        )
    if not _is_number(planned_offset_s):
        raise CaptureError([_issue("planned_offset_s", "must be a finite number")])

    if isinstance(planned, dict):
        planned_document = load_planned_trajectory(planned)
        planned_points = select_planned_joints(planned_document, joint_names)
        source = planned_source or planned_document.get("source") or ""
        planned_metadata = planned_document.get("metadata") or {}
    else:
        planned_points = planned
        source = planned_source
        planned_metadata = {}

    dataset = {
        "schema_version": DATASET_SCHEMA_VERSION,
        "name": name.strip(),
        "description": description[:2000],
        "timebase": TIMEBASE,
        "joint_names": list(joint_names),
        "planned": _validate_stream(
            planned_points,
            name="planned",
            joint_count=len(joint_names),
            truth="planned",
            source=source,
            offset_s=float(planned_offset_s),
        ),
        "commanded": _validate_stream(
            commanded,
            name="commanded",
            joint_count=len(joint_names),
            truth="commanded",
            source=commanded_source,
        ),
        "feedback": _validate_stream(
            feedback,
            name="feedback",
            joint_count=len(joint_names),
            truth=feedback_truth,
            source=feedback_source,
        ),
        "executor": executor,
        "metadata": {
            "recorder": "armpi_studio_trajectory_recorder",
            "capture_schema_version": CAPTURE_SCHEMA_VERSION,
            "planned_metadata": planned_metadata,
            **(metadata or {}),
        },
    }
    return dataset


def dataset_from_capture(
    capture: dict[str, Any],
    planned: dict[str, Any] | list[dict[str, Any]],
    *,
    name: str | None = None,
    description: str = "",
    planned_offset_s: float = 0.0,
    executor: dict[str, Any] | None = None,
    metadata: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Attach a planned stream to a raw recorder capture and build a dataset.

    This is the offline path for a real motion: the recorder writes the raw
    capture while the robot moves, and the MoveIt plan JSON is attached
    afterwards. No ROS connection is required.
    """
    if not isinstance(capture, dict):
        raise CaptureError([_issue("$", "capture must be a JSON object")])
    if capture.get("kind") != CAPTURE_KIND:
        raise CaptureError([_issue("kind", f"must be {CAPTURE_KIND!r}")])
    if capture.get("timebase") != TIMEBASE:
        raise CaptureError([_issue("timebase", f"must be {TIMEBASE!r}")])
    joint_names = capture.get("joint_names")
    if (
        not isinstance(joint_names, list)
        or not 1 <= len(joint_names) <= 8
        or len(set(joint_names)) != len(joint_names)
    ):
        raise CaptureError([_issue("joint_names", "must be 1..8 unique names")])
    streams = capture.get("streams")
    if not isinstance(streams, dict):
        raise CaptureError([_issue("streams", "must be an object")])
    commanded = streams.get("commanded")
    feedback = streams.get("feedback")
    if not isinstance(commanded, dict):
        raise CaptureError([_issue("streams.commanded", "must be an object")])
    if not isinstance(feedback, dict):
        raise CaptureError([_issue("streams.feedback", "must be an object")])
    feedback_truth = feedback.get("truth", "measured")
    if feedback_truth == "unknown":
        # The recorder always labels measured feedback; refuse to silently
        # upgrade an unknown stream into tracking evidence.
        raise CaptureError(
            [_issue("streams.feedback.truth", "unknown truth cannot be used for tracking")]
        )
    capture_metadata = {
        "capture_created_at": capture.get("created_at"),
        "capture_started_at": capture.get("started_at"),
        "capture_ended_at": capture.get("ended_at"),
        "capture_notes": capture.get("notes", []),
        "capture_topics": capture.get("topics", {}),
        "capture_motion_start_s": capture.get("motion_start_s"),
        "capture_aborted": capture.get("aborted", False),
        "capture_recorder": capture.get("recorder", {}),
    }
    return build_dataset(
        name=name or f"recorded capture {capture.get('created_at', '')}".strip(),
        joint_names=joint_names,
        planned=planned,
        commanded=commanded.get("points", []),
        feedback=feedback.get("points", []),
        description=description,
        planned_source="",
        planned_offset_s=planned_offset_s,
        commanded_source=str(commanded.get("source", "")),
        feedback_source=str(feedback.get("source", "")),
        feedback_truth=feedback_truth,
        executor=executor,
        metadata={**capture_metadata, **(metadata or {})},
    )


def write_json_atomic(path: str, payload: Any) -> None:
    """Write JSON through a sibling temporary file, then replace."""
    import os

    temporary = f"{path}.tmp"
    with open(temporary, "w", encoding="utf-8") as handle:
        json.dump(payload, handle, indent=2, sort_keys=True)
        handle.write("\n")
    os.replace(temporary, path)
