"""Supervised motion guard for the gateway.

The gateway is the last validation layer before a vendor ROS message reaches
the servo controller. It is deliberately independent of the Studio backend:
even a backend bug cannot publish a pulse outside the configured limits, a
step larger than the supervised cap, a stale-feedback command, or a second
command while one is still executing.

This module has no ROS import and no publisher. It validates a request against
a JSON limits file and derives completion from measured ``servo_states``
pulses. The node owns the publisher and the measured subscription.
"""

from __future__ import annotations

import json
import math
import threading
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

from .chassis import ChassisConfig, chassis_config_from_dict

MOTION_SCHEMA_VERSION = 1
ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5")
SERVO_IDS = {
    "joint1": 6,
    "joint2": 5,
    "joint3": 4,
    "joint4": 3,
    "joint5": 2,
}
SERVO_INIT_PULSE = 500
GRIPPER_JOINT = "r_joint"
GRIPPER_SERVO_ID = 1
GRIPPER_INIT_PULSE = 700
MOTION_SERVO_IDS = {**SERVO_IDS, GRIPPER_JOINT: GRIPPER_SERVO_ID}
MOTION_MODES = ("step", "pose")


class MotionConfigError(ValueError):
    """The motion limits file is missing or malformed."""


class MotionRejected(RuntimeError):
    """A motion request failed gateway-side validation."""

    def __init__(
        self,
        code: str,
        message: str,
        *,
        http_status: int = 400,
        details: dict[str, Any] | None = None,
    ) -> None:
        super().__init__(message)
        self.code = code
        self.http_status = http_status
        self.details = details or {}


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _is_number(value: Any) -> bool:
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise MotionConfigError(message)


@dataclass(frozen=True)
class JointMotionLimit:
    joint: str
    servo_id: int
    min_pulse: int
    max_pulse: int
    init_pulse: int
    direction: int
    radians_per_tick: float

    def rad_to_pulse(self, radians: float) -> float:
        return self.init_pulse + self.direction * float(radians) / self.radians_per_tick

    def pulse_to_rad(self, pulse: int) -> float:
        return self.direction * (int(pulse) - self.init_pulse) * self.radians_per_tick

    @property
    def min_rad(self) -> float:
        return min(self.pulse_to_rad(self.min_pulse), self.pulse_to_rad(self.max_pulse))

    @property
    def max_rad(self) -> float:
        return max(self.pulse_to_rad(self.min_pulse), self.pulse_to_rad(self.max_pulse))

    @property
    def max_step_pulses(self) -> int:
        """Pulses covered by the supervised step cap, rounded down."""
        return int(math.floor(self._step_limit_rad / self.radians_per_tick + 1e-9))

    # The step cap is injected by MotionConfig before validation; keeping it as
    # a plain attribute avoids duplicating the value in every joint record.
    _step_limit_rad: float = 0.0


@dataclass(frozen=True)
class GripperLimit:
    """The vendor maps r_joint around init pulse 700.

    Open raw 170 measures register 167; an empty close stalls at register 640
    while the command is 700, so completion is timed rather than positional.
    """

    joint: str
    servo_id: int
    min_pulse: int
    max_pulse: int
    init_pulse: int
    radians_per_tick: float
    speed_raw_per_s: float
    min_duration_s: float
    max_duration_s: float
    completion: str = "timed"

    def rad_to_pulse(self, radians: float) -> float:
        return self.init_pulse + float(radians) / self.radians_per_tick

    def pulse_to_rad(self, pulse: int) -> float:
        return (int(pulse) - self.init_pulse) * self.radians_per_tick

    @property
    def min_rad(self) -> float:
        return min(self.pulse_to_rad(self.min_pulse), self.pulse_to_rad(self.max_pulse))

    @property
    def max_rad(self) -> float:
        return max(self.pulse_to_rad(self.min_pulse), self.pulse_to_rad(self.max_pulse))


@dataclass(frozen=True)
class MotionSafety:
    max_step_rad: float
    min_duration_s: float
    max_duration_s: float
    feedback_max_age_ms: int
    tolerance_pulses: int
    settle_samples: int
    timeout_margin_s: float
    single_joint_only: bool
    mode: str = "step"
    max_velocity_rad_s: float | None = None
    pose_requires_all_arm_joints: bool = True
    # The vendor ``/controller_manager/servo_states`` topic is a command cache,
    # not a bus read. When ``verification`` is on, an arm command is completed
    # only after ``GetBusServoState`` returns the actual registers.
    verification: bool = False
    verification_tolerance_pulses: int = 12
    verification_settle_s: float = 0.5
    verification_margin_pulses: int = 25


@dataclass(frozen=True)
class MotionConfig:
    schema_version: int
    mapping_version: str
    radians_per_tick: float
    joints: dict[str, JointMotionLimit]
    safety: MotionSafety
    gripper: GripperLimit | None = None
    chassis: ChassisConfig | None = None


@dataclass
class MotionTarget:
    joint: str
    servo_id: int
    target_pulse: int
    target_rad: float
    completion: str = "position"


@dataclass
class MotionPlan:
    command_id: str
    duration_s: float
    targets: list[MotionTarget]
    started_at: float
    deadline: float
    started_wall: str
    mode: str = "step"


def motion_config_from_dict(raw: dict[str, Any]) -> MotionConfig:
    """Validate the gateway motion JSON contract and return typed limits."""
    _require(isinstance(raw, dict), "motion config root must be an object")
    schema_version = raw.get("schema_version", MOTION_SCHEMA_VERSION)
    _require(
        int(schema_version) == MOTION_SCHEMA_VERSION,
        f"unsupported motion schema_version: {schema_version!r}",
    )
    mapping_version = str(raw.get("mapping_version") or "").strip()
    _require(bool(mapping_version), "mapping_version is required")

    radians_per_tick = float(raw.get("radians_per_tick", 0.0))
    _require(radians_per_tick > 0, "radians_per_tick must be positive")

    safety_raw = raw.get("safety")
    _require(isinstance(safety_raw, dict), "safety must be an object")
    mode = str(raw.get("mode", safety_raw.get("mode", "step"))).strip().lower()
    _require(mode in MOTION_MODES, f"unknown motion mode: {mode!r}")
    max_velocity_raw = safety_raw.get("max_velocity_rad_s")
    max_velocity = None
    if max_velocity_raw is not None:
        max_velocity = float(max_velocity_raw)
        _require(0.05 <= max_velocity <= 2.0, "max_velocity_rad_s must be in [0.05, 2.0]")
    single_joint_only = bool(safety_raw.get("single_joint_only", mode == "step"))
    verification = bool(safety_raw.get("verification", mode == "pose"))
    verification_tolerance = int(
        safety_raw.get("verification_tolerance_pulses", 12)
    )
    verification_settle = float(safety_raw.get("verification_settle_s", 0.5))
    verification_margin = int(safety_raw.get("verification_margin_pulses", 25))
    safety = MotionSafety(
        max_step_rad=float(safety_raw["max_step_rad"]),
        min_duration_s=float(safety_raw["min_duration_s"]),
        max_duration_s=float(safety_raw["max_duration_s"]),
        feedback_max_age_ms=int(safety_raw["feedback_max_age_ms"]),
        tolerance_pulses=int(safety_raw["tolerance_pulses"]),
        settle_samples=int(safety_raw["settle_samples"]),
        timeout_margin_s=float(safety_raw["timeout_margin_s"]),
        single_joint_only=single_joint_only,
        mode=mode,
        max_velocity_rad_s=max_velocity,
        pose_requires_all_arm_joints=bool(
            safety_raw.get("pose_requires_all_arm_joints", True)
        ),
        verification=verification,
        verification_tolerance_pulses=verification_tolerance,
        verification_settle_s=verification_settle,
        verification_margin_pulses=verification_margin,
    )
    step_cap = 4.0 if mode == "pose" else 0.35
    _require(
        0 < safety.max_step_rad <= step_cap,
        f"max_step_rad must be in (0, {step_cap}] for mode {mode}",
    )
    _require(0 <= safety.min_duration_s <= safety.max_duration_s, "invalid duration range")
    _require(safety.feedback_max_age_ms > 0, "feedback_max_age_ms must be positive")
    _require(0 <= safety.tolerance_pulses <= 10, "tolerance_pulses must be 0..10")
    _require(1 <= safety.settle_samples <= 10, "settle_samples must be 1..10")
    _require(0 <= safety.timeout_margin_s <= 10, "timeout_margin_s must be 0..10")
    _require(
        0 <= safety.verification_tolerance_pulses <= 50,
        "verification_tolerance_pulses must be 0..50",
    )
    _require(
        0.0 <= safety.verification_settle_s <= 5.0,
        "verification_settle_s must be 0..5",
    )
    _require(
        0 <= safety.verification_margin_pulses <= 100,
        "verification_margin_pulses must be 0..100",
    )
    if mode == "pose":
        _require(not single_joint_only, "pose mode must allow a whole pose")
        _require(
            max_velocity is not None,
            "pose mode requires safety.max_velocity_rad_s",
        )
        _require(
            verification,
            "pose mode requires bus verification (safety.verification=true)",
        )

    gripper: GripperLimit | None = None
    if mode == "pose":
        gripper_raw = raw.get("gripper")
        _require(isinstance(gripper_raw, dict), "pose mode requires a gripper block")
        _require(
            str(gripper_raw.get("joint", GRIPPER_JOINT)) == GRIPPER_JOINT,
            f"gripper.joint must be {GRIPPER_JOINT}",
        )
        gripper_servo = int(gripper_raw.get("servo_id", GRIPPER_SERVO_ID))
        _require(
            gripper_servo == GRIPPER_SERVO_ID,
            f"gripper.servo_id must be {GRIPPER_SERVO_ID}",
        )
        gripper_init = int(gripper_raw.get("init_pulse", GRIPPER_INIT_PULSE))
        _require(gripper_init == GRIPPER_INIT_PULSE, "gripper.init_pulse must be 700")
        gripper_min = int(gripper_raw["min_pulse"])
        gripper_max = int(gripper_raw["max_pulse"])
        _require(
            0 <= gripper_min < gripper_max <= 1000,
            "gripper pulse range is invalid",
        )
        speed = float(gripper_raw.get("speed_raw_per_s", 120.0))
        _require(speed > 0, "gripper.speed_raw_per_s must be positive")
        gmin = float(gripper_raw.get("min_duration_s", 1.0))
        gmax = float(gripper_raw.get("max_duration_s", 6.0))
        _require(0.5 <= gmin <= gmax <= 30.0, "invalid gripper duration range")
        completion = str(gripper_raw.get("completion", "timed"))
        _require(
            completion in {"timed", "position"},
            "gripper.completion must be 'timed' or 'position'",
        )
        gripper = GripperLimit(
            joint=GRIPPER_JOINT,
            servo_id=GRIPPER_SERVO_ID,
            min_pulse=gripper_min,
            max_pulse=gripper_max,
            init_pulse=gripper_init,
            radians_per_tick=radians_per_tick,
            speed_raw_per_s=speed,
            min_duration_s=gmin,
            max_duration_s=gmax,
            completion=completion,
        )

    joints_raw = raw.get("joints")
    _require(isinstance(joints_raw, dict), "joints must be an object")
    joints: dict[str, JointMotionLimit] = {}
    for joint in ARM_JOINTS:
        item = joints_raw.get(joint)
        _require(isinstance(item, dict), f"missing joint limit: {joint}")
        servo_id = int(item["servo_id"])
        _require(
            servo_id == SERVO_IDS[joint],
            f"{joint}: servo_id must be {SERVO_IDS[joint]}, got {servo_id}",
        )
        min_pulse = int(item["min_pulse"])
        max_pulse = int(item["max_pulse"])
        init_pulse = int(item.get("init_pulse", SERVO_INIT_PULSE))
        _require(0 <= min_pulse < max_pulse <= 1000, f"{joint}: invalid pulse range")
        _require(init_pulse == SERVO_INIT_PULSE, f"{joint}: init_pulse must be 500")
        direction = int(item.get("direction", 1))
        _require(direction in (-1, 1), f"{joint}: direction must be 1 or -1")
        tick = float(item.get("radians_per_tick", radians_per_tick))
        _require(tick > 0, f"{joint}: radians_per_tick must be positive")
        joints[joint] = JointMotionLimit(
            joint=joint,
            servo_id=servo_id,
            min_pulse=min_pulse,
            max_pulse=max_pulse,
            init_pulse=init_pulse,
            direction=direction,
            radians_per_tick=tick,
            _step_limit_rad=safety.max_step_rad,
        )
    return MotionConfig(
        schema_version=MOTION_SCHEMA_VERSION,
        mapping_version=mapping_version,
        radians_per_tick=radians_per_tick,
        joints=joints,
        safety=safety,
        gripper=gripper,
        chassis=chassis_config_from_dict(raw.get("chassis")),
    )


def load_motion_config(path: str | Path) -> MotionConfig:
    path = Path(path)
    if not path.is_file():
        raise MotionConfigError(f"motion limits file not found: {path}")
    try:
        raw = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise MotionConfigError(f"invalid JSON in {path}: {exc}") from exc
    return motion_config_from_dict(raw)


def _measured_item(measured: dict[str, Any], joint: str, now: float) -> tuple[int, float]:
    item = measured.get(joint)
    if not isinstance(item, dict):
        raise MotionRejected(
            "MOTION_STALE_FEEDBACK",
            f"{joint}: no measured servo position is available",
            http_status=409,
        )
    raw = item.get("raw_position")
    age_ms = item.get("sample_age_ms")
    if isinstance(raw, bool) or not isinstance(raw, (int, float)):
        raise MotionRejected(
            "MOTION_STALE_FEEDBACK",
            f"{joint}: measured pulse is unavailable",
            http_status=409,
        )
    if not _is_number(age_ms):
        raise MotionRejected(
            "MOTION_STALE_FEEDBACK",
            f"{joint}: measured sample age is unknown",
            http_status=409,
        )
    return int(raw), float(age_ms)


def _guarded(method):
    """Serialize MotionGuard state: HTTP, servo cache and bus-read threads."""

    def wrapper(self, *args, **kwargs):
        with self._lock:
            return method(self, *args, **kwargs)

    wrapper.__name__ = method.__name__
    wrapper.__doc__ = method.__doc__
    return wrapper


class MotionGuard:
    """Validate one supervised command at a time and derive completion."""

    def __init__(self, config: MotionConfig) -> None:
        self.config = config
        self._lock = threading.RLock()
        self._active: MotionPlan | None = None
        self._state = "idle"
        self._settle_count = 0
        self._last_targets: list[MotionTarget] = []
        self._last_duration_s: float | None = None
        self._last_command_id: str | None = None
        self._last_started_wall: str | None = None
        self._last_measured: dict[str, dict[str, Any]] = {}
        self._finished_at: str | None = None
        self._last_stop: dict[str, Any] | None = None
        # Bus-verification state. ``None`` until a command needs it.
        self._verification: str | None = None
        self._verification_reason: str | None = None
        self._actual: dict[str, dict[str, Any]] = {}
        self._actual_at: str | None = None

    # ------------------------------------------------------------- validation
    @_guarded
    def validate(
        self,
        payload: dict[str, Any],
        measured: dict[str, Any] | None,
        now: float,
    ) -> MotionPlan:
        if not isinstance(payload, dict):
            raise MotionRejected("MOTION_BAD_REQUEST", "request body must be an object")
        command_id = payload.get("command_id")
        if not isinstance(command_id, str) or not command_id.strip():
            raise MotionRejected("MOTION_BAD_REQUEST", "command_id is required")
        if len(command_id) > 128:
            raise MotionRejected("MOTION_BAD_REQUEST", "command_id is too long")

        if self._active is not None:
            raise MotionRejected(
                "MOTION_BUSY",
                f"command {self._active.command_id} is still executing",
                http_status=409,
            )

        duration = payload.get("duration_s")
        if not _is_number(duration):
            raise MotionRejected("MOTION_BAD_REQUEST", "duration_s must be finite")
        duration = float(duration)
        safety = self.config.safety
        if not safety.min_duration_s <= duration <= safety.max_duration_s:
            raise MotionRejected(
                "MOTION_BAD_REQUEST",
                f"duration_s must be within [{safety.min_duration_s}, "
                f"{safety.max_duration_s}]",
            )

        raw_joints = payload.get("joints")
        if not isinstance(raw_joints, list) or not raw_joints:
            raise MotionRejected("MOTION_BAD_REQUEST", "joints must be a non-empty list")
        if safety.single_joint_only and len(raw_joints) != 1:
            raise MotionRejected(
                "MOTION_BAD_REQUEST",
                "supervised motion accepts exactly one joint per command",
            )
        if len(raw_joints) > len(ARM_JOINTS):
            raise MotionRejected("MOTION_BAD_REQUEST", "too many joints")

        gripper = self.config.gripper
        if safety.mode == "pose":
            names = [
                str(item.get("joint") or "").strip()
                for item in raw_joints
                if isinstance(item, dict)
            ]
            gripper_only = gripper is not None and names == [gripper.joint]
            if not gripper_only and safety.pose_requires_all_arm_joints:
                if set(names) != set(ARM_JOINTS):
                    raise MotionRejected(
                        "MOTION_BAD_REQUEST",
                        "pose mode requires exactly the five arm joints or "
                        f"exactly {GRIPPER_JOINT}; got {sorted(set(names))}",
                    )
            if gripper is not None and gripper.joint in names and not gripper_only:
                raise MotionRejected(
                    "MOTION_BAD_REQUEST",
                    "a gripper command cannot be mixed with arm joints",
                )

        measured = measured if isinstance(measured, dict) else {}
        targets: list[MotionTarget] = []
        seen: set[str] = set()
        for index, raw in enumerate(raw_joints):
            if not isinstance(raw, dict):
                raise MotionRejected(
                    "MOTION_BAD_REQUEST", f"joints[{index}] must be an object"
                )
            joint = str(raw.get("joint") or "").strip()
            if gripper is not None and joint == gripper.joint:
                if len(raw_joints) != 1:
                    raise MotionRejected(
                        "MOTION_BAD_REQUEST",
                        "a gripper command must be the only target",
                    )
                targets.append(
                    self._validate_gripper(raw, measured, now, duration)
                )
                continue
            if joint not in self.config.joints:
                raise MotionRejected(
                    "MOTION_BAD_REQUEST", f"unknown joint: {joint!r}"
                )
            if joint in seen:
                raise MotionRejected("MOTION_BAD_REQUEST", f"duplicate joint: {joint}")
            seen.add(joint)
            targets.append(
                self._validate_arm(joint, raw, measured, now, duration)
            )

        return MotionPlan(
            command_id=command_id,
            duration_s=duration,
            targets=targets,
            started_at=now,
            deadline=(
                now
                + duration
                + (safety.verification_settle_s if safety.verification else 0.0)
                + safety.timeout_margin_s
            ),
            started_wall=utc_now_iso(),
            mode=safety.mode,
        )

    # --------------------------------------------------------- target checks
    def _validate_arm(
        self,
        joint: str,
        raw: dict[str, Any],
        measured: dict[str, Any],
        now: float,
        duration: float,
    ) -> MotionTarget:
        limit = self.config.joints[joint]
        safety = self.config.safety
        servo_id = raw.get("servo_id", limit.servo_id)
        if int(servo_id) != limit.servo_id:
            raise MotionRejected(
                "MOTION_BAD_REQUEST",
                f"{joint}: servo_id must be {limit.servo_id}",
            )
        pulse = raw.get("target_pulse")
        if isinstance(pulse, bool) or not isinstance(pulse, (int, float)):
            raise MotionRejected(
                "MOTION_BAD_REQUEST", f"{joint}: target_pulse is required"
            )
        if int(pulse) != float(pulse):
            raise MotionRejected(
                "MOTION_BAD_REQUEST", f"{joint}: target_pulse must be an integer"
            )
        target_pulse = int(pulse)
        if not limit.min_pulse <= target_pulse <= limit.max_pulse:
            raise MotionRejected(
                "MOTION_OUT_OF_LIMIT",
                f"{joint}: pulse {target_pulse} outside "
                f"[{limit.min_pulse}, {limit.max_pulse}]",
            )
        target_rad = raw.get("target_rad")
        if not _is_number(target_rad):
            raise MotionRejected(
                "MOTION_BAD_REQUEST", f"{joint}: target_rad is required"
            )
        if abs(limit.rad_to_pulse(float(target_rad)) - target_pulse) > 1.5:
            raise MotionRejected(
                "MOTION_BAD_REQUEST",
                f"{joint}: target_rad disagrees with target_pulse",
            )

        measured_pulse, age_ms = _measured_item(measured, joint, now)
        if age_ms > safety.feedback_max_age_ms:
            raise MotionRejected(
                "MOTION_STALE_FEEDBACK",
                f"{joint}: measured sample is {age_ms:.0f} ms old "
                f"(limit {safety.feedback_max_age_ms} ms)",
                http_status=409,
            )
        step = abs(target_pulse - measured_pulse)
        if step > limit.max_step_pulses:
            raise MotionRejected(
                "MOTION_STEP_EXCEEDED",
                f"{joint}: step {step} pulses exceeds supervised "
                f"{limit.max_step_pulses} pulses "
                f"({safety.max_step_rad} rad)",
                http_status=409,
            )
        if safety.max_velocity_rad_s is not None:
            velocity = step * limit.radians_per_tick / duration
            if velocity > safety.max_velocity_rad_s + 1e-9:
                raise MotionRejected(
                    "MOTION_VELOCITY_EXCEEDED",
                    f"{joint}: {velocity:.3f} rad/s exceeds the pose cap "
                    f"{safety.max_velocity_rad_s:.3f} rad/s",
                    http_status=409,
                )
        return MotionTarget(
            joint=joint,
            servo_id=limit.servo_id,
            target_pulse=target_pulse,
            target_rad=float(target_rad),
        )

    def _validate_gripper(
        self,
        raw: dict[str, Any],
        measured: dict[str, Any],
        now: float,
        duration: float,
    ) -> MotionTarget:
        gripper = self.config.gripper
        safety = self.config.safety
        if gripper is None:  # guarded by the caller
            raise MotionRejected("MOTION_BAD_REQUEST", "no gripper contract")
        if not gripper.min_duration_s <= duration <= gripper.max_duration_s:
            raise MotionRejected(
                "MOTION_BAD_REQUEST",
                f"{gripper.joint}: duration must be within "
                f"[{gripper.min_duration_s}, {gripper.max_duration_s}] s",
            )
        servo_id = raw.get("servo_id", gripper.servo_id)
        if int(servo_id) != gripper.servo_id:
            raise MotionRejected(
                "MOTION_BAD_REQUEST",
                f"{gripper.joint}: servo_id must be {gripper.servo_id}",
            )
        pulse = raw.get("target_pulse")
        if isinstance(pulse, bool) or not isinstance(pulse, (int, float)):
            raise MotionRejected(
                "MOTION_BAD_REQUEST", f"{gripper.joint}: target_pulse is required"
            )
        if int(pulse) != float(pulse):
            raise MotionRejected(
                "MOTION_BAD_REQUEST",
                f"{gripper.joint}: target_pulse must be an integer",
            )
        target_pulse = int(pulse)
        if not gripper.min_pulse <= target_pulse <= gripper.max_pulse:
            raise MotionRejected(
                "MOTION_OUT_OF_LIMIT",
                f"{gripper.joint}: pulse {target_pulse} outside "
                f"[{gripper.min_pulse}, {gripper.max_pulse}]",
            )
        target_rad = raw.get("target_rad")
        if not _is_number(target_rad):
            raise MotionRejected(
                "MOTION_BAD_REQUEST", f"{gripper.joint}: target_rad is required"
            )
        if abs(gripper.rad_to_pulse(float(target_rad)) - target_pulse) > 1.5:
            raise MotionRejected(
                "MOTION_BAD_REQUEST",
                f"{gripper.joint}: target_rad disagrees with target_pulse",
            )
        measured_pulse, age_ms = _measured_item(measured, gripper.joint, now)
        if age_ms > safety.feedback_max_age_ms:
            raise MotionRejected(
                "MOTION_STALE_FEEDBACK",
                f"{gripper.joint}: measured sample is {age_ms:.0f} ms old "
                f"(limit {safety.feedback_max_age_ms} ms)",
                http_status=409,
            )
        delta = abs(target_pulse - measured_pulse)
        velocity = delta / duration
        if velocity > gripper.speed_raw_per_s + 1e-9:
            raise MotionRejected(
                "MOTION_VELOCITY_EXCEEDED",
                f"{gripper.joint}: {velocity:.1f} raw/s exceeds the measured "
                f"{gripper.speed_raw_per_s:.0f} raw/s tracking speed",
                http_status=409,
            )
        return MotionTarget(
            joint=gripper.joint,
            servo_id=gripper.servo_id,
            target_pulse=target_pulse,
            target_rad=float(target_rad),
            completion=gripper.completion,
        )

    # ------------------------------------------------------------------- state
    @_guarded
    def start(self, plan: MotionPlan, measured: dict[str, Any] | None = None) -> None:
        self._active = plan
        self._state = "executing"
        self._settle_count = 0
        self._last_targets = list(plan.targets)
        self._last_duration_s = plan.duration_s
        self._last_command_id = plan.command_id
        self._last_started_wall = plan.started_wall
        self._last_measured = _measured_snapshot(measured)
        self._finished_at = None
        self._verification = "pending" if self.config.safety.verification else None
        self._verification_reason = None

    @_guarded
    def observe(
        self,
        measured: dict[str, Any] | None,
        now: float,
    ) -> None:
        """Update completion from a ``servo_states`` sample.

        ``servo_states`` is the vendor command cache, not a bus read. When
        ``verification`` is enabled an arm target is never completed from this
        stream; the node calls :meth:`mark_verified` with a real bus read.
        Timed gripper targets still complete from the elapsed time.
        """
        snapshot = _measured_snapshot(measured)
        if snapshot:
            self._last_measured = snapshot
        plan = self._active
        if plan is None:
            return
        if now > plan.deadline:
            self._state = "unknown"
            self._verification_reason = (
                "no bus verification before the deadline"
                if self.config.safety.verification
                else None
            )
            self._finished_at = utc_now_iso()
            self._active = None
            return
        if not snapshot:
            return
        if self.config.safety.verification:
            # The command cache cannot prove any target. Even a timed gripper
            # waits for the post-interpolation bus read so its register is
            # recorded in the same completion.
            return
        tolerance = self.config.safety.tolerance_pulses
        elapsed = now - plan.started_at
        reached = True
        for target in plan.targets:
            if target.completion == "timed":
                # The gripper stalls on contact; its completion is the
                # commanded interpolation time, not the measured pulse.
                if elapsed < plan.duration_s:
                    reached = False
                    break
                continue
            item = snapshot.get(target.joint)
            if not item or item.get("raw_position") is None:
                reached = False
                break
            if abs(int(item["raw_position"]) - target.target_pulse) > tolerance:
                reached = False
                break
        if reached:
            self._settle_count += 1
        else:
            self._settle_count = 0
        if self._settle_count >= self.config.safety.settle_samples:
            self._state = "completed"
            self._verification = (
                "timed"
                if all(target.completion == "timed" for target in plan.targets)
                else "cache"
            )
            self._finished_at = utc_now_iso()
            self._active = None

    @_guarded
    def mark_verified(
        self,
        actual: dict[str, Any] | None,
        now: float,
        *,
        read_ok: bool,
        reason: str | None = None,
        tolerance_pulses: int | None = None,
    ) -> dict[str, Any]:
        """Record a real ``GetBusServoState`` read for the active command.

        An arm command is ``completed`` only when every position target is
        within ``tolerance_pulses`` of the actual register. A gripper-only
        command keeps its timed completion and records the register as
        evidence. A failed or unsafe read makes an arm command ``unknown``.
        """
        snapshot = _measured_snapshot(actual)
        if snapshot:
            self._actual = snapshot
            self._actual_at = utc_now_iso()
        plan = self._active
        if plan is None:
            return self.status()
        timed_only = all(target.completion == "timed" for target in plan.targets)
        if timed_only:
            self._state = "completed"
            self._verification = "timed" if not read_ok else "bus"
            self._verification_reason = reason
            self._finished_at = utc_now_iso()
            self._active = None
            return self.status()
        if not read_ok or not snapshot:
            self._state = "unknown"
            self._verification = "failed"
            self._verification_reason = reason or "bus read failed"
            self._finished_at = utc_now_iso()
            self._active = None
            return self.status()
        tolerance = (
            self.config.safety.verification_tolerance_pulses
            if tolerance_pulses is None
            else int(tolerance_pulses)
        )
        deviations: dict[str, int] = {}
        reached = True
        for target in plan.targets:
            item = snapshot.get(target.joint)
            if not item or item.get("raw_position") is None:
                reached = False
                deviations[target.joint] = 0
                continue
            delta = int(item["raw_position"]) - target.target_pulse
            deviations[target.joint] = delta
            if abs(delta) > tolerance:
                reached = False
        self._verification = "bus"
        self._verification_reason = None
        if reached:
            self._state = "completed"
        else:
            self._state = "unknown"
            self._verification = "failed"
            self._verification_reason = (
                "actual registers outside +/-{} pulses: {}".format(
                    tolerance,
                    ", ".join(
                        f"{joint}={delta:+d}"
                        for joint, delta in sorted(deviations.items())
                    ),
                )
            )
        self._finished_at = utc_now_iso()
        self._active = None
        return self.status()

    @_guarded
    def mark_stopped(
        self,
        measured: dict[str, Any] | None,
        now: float,
    ) -> dict[str, Any]:
        snapshot = _measured_snapshot(measured)
        if snapshot:
            self._last_measured = snapshot
        if self._active is not None:
            self._last_targets = list(self._active.targets)
            self._last_duration_s = self._active.duration_s
            self._active = None
            self._state = "stopped"
            self._finished_at = utc_now_iso()
        self._last_stop = {
            "requested_at": utc_now_iso(),
            "measured": self._last_measured,
            "detail": "hold command published; physical stop not confirmed",
        }
        return self.status()

    @_guarded
    def status(self) -> dict[str, Any]:
        active = self._active
        state = self._state
        if active is not None and state == "executing":
            pass
        elif active is None and state == "executing":
            state = "unknown"
        payload: dict[str, Any] = {
            "state": state,
            "mode": self.config.safety.mode,
            "max_velocity_rad_s": self.config.safety.max_velocity_rad_s,
            "command_id": (
                active.command_id if active else self._last_command_id
            ),
            "duration_s": (
                active.duration_s if active else self._last_duration_s
            ),
            "targets": [
                {
                    "joint": target.joint,
                    "servo_id": target.servo_id,
                    "target_pulse": target.target_pulse,
                    "target_rad": target.target_rad,
                    "completion": target.completion,
                }
                for target in (active.targets if active else self._last_targets)
            ],
            "measured": self._last_measured,
            "measured_truth": "commanded",
            "actual": self._actual,
            "actual_at": self._actual_at,
            "verification": self._verification,
            "verification_reason": self._verification_reason,
            "verification_tolerance_pulses": (
                self.config.safety.verification_tolerance_pulses
            ),
            "tolerance_pulses": self.config.safety.tolerance_pulses,
            "started_at": (
                active.started_wall if active else self._last_started_wall
            ),
            "finished_at": self._finished_at,
            "last_stop": self._last_stop,
            "gripper": (
                {
                    "joint": self.config.gripper.joint,
                    "servo_id": self.config.gripper.servo_id,
                    "min_pulse": self.config.gripper.min_pulse,
                    "max_pulse": self.config.gripper.max_pulse,
                    "min_rad": self.config.gripper.min_rad,
                    "max_rad": self.config.gripper.max_rad,
                    "speed_raw_per_s": self.config.gripper.speed_raw_per_s,
                    "completion": self.config.gripper.completion,
                }
                if self.config.gripper is not None
                else None
            ),
        }
        if active is not None:
            payload["command_id"] = active.command_id
            payload["deadline_monotonic"] = active.deadline
        return payload


def _measured_snapshot(measured: dict[str, Any] | None) -> dict[str, dict[str, Any]]:
    if not isinstance(measured, dict):
        return {}
    snapshot: dict[str, dict[str, Any]] = {}
    for joint, item in measured.items():
        if not isinstance(item, dict):
            continue
        raw = item.get("raw_position")
        if isinstance(raw, bool) or not isinstance(raw, (int, float)):
            continue
        snapshot[joint] = {
            "raw_position": int(raw),
            "sample_age_ms": item.get("sample_age_ms"),
            "received_at": item.get("received_at"),
        }
    return snapshot
