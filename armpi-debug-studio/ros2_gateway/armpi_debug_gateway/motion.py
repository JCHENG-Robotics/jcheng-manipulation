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
class ProfileSafety:
    """Bounds for one dense S-curve command (G8 fast task path).

    The backend builds the waypoints; the gateway checks every segment again,
    so a backend bug still cannot publish a segment outside the measured
    envelope. The velocity is the peak joint velocity of the profile, not a
    per-command average.
    """

    max_velocity_rad_s: float = 4.0
    max_accel_rad_s2: float = 20.0
    min_segment_s: float = 0.025
    max_segment_s: float = 0.5
    max_points: int = 400
    max_total_duration_s: float = 6.0
    start_tolerance_pulses: int = 20
    gripper_speed_raw_per_s: float = 900.0
    gripper_min_duration_s: float = 0.25
    gripper_settle_s: float = 0.6


@dataclass(frozen=True)
class MotionConfig:
    schema_version: int
    mapping_version: str
    radians_per_tick: float
    joints: dict[str, JointMotionLimit]
    safety: MotionSafety
    gripper: GripperLimit | None = None
    chassis: ChassisConfig | None = None
    profile: ProfileSafety | None = None


@dataclass
class MotionTarget:
    joint: str
    servo_id: int
    target_pulse: int
    target_rad: float
    completion: str = "position"


@dataclass
class ProfilePoint:
    """One segment: publish these targets with ``duration_s``, then wait."""

    duration_s: float
    targets: list[MotionTarget]


@dataclass
class MotionPlan:
    command_id: str
    duration_s: float
    targets: list[MotionTarget]
    started_at: float
    deadline: float
    started_wall: str
    mode: str = "step"
    points: list[ProfilePoint] | None = None
    profile: ProfileSafety | None = None
    fast: bool = False
    settle_s: float = 0.0


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
    profile: ProfileSafety | None = None
    profile_raw = safety_raw.get("profile")
    if profile_raw is not None:
        _require(isinstance(profile_raw, dict), "safety.profile must be an object")
        _require(mode == "pose", "safety.profile requires pose mode")
        profile = ProfileSafety(
            max_velocity_rad_s=float(
                profile_raw.get("max_velocity_rad_s", 4.0)
            ),
            max_accel_rad_s2=float(profile_raw.get("max_accel_rad_s2", 20.0)),
            min_segment_s=float(profile_raw.get("min_segment_s", 0.025)),
            max_segment_s=float(profile_raw.get("max_segment_s", 0.5)),
            max_points=int(profile_raw.get("max_points", 400)),
            max_total_duration_s=float(
                profile_raw.get("max_total_duration_s", 6.0)
            ),
            start_tolerance_pulses=int(
                profile_raw.get("start_tolerance_pulses", 20)
            ),
            gripper_speed_raw_per_s=float(
                profile_raw.get("gripper_speed_raw_per_s", 900.0)
            ),
            gripper_min_duration_s=float(
                profile_raw.get("gripper_min_duration_s", 0.25)
            ),
            gripper_settle_s=float(profile_raw.get("gripper_settle_s", 0.6)),
        )
        _require(
            0.0 < profile.max_velocity_rad_s <= 4.0,
            "safety.profile.max_velocity_rad_s must be in (0, 4.0]",
        )
        _require(
            0.0 < profile.max_accel_rad_s2 <= 20.0,
            "safety.profile.max_accel_rad_s2 must be in (0, 20.0]",
        )
        _require(
            0.02 <= profile.min_segment_s < profile.max_segment_s <= 1.0,
            "safety.profile segment range is invalid",
        )
        _require(
            1 <= profile.max_points <= 1000,
            "safety.profile.max_points must be in [1, 1000]",
        )
        _require(
            0.0 < profile.max_total_duration_s
            <= safety.max_duration_s + 1e-9,
            "safety.profile.max_total_duration_s exceeds the pose ceiling",
        )
        _require(
            0 <= profile.start_tolerance_pulses <= 50,
            "safety.profile.start_tolerance_pulses must be in [0, 50]",
        )
        _require(
            100.0 <= profile.gripper_speed_raw_per_s <= 1000.0,
            "safety.profile.gripper_speed_raw_per_s must be in [100, 1000]",
        )
        _require(
            0.1 <= profile.gripper_min_duration_s <= 1.0,
            "safety.profile.gripper_min_duration_s must be in [0.1, 1.0]",
        )
        _require(
            0.1 <= profile.gripper_settle_s <= 2.0,
            "safety.profile.gripper_settle_s must be in [0.1, 2.0]",
        )
    return MotionConfig(
        schema_version=MOTION_SCHEMA_VERSION,
        mapping_version=mapping_version,
        radians_per_tick=radians_per_tick,
        joints=joints,
        safety=safety,
        gripper=gripper,
        chassis=chassis_config_from_dict(raw.get("chassis")),
        profile=profile,
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
        # External /safety_stop latch. Once set, no new command is accepted
        # until armpi_safety clears it.
        self._safety_stop = False
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
        if self._safety_stop:
            raise MotionRejected(
                "SAFETY_STOP",
                "the safety supervisor has latched a stop; reset it before motion",
                http_status=409,
            )
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

        profile_raw = payload.get("profile")
        fast = bool(payload.get("fast", False))
        profile_config = self.config.profile
        if profile_raw is not None and not isinstance(profile_raw, dict):
            raise MotionRejected(
                "MOTION_BAD_REQUEST", "profile must be an object"
            )
        if profile_raw is not None and profile_config is None:
            raise MotionRejected(
                "MOTION_PROFILE_DISABLED",
                "this gateway has no safety.profile block",
                http_status=403,
            )
        if fast and profile_config is None:
            raise MotionRejected(
                "MOTION_FAST_DISABLED",
                "this gateway has no safety.profile block",
                http_status=403,
            )

        duration = payload.get("duration_s")
        if not _is_number(duration):
            raise MotionRejected("MOTION_BAD_REQUEST", "duration_s must be finite")
        duration = float(duration)
        safety = self.config.safety
        floor = safety.min_duration_s
        if profile_raw is not None:
            # A profile is validated segment by segment below; its total can
            # be shorter than the single-command duration floor.
            floor = 0.0
        elif fast and profile_config is not None:
            floor = profile_config.gripper_min_duration_s
        if not floor <= duration <= safety.max_duration_s:
            raise MotionRejected(
                "MOTION_BAD_REQUEST",
                f"duration_s must be within [{floor}, "
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
        if profile_raw is not None and safety.mode != "pose":
            raise MotionRejected(
                "MOTION_BAD_REQUEST", "profiles require pose mode"
            )
        if profile_raw is not None and (
            gripper is not None
            and any(
                str(item.get("joint") or "").strip() == gripper.joint
                for item in raw_joints
                if isinstance(item, dict)
            )
        ):
            raise MotionRejected(
                "MOTION_BAD_REQUEST",
                "a profile cannot carry a gripper target",
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
                    self._validate_gripper(
                        raw, measured, now, duration, fast=fast
                    )
                )
                continue
            if joint not in self.config.joints:
                raise MotionRejected(
                    "MOTION_BAD_REQUEST", f"unknown joint: {joint!r}"
                )
            if joint in seen:
                raise MotionRejected("MOTION_BAD_REQUEST", f"duplicate joint: {joint}")
            seen.add(joint)
            if profile_raw is not None:
                targets.append(
                    self._validate_endpoint(joint, raw, measured, now)
                )
            else:
                targets.append(
                    self._validate_arm(joint, raw, measured, now, duration)
                )

        points: list[ProfilePoint] | None = None
        if profile_raw is not None:
            points, duration = self._validate_profile(
                profile_raw, targets, measured, now
            )

        settle_s = 0.0
        if safety.verification:
            settle_s = safety.verification_settle_s
            if (
                fast
                and profile_config is not None
                and any(target.completion == "timed" for target in targets)
            ):
                # The fast task uses raw bus-servo gripper durations; the
                # measured queue delay is ~0.1 s, so 0.6 s records the settled
                # register instead of the slow path's 1.2 s.
                settle_s = max(settle_s, profile_config.gripper_settle_s)
            elif any(target.completion == "timed" for target in targets):
                settle_s = max(settle_s, 1.2)
        return MotionPlan(
            command_id=command_id,
            duration_s=duration,
            targets=targets,
            started_at=now,
            deadline=(
                now + duration + settle_s + safety.timeout_margin_s
            ),
            started_wall=utc_now_iso(),
            mode=safety.mode,
            points=points,
            profile=profile_config if points is not None else None,
            fast=fast,
            settle_s=settle_s,
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
        *,
        fast: bool = False,
    ) -> MotionTarget:
        gripper = self.config.gripper
        safety = self.config.safety
        profile = self.config.profile
        if gripper is None:  # guarded by the caller
            raise MotionRejected("MOTION_BAD_REQUEST", "no gripper contract")
        floor = gripper.min_duration_s
        speed = gripper.speed_raw_per_s
        if fast and profile is not None:
            floor = profile.gripper_min_duration_s
            speed = max(speed, profile.gripper_speed_raw_per_s)
        if not floor <= duration <= gripper.max_duration_s:
            raise MotionRejected(
                "MOTION_BAD_REQUEST",
                f"{gripper.joint}: duration must be within "
                f"[{floor}, {gripper.max_duration_s}] s",
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
        if velocity > speed + 1e-9:
            raise MotionRejected(
                "MOTION_VELOCITY_EXCEEDED",
                f"{gripper.joint}: {velocity:.1f} raw/s exceeds the measured "
                f"{speed:.0f} raw/s tracking speed",
                http_status=409,
            )
        return MotionTarget(
            joint=gripper.joint,
            servo_id=gripper.servo_id,
            target_pulse=target_pulse,
            target_rad=float(target_rad),
            completion=gripper.completion,
        )

    def _validate_endpoint(
        self,
        joint: str,
        raw: dict[str, Any],
        measured: dict[str, Any],
        now: float,
    ) -> MotionTarget:
        """Validate one profile endpoint without a step/velocity check.

        Every segment of the profile is checked separately; the endpoint only
        has to be inside the joint limits, be pulse-consistent, and have a
        usable measured baseline for the profile's first segment.
        """
        limit = self.config.joints[joint]
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
        _measured_pulse, age_ms = _measured_item(measured, joint, now)
        if age_ms > self.config.safety.feedback_max_age_ms:
            raise MotionRejected(
                "MOTION_STALE_FEEDBACK",
                f"{joint}: measured sample is {age_ms:.0f} ms old "
                f"(limit {self.config.safety.feedback_max_age_ms} ms)",
                http_status=409,
            )
        return MotionTarget(
            joint=joint,
            servo_id=limit.servo_id,
            target_pulse=target_pulse,
            target_rad=float(target_rad),
        )

    def _validate_profile(
        self,
        raw: dict[str, Any],
        endpoint: list[MotionTarget],
        measured: dict[str, Any],
        now: float,
    ) -> tuple[list[ProfilePoint], float]:
        """Validate every dense waypoint; return the segments and total time."""
        config = self.config.profile
        if config is None:  # guarded by the caller
            raise MotionRejected("MOTION_PROFILE_DISABLED", "no profile contract")
        points_raw = raw.get("points")
        if not isinstance(points_raw, list) or not points_raw:
            raise MotionRejected(
                "MOTION_BAD_REQUEST", "profile.points must be a non-empty list"
            )
        if len(points_raw) > config.max_points:
            raise MotionRejected(
                "MOTION_BAD_REQUEST",
                f"profile has {len(points_raw)} points; the cap is "
                f"{config.max_points}",
            )
        measured_snapshot = _measured_snapshot(measured)
        previous: dict[str, int] | None = None
        previous_velocity: dict[str, float] | None = None
        previous_duration = 0.0
        total = 0.0
        points: list[ProfilePoint] = []
        for index, item in enumerate(points_raw):
            if not isinstance(item, dict):
                raise MotionRejected(
                    "MOTION_BAD_REQUEST", f"profile.points[{index}] must be an object"
                )
            segment = item.get("duration_s")
            if not _is_number(segment):
                raise MotionRejected(
                    "MOTION_BAD_REQUEST",
                    f"profile.points[{index}].duration_s must be finite",
                )
            segment = float(segment)
            if not config.min_segment_s <= segment <= config.max_segment_s:
                raise MotionRejected(
                    "MOTION_BAD_REQUEST",
                    f"profile.points[{index}].duration_s must be within "
                    f"[{config.min_segment_s}, {config.max_segment_s}] s",
                )
            raw_joints = item.get("joints")
            if not isinstance(raw_joints, list) or not raw_joints:
                raise MotionRejected(
                    "MOTION_BAD_REQUEST",
                    f"profile.points[{index}].joints must be a non-empty list",
                )
            names = [
                str(entry.get("joint") or "").strip()
                for entry in raw_joints
                if isinstance(entry, dict)
            ]
            if set(names) != set(ARM_JOINTS) or len(names) != len(ARM_JOINTS):
                raise MotionRejected(
                    "MOTION_BAD_REQUEST",
                    f"profile.points[{index}] must carry exactly the five arm "
                    f"joints; got {sorted(set(names))}",
                )
            targets: list[MotionTarget] = []
            for entry in raw_joints:
                if not isinstance(entry, dict):
                    raise MotionRejected(
                        "MOTION_BAD_REQUEST",
                        f"profile.points[{index}] joints must be objects",
                    )
                joint = str(entry.get("joint") or "").strip()
                targets.append(
                    self._validate_endpoint(joint, entry, measured, now)
                )
            total += segment
            if total > config.max_total_duration_s + 1e-9:
                raise MotionRejected(
                    "MOTION_BAD_REQUEST",
                    f"profile total {total:.3f} s exceeds "
                    f"{config.max_total_duration_s:.3f} s",
                )
            current = {target.joint: target.target_pulse for target in targets}
            velocity: dict[str, float] = {}
            for target in targets:
                limit = self.config.joints[target.joint]
                baseline = (
                    previous[target.joint]
                    if previous is not None
                    else measured_snapshot.get(target.joint, {}).get(
                        "raw_position"
                    )
                )
                if baseline is None:
                    raise MotionRejected(
                        "MOTION_STALE_FEEDBACK",
                        f"{target.joint}: no measured servo position",
                        http_status=409,
                    )
                step = abs(target.target_pulse - int(baseline))
                velocity[target.joint] = (
                    step * limit.radians_per_tick / segment
                )
                # The first segment starts from the command cache; a fresh
                # bus actual can differ by a few pulses, so give that segment
                # the same pulse tolerance as the start check.
                velocity_cap = config.max_velocity_rad_s
                velocity_cap += (
                    config.start_tolerance_pulses
                    * limit.radians_per_tick
                    / segment
                    if previous is None
                    else 0.15
                )
                if velocity[target.joint] > velocity_cap:
                    raise MotionRejected(
                        "MOTION_PROFILE_VELOCITY",
                        f"{target.joint}: segment {index} is "
                        f"{velocity[target.joint]:.3f} rad/s (cap "
                        f"{velocity_cap:.3f})",
                        http_status=409,
                    )
                if previous is None and step > (
                    config.max_velocity_rad_s * segment / limit.radians_per_tick
                    + config.start_tolerance_pulses
                ):
                    raise MotionRejected(
                        "MOTION_PROFILE_START",
                        f"{target.joint}: the first segment starts {step} pulses "
                        "from the measured cache; the profile must start at the "
                        "current pose",
                        http_status=409,
                    )
                if previous_velocity is not None and index >= 2:
                    accel = abs(
                        velocity[target.joint] - previous_velocity[target.joint]
                    ) / ((segment + previous_duration) / 2.0)
                    if accel > config.max_accel_rad_s2 * 1.25 + 0.5:
                        raise MotionRejected(
                            "MOTION_PROFILE_ACCEL",
                            f"{target.joint}: segment {index} changes velocity at "
                            f"{accel:.1f} rad/s^2 (cap "
                            f"{config.max_accel_rad_s2:.1f})",
                            http_status=409,
                        )
            previous = current
            previous_velocity = velocity
            previous_duration = segment
            points.append(ProfilePoint(duration_s=segment, targets=targets))

        endpoint_map = {target.joint: target.target_pulse for target in endpoint}
        if previous != endpoint_map:
            raise MotionRejected(
                "MOTION_BAD_REQUEST",
                "the profile's last point must equal the requested endpoint",
            )
        return points, total

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
    def set_safety_stop(self, active: bool) -> None:
        """Latch or clear the external ``/safety_stop`` input."""
        self._safety_stop = bool(active)

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
            "safety_stop": self._safety_stop,
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
            "profile": (
                {
                    "points": len(active.points),
                    "max_velocity_rad_s": active.profile.max_velocity_rad_s,
                    "max_accel_rad_s2": active.profile.max_accel_rad_s2,
                }
                if active is not None
                and active.points is not None
                and active.profile is not None
                else None
            ),
            "fast": bool(active.fast) if active is not None else False,
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
