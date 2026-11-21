"""Open-loop chassis pulse guard for the G8 task approach.

The base has four duty-driven motors and **no odometry** (2026-11-18 probe).
A pulse is therefore a bounded, open-loop command: the caller measures the
scene, pulses, waits for the coast, and measures again. This module keeps the
command inside the measured envelope before it reaches the vendor topic:

- ``ros_robot_controller/set_motor_speeds`` (``MotorsSpeedControl``),
- the operator-confirmed sign tables (forward ``[+,-,-,+]``, left ``[+,+,+,+]``),
- the measured scale 48-51 mm/s at duty 10 (5 mm/s per duty; the guard uses
  the conservative 4.9),
- a per-pulse duty cap (20) and wheel cap (30, because a diagonal at duty 20
  commands 28.3 on two wheels),
- a pulse duration window (0.2-3.0 s: below ~0.15 s the base stays in the
  dead band, and the 2026-11-19 trials never used more than 3 s in one go).

No ROS import: the node owns the publisher and the timing.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any

CHASSIS_TOPIC = "/ros_robot_controller/set_motor_speeds"
PUBLISH_HZ = 20.0
MOTOR_IDS = (1, 2, 3, 4)
#: operator-confirmed 2026-11-18, IDs 1..4 = RF, RR, LF, LR
SIGN_X = {1: 1.0, 2: -1.0, 3: -1.0, 4: 1.0}
SIGN_Y = {1: 1.0, 2: 1.0, 3: 1.0, 4: 1.0}
DEFAULT_MAX_DUTY = 20.0
DEFAULT_WHEEL_MAX_DUTY = 30.0
DEFAULT_SPEED_MM_S_PER_DUTY = 4.9
DEFAULT_MIN_DURATION_S = 0.20
DEFAULT_MAX_DURATION_S = 3.0
DISCOVERY_TIMEOUT_S = 3.0


class ChassisConfigError(ValueError):
    """The chassis block is missing or malformed."""


class ChassisRejected(RuntimeError):
    """A chassis pulse failed gateway-side validation."""

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


@dataclass(frozen=True)
class ChassisConfig:
    enabled: bool = True
    max_duty: float = DEFAULT_MAX_DUTY
    wheel_max_duty: float = DEFAULT_WHEEL_MAX_DUTY
    speed_mm_s_per_duty: float = DEFAULT_SPEED_MM_S_PER_DUTY
    min_duration_s: float = DEFAULT_MIN_DURATION_S
    max_duration_s: float = DEFAULT_MAX_DURATION_S

    def to_public_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "max_duty": self.max_duty,
            "wheel_max_duty": self.wheel_max_duty,
            "speed_mm_s_per_duty": self.speed_mm_s_per_duty,
            "min_duration_s": self.min_duration_s,
            "max_duration_s": self.max_duration_s,
        }


@dataclass(frozen=True)
class ChassisPulse:
    vx: float
    vy: float
    duty: float
    duration_s: float
    wheels: dict[int, float]
    nominal_travel_mm: float

    def to_public_dict(self) -> dict[str, Any]:
        return {
            "vx": self.vx,
            "vy": self.vy,
            "duty": self.duty,
            "duration_s": self.duration_s,
            "wheels": {str(mid): value for mid, value in self.wheels.items()},
            "nominal_travel_mm": self.nominal_travel_mm,
        }


def chassis_config_from_dict(raw: Any) -> ChassisConfig:
    """Validate an optional ``chassis`` block; absent means disabled."""
    if raw is None:
        return ChassisConfig(enabled=False)
    if not isinstance(raw, dict):
        raise ChassisConfigError("chassis must be an object")
    enabled = bool(raw.get("enabled", False))
    max_duty = float(raw.get("max_duty", DEFAULT_MAX_DUTY))
    wheel_max = float(raw.get("wheel_max_duty", DEFAULT_WHEEL_MAX_DUTY))
    speed = float(raw.get("speed_mm_s_per_duty", DEFAULT_SPEED_MM_S_PER_DUTY))
    min_duration = float(raw.get("min_duration_s", DEFAULT_MIN_DURATION_S))
    max_duration = float(raw.get("max_duration_s", DEFAULT_MAX_DURATION_S))
    if not 0.0 < max_duty <= DEFAULT_MAX_DUTY:
        raise ChassisConfigError("chassis.max_duty must be in (0, 20]")
    if not max_duty <= wheel_max <= DEFAULT_WHEEL_MAX_DUTY:
        raise ChassisConfigError("chassis.wheel_max_duty must be in [max_duty, 30]")
    if not 0.5 <= speed <= 10.0:
        raise ChassisConfigError("chassis.speed_mm_s_per_duty must be in [0.5, 10]")
    if not 0.0 < min_duration < max_duration <= DEFAULT_MAX_DURATION_S:
        raise ChassisConfigError(
            "chassis durations must satisfy 0 < min < max <= 3.0 s"
        )
    return ChassisConfig(
        enabled=enabled,
        max_duty=max_duty,
        wheel_max_duty=wheel_max,
        speed_mm_s_per_duty=speed,
        min_duration_s=min_duration,
        max_duration_s=max_duration,
    )


def plan_chassis_pulse(
    payload: Any,
    config: ChassisConfig | None,
) -> ChassisPulse:
    """Validate one pulse and return its wheel duties.

    ``payload`` is ``{"vx", "vy", "duty", "duration_s"}``; the direction is
    normalised, so the caller may pass the raw cube displacement.
    """
    if config is None or not config.enabled:
        raise ChassisRejected(
            "CHASSIS_DISABLED",
            "the chassis write path is disabled on this gateway",
            http_status=403,
        )
    if not isinstance(payload, dict):
        raise ChassisRejected("CHASSIS_BAD_REQUEST", "body must be an object")
    try:
        vx = float(payload["vx"])
        vy = float(payload["vy"])
        duty = float(payload["duty"])
        duration = float(payload["duration_s"])
    except (KeyError, TypeError, ValueError) as exc:
        raise ChassisRejected(
            "CHASSIS_BAD_REQUEST",
            "vx, vy, duty and duration_s are required numbers",
        ) from exc
    if not all(math.isfinite(value) for value in (vx, vy, duty, duration)):
        raise ChassisRejected("CHASSIS_BAD_REQUEST", "values must be finite")
    norm = math.hypot(vx, vy)
    if norm < 1e-6:
        raise ChassisRejected("CHASSIS_BAD_REQUEST", "zero direction")
    vx, vy = vx / norm, vy / norm
    if not 0.0 < duty <= config.max_duty:
        raise ChassisRejected(
            "CHASSIS_DUTY",
            f"duty {duty:.2f} outside (0, {config.max_duty:.1f}]",
        )
    if not config.min_duration_s <= duration <= config.max_duration_s:
        raise ChassisRejected(
            "CHASSIS_DURATION",
            f"duration {duration:.2f} s outside "
            f"{config.min_duration_s:.2f}-{config.max_duration_s:.2f} s",
        )
    wheels = {
        mid: duty * (vx * SIGN_X[mid] + vy * SIGN_Y[mid])
        for mid in MOTOR_IDS
    }
    peak = max(abs(value) for value in wheels.values())
    if peak > config.wheel_max_duty:
        raise ChassisRejected(
            "CHASSIS_WHEEL_DUTY",
            f"wheel duty {peak:.1f} above {config.wheel_max_duty:.1f}",
        )
    travel = duty * config.speed_mm_s_per_duty * duration
    return ChassisPulse(
        vx=round(vx, 6),
        vy=round(vy, 6),
        duty=duty,
        duration_s=duration,
        wheels={mid: round(value, 3) for mid, value in wheels.items()},
        nominal_travel_mm=round(travel, 1),
    )
