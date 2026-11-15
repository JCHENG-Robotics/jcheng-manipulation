"""Backend-side target validation.

Every motion request is validated here, before any adapter dispatch. The
vendor stack silently clamps out-of-range values; this layer rejects them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Iterable

from ..domain.enums import ErrorCode
from ..domain.mapping import JointMapping, MappingConfig
from ..domain.models import MotionTarget, ValidationIssue


@dataclass
class ValidationOutcome:
    errors: list[ValidationIssue] = field(default_factory=list)
    warnings: list[ValidationIssue] = field(default_factory=list)
    normalized: list[dict] = field(default_factory=list)

    @property
    def valid(self) -> bool:
        return not self.errors


def _issue(code: ErrorCode | str, message: str, joint: str | None = None, **details) -> ValidationIssue:
    code_value = code.value if isinstance(code, ErrorCode) else str(code)
    return ValidationIssue(code=code_value, message=message, joint=joint, details=details)


def _raw_bounds(mapping: JointMapping) -> tuple[int, int]:
    return min(mapping.min_raw, mapping.max_raw), max(mapping.min_raw, mapping.max_raw)


def validate_targets(
    mapping: MappingConfig,
    targets: Iterable[MotionTarget],
    *,
    current_targets: dict[str, float] | None = None,
) -> ValidationOutcome:
    outcome = ValidationOutcome()
    current = current_targets or {}
    seen: set[str] = set()
    items = list(targets)

    if not items:
        outcome.errors.append(_issue(ErrorCode.NO_TARGETS, "at least one target is required"))
        return outcome

    for target in items:
        name = (target.joint or "").strip()
        if not name:
            outcome.errors.append(_issue(ErrorCode.BAD_REQUEST, "joint name is required"))
            continue
        if name in seen:
            outcome.errors.append(
                _issue(ErrorCode.DUPLICATE_JOINT, f"joint {name} appears more than once", name)
            )
            continue
        seen.add(name)

        if name in {item.get("name") for item in mapping.excluded}:
            outcome.errors.append(
                _issue(
                    ErrorCode.GRIPPER_NOT_IN_G1,
                    f"joint {name} is excluded from G1",
                    name,
                )
            )
            continue

        joint = mapping.get_joint(name)
        if joint is None:
            outcome.errors.append(
                _issue(ErrorCode.UNKNOWN_JOINT, f"unknown joint: {name}", name)
            )
            continue

        value = target.target_rad
        if not isinstance(value, (int, float)) or isinstance(value, bool) or not math.isfinite(float(value)):
            outcome.errors.append(
                _issue(
                    ErrorCode.NON_FINITE_TARGET,
                    f"{name}: target must be a finite number",
                    name,
                    value=repr(value),
                )
            )
            continue
        target_rad = float(value)

        if target_rad < joint.min_rad or target_rad > joint.max_rad:
            outcome.errors.append(
                _issue(
                    ErrorCode.OUT_OF_LIMIT,
                    f"{name}: {target_rad:.6f} rad outside provisional limit "
                    f"[{joint.min_rad:.6f}, {joint.max_rad:.6f}]",
                    name,
                    target_rad=target_rad,
                    min_rad=joint.min_rad,
                    max_rad=joint.max_rad,
                )
            )
            continue

        from_rad = float(current.get(name, 0.0))
        delta_rad = target_rad - from_rad
        if abs(delta_rad) > mapping.safety.max_step_rad + 1e-12:
            outcome.errors.append(
                _issue(
                    ErrorCode.STEP_EXCEEDED,
                    f"{name}: step {delta_rad:.6f} rad exceeds max_step_rad "
                    f"{mapping.safety.max_step_rad:.6f}",
                    name,
                    delta_rad=delta_rad,
                    max_step_rad=mapping.safety.max_step_rad,
                )
            )
            continue

        duration = target.duration_s
        if (
            not isinstance(duration, (int, float))
            or isinstance(duration, bool)
            or not math.isfinite(float(duration))
            or float(duration) < mapping.safety.min_duration_s
            or float(duration) > mapping.safety.max_duration_s
        ):
            outcome.errors.append(
                _issue(
                    ErrorCode.DURATION_RANGE,
                    f"{name}: duration must be within "
                    f"[{mapping.safety.min_duration_s}, {mapping.safety.max_duration_s}] s",
                    name,
                    duration_s=repr(duration),
                )
            )
            continue

        raw_float = joint.rad_to_raw(target_rad)
        raw_int = int(round(raw_float))
        raw_low, raw_high = _raw_bounds(joint)
        if raw_int < raw_low or raw_int > raw_high:
            outcome.errors.append(
                _issue(
                    ErrorCode.OUT_OF_LIMIT,
                    f"{name}: mapped raw {raw_int} outside [{raw_low}, {raw_high}]",
                    name,
                    raw_position=raw_int,
                )
            )
            continue

        if abs(target_rad - from_rad) > mapping.safety.max_step_rad * 0.8:
            outcome.warnings.append(
                _issue(
                    "LARGE_STEP",
                    f"{name}: step is close to the configured maximum",
                    name,
                    delta_rad=delta_rad,
                )
            )
        if abs(target_rad - joint.min_rad) < 0.02 or abs(target_rad - joint.max_rad) < 0.02:
            outcome.warnings.append(
                _issue(
                    "NEAR_LIMIT",
                    f"{name}: target is within 0.02 rad of a provisional limit",
                    name,
                )
            )

        outcome.normalized.append(
            {
                "joint": name,
                "servo_id": joint.servo_id,
                "from_rad": from_rad,
                "target_rad": target_rad,
                "delta_rad": delta_rad,
                "duration_s": float(duration),
                "target_raw": raw_int,
                "limit_min_rad": joint.min_rad,
                "limit_max_rad": joint.max_rad,
                "limit_verified": joint.verified,
                "limit_source": joint.limit_source,
            }
        )

    if not mapping.verified:
        outcome.warnings.append(
            _issue(
                "MAPPING_PROVISIONAL",
                "joint mapping is provisional; limits are not hardware-verified",
            )
        )

    return outcome
