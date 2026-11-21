"""G8 task plan: the validated floor-cube sequence, step by step.

The plan is built from a measured grasp point and the deep-family fit, then
each pose is validated (limits, velocity, FK gate) **from the previous step's
target**. The UI executes the steps through the ordinary motion
preview/execute endpoints, so the gateway's bus verification applies to every
arm move. The plan is only a proposal: nothing here dispatches motion.
"""

from __future__ import annotations

import math
from typing import Any

from .mapping import MappingConfig
from .motion import EffectiveMotionLimits
from .models import MotionTarget
from .takeover import TakeoverConfig
from ..safety.pose_gate import PoseGate
from ..safety.validation import validate_targets

ARM_JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5")


def _pose_duration(
    start: dict[str, float],
    target: dict[str, float],
    *,
    vmax: float,
    floor: float,
    ceiling: float,
) -> float:
    delta = max(
        abs(float(target[name]) - float(start.get(name, 0.0)))
        for name in ARM_JOINTS
    )
    value = math.ceil((delta / vmax) * 10.0) / 10.0
    return min(ceiling, max(floor, value))


def _targets(
    joints: dict[str, float], duration: float
) -> list[dict[str, Any]]:
    return [
        {
            "joint": name,
            "target_rad": float(joints[name]),
            "duration_s": float(duration),
        }
        for name in ARM_JOINTS
    ]


def _normalized_map(normalized: list[dict[str, Any]]) -> dict[str, Any]:
    return {item["joint"]: item for item in normalized}


def build_task_plan(
    *,
    takeover: TakeoverConfig,
    mapping: MappingConfig,
    limits: EffectiveMotionLimits,
    gate: PoseGate,
    fit_result: dict[str, Any],
    point: list[float],
    start_joints: dict[str, float],
    start_gripper_rad: float,
    source: str,
    locate_summary: dict[str, Any] | None = None,
    hand_eye_summary: dict[str, Any] | None = None,
    simulated: bool = False,
) -> dict[str, Any]:
    """Build and validate the full task plan; return the public plan."""
    task = takeover.task
    if task is None:
        raise ValueError("the takeover contract has no task block")
    view = takeover.preset(task.view_preset)
    lift = takeover.preset(task.lift_preset)
    if view is None or lift is None:
        raise ValueError("task presets are missing")
    fit = {
        name: float(fit_result["joints"][index])
        for index, name in enumerate(ARM_JOINTS)
    }
    approach = {
        "joint1": fit["joint1"],
        "joint2": task.grasp_j2,
        "joint3": task.grasp_j3,
        "joint4": 0.0,
        "joint5": task.grasp_j5,
    }
    sign = 1.0 if fit["joint4"] >= approach["joint4"] else -1.0
    pre_grasp = dict(fit)
    pre_grasp["joint4"] = fit["joint4"] - sign * task.pre_grasp_offset_rad
    view_joints = dict(view.joints)
    lift_joints = dict(lift.joints)

    errors: list[dict[str, Any]] = []
    warnings: list[dict[str, Any]] = []
    steps: list[dict[str, Any]] = []

    def fail(label: str, code: str, message: str, joint: str | None = None):
        errors.append(
            {
                "label": label,
                "code": code,
                "message": message,
                "joint": joint,
            }
        )

    def pose_step(
        label: str,
        start: dict[str, float],
        target: dict[str, float],
    ) -> dict[str, float]:
        duration = _pose_duration(
            start,
            target,
            vmax=float(limits.max_velocity_rad_s or 0.5),
            floor=limits.min_duration_s,
            ceiling=limits.max_duration_s,
        )
        targets = _targets(target, duration)
        outcome = validate_targets(
            mapping,
            [MotionTarget(item["joint"], item["target_rad"], duration) for item in targets],
            current_targets=start,
            max_step_rad=limits.max_step_rad,
            min_duration_s=limits.min_duration_s,
            max_duration_s=limits.max_duration_s,
            single_joint_only=False,
            scope="pose",
            max_velocity_rad_s=limits.max_velocity_rad_s,
            gripper=takeover.gripper,
        )
        check = gate.check(start, target)
        if not outcome.valid:
            for issue in outcome.errors:
                fail(label, issue.code, issue.message, issue.joint)
        if not check.ok:
            for violation in check.violations:
                fail(
                    label,
                    violation.code,
                    violation.message,
                    violation.joint,
                )
        if not check.start_compliant:
            warnings.append(
                {
                    "label": label,
                    "code": "START_OUTSIDE_GATE",
                    "message": "the planned start is outside the FK gate; it "
                    "is exempt so the arm can be commanded back",
                }
            )
        normalized = _normalized_map(outcome.normalized)
        step = {
            "kind": "pose",
            "label": label,
            "joints": {name: round(value, 6) for name, value in target.items()},
            "from_joints": {name: round(value, 6) for name, value in start.items()},
            "targets": targets,
            "duration_s": duration,
            "target_raw": {
                name: normalized.get(name, {}).get("target_raw")
                for name in ARM_JOINTS
            },
            "pose_check": check.to_public_dict(),
            "status": "pending",
        }
        steps.append(step)
        return target

    def grip_step(
        label: str,
        start_rad: float,
        target_rad: float,
        duration: float,
        *,
        configured_duration: bool = False,
    ) -> float:
        if not configured_duration:
            delta_raw = abs(
                takeover.gripper.rad_to_raw(target_rad)
                - takeover.gripper.rad_to_raw(start_rad)
            )
            duration = takeover.gripper.suggested_duration_s(delta_raw)
        targets = [
            {
                "joint": takeover.gripper.joint,
                "target_rad": float(target_rad),
                "duration_s": float(duration),
            }
        ]
        outcome = validate_targets(
            mapping,
            [
                MotionTarget(
                    takeover.gripper.joint, float(target_rad), float(duration)
                )
            ],
            current_targets={takeover.gripper.joint: float(start_rad)},
            max_step_rad=limits.max_step_rad,
            min_duration_s=limits.min_duration_s,
            max_duration_s=limits.max_duration_s,
            single_joint_only=False,
            scope="pose",
            max_velocity_rad_s=limits.max_velocity_rad_s,
            gripper=takeover.gripper,
        )
        if not outcome.valid:
            for issue in outcome.errors:
                fail(label, issue.code, issue.message, issue.joint)
        normalized = _normalized_map(outcome.normalized)
        step = {
            "kind": "grip",
            "label": label,
            "target_rad": round(float(target_rad), 6),
            "from_rad": round(float(start_rad), 6),
            "targets": targets,
            "duration_s": float(duration),
            "target_raw": normalized.get(takeover.gripper.joint, {}).get(
                "target_raw"
            ),
            "status": "pending",
        }
        steps.append(step)
        return float(target_rad)

    current = {name: float(start_joints.get(name, 0.0)) for name in ARM_JOINTS}
    gripper_now = float(start_gripper_rad)

    gripper_now = grip_step(
        "open_gripper",
        gripper_now,
        takeover.gripper.open_rad,
        0.0,
    )
    current = pose_step("approach", current, approach)
    current = pose_step("pre_grasp", current, pre_grasp)
    current = pose_step("grasp", current, fit)
    gripper_now = grip_step(
        "close_pre",
        gripper_now,
        task.preclose_rad,
        task.grip_preclose_duration_s,
        configured_duration=True,
    )
    gripper_now = grip_step(
        "close_grip",
        gripper_now,
        task.grip_rad,
        task.grip_duration_s,
        configured_duration=True,
    )
    current = pose_step("lift", current, lift_joints)
    current = pose_step("view_verify", current, view_joints)
    steps.append(
        {
            "kind": "verify",
            "label": "grip_verify",
            "status": "pending",
            "detail": "fetch a cloud at the view and check the cube box",
        }
    )
    current = pose_step("putback_pre_grasp", current, pre_grasp)
    current = pose_step("putback_grasp", current, fit)
    gripper_now = grip_step(
        "release_open",
        gripper_now,
        takeover.gripper.open_rad,
        task.release_open_duration_s,
        configured_duration=True,
    )
    pose_step("return_view", current, view_joints)
    del gripper_now

    if simulated:
        warnings.append(
            {
                "label": "task",
                "code": "SIMULATED_LOCATE",
                "message": "mock mode uses a fixed cube point; no camera was read",
            }
        )
    if source == "commanded":
        warnings.append(
            {
                "label": "task",
                "code": "START_ECHO",
                "message": "the locate used the command cache (up to ~8 pulses "
                "from the actual); each move is bus-verified",
            }
        )
    return {
        "schema_version": 1,
        "valid": not errors,
        "errors": errors,
        "warnings": warnings,
        "simulated": simulated,
        "point": [round(float(value), 6) for value in point],
        "target": [
            round(float(value) + float(task.correction_mm[index]) / 1000.0, 6)
            for index, value in enumerate(point)
        ],
        "correction_mm": list(task.correction_mm),
        "fit": fit_result,
        "grasp_joints": {name: round(value, 6) for name, value in fit.items()},
        "approach_joints": approach,
        "pre_grasp_joints": pre_grasp,
        "view_joints": view_joints,
        "lift_joints": lift_joints,
        "view_preset": task.view_preset,
        "lift_preset": task.lift_preset,
        "start_joints": {name: round(value, 6) for name, value in start_joints.items()},
        "start_gripper_rad": round(float(start_gripper_rad), 6),
        "source": source,
        "locate": locate_summary or {},
        "hand_eye": hand_eye_summary or {},
        "steps": steps,
    }
