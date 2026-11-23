#!/usr/bin/env python3
"""Read-only consistency check for ArmPi limits and calibration constants.

The same physical joint limits are consumed by several independent files,
including the phase-2 tool-side ``vendor_servo_map.py``; the constant
low-configuration correction is repeated in several runners and the Studio
takeover config; and the fast-motion defaults / phase-3 view pose are copied
across runners.  A drift between them is a safety defect (the 2026-11-19
joint2 incident) and a reproducibility defect.  This module checks that the
sources agree; it never writes or commands anything.
"""

from __future__ import annotations

import argparse
import ast
import json
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import yaml

JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5")

XACRO = "ros2_ws/src/armpi_ultra_description/urdf/armpi_ultra.xacro"
MOVEIT = "ros2_ws/src/armpi_ultra_moveit_config/config/joint_limits.yaml"
SAFETY = "ros2_ws/src/armpi_safety/config/safety_limits.yaml"
MAPPING = "armpi-debug-studio/configs/joint_mapping.yaml"
TAKEOVER = "armpi-debug-studio/configs/takeover.yaml"
VENDOR = "tools/phase2/vendor_servo_map.py"
FAST_MOTION = "tools/phase2/fast_motion.py"

VIEW_POSE_FILES = (
    "tools/phase3/execution_fixture_run.py",
    "tools/phase3/fast_motion_e2e.py",
    "tools/phase3/gripper_envelope_run.py",
    "tools/phase3/paired_trial_session.py",
    "tools/phase3/visual_displacement_run.py",
)

CANONICAL_ARM_JOINTS = (
    "joint1",
    "joint2",
    "joint3",
    "joint4",
    "joint5",
)
CANONICAL_SERVO_IDS = {
    "joint1": 6,
    "joint2": 5,
    "joint3": 4,
    "joint4": 3,
    "joint5": 2,
}
ARM_JOINT_FILES = (
    "armpi-debug-studio/ros2_gateway/armpi_debug_gateway/motion.py",
    "armpi-debug-studio/ros2_gateway/armpi_debug_gateway/"
    "trajectory_capture.py",
    "armpi-debug-studio/backend/app/domain/takeover.py",
    "armpi-debug-studio/backend/app/domain/task.py",
    "ros2_ws/src/armpi_safety/armpi_safety/policy.py",
    "ros2_ws/src/armpi_grasp_planner/armpi_grasp_planner/evaluator.py",
    "tools/phase2/vendor_servo_map.py",
    "tools/phase2/bus_pulses.py",
    "tools/phase2/chessboard_capture.py",
)
SERVO_ID_FILES = (
    "armpi-debug-studio/ros2_gateway/armpi_debug_gateway/motion.py",
    "armpi-debug-studio/ros2_gateway/armpi_debug_gateway/"
    "trajectory_capture.py",
    "tools/phase2/vendor_servo_map.py",
    "tools/phase2/bus_pulses.py",
    "tools/phase2/chessboard_capture.py",
)
GRIPPER_ID_FILES = (
    "armpi-debug-studio/ros2_gateway/armpi_debug_gateway/motion.py",
    "armpi-debug-studio/backend/app/domain/takeover.py",
)

CORRECTION_FILES = (
    "tools/phase2/low_config_model_fit.py",
    "tools/phase3/execution_fixture_run.py",
    "tools/phase3/fast_motion_e2e.py",
    "tools/phase3/gripper_envelope_run.py",
    "tools/phase3/paired_trial_session.py",
)


@dataclass
class Report:
    limits: dict[str, dict[str, tuple[float, float]]] = field(
        default_factory=dict
    )
    corrections: dict[str, tuple[float, float, float]] = field(
        default_factory=dict
    )
    fast_motion: dict[str, float] = field(default_factory=dict)
    view_poses: dict[str, tuple[float, ...]] = field(default_factory=dict)
    arm_joints: dict[str, tuple[str, ...]] = field(default_factory=dict)
    servo_ids: dict[str, dict[str, int]] = field(default_factory=dict)
    gripper_servo_ids: dict[str, int] = field(default_factory=dict)
    profiles: dict[str, dict[str, Any]] = field(default_factory=dict)
    issues: list[str] = field(default_factory=list)


def _float(value: Any, where: str) -> float:
    try:
        return float(value)
    except (TypeError, ValueError) as exc:
        raise ValueError(f"{where}: not a number: {value!r}") from exc


def xacro_limits(path: Path) -> dict[str, tuple[float, float]]:
    """Parse the five numeric <limit> blocks from the ArmPi xacro."""
    text = path.read_text(encoding="utf-8")
    limits: dict[str, tuple[float, float]] = {}
    for joint in JOINTS:
        block = re.search(
            rf'<joint\s+name="{re.escape(joint)}".*?</joint>',
            text,
            re.S,
        )
        if block is None:
            raise ValueError(f"{path}: joint block not found: {joint}")
        limit = re.search(r"<limit\b[^>]*>", block.group(0))
        if limit is None:
            raise ValueError(f"{path}: <limit> not found: {joint}")
        attrs = dict(re.findall(r'(\w+)="([^"]*)"', limit.group(0)))
        limits[joint] = (
            _float(attrs.get("lower"), f"{joint}.lower"),
            _float(attrs.get("upper"), f"{joint}.upper"),
        )
    return limits


def moveit_limits(path: Path) -> dict[str, tuple[float, float]]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    raw = data.get("joint_limits") or {}
    limits: dict[str, tuple[float, float]] = {}
    for joint in JOINTS:
        entry = raw.get(joint) or {}
        limits[joint] = (
            _float(entry.get("min_position"), f"{joint}.min_position"),
            _float(entry.get("max_position"), f"{joint}.max_position"),
        )
    return limits


def safety_limits(path: Path) -> dict[str, tuple[float, float]]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    raw = data.get("joint_limits") or {}
    limits: dict[str, tuple[float, float]] = {}
    for joint in JOINTS:
        entry = raw.get(joint) or {}
        limits[joint] = (
            _float(entry.get("lower"), f"{joint}.lower"),
            _float(entry.get("upper"), f"{joint}.upper"),
        )
    return limits


def safety_profiles(path: Path) -> dict[str, dict[str, Any]]:
    """Read the path-bound limit profiles from the safety config."""
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    raw = data.get("profiles") or {}
    if not isinstance(raw, dict):
        raise ValueError(f"{path}: profiles must be a mapping")
    return raw


def vendor_limits(path: Path) -> dict[str, tuple[float, float]]:
    """Load the phase-2 ADOPTED_BOUNDS and convert them to radians."""
    import importlib.util

    spec = importlib.util.spec_from_file_location(
        "_vendor_servo_map_for_consistency", path
    )
    if spec is None or spec.loader is None:
        raise ValueError(f"{path}: cannot load module")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    limits: dict[str, tuple[float, float]] = {}
    for joint in JOINTS:
        low, high = module.ADOPTED_BOUNDS[joint]
        hard_low, hard_high = module.HARD_BOUNDS[joint]
        if not hard_low <= low <= high <= hard_high:
            raise ValueError(
                f"{joint}: adopted {low}..{high} outside hard "
                f"{hard_low}..{hard_high}"
            )
        a = module.pulse_to_rad(joint, low)
        b = module.pulse_to_rad(joint, high)
        limits[joint] = (min(a, b), max(a, b))
    return limits


def mapping_limits(path: Path) -> dict[str, tuple[float, float]]:
    """Convert the Studio's raw pulse bounds to radians.

    The vendor mapping is linear: ``rad = (raw - init_raw) * ticks *
    direction``.  joint3 is the one-sided/inverted joint (direction -1).
    """
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    ticks = _float(data.get("radians_per_tick"), "radians_per_tick")
    limits: dict[str, tuple[float, float]] = {}
    by_name = {entry.get("name"): entry for entry in data.get("joints") or []}
    for joint in JOINTS:
        entry = by_name.get(joint)
        if entry is None:
            raise ValueError(f"{path}: joint not found: {joint}")
        init = _float(entry.get("init_raw"), f"{joint}.init_raw")
        direction = _float(entry.get("direction"), f"{joint}.direction")
        low = (_float(entry.get("min_raw"), f"{joint}.min_raw") - init) * (
            ticks
        ) * direction
        high = (_float(entry.get("max_raw"), f"{joint}.max_raw") - init) * (
            ticks
        ) * direction
        limits[joint] = (min(low, high), max(low, high))
    return limits


def compare_limits(
    joint: str,
    sources: dict[str, tuple[float, float]],
    tolerance: float,
) -> list[str]:
    """Return one issue per source that disagrees with the xacro."""
    issues: list[str] = []
    reference = sources["xacro"]
    for name, value in sources.items():
        if name == "xacro":
            continue
        for index, label in enumerate(("lower", "upper")):
            if abs(value[index] - reference[index]) > tolerance:
                issues.append(
                    f"{joint} {label}: xacro={reference[index]:.6f} "
                    f"{name}={value[index]:.6f} "
                    f"(delta {value[index] - reference[index]:+.6f})"
                )
    return issues


def correction_values(path: Path) -> tuple[float, float, float]:
    """Extract the constant correction from a Python runner."""
    text = path.read_text(encoding="utf-8")
    match = re.search(
        r"(?:EMPIRICAL_CORRECTION_MM|DEFAULT_CORRECTION_MM)\s*=\s*"
        r"(?:\(|\[|')([^)\]\n']+)",
        text,
    )
    if match is None:
        raise ValueError(f"{path}: correction constant not found")
    numbers = [
        float(part) for part in re.split(r"[,\s]+", match.group(1)) if part
    ]
    if len(numbers) != 3:
        raise ValueError(
            f"{path}: expected 3 correction values, got {numbers}"
        )
    return (numbers[0], numbers[1], numbers[2])


def takeover_correction(path: Path) -> tuple[float, float, float]:
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    raw = (data.get("task") or {}).get("correction_mm")
    if not isinstance(raw, list) or len(raw) != 3:
        raise ValueError(f"{path}: task.correction_mm must be a 3-list")
    return (_float(raw[0], "correction_mm[0]"),
            _float(raw[1], "correction_mm[1]"),
            _float(raw[2], "correction_mm[2]"))


def fast_motion_defaults(path: Path) -> dict[str, float]:
    """Parse the fast_motion CLI defaults that mirror task.fast."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    found: dict[str, float] = {}
    for node in ast.walk(tree):
        if not isinstance(node, ast.Call):
            continue
        if not isinstance(node.func, ast.Attribute):
            continue
        if node.func.attr != "add_argument" or not node.args:
            continue
        flag = node.args[0]
        if not isinstance(flag, ast.Constant):
            continue
        if flag.value not in {"--vmax", "--amax"}:
            continue
        for keyword in node.keywords:
            if keyword.arg != "default":
                continue
            if not isinstance(keyword.value, ast.Constant):
                continue
            found[str(flag.value)] = float(keyword.value.value)
    missing = {"--vmax", "--amax"} - set(found)
    if missing:
        raise ValueError(f"{path}: missing defaults for {sorted(missing)}")
    return {
        "vmax_rad_s": found["--vmax"],
        "amax_rad_s2": found["--amax"],
    }


def view_pose(path: Path) -> tuple[float, ...]:
    """Parse a DEFAULT_VIEW_POSE list/tuple or comma-separated string."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in tree.body:
        if not isinstance(node, ast.Assign):
            continue
        if len(node.targets) != 1:
            continue
        target = node.targets[0]
        if (
            not isinstance(target, ast.Name)
            or target.id != "DEFAULT_VIEW_POSE"
        ):
            continue
        value = ast.literal_eval(node.value)
        if isinstance(value, str):
            parts = [float(item) for item in value.split(",")]
        elif isinstance(value, (list, tuple)):
            parts = [float(item) for item in value]
        else:
            raise ValueError(f"{path}: unsupported DEFAULT_VIEW_POSE")
        if len(parts) != 5:
            raise ValueError(f"{path}: DEFAULT_VIEW_POSE must have 5 values")
        return tuple(parts)
    raise ValueError(f"{path}: DEFAULT_VIEW_POSE not found")


def module_literal(path: Path, name: str) -> Any:
    """Return a module-level literal assignment by name."""
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    for node in tree.body:
        target = None
        value = None
        if isinstance(node, ast.Assign) and len(node.targets) == 1:
            target = node.targets[0]
            value = node.value
        elif isinstance(node, ast.AnnAssign):
            target = node.target
            value = node.value
        if isinstance(target, ast.Name) and target.id == name:
            if value is None:
                raise ValueError(f"{path}: {name} has no value")
            return ast.literal_eval(value)
    raise ValueError(f"{path}: {name} not found")


def arm_joints(path: Path) -> tuple[str, ...]:
    value = module_literal(path, "ARM_JOINTS")
    if not isinstance(value, (list, tuple)):
        raise ValueError(f"{path}: ARM_JOINTS must be a sequence")
    return tuple(str(item) for item in value)


def servo_ids(path: Path) -> dict[str, int]:
    value = module_literal(path, "SERVO_IDS")
    if not isinstance(value, dict):
        raise ValueError(f"{path}: SERVO_IDS must be a mapping")
    return {str(key): int(item) for key, item in value.items()}


def gripper_servo_id(path: Path) -> int:
    value = module_literal(path, "GRIPPER_SERVO_ID")
    if not isinstance(value, int):
        raise ValueError(f"{path}: GRIPPER_SERVO_ID must be an int")
    return value


def check_repo(root: Path, tolerance: float = 1e-4) -> Report:
    report = Report()
    paths = {
        "xacro": root / XACRO,
        "moveit": root / MOVEIT,
        "safety": root / SAFETY,
        "mapping": root / MAPPING,
        "vendor": root / VENDOR,
    }
    parsed: dict[str, dict[str, tuple[float, float]]] = {}
    for name, path in paths.items():
        if not path.is_file():
            report.issues.append(f"missing limit source: {path}")
            continue
        try:
            if name == "xacro":
                parsed[name] = xacro_limits(path)
            elif name == "moveit":
                parsed[name] = moveit_limits(path)
            elif name == "safety":
                parsed[name] = safety_limits(path)
            elif name == "mapping":
                parsed[name] = mapping_limits(path)
            else:
                parsed[name] = vendor_limits(path)
        except (ValueError, KeyError) as exc:
            report.issues.append(f"{path}: {exc}")

    if len(parsed) == len(paths):
        for joint in JOINTS:
            sources = {name: values[joint] for name, values in parsed.items()}
            report.limits[joint] = sources
            report.issues.extend(compare_limits(joint, sources, tolerance))

    try:
        for relative in CORRECTION_FILES:
            path = root / relative
            if not path.is_file():
                report.issues.append(f"missing correction source: {path}")
                continue
            report.corrections[relative] = correction_values(path)
        takeover = root / TAKEOVER
        if takeover.is_file():
            report.corrections[TAKEOVER] = takeover_correction(takeover)
        else:
            report.issues.append(f"missing correction source: {takeover}")
    except (ValueError, KeyError) as exc:
        report.issues.append(str(exc))

    if report.corrections:
        values = list(report.corrections.values())
        reference = values[0]
        for path, value in report.corrections.items():
            if any(abs(value[i] - reference[i]) > tolerance for i in range(3)):
                report.issues.append(
                    f"correction mismatch: {path}={value} vs {reference}"
                )

    try:
        takeover_data = yaml.safe_load(
            (root / TAKEOVER).read_text(encoding="utf-8")
        ) or {}
        safety_data = yaml.safe_load(
            (root / SAFETY).read_text(encoding="utf-8")
        ) or {}
        profiles = safety_profiles(root / SAFETY)
        report.profiles = profiles
        for name in ("moveit", "step", "pose", "fast"):
            if name not in profiles:
                report.issues.append(f"missing limit profile: {name}")

        moveit = profiles.get("moveit") or {}
        for joint, raw in (safety_data.get("joint_limits") or {}).items():
            for key, profile_key in (
                ("max_velocity", "max_velocity_rad_s"),
                ("max_acceleration", "max_acceleration_rad_s2"),
            ):
                actual = _float(raw.get(key), f"{joint}.{key}")
                expected = _float(
                    moveit.get(profile_key),
                    f"profiles.moveit.{profile_key}",
                )
                if abs(actual - expected) > tolerance:
                    report.issues.append(
                        f"safety {joint}.{key}={actual} vs "
                        f"profiles.moveit.{profile_key}={expected}"
                    )

        mapping_data = yaml.safe_load(
            (root / MAPPING).read_text(encoding="utf-8")
        ) or {}
        step = profiles.get("step") or {}
        for key in ("max_step_rad", "min_duration_s", "max_duration_s"):
            actual = _float(
                (mapping_data.get("safety") or {}).get(key),
                f"mapping.safety.{key}",
            )
            expected = _float(step.get(key), f"profiles.step.{key}")
            if abs(actual - expected) > tolerance:
                report.issues.append(
                    f"mapping.safety.{key}={actual} vs "
                    f"profiles.step.{key}={expected}"
                )

        pose = profiles.get("pose") or {}
        actual_pose = _float(
            takeover_data.get("max_velocity_rad_s"),
            "takeover.max_velocity_rad_s",
        )
        expected_pose = _float(
            pose.get("max_velocity_rad_s"),
            "profiles.pose.max_velocity_rad_s",
        )
        if abs(actual_pose - expected_pose) > tolerance:
            report.issues.append(
                f"takeover max_velocity_rad_s={actual_pose} vs "
                f"profiles.pose.max_velocity_rad_s={expected_pose}"
            )

        fast = (takeover_data.get("task") or {}).get("fast") or {}
        defaults = fast_motion_defaults(root / FAST_MOTION)
        report.fast_motion = defaults
        fast_profile = profiles.get("fast") or {}
        for key, profile_key in (
            ("vmax_rad_s", "max_velocity_rad_s"),
            ("amax_rad_s2", "max_acceleration_rad_s2"),
        ):
            expected = _float(fast.get(key), f"task.fast.{key}")
            if abs(defaults[key] - expected) > tolerance:
                report.issues.append(
                    f"fast_motion {key}={defaults[key]} vs "
                    f"takeover task.fast {key}={expected}"
                )
            profile_value = _float(
                fast_profile.get(profile_key),
                f"profiles.fast.{profile_key}",
            )
            if abs(expected - profile_value) > tolerance:
                report.issues.append(
                    f"takeover task.fast.{key}={expected} vs "
                    f"profiles.fast.{profile_key}={profile_value}"
                )
        if fast_profile.get("requires_bus_verification") is not True:
            report.issues.append(
                "profiles.fast.requires_bus_verification must be true"
            )
    except (ValueError, KeyError, OSError) as exc:
        report.issues.append(str(exc))

    poses: dict[str, tuple[float, ...]] = {}
    for relative in VIEW_POSE_FILES:
        path = root / relative
        if not path.is_file():
            report.issues.append(f"missing view-pose source: {path}")
            continue
        try:
            poses[relative] = view_pose(path)
        except (ValueError, SyntaxError) as exc:
            report.issues.append(str(exc))
    report.view_poses = poses
    if poses:
        reference_path, reference = next(iter(poses.items()))
        for path, pose in poses.items():
            if any(abs(a - b) > tolerance for a, b in zip(pose, reference)):
                report.issues.append(
                    f"DEFAULT_VIEW_POSE mismatch: {path}={pose} vs "
                    f"{reference_path}={reference}"
                )

    for relative in ARM_JOINT_FILES:
        path = root / relative
        if not path.is_file():
            report.issues.append(f"missing arm-joint source: {path}")
            continue
        try:
            joints = arm_joints(path)
        except (ValueError, SyntaxError) as exc:
            report.issues.append(str(exc))
            continue
        report.arm_joints[relative] = joints
        if joints != CANONICAL_ARM_JOINTS:
            report.issues.append(
                f"ARM_JOINTS mismatch: {relative}={joints} vs "
                f"{CANONICAL_ARM_JOINTS}"
            )

    for relative in SERVO_ID_FILES:
        path = root / relative
        if not path.is_file():
            report.issues.append(f"missing servo-id source: {path}")
            continue
        try:
            ids = servo_ids(path)
        except (ValueError, SyntaxError) as exc:
            report.issues.append(str(exc))
            continue
        report.servo_ids[relative] = ids
        for joint, expected in CANONICAL_SERVO_IDS.items():
            if ids.get(joint) != expected:
                report.issues.append(
                    f"SERVO_IDS mismatch: {relative}[{joint}]="
                    f"{ids.get(joint)} vs {expected}"
                )
        extra = set(ids) - set(CANONICAL_SERVO_IDS) - {"gripper"}
        if extra:
            report.issues.append(
                f"SERVO_IDS has unknown joints in {relative}: {sorted(extra)}"
            )
        if "gripper" in ids and ids["gripper"] != 1:
            report.issues.append(
                f"SERVO_IDS gripper in {relative}={ids['gripper']} vs 1"
            )

    for relative in GRIPPER_ID_FILES:
        path = root / relative
        if not path.is_file():
            report.issues.append(f"missing gripper-id source: {path}")
            continue
        try:
            value = gripper_servo_id(path)
        except (ValueError, SyntaxError) as exc:
            report.issues.append(str(exc))
            continue
        report.gripper_servo_ids[relative] = value
        if value != 1:
            report.issues.append(
                f"GRIPPER_SERVO_ID mismatch: {relative}={value} vs 1"
            )

    # The G8 task preset must stay inside the adopted joint2/joint3 bounds.
    try:
        takeover_data = yaml.safe_load(
            (root / TAKEOVER).read_text(encoding="utf-8")
        ) or {}
        task = takeover_data.get("task") or {}
        for key, joint in (("grasp_j2", "joint2"), ("grasp_j3", "joint3")):
            value = task.get(key)
            if value is None or "safety" not in parsed:
                continue
            lower, upper = parsed["safety"][joint]
            if not lower - tolerance <= float(value) <= upper + tolerance:
                report.issues.append(
                    f"{key}={value} outside safety {joint} "
                    f"[{lower:.5f}, {upper:.5f}]"
                )
    except (ValueError, KeyError) as exc:
        report.issues.append(f"{TAKEOVER}: {exc}")

    return report


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--root",
        type=Path,
        default=Path(__file__).resolve().parents[1],
        help="repository root (default: the checkout containing this script)",
    )
    parser.add_argument(
        "--tolerance",
        type=float,
        default=1e-4,
        help="radians; the Studio mapping is rounded from integer pulses",
    )
    parser.add_argument("--json", action="store_true")
    args = parser.parse_args(argv)

    report = check_repo(args.root.resolve(), tolerance=args.tolerance)
    if args.json:
        payload = {
            "limits": {
                joint: {
                    name: list(value)
                    for name, value in sources.items()
                }
                for joint, sources in report.limits.items()
            },
            "corrections": {
                path: list(value)
                for path, value in report.corrections.items()
            },
            "fast_motion": report.fast_motion,
            "view_poses": {
                path: list(value)
                for path, value in report.view_poses.items()
            },
            "profiles": report.profiles,
            "arm_joints": {
                path: list(value)
                for path, value in report.arm_joints.items()
            },
            "servo_ids": report.servo_ids,
            "gripper_servo_ids": report.gripper_servo_ids,
            "issues": report.issues,
        }
        print(json.dumps(payload, indent=2, sort_keys=True))
    else:
        if report.issues:
            print("CONFIG INCONSISTENT", file=sys.stderr)
            for issue in report.issues:
                print(f"  - {issue}", file=sys.stderr)
        else:
            print(
                "CONFIG OK: 5 joints agree across xacro / MoveIt / safety / "
                "mapping / vendor; the fast-motion defaults match "
                f"task.fast; {len(report.view_poses)} view poses agree; "
                f"{len(report.arm_joints)} arm-joint, "
                f"{len(report.servo_ids)} servo-id and "
                f"{len(report.gripper_servo_ids)} gripper-id sources agree; "
                f"{len(report.profiles)} limit profiles agree; "
                "the constant correction agrees in "
                f"{len(report.corrections)} sources"
            )
    return 1 if report.issues else 0


if __name__ == "__main__":
    raise SystemExit(main())
