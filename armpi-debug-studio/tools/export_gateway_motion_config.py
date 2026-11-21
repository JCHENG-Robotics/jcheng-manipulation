#!/usr/bin/env python3
"""Export the Studio motion contract as the gateway's motion limits JSON.

The gateway does not parse YAML and does not import the backend. This tool
materialises the adopted pulse bounds into a small JSON file that the gateway
loads with ``--motion-config``:

    # G6 supervised single-joint instance
    backend/.venv/bin/python tools/export_gateway_motion_config.py \
        --out /tmp/studio_motion_limits.json

    # G7 pose takeover instance
    backend/.venv/bin/python tools/export_gateway_motion_config.py \
        --mode pose --out /tmp/studio_takeover_limits.json

Step defaults are intentionally narrower than ``configs/joint_mapping.yaml``:
0.10 rad per command, a 1.0 s duration floor and one joint per command. Pose
mode exports the five-joint set plus the gripper contract from
``configs/takeover.yaml``; it never invents pulse bounds.

The export is validated by the gateway's own parser before it is written.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import yaml

STUDIO = Path(__file__).resolve().parents[1]
GATEWAY = STUDIO / "ros2_gateway"
if str(GATEWAY) not in sys.path:
    sys.path.insert(0, str(GATEWAY))

from armpi_debug_gateway.motion import (  # noqa: E402
    ARM_JOINTS,
    GRIPPER_JOINT,
    GRIPPER_SERVO_ID,
    MotionConfigError,
    motion_config_from_dict,
)

DEFAULT_MAPPING = STUDIO / "configs" / "joint_mapping.yaml"
DEFAULT_TAKEOVER = STUDIO / "configs" / "takeover.yaml"
DEFAULT_MAX_STEP_RAD = 0.10
DEFAULT_MIN_DURATION_S = 1.0
DEFAULT_TOLERANCE_PULSES = 3
DEFAULT_SETTLE_SAMPLES = 3
DEFAULT_TIMEOUT_MARGIN_S = 2.0
DEFAULT_POSE_TIMEOUT_MARGIN_S = 3.0
DEFAULT_MAX_VELOCITY_RAD_S = 0.5


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export joint_mapping.yaml/takeover.yaml to the gateway motion JSON"
    )
    parser.add_argument("--mode", choices=("step", "pose"), default="step")
    parser.add_argument("--mapping", default=str(DEFAULT_MAPPING))
    parser.add_argument("--takeover", default=str(DEFAULT_TAKEOVER))
    parser.add_argument("--out", required=True)
    parser.add_argument("--max-step-rad", type=float, default=None)
    parser.add_argument(
        "--max-velocity-rad-s", type=float, default=DEFAULT_MAX_VELOCITY_RAD_S
    )
    parser.add_argument(
        "--min-duration-s", type=float, default=DEFAULT_MIN_DURATION_S
    )
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args(argv)


def _load_joints(mapping_path: Path) -> tuple[dict, dict, float]:
    raw = yaml.safe_load(mapping_path.read_text(encoding="utf-8"))
    joints_raw = raw.get("joints")
    if not isinstance(joints_raw, list) or not joints_raw:
        raise SystemExit("mapping has no joints")
    by_name = {
        str(item["name"]): item for item in joints_raw if isinstance(item, dict)
    }
    missing = [joint for joint in ARM_JOINTS if joint not in by_name]
    if missing:
        raise SystemExit(f"mapping is missing arm joints: {', '.join(missing)}")
    return raw, by_name, float(raw["radians_per_tick"])


def _widest_span_rad(by_name: dict, radians_per_tick: float) -> float:
    widest = 0.0
    for joint in ARM_JOINTS:
        item = by_name[joint]
        low = min(int(item["min_raw"]), int(item["max_raw"]))
        high = max(int(item["min_raw"]), int(item["max_raw"]))
        widest = max(widest, (high - low) * radians_per_tick)
    return widest


def build_config(
    mapping_path: Path,
    *,
    mode: str = "step",
    takeover_path: Path | None = None,
    max_step_rad: float | None = None,
    min_duration_s: float = DEFAULT_MIN_DURATION_S,
    max_velocity_rad_s: float = DEFAULT_MAX_VELOCITY_RAD_S,
) -> dict:
    raw, by_name, radians_per_tick = _load_joints(mapping_path)
    safety_raw = raw.get("safety") or {}
    joints = {}
    for joint in ARM_JOINTS:
        item = by_name[joint]
        low = min(int(item["min_raw"]), int(item["max_raw"]))
        high = max(int(item["min_raw"]), int(item["max_raw"]))
        joints[joint] = {
            "servo_id": int(item["servo_id"]),
            "min_pulse": low,
            "max_pulse": high,
            "init_pulse": int(item["init_raw"]),
            "direction": int(item.get("direction", 1)),
            "radians_per_tick": radians_per_tick,
        }

    config: dict = {
        "schema_version": 1,
        "mode": mode,
        "mapping_version": str(raw["mapping_version"]),
        "radians_per_tick": radians_per_tick,
        "source": str(raw.get("source", "")),
        "joints": joints,
        "safety": {
            "max_step_rad": (
                float(max_step_rad)
                if max_step_rad is not None
                else (
                    math.ceil(_widest_span_rad(by_name, radians_per_tick) * 100.0)
                    / 100.0
                    if mode == "pose"
                    else DEFAULT_MAX_STEP_RAD
                )
            ),
            "min_duration_s": float(min_duration_s),
            "max_duration_s": float(safety_raw.get("max_duration_s", 10.0)),
            "feedback_max_age_ms": int(
                safety_raw.get("feedback_max_age_ms", 500)
            ),
            "tolerance_pulses": DEFAULT_TOLERANCE_PULSES,
            "settle_samples": DEFAULT_SETTLE_SAMPLES,
            "timeout_margin_s": (
                DEFAULT_POSE_TIMEOUT_MARGIN_S
                if mode == "pose"
                else DEFAULT_TIMEOUT_MARGIN_S
            ),
            "single_joint_only": mode == "step",
            # servo_states is the vendor command cache; every arm command is
            # completed only after a GetBusServoState bus read.
            "verification": True,
            "verification_tolerance_pulses": 12 if mode == "pose" else 8,
            "verification_settle_s": 0.5,
            "verification_margin_pulses": 25,
        },
    }
    if mode == "pose":
        if takeover_path is None:
            raise SystemExit("pose mode needs --takeover")
        takeover_raw = yaml.safe_load(takeover_path.read_text(encoding="utf-8"))
        gripper_raw = takeover_raw.get("gripper")
        if not isinstance(gripper_raw, dict):
            raise SystemExit("takeover config has no gripper block")
        contract_velocity = float(
            takeover_raw.get("max_velocity_rad_s", max_velocity_rad_s)
        )
        config["safety"]["max_velocity_rad_s"] = min(
            float(max_velocity_rad_s), contract_velocity
        )
        config["safety"]["pose_requires_all_arm_joints"] = True
        config["gripper"] = {
            "joint": GRIPPER_JOINT,
            "servo_id": GRIPPER_SERVO_ID,
            "min_pulse": int(gripper_raw["open_raw"]),
            "max_pulse": int(gripper_raw["close_raw"]),
            "init_pulse": int(gripper_raw.get("init_raw", 700)),
            "speed_raw_per_s": float(gripper_raw.get("speed_raw_per_s", 120.0)),
            "min_duration_s": float(gripper_raw.get("min_duration_s", 1.0)),
            "max_duration_s": float(gripper_raw.get("max_duration_s", 6.0)),
            "completion": str(gripper_raw.get("completion", "timed")),
        }
        chassis_raw = takeover_raw.get("chassis")
        if chassis_raw is not None:
            if not isinstance(chassis_raw, dict):
                raise SystemExit("takeover chassis block must be a mapping")
            config["chassis"] = {
                "enabled": bool(chassis_raw.get("enabled", False)),
                "max_duty": float(chassis_raw.get("max_duty", 20.0)),
                "wheel_max_duty": float(
                    chassis_raw.get("wheel_max_duty", 30.0)
                ),
                "speed_mm_s_per_duty": float(
                    chassis_raw.get("speed_mm_s_per_duty", 4.9)
                ),
                "min_duration_s": float(
                    chassis_raw.get("min_duration_s", 0.20)
                ),
                "max_duration_s": float(
                    chassis_raw.get("max_duration_s", 3.0)
                ),
            }
        task_raw = takeover_raw.get("task") or {}
        fast_raw = task_raw.get("fast") if isinstance(task_raw, dict) else None
        if isinstance(fast_raw, dict) and bool(fast_raw.get("enabled", False)):
            # The G8 fast task path: dense S-curve waypoints and the measured
            # gripper timings. The per-command pose cap above stays 0.5 rad/s;
            # this block only applies to a profile payload or a fast gripper.
            config["safety"]["profile"] = {
                "max_velocity_rad_s": float(fast_raw.get("vmax_rad_s", 4.0)),
                "max_accel_rad_s2": float(fast_raw.get("amax_rad_s2", 20.0)),
                "min_segment_s": 0.025,
                "max_segment_s": 0.5,
                "max_points": int(fast_raw.get("max_points", 400)),
                "max_total_duration_s": float(
                    fast_raw.get("max_total_duration_s", 6.0)
                ),
                "start_tolerance_pulses": 20,
                "gripper_speed_raw_per_s": float(
                    fast_raw.get("gripper_speed_raw_per_s", 900.0)
                ),
                "gripper_min_duration_s": float(
                    fast_raw.get("gripper_min_duration_s", 0.25)
                ),
                "gripper_settle_s": float(
                    fast_raw.get("gripper_settle_s", 0.6)
                ),
            }
            config["safety"]["verification_settle_s"] = float(
                fast_raw.get("verify_settle_s", 0.35)
            )
    return config


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    mapping_path = Path(args.mapping)
    if not mapping_path.is_file():
        raise SystemExit(f"mapping not found: {mapping_path}")
    takeover_path = Path(args.takeover) if args.mode == "pose" else None
    if takeover_path is not None and not takeover_path.is_file():
        raise SystemExit(f"takeover config not found: {takeover_path}")
    out = Path(args.out)
    if out.exists() and not args.overwrite:
        raise SystemExit(f"refusing to overwrite {out}; pass --overwrite")

    config = build_config(
        mapping_path,
        mode=args.mode,
        takeover_path=takeover_path,
        max_step_rad=args.max_step_rad,
        min_duration_s=args.min_duration_s,
        max_velocity_rad_s=args.max_velocity_rad_s,
    )
    try:
        parsed = motion_config_from_dict(config)
    except MotionConfigError as exc:
        raise SystemExit(f"generated config failed validation: {exc}") from exc

    out.write_text(
        json.dumps(config, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(f"wrote {out}")
    print(
        f"  mode={parsed.safety.mode} mapping={parsed.mapping_version} "
        f"joints={len(parsed.joints)} "
        f"step<={parsed.safety.max_step_rad} rad "
        f"velocity<={parsed.safety.max_velocity_rad_s} rad/s "
        f"duration>={parsed.safety.min_duration_s} s "
        f"single_joint={parsed.safety.single_joint_only}"
    )
    if parsed.chassis is not None and parsed.chassis.enabled:
        chassis = parsed.chassis
        print(
            f"  chassis enabled: duty<={chassis.max_duty} "
            f"wheel<={chassis.wheel_max_duty} "
            f"duration {chassis.min_duration_s}-{chassis.max_duration_s} s "
            f"({chassis.speed_mm_s_per_duty} mm/s per duty)"
        )
    if parsed.gripper is not None:
        print(
            f"  gripper={parsed.gripper.joint} "
            f"pulse {parsed.gripper.min_pulse}..{parsed.gripper.max_pulse} "
            f"completion={parsed.gripper.completion}"
        )
    if parsed.profile is not None:
        profile = parsed.profile
        print(
            f"  fast profile enabled: velocity<={profile.max_velocity_rad_s} "
            f"rad/s accel<={profile.max_accel_rad_s2} rad/s^2 "
            f"segments<={profile.max_points} "
            f"total<={profile.max_total_duration_s} s"
        )
        print(
            f"  fast gripper: duration>={profile.gripper_min_duration_s} s "
            f"rate<={profile.gripper_speed_raw_per_s} raw/s "
            f"settle={profile.gripper_settle_s} s"
        )
    print("  add --enable-motion --motion-config to the motion gateway launch")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
