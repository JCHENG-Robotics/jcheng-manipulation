#!/usr/bin/env python3
"""Export the Studio joint mapping as the gateway's motion limits JSON.

The gateway does not parse YAML and does not import the backend. This tool
materialises the adopted pulse bounds and the *supervised* caps into a small
JSON file that the gateway loads with ``--motion-config``:

    backend/.venv/bin/python tools/export_gateway_motion_config.py \
        --out /tmp/studio_motion_limits.json

Defaults are intentionally narrower than ``configs/joint_mapping.yaml``:
0.10 rad per command, a 1.0 s duration floor and one joint per command. Pass
``--max-step-rad``/``--min-duration-s`` only with a reviewed reason; the
gateway rejects values above 0.35 rad.

The export is validated by the gateway's own parser before it is written.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import yaml

STUDIO = Path(__file__).resolve().parents[1]
GATEWAY = STUDIO / "ros2_gateway"
if str(GATEWAY) not in sys.path:
    sys.path.insert(0, str(GATEWAY))

from armpi_debug_gateway.motion import (  # noqa: E402
    ARM_JOINTS,
    MotionConfigError,
    motion_config_from_dict,
)

DEFAULT_MAPPING = STUDIO / "configs" / "joint_mapping.yaml"
DEFAULT_MAX_STEP_RAD = 0.10
DEFAULT_MIN_DURATION_S = 1.0
DEFAULT_TOLERANCE_PULSES = 3
DEFAULT_SETTLE_SAMPLES = 3
DEFAULT_TIMEOUT_MARGIN_S = 2.0


def _parse_args(argv: list[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Export joint_mapping.yaml to the gateway motion JSON"
    )
    parser.add_argument("--mapping", default=str(DEFAULT_MAPPING))
    parser.add_argument("--out", required=True)
    parser.add_argument("--max-step-rad", type=float, default=DEFAULT_MAX_STEP_RAD)
    parser.add_argument(
        "--min-duration-s", type=float, default=DEFAULT_MIN_DURATION_S
    )
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args(argv)


def build_config(
    mapping_path: Path,
    *,
    max_step_rad: float,
    min_duration_s: float,
) -> dict:
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

    radians_per_tick = float(raw["radians_per_tick"])
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

    return {
        "schema_version": 1,
        "mapping_version": str(raw["mapping_version"]),
        "radians_per_tick": radians_per_tick,
        "source": str(raw.get("source", "")),
        "joints": joints,
        "safety": {
            "max_step_rad": float(max_step_rad),
            "min_duration_s": float(min_duration_s),
            "max_duration_s": float(safety_raw.get("max_duration_s", 10.0)),
            "feedback_max_age_ms": int(
                safety_raw.get("feedback_max_age_ms", 500)
            ),
            "tolerance_pulses": DEFAULT_TOLERANCE_PULSES,
            "settle_samples": DEFAULT_SETTLE_SAMPLES,
            "timeout_margin_s": DEFAULT_TIMEOUT_MARGIN_S,
            "single_joint_only": True,
        },
    }


def main(argv: list[str] | None = None) -> int:
    args = _parse_args(argv)
    mapping_path = Path(args.mapping)
    if not mapping_path.is_file():
        raise SystemExit(f"mapping not found: {mapping_path}")
    out = Path(args.out)
    if out.exists() and not args.overwrite:
        raise SystemExit(f"refusing to overwrite {out}; pass --overwrite")

    config = build_config(
        mapping_path,
        max_step_rad=args.max_step_rad,
        min_duration_s=args.min_duration_s,
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
        f"  mapping={parsed.mapping_version} joints={len(parsed.joints)} "
        f"step<={parsed.safety.max_step_rad} rad "
        f"duration>={parsed.safety.min_duration_s} s "
        f"single_joint={parsed.safety.single_joint_only}"
    )
    print("  add --enable-motion --motion-config to the supervised gateway launch")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
