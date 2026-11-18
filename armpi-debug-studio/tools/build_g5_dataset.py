#!/usr/bin/env python3
"""Attach a MoveIt plan JSON to a G5 recorder capture and write a G2 dataset.

The recorder writes the raw capture while the robot moves. Run the plan-only
MoveIt tool with ``--trajectory-json`` first (or use the JSON written by the
same run), then assemble offline:

    armpi-debug-studio/backend/.venv/bin/python \
      armpi-debug-studio/tools/build_g5_dataset.py \
      --capture /tmp/g5_capture.json \
      --planned-json /tmp/camera_forward_plan.json \
      --out /tmp/g5_dataset.json \
      --name "camera_forward real capture"

No ROS connection, gateway call or robot access is used.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

GATEWAY = Path(__file__).resolve().parents[1] / "ros2_gateway"
if str(GATEWAY) not in sys.path:
    sys.path.insert(0, str(GATEWAY))

from armpi_debug_gateway.trajectory_capture import (  # noqa: E402
    CaptureError,
    dataset_from_capture,
    load_planned_trajectory,
    write_json_atomic,
)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--capture", required=True, help="raw recorder capture JSON")
    parser.add_argument("--planned-json", required=True, help="MoveIt plan JSON")
    parser.add_argument("--out", required=True, help="output G2 dataset JSON")
    parser.add_argument("--name", default="")
    parser.add_argument("--description", default="")
    parser.add_argument("--planned-offset-s", type=float, default=0.0)
    parser.add_argument("--executor-status", default="")
    parser.add_argument("--executor-detail", default="")
    parser.add_argument("--executor-truth", default="operator_reported")
    parser.add_argument("--overwrite", action="store_true")
    args = parser.parse_args(argv)

    output = Path(args.out)
    if output.exists() and not args.overwrite:
        print(f"error: {output} exists; pass --overwrite", file=sys.stderr)
        return 2
    try:
        with open(args.capture, encoding="utf-8") as handle:
            capture = json.load(handle)
        with open(args.planned_json, encoding="utf-8") as handle:
            planned_raw = json.load(handle)
        planned = load_planned_trajectory(planned_raw)
        executor = None
        if args.executor_status:
            executor = {
                "status": args.executor_status,
                "detail": args.executor_detail,
                "truth": args.executor_truth,
            }
        dataset = dataset_from_capture(
            capture,
            planned,
            name=args.name or None,
            description=args.description,
            planned_offset_s=args.planned_offset_s,
            executor=executor,
        )
    except (OSError, json.JSONDecodeError, CaptureError) as exc:
        if isinstance(exc, CaptureError):
            print("error: validation failed", file=sys.stderr)
            for issue in exc.issues:
                print(f"  {issue['path']}: {issue['message']}", file=sys.stderr)
        else:
            print(f"error: {exc}", file=sys.stderr)
        return 1
    write_json_atomic(str(output), dataset)
    print(f"dataset: {output}")
    print(
        "samples: "
        f"planned={len(dataset['planned']['points'])} "
        f"commanded={len(dataset['commanded']['points'])} "
        f"feedback={len(dataset['feedback']['points'])}"
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
