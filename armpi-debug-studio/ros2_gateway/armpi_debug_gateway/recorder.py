"""Read-only trajectory recorder for the Studio G5 real-data bridge.

Run inside the robot's vendor container, with the vendor ROS environment
sourced (CycloneDDS):

    python3 -m armpi_debug_gateway.recorder --duration 10 \
        --planned-json /tmp/camera_forward_plan.json \
        --capture-out /tmp/g5_capture.json \
        --dataset-out /tmp/g5_dataset.json

The recorder only creates subscriptions. It never creates a publisher, a
service client, or a timer that writes. It refuses to overwrite files unless
``--overwrite`` is passed.

Streams
-------
``commanded``
    ``/controller_manager/joint_states`` - the vendor controller's command
    echo, labelled ``commanded``.
``feedback``
    ``/controller_manager/servo_states`` - raw bus-servo positions converted
    from pulses to radians, labelled ``measured``. At the time of writing this
    topic publishes at about 47.7 Hz and matches the joint-state values.
``planned``
    Supplied as a JSON file (``real_moveit_execute.py --trajectory-json``).
    The recorder does not plan or execute motion.

Start detection
---------------
In the default ``--mode motion``, recording waits for a commanded position to
change by at least ``--motion-threshold`` radians. The last unchanged sample
defines the shared trajectory start, so the planned ``t=0`` and the first
moving samples are aligned within one controller period. ``--mode manual``
starts at the first received sample instead (handy for a no-motion check, but
the planned offset is then the operator's responsibility).

This tool is read-only and is not part of the gateway's HTTP service.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from collections import deque
from datetime import datetime, timezone
from typing import Any

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

try:
    from servo_controller_msgs.msg import ServoStateList
except ImportError as exc:  # pragma: no cover - depends on the vendor image
    raise SystemExit(
        "servo_controller_msgs is not available; run this inside the vendor "
        "ROS 2 container after sourcing /home/ubuntu/ros2_ws/install/setup.bash"
    ) from exc

from .trajectory_capture import (
    ARM_JOINTS,
    CaptureBuffer,
    CaptureError,
    build_dataset,
    build_static_planned,
    load_planned_trajectory,
    servo_states_to_positions,
    utc_now_iso,
    write_json_atomic,
)


def _stamp_text() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


class RecorderNode(Node):
    """Subscribe to commanded and measured streams and buffer samples."""

    def __init__(
        self,
        args: argparse.Namespace,
        buffer: CaptureBuffer,
    ) -> None:
        super().__init__("armpi_studio_recorder")
        self.args = args
        self.buffer = buffer
        self.joint_names = list(args.joints)
        self._baseline: list[float] | None = None
        self._last_stable: tuple[int, list[float]] | None = None
        self._pending_measured: deque[tuple[int, list[float]]] = deque(maxlen=200)
        self._t0_ns: int | None = None
        self._motion_detected = False
        self._start_ns = time.monotonic_ns()
        self._last_status_ns = self._start_ns
        self._done_reason: str | None = None
        self._abort: list[dict[str, str]] | None = None
        self._commanded_seen = 0
        self._measured_seen = 0

        self.create_subscription(
            JointState,
            args.commanded_topic,
            self._on_commanded,
            50,
        )
        self.create_subscription(
            ServoStateList,
            args.measured_topic,
            self._on_measured,
            50,
        )
        self.get_logger().info(
            f"read-only recorder: commanded={args.commanded_topic} "
            f"measured={args.measured_topic} mode={args.mode} "
            f"duration={args.duration}s"
        )

    # ------------------------------------------------------------------ time
    def _elapsed_s(self) -> float:
        if self._t0_ns is None:
            return 0.0
        return (time.monotonic_ns() - self._t0_ns) / 1e9

    def _since_start_s(self) -> float:
        return (time.monotonic_ns() - self._start_ns) / 1e9

    def _arm(self, t0_ns: int, reason: str) -> None:
        self._t0_ns = t0_ns
        self.get_logger().info(f"recording armed ({reason})")
        while self._pending_measured:
            sample_ns, positions = self._pending_measured.popleft()
            if sample_ns >= t0_ns:
                self.buffer.add("feedback", (sample_ns - t0_ns) / 1e9, positions)

    # ------------------------------------------------------------- callbacks
    def _positions_from_joint_state(self, message: JointState) -> list[float]:
        index = {name: position for position, name in enumerate(message.name)}
        missing = [name for name in self.joint_names if name not in index]
        if missing:
            raise CaptureError(
                [
                    {
                        "path": "commanded.joint_names",
                        "message": f"missing joints: {', '.join(missing)}",
                    }
                ]
            )
        positions: list[float] = []
        for name in self.joint_names:
            position = index[name]
            if position >= len(message.position):
                raise CaptureError(
                    [
                        {
                            "path": "commanded.position",
                            "message": f"no position value for {name}",
                        }
                    ]
                )
            positions.append(float(message.position[position]))
        return positions

    def _on_commanded(self, message: JointState) -> None:
        if self._done_reason is not None:
            return
        now_ns = time.monotonic_ns()
        self._commanded_seen += 1
        try:
            positions = self._positions_from_joint_state(message)
        except CaptureError as exc:
            self._abort = exc.issues
            self._done_reason = "invalid_commanded_message"
            return

        if self.args.mode == "manual":
            if self._t0_ns is None:
                self._arm(now_ns, "manual start")
                self.buffer.add("commanded", 0.0, positions)
            else:
                self.buffer.add("commanded", (now_ns - self._t0_ns) / 1e9, positions)
            return

        if self._baseline is None:
            self._baseline = positions
            self._last_stable = (now_ns, positions)
            return
        if not self._motion_detected:
            delta = max(
                abs(value - baseline)
                for value, baseline in zip(positions, self._baseline)
            )
            if delta >= self.args.motion_threshold:
                t0_ns = self._last_stable[0] if self._last_stable else now_ns
                self._motion_detected = True
                self._arm(t0_ns, f"commanded delta {delta:.4f} rad")
                if self._last_stable is not None:
                    stable_ns, stable_positions = self._last_stable
                    self.buffer.add(
                        "commanded", (stable_ns - t0_ns) / 1e9, stable_positions
                    )
                self.buffer.add("commanded", (now_ns - t0_ns) / 1e9, positions)
            else:
                self._last_stable = (now_ns, positions)
            return
        self.buffer.add("commanded", (now_ns - self._t0_ns) / 1e9, positions)

    def _on_measured(self, message: Any) -> None:
        if self._done_reason is not None:
            return
        now_ns = time.monotonic_ns()
        self._measured_seen += 1
        try:
            by_joint = servo_states_to_positions(
                getattr(message, "servo_state", []),
                tuple(self.joint_names),
            )
        except CaptureError as exc:
            # A pulse outside 0..1000 is the known vendor-node crash trigger.
            # Stop immediately and keep the samples already collected.
            self._abort = exc.issues
            self._done_reason = "measured_out_of_range"
            return
        positions = [by_joint[name] for name in self.joint_names]
        if self._t0_ns is None:
            self._pending_measured.append((now_ns, positions))
            return
        self.buffer.add("feedback", (now_ns - self._t0_ns) / 1e9, positions)

    # ---------------------------------------------------------------- finish
    def check_deadline(self) -> None:
        """Set the stop reason when the recording window has elapsed."""
        if self._done_reason is not None:
            return
        if self._t0_ns is not None:
            if self._elapsed_s() >= self.args.duration:
                self._done_reason = "duration_elapsed"
            return
        if self._since_start_s() >= self.args.start_timeout:
            self._done_reason = "start_timeout"

    def status(self) -> None:
        now_ns = time.monotonic_ns()
        if now_ns - self._last_status_ns < 2_000_000_000:
            return
        self._last_status_ns = now_ns
        self.get_logger().info(
            f"commanded={self.buffer.count('commanded')} "
            f"measured={self.buffer.count('feedback')} "
            f"armed={self._t0_ns is not None} "
            f"elapsed={self._elapsed_s():.1f}s"
        )

    def stop_reason(self) -> str | None:
        return self._done_reason

    def abort_issues(self) -> list[dict[str, str]] | None:
        return self._abort

    def source_counts(self) -> dict[str, int]:
        return {
            "commanded_messages": self._commanded_seen,
            "measured_messages": self._measured_seen,
        }

    def started_at_iso(self) -> str:
        """Return the recorder start as an ISO-8601 UTC timestamp."""
        return (
            datetime.fromtimestamp(self._start_ns / 1e9, tz=timezone.utc)
            .isoformat()
            .replace("+00:00", "Z")
        )


def _load_planned(args: argparse.Namespace, buffer: CaptureBuffer) -> dict[str, Any] | None:
    if args.planned_json:
        with open(args.planned_json, encoding="utf-8") as handle:
            raw = json.load(handle)
        return load_planned_trajectory(raw)
    if args.static_planned:
        commanded = buffer.samples["commanded"]
        if not commanded:
            return None
        return build_static_planned(
            buffer.joint_names,
            commanded[0]["positions"],
            duration_s=max(1.0, args.duration),
        )
    return None


def _write_outputs(
    args: argparse.Namespace,
    buffer: CaptureBuffer,
    node: RecorderNode,
) -> int:
    capture = buffer.capture_document(
        started_at=node.started_at_iso(),
        ended_at=utc_now_iso(),
        commanded_source=f"{args.commanded_topic} (vendor command echo)",
        feedback_source=f"{args.measured_topic} (bus-servo position, pulse converted)",
        feedback_truth="measured",
        topics={
            "commanded": args.commanded_topic,
            "measured": args.measured_topic,
            "planned": args.planned_json or "",
        },
        time_source="monotonic_receive",
        motion_start_s=0.0 if node._t0_ns is not None else None,
        notes=list(args.note or []),
        aborted=node.abort_issues() is not None,
    )
    capture["recorder"] = {
        "mode": args.mode,
        "motion_threshold_rad": args.motion_threshold,
        "duration_s": args.duration,
        "start_timeout_s": args.start_timeout,
        "counts": node.source_counts(),
        "dropped": buffer.dropped,
        "out_of_order": buffer.out_of_order,
        "abort_issues": node.abort_issues(),
    }
    write_json_atomic(args.capture_out, capture)
    print(f"capture: {args.capture_out}")

    if not args.dataset_out:
        return 0
    if buffer.count("commanded") < 2 or buffer.count("feedback") < 2:
        print(
            "dataset not written: commanded and feedback need at least 2 "
            f"samples (got commanded={buffer.count('commanded')}, "
            f"feedback={buffer.count('feedback')})",
            file=sys.stderr,
        )
        return 3 if node.stop_reason() == "start_timeout" else 2
    planned = _load_planned(args, buffer)
    if planned is None:
        print(
            "dataset not written: pass --planned-json or --static-planned",
            file=sys.stderr,
        )
        return 2
    executor = None
    if args.executor_status:
        executor = {
            "status": args.executor_status,
            "detail": args.executor_detail or "",
            "truth": args.executor_truth,
        }
    try:
        dataset = build_dataset(
            name=args.name or f"recorded capture {_stamp_text()}",
            description=args.description or "",
            joint_names=buffer.joint_names,
            planned=planned,
            commanded=buffer.samples["commanded"],
            feedback=buffer.samples["feedback"],
            planned_source=args.planned_source or planned.get("source", ""),
            planned_offset_s=args.planned_offset_s,
            commanded_source=f"{args.commanded_topic} (vendor command echo)",
            feedback_source=f"{args.measured_topic} (bus-servo position, pulse converted)",
            feedback_truth="measured",
            executor=executor,
            metadata={
                "operator": args.operator or "",
                "notes": list(args.note or []),
                "motion_detected": node._motion_detected,
                "motion_threshold_rad": args.motion_threshold,
                "capture_file": args.capture_out,
                "commanded_topic": args.commanded_topic,
                "measured_topic": args.measured_topic,
                "planned_file": args.planned_json or "",
                "recorder_mode": args.mode,
            },
        )
    except CaptureError as exc:
        print("dataset not written: validation failed", file=sys.stderr)
        for issue in exc.issues:
            print(f"  {issue['path']}: {issue['message']}", file=sys.stderr)
        return 2
    write_json_atomic(args.dataset_out, dataset)
    print(f"dataset: {args.dataset_out}")
    print(
        "samples: "
        f"planned={len(dataset['planned']['points'])} "
        f"commanded={len(dataset['commanded']['points'])} "
        f"feedback={len(dataset['feedback']['points'])}"
    )
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode",
        choices=("motion", "manual"),
        default="motion",
        help="motion: start on the first commanded change; manual: start immediately",
    )
    parser.add_argument("--duration", type=float, default=15.0, help="recording seconds after start")
    parser.add_argument(
        "--start-timeout",
        type=float,
        default=30.0,
        help="in motion mode, fail if no motion is seen within this many seconds",
    )
    parser.add_argument(
        "--motion-threshold",
        type=float,
        default=0.002,
        help=(
            "commanded delta in radians that arms the recorder; keep it below "
            "one servo tick (0.00419 rad) so a single-tick move is not missed"
        ),
    )
    parser.add_argument(
        "--joints",
        nargs="+",
        default=list(ARM_JOINTS),
        help="arm joints to record, in dataset order",
    )
    parser.add_argument(
        "--commanded-topic",
        default="/controller_manager/joint_states",
        help="vendor command echo (sensor_msgs/JointState)",
    )
    parser.add_argument(
        "--measured-topic",
        default="/controller_manager/servo_states",
        help="bus-servo positions (servo_controller_msgs/ServoStateList)",
    )
    parser.add_argument("--planned-json", help="MoveIt planned-trajectory JSON")
    parser.add_argument(
        "--static-planned",
        action="store_true",
        help="build a two-point hold plan from the first commanded sample (no motion)",
    )
    parser.add_argument("--planned-source", default="", help="override planned source label")
    parser.add_argument(
        "--planned-offset-s",
        type=float,
        default=0.0,
        help="added to planned timestamps; use only with a documented alignment",
    )
    parser.add_argument("--capture-out", default="trajectory_capture.json")
    parser.add_argument("--dataset-out", help="write a G2 dataset JSON as well")
    parser.add_argument("--name", default="")
    parser.add_argument("--description", default="")
    parser.add_argument("--operator", default="")
    parser.add_argument("--note", action="append", default=[], help="repeatable note")
    parser.add_argument("--executor-status", default="")
    parser.add_argument("--executor-detail", default="")
    parser.add_argument("--executor-truth", default="operator_reported")
    parser.add_argument("--overwrite", action="store_true")
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    if args.duration <= 0 or args.duration > 300:
        print("error: --duration must be in (0, 300] seconds", file=sys.stderr)
        return 2
    if args.start_timeout <= 0 or args.start_timeout > 300:
        print("error: --start-timeout must be in (0, 300] seconds", file=sys.stderr)
        return 2
    if len(set(args.joints)) != len(args.joints):
        print("error: --joints must be unique", file=sys.stderr)
        return 2
    unknown_joints = [name for name in args.joints if name not in ARM_JOINTS]
    if unknown_joints:
        print(
            "error: unknown arm joints: " + ", ".join(unknown_joints),
            file=sys.stderr,
        )
        return 2
    for path in (args.capture_out, args.dataset_out):
        if path and not args.overwrite:
            try:
                with open(path, "rb"):
                    pass
            except FileNotFoundError:
                continue
            print(f"error: {path} exists; pass --overwrite to replace it", file=sys.stderr)
            return 2
    if args.dataset_out and not (args.planned_json or args.static_planned):
        print(
            "error: --dataset-out requires --planned-json or --static-planned",
            file=sys.stderr,
        )
        return 2

    try:
        buffer = CaptureBuffer(args.joints)
    except CaptureError as exc:
        for issue in exc.issues:
            print(f"error: {issue['path']}: {issue['message']}", file=sys.stderr)
        return 2

    rclpy.init(args=None)
    node = RecorderNode(args, buffer)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            node.check_deadline()
            node.status()
            if node.stop_reason() is not None:
                break
    except KeyboardInterrupt:
        node._done_reason = "keyboard_interrupt"
    finally:
        node.destroy_node()
        rclpy.shutdown()

    print(f"stop reason: {node.stop_reason()}")
    issues = node.abort_issues()
    if issues:
        for issue in issues:
            print(f"  {issue['path']}: {issue['message']}", file=sys.stderr)
    code = _write_outputs(args, buffer, node)
    if issues:
        return 4
    if code != 0:
        return code
    if node.stop_reason() == "start_timeout":
        print("no commanded motion was detected; capture contains no stream samples", file=sys.stderr)
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
