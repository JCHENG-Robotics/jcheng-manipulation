#!/usr/bin/env python3
"""Pulse one wheel motor at a time and prove the write path with motion.

The ArmPi Ultra chassis has **no motor feedback**: the vendor SDK only writes
duty (0x05) and the firmware reports no motor state, so the only way to prove
a motor channel is connected is to command a small duty and watch the wheel.

This tool publishes ``MotorsSpeedControl`` directly on
``/ros_robot_controller/set_motor_speeds`` (bypassing ``mecanum_chassis_node``
and ``/cmd_vel``), holds a short pulse at 20 Hz, then writes an explicit zero
and verifies the echoed command is zero. It refuses to run without ``--yes``
and hard-caps duty at 20 % and duration at 1.0 s.

Run inside the robot's vendor container (ROS 2 Humble)::

    sudo docker cp tools/phase2/motor_pulse.py ArmPiUltra:/tmp/
    sudo docker exec -it -u ubuntu ArmPiUltra /bin/zsh -c \
      'source ~/.zshrc >/dev/null 2>&1; \
       python3 /tmp/motor_pulse.py --ids 1 --duty 10 --duration 0.3 --yes'

Safety: there is no e-stop and no brake. Lift the robot so all four wheels
are off the ground (or put it on the floor with >=1 m clear), keep the power
switch within reach, and watch the wheels during the pulse. A nonzero duty
moves the base. The arm must be clear of the wheels.

Reading the result: ``echoed_zero=true`` only proves the ROS node accepted a
zero command, **not** that the MCU executed anything. The physical wheel
motion is the evidence.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import MotorSpeedControl, MotorsSpeedControl
from std_msgs.msg import UInt16

MOTOR_TOPIC = "/ros_robot_controller/set_motor_speeds"
BATTERY_TOPIC = "/ros_robot_controller/battery"
PUBLISH_HZ = 20.0
ZERO_HOLD_S = 0.5
MAX_DUTY = 20.0
MAX_DURATION = 1.0


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class MotorPulse(Node):
    def __init__(self) -> None:
        super().__init__("motor_pulse")
        self.publisher = self.create_publisher(
            MotorsSpeedControl, MOTOR_TOPIC, 10
        )
        self.create_subscription(
            MotorsSpeedControl, MOTOR_TOPIC, self._on_motors, 10
        )
        self.create_subscription(UInt16, BATTERY_TOPIC, self._on_battery, 10)
        self.last_motors: list[dict[str, float]] | None = None
        self.last_battery_mv: int | None = None
        self.motor_messages = 0

    def _on_motors(self, message: MotorsSpeedControl) -> None:
        self.motor_messages += 1
        self.last_motors = [
            {"id": int(item.id), "speed": float(item.speed)}
            for item in message.data
        ]

    def _on_battery(self, message: UInt16) -> None:
        self.last_battery_mv = int(message.data)

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def publish_duty(
        self, ids: list[int], duties: list[float], duration: float
    ) -> int:
        message = MotorsSpeedControl()
        message.data = [
            MotorSpeedControl(id=servo_id, speed=duty)
            for servo_id, duty in zip(ids, duties, strict=True)
        ]
        count = 0
        period = 1.0 / PUBLISH_HZ
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            self.publisher.publish(message)
            self.spin_for(period)
            count += 1
        return count

    def zero(self) -> None:
        self.publish_duty([1, 2, 3, 4], [0.0, 0.0, 0.0, 0.0], ZERO_HOLD_S)

    def echoed_zero(self) -> bool:
        if self.last_motors is None:
            return False
        return all(abs(item["speed"]) < 1e-9 for item in self.last_motors)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ids", default="1", help="motor IDs, e.g. 1 or 1,3")
    parser.add_argument("--duty", type=float, default=10.0,
                        help="duty percent, |duty| <= "
                             f"{MAX_DUTY} (default 10)")
    parser.add_argument("--duties", default=None,
                        help="per-ID duties, e.g. "
                             "10,-10,-10,10 (overrides --duty)")
    parser.add_argument("--duration", type=float, default=0.3,
                        help=f"pulse seconds, <= {MAX_DURATION} (default 0.3)")
    parser.add_argument("--yes", action="store_true",
                        help="confirm the wheels may move")
    parser.add_argument("--record", type=Path, default=None)
    args = parser.parse_args(argv)

    ids = [int(part) for part in args.ids.split(",") if part.strip()]
    if args.duties is not None:
        duties = [
            float(part) for part in args.duties.split(",") if part.strip()
        ]
    else:
        duties = [args.duty] * len(ids)
    if not args.yes:
        print(
            "refusing to move: pass --yes after clearing the wheels",
            file=sys.stderr,
        )
        return 2
    if not ids or any(i < 1 or i > 4 for i in ids):
        print("motor IDs must be 1..4", file=sys.stderr)
        return 2
    if len(duties) != len(ids):
        print("--duties must have one value per ID", file=sys.stderr)
        return 2
    if any(abs(d) > MAX_DUTY for d in duties) or args.duration > MAX_DURATION:
        print(
            f"pulse exceeds cap (|duty| <= {MAX_DUTY}, "
            f"duration <= {MAX_DURATION})",
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    node = MotorPulse()
    record = {
        "started_at": utc_now_iso(),
        "ids": ids,
        "duties": duties,
        "duration_s": args.duration,
        "battery_mv_before": None,
        "echoed_zero": False,
        "publish_count": 0,
    }
    try:
        node.spin_for(2.0)  # DDS discovery + battery
        record["battery_mv_before"] = node.last_battery_mv
        if node.count_subscribers(MOTOR_TOPIC) == 0:
            print("no subscriber on set_motor_speeds", file=sys.stderr)
            return 1

        print(f"pulse ids={ids} duties={duties}% for {args.duration:.2f} s")
        record["publish_count"] = node.publish_duty(ids, duties, args.duration)
        node.zero()
        record["echoed_zero"] = node.echoed_zero()
        print(f"echoed zero: {record['echoed_zero']}  last={node.last_motors}")
        record["battery_mv_after"] = node.last_battery_mv
        print("verification is visual: did the wheel(s) spin?")
    finally:
        if not record["echoed_zero"]:
            node.zero()
        record["finished_at"] = utc_now_iso()
        if args.record:
            args.record.write_text(json.dumps(record, indent=2) + "\n")
            print(f"record: {args.record}")
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
