#!/usr/bin/env python3
"""Read the chassis link state, or jog the mecanum base for a bounded pulse.

Run inside the robot's vendor container (ROS 2 Humble):

    sudo docker cp tools/phase2/chassis_jog.py ArmPiUltra:/tmp/
    sudo docker exec -it -u ubuntu ArmPiUltra /bin/zsh -c \
      'source ~/.zshrc >/dev/null 2>&1; \
       python3 /tmp/chassis_jog.py --status'

``--status`` is read-only: it reports whether ``mecanum_chassis_node`` is up
(by subscriber counts, because ``get_node_names()`` is unreliable in this
container), the last ``MotorsSpeedControl`` command, and the battery voltage.

``--dry-run`` computes the motor values the chassis node would publish and
prints them without touching ROS.

A motion pulse requires ``--yes`` and stays inside hard caps: 0.30 m/s
linear, 0.60 rad/s angular, 3.0 s. The tool publishes at 20 Hz so the node's
0.5 s watchdog does not zero the command mid-pulse, then publishes an
explicit zero and verifies the echoed motor command is zero.

There is **no e-stop** on this robot and no brake: put the robot on the
floor, tuck the arm, keep the power switch within reach, and measure the
displacement by hand. The chassis has no odometry; the travel estimate uses
the measured duty scale, not a wheel encoder.

The node now converts m/s to motor **duty** with the 2026-11-18 measurement:
10 % duty gave 55 mm/s and 20 % gave 110 mm/s, so ``duty = v[m/s] * 182``,
capped at 60 duty (~0.33 m/s). The vendor controller feeds
``MotorSpeedControl.speed`` to ``set_motor_duty`` (0x05), so a duty value is
what the MCU actually receives.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from ros_robot_controller_msgs.msg import MotorsSpeedControl
from std_msgs.msg import UInt16

CMD_VEL_TOPIC = "/cmd_vel"
MOTOR_TOPIC = "/ros_robot_controller/set_motor_speeds"
BATTERY_TOPIC = "/ros_robot_controller/battery"

WHEELBASE = 0.195
TRACK_WIDTH = 0.22
DUTY_PER_MPS = 182.0
MAX_DUTY = 60.0

CAP_LINEAR = 0.30
CAP_ANGULAR = 0.60
CAP_DURATION = 3.0
PUBLISH_HZ = 20.0
ZERO_HOLD_S = 0.35


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def predict_motors(
    linear_x: float, linear_y: float, angular_z: float
) -> list[float]:
    """Mirror MecanumChassis.set_velocity(), including scaling and clamping."""
    k = (WHEELBASE + TRACK_WIDTH) / 2.0
    wheel = [
        linear_x + linear_y + angular_z * k,
        linear_x - linear_y + angular_z * k,
        linear_x - linear_y - angular_z * k,
        linear_x + linear_y - angular_z * k,
    ]
    duty = [
        wheel[0] * DUTY_PER_MPS,
        -wheel[1] * DUTY_PER_MPS,
        -wheel[2] * DUTY_PER_MPS,
        wheel[3] * DUTY_PER_MPS,
    ]
    peak = max(abs(v) for v in duty)
    if peak > MAX_DUTY:
        duty = [v * (MAX_DUTY / peak) for v in duty]
    return [max(min(v, MAX_DUTY), -MAX_DUTY) for v in duty]


class ChassisJog(Node):
    def __init__(self) -> None:
        super().__init__("chassis_jog")
        self.publisher = self.create_publisher(Twist, CMD_VEL_TOPIC, 10)
        self.create_subscription(
            MotorsSpeedControl, MOTOR_TOPIC, self._on_motors, 10
        )
        self.create_subscription(UInt16, BATTERY_TOPIC, self._on_battery, 10)
        self.last_motors: dict[str, Any] | None = None
        self.last_battery_mv: int | None = None
        self.motor_messages = 0

    def _on_motors(self, message: MotorsSpeedControl) -> None:
        self.motor_messages += 1
        self.last_motors = {
            "received_at": utc_now_iso(),
            "data": [
                {"id": int(item.id), "speed": float(item.speed)}
                for item in message.data
            ],
        }

    def _on_battery(self, message: UInt16) -> None:
        self.last_battery_mv = int(message.data)

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def motors_are_zero(self) -> bool:
        if self.last_motors is None:
            return False
        return all(
            abs(item["speed"]) < 1e-9 for item in self.last_motors["data"]
        )

    def publish_for(self, twist: Twist, duration: float) -> int:
        period = 1.0 / PUBLISH_HZ
        deadline = time.monotonic() + duration
        count = 0
        while time.monotonic() < deadline:
            self.publisher.publish(twist)
            count += 1
            self.spin_for(period)
        return count


def twist_from(axis: str, speed: float) -> Twist:
    twist = Twist()
    if axis == "x":
        twist.linear.x = speed
    elif axis == "y":
        twist.linear.y = speed
    else:
        twist.angular.z = speed
    return twist


def wait_for(node: ChassisJog, predicate, timeout: float) -> bool:
    """Poll a predicate while spinning, so subscriptions keep updating."""
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return True
        node.spin_for(0.05)
    return predicate()


def print_status(node: ChassisJog, wait: float) -> dict[str, Any]:
    node.spin_for(wait)
    status = {
        "checked_at": utc_now_iso(),
        "cmd_vel_subscribers": node.count_subscribers(CMD_VEL_TOPIC),
        "motor_publishers": node.count_publishers(MOTOR_TOPIC),
        "motor_subscribers_including_self": node.count_subscribers(
            MOTOR_TOPIC
        ),
        "last_motor_command": node.last_motors,
        "battery_mv": node.last_battery_mv,
    }
    print(json.dumps(status, indent=2))
    if status["last_motor_command"] is None:
        print(
            "note: no MotorSpeedControl seen during the wait window; "
            "the chassis node publishes only after a /cmd_vel message",
            file=sys.stderr,
        )
    return status


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="Read chassis link state or jog the mecanum base.",
    )
    parser.add_argument(
        "--status", action="store_true", help="read-only link report"
    )
    parser.add_argument(
        "--dry-run", action="store_true",
        help="print predicted motor values",
    )
    parser.add_argument("--axis", choices=("x", "y", "yaw"), default="x")
    parser.add_argument("--speed", type=float, default=0.02,
                        help="m/s for x/y, rad/s for yaw (default 0.02)")
    parser.add_argument(
        "--duration", type=float, default=0.5,
        help="pulse seconds (max 3.0)",
    )
    parser.add_argument(
        "--yes", action="store_true", help="confirm the robot may move"
    )
    parser.add_argument(
        "--record", type=Path, default=None, help="JSON record path"
    )
    parser.add_argument(
        "--wait", type=float, default=3.0,
        help="DDS discovery wait before checks (default 3.0 s)",
    )
    args = parser.parse_args(argv)

    if args.status and not args.dry_run:
        rclpy.init()
        node = ChassisJog()
        try:
            print_status(node, args.wait)
        finally:
            node.destroy_node()
            rclpy.shutdown()
        return 0

    if args.dry_run and not args.yes:
        predicted = predict_motors(
            args.speed if args.axis == "x" else 0.0,
            args.speed if args.axis == "y" else 0.0,
            args.speed if args.axis == "yaw" else 0.0,
        )
        print(json.dumps({
            "axis": args.axis,
            "speed": args.speed,
            "duration": args.duration,
            "predicted_motor_speeds": [round(v, 6) for v in predicted],
            "estimated_travel_m": (
                abs(args.speed) * args.duration
                if args.axis in ("x", "y") else None
            ),
            "estimated_turn_rad": (
                abs(args.speed) * args.duration if args.axis == "yaw" else None
            ),
            "note": (
                "travel uses the measured duty scale; the base has no "
                "odometry"
            ),
        }, indent=2))
        return 0

    cap = CAP_ANGULAR if args.axis == "yaw" else CAP_LINEAR
    if abs(args.speed) > cap:
        print(
            f"refused: |speed| {abs(args.speed)} > cap {cap}",
            file=sys.stderr,
        )
        return 2
    if args.duration <= 0.0 or args.duration > CAP_DURATION:
        print(
            f"refused: duration must be in (0, {CAP_DURATION}]",
            file=sys.stderr,
        )
        return 2
    if not args.yes:
        print(
            "refused: motion pulse needs --yes (put the robot on the floor, "
            "tuck the arm, keep the power switch within reach)",
            file=sys.stderr,
        )
        return 2

    rclpy.init()
    node = ChassisJog()
    record: dict[str, Any] = {
        "started_at": utc_now_iso(),
        "axis": args.axis,
        "speed": args.speed,
        "duration_s": args.duration,
        "predicted_motor_speeds": [
            round(v, 6) for v in predict_motors(
                args.speed if args.axis == "x" else 0.0,
                args.speed if args.axis == "y" else 0.0,
                args.speed if args.axis == "yaw" else 0.0,
            )
        ],
    }
    try:
        node.spin_for(args.wait)
        subscribers = node.count_subscribers(CMD_VEL_TOPIC)
        publishers = node.count_publishers(MOTOR_TOPIC)
        record["preflight"] = {
            "cmd_vel_subscribers": subscribers,
            "motor_publishers": publishers,
            "motor_subscribers_including_self": node.count_subscribers(
                MOTOR_TOPIC
            ),
            "battery_mv": node.last_battery_mv,
        }
        if subscribers < 1:
            print(
                "refused: no /cmd_vel subscriber; "
                "start mecanum_chassis_node first",
                file=sys.stderr,
            )
            return 2
        if publishers < 1:
            print(
                "refused: no publisher on the motor topic; "
                "the chassis node is not up",
                file=sys.stderr,
            )
            return 2
        if node.count_subscribers(MOTOR_TOPIC) < 2:
            print(
                "refused: ros_robot_controller is not subscribed "
                "to the motor topic (only this tool is)",
                file=sys.stderr,
            )
            return 2

        pulse = twist_from(args.axis, args.speed)
        record["zero_before"] = node.last_motors
        print(f"pulse: {args.axis} {args.speed} for {args.duration}s")
        published = node.publish_for(pulse, args.duration)
        record["pulse_messages_sent"] = published
        node.spin_for(0.15)
        record["during_pulse_command"] = node.last_motors
        print(f"motor command during pulse: {node.last_motors}")

        zero = Twist()
        node.publish_for(zero, ZERO_HOLD_S)
        stopped = wait_for(node, node.motors_are_zero, 1.0)
        record["after_zero_command"] = node.last_motors
        record["zero_confirmed"] = stopped
        record["finished_at"] = utc_now_iso()
        print(f"zero command after pulse: {node.last_motors}")
        if not stopped:
            print(
                "warning: the last motor command is not zero; "
                "publishing another zero",
                file=sys.stderr,
            )
            node.publish_for(zero, ZERO_HOLD_S)
            record["after_zero_retry"] = node.last_motors
            record["zero_confirmed"] = node.motors_are_zero()
        if not stopped:
            return 3
        return 0
    finally:
        record_path = args.record
        if record_path is None:
            stamp = datetime.now().strftime("%Y%m%dT%H%M%S")
            record_path = Path(f"/tmp/chassis_jog_{stamp}.json")
        record_path.write_text(json.dumps(record, indent=2) + "\n")
        print(f"record: {record_path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
