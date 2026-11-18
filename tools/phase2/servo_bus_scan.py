#!/usr/bin/env python3
"""Read-only scan of the bus-servo chain behind ``ros_robot_controller``.

The ArmPi Ultra drives its mecanum wheels with four DC motors on the STM32
(``set_motor_duty``, sub-command 0x05). The only serial bus servos on the
robot are the six arm joints (vendor ``servo_controller.yaml``):
``joint1..joint5`` = IDs 6..2, ``r_joint`` = ID 1. This tool exists to prove
that on the live bus: it asks every candidate ID for its ``present_id`` and
never writes position, torque or EEPROM.

Read-only by construction: only ``get_id`` (and optionally ``get_position``
when ``--positions`` is passed) is requested.

Run inside the robot's vendor container (ROS 2 Humble)::

    sudo docker cp tools/phase2/servo_bus_scan.py ArmPiUltra:/tmp/
    sudo docker exec -u ubuntu ArmPiUltra /bin/zsh -c \
      'source ~/.zshrc >/dev/null 2>&1; \
       python3 /tmp/servo_bus_scan.py --ids 1-12'

Do **not** pass ``--positions`` when a joint sits at or past pulse 0: the
vendor driver raises a uint16 ``AssertionError`` on a negative position reply
and the node exits (see the 2026-11-18 servo survey notes).
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import GetBusServoCmd
from ros_robot_controller_msgs.srv import GetBusServoState

SERVICE = "/ros_robot_controller/bus_servo/get_state"
SERVO_IDS = [1, 2, 3, 4, 5, 6]
JOINT_FOR_ID = {
    6: "joint1",
    5: "joint2",
    4: "joint3",
    3: "joint4",
    2: "joint5",
    1: "r_joint (gripper)",
}


def parse_ids(text: str) -> list[int]:
    """Parse ``1-12`` or ``1,3,5`` into a list of IDs."""
    if "-" in text:
        low, high = text.split("-", 1)
        return list(range(int(low), int(high) + 1))
    return [int(part) for part in text.split(",") if part.strip()]


class BusScanner(Node):
    def __init__(self) -> None:
        super().__init__("servo_bus_scan")
        self.client = self.create_client(GetBusServoState, SERVICE)

    def wait_for_service(self, timeout: float = 5.0) -> bool:
        return self.client.wait_for_service(timeout_sec=timeout)

    def read_one(
        self, servo_id: int, with_position: bool, timeout: float = 1.5
    ):
        command = GetBusServoCmd()
        command.id = servo_id
        command.get_id = 1
        if with_position:
            command.get_position = 1
        request = GetBusServoState.Request()
        request.cmd = [command]
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            return None
        result = future.result()
        if not result.state:
            return None
        state = result.state[0]
        present = list(state.present_id) if state.present_id else []
        position = list(state.position) if state.position else []
        return present, position


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ids", default="1-12", help="IDs to scan, e.g. 1-12")
    parser.add_argument(
        "--positions", action="store_true",
        help="also read the position register (unsafe near negative pulses)")
    parser.add_argument("--timeout", type=float, default=1.5)
    args = parser.parse_args()

    ids = parse_ids(args.ids)
    rclpy.init()
    node = BusScanner()
    try:
        if not node.wait_for_service():
            print(f"service {SERVICE} not available", file=sys.stderr)
            return 1

        found = []
        for servo_id in ids:
            result = node.read_one(servo_id, args.positions, args.timeout)
            if result is None:
                print(f"ID {servo_id:>3}: no reply")
                continue
            present, position = result
            if not present:
                print(f"ID {servo_id:>3}: no servo")
                continue
            found.append((servo_id, present, position))
            label = JOINT_FOR_ID.get(servo_id, "NOT IN VENDOR CONFIG")
            extra = f" position={position}" if position else ""
            print(f"ID {servo_id:>3}: present={present} -> {label}{extra}")

        print(f"\n{len(found)} servo(s) responded on IDs {args.ids}")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
