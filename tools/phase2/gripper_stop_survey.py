#!/usr/bin/env python3
"""Staircase sweep of the gripper servo (bus servo ID 1) to find its stops.

The vendor path is ``ServosPosition`` on
``/ros_robot_controller/bus_servo/set_position``; the MCU interpolates to the
target over ``duration`` and reports the *linkage shaft* register, which is
not the command. This tool sends one command at a time, waits for the register
to settle, and prints ``command -> register`` so the real mechanical stops can
be read off the table.

Sequence: hold every other servo at its measured register, command the gripper
from CLOSED to OPEN in even steps, then back. The operator watches; the
gripper must be empty. A register that stops changing while the command keeps
moving is a stop (or a bind - repeatability between the two directions tells
them apart).

Run inside the robot's vendor container (ROS 2 Humble)::

    sudo docker cp tools/phase2/gripper_stop_survey.py ArmPiUltra:/tmp/
    sudo docker exec -u ubuntu ArmPiUltra bash -lc \
      'source /opt/ros/humble/setup.bash; \
       source ~/ros2_ws/install/setup.bash; \
       export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp; \
       export CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml; \
       python3 /tmp/gripper_stop_survey.py --yes --start 700 --end 0 --step 40'

Without ``RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`` and ``CYCLONEDDS_URI`` the
shell discovers the workspace's nodes over Fast DDS and every service call
times out.
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import (
    GetBusServoCmd, ServoPosition, ServosPosition)
from ros_robot_controller_msgs.srv import GetBusServoState

STATE_SERVICE = '/ros_robot_controller/bus_servo/get_state'
POSITION_TOPIC = '/ros_robot_controller/bus_servo/set_position'
GRIPPER_ID = 1
IDS = [1, 2, 3, 4, 5, 6]


class GripperSweep(Node):
    def __init__(self) -> None:
        super().__init__('gripper_stop_survey')
        self.client = self.create_client(GetBusServoState, STATE_SERVICE)
        self.publisher = self.create_publisher(
            ServosPosition, POSITION_TOPIC, 10)

    def read(self) -> dict[int, int] | None:
        request = GetBusServoState.Request()
        for servo_id in IDS:
            cmd = GetBusServoCmd()
            cmd.id = servo_id
            cmd.get_position = 1
            cmd.get_torque_state = 1
            request.cmd.append(cmd)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.success:
            return None
        return {
            int(cmd.id): (int(state.position[0]) if state.position else -1)
            for cmd, state in zip(request.cmd, response.state)
        }

    def command(self, held: dict[int, int], target: int,
                duration: float) -> None:
        message = ServosPosition()
        message.duration = duration
        for servo_id in IDS:
            entry = ServoPosition()
            entry.id = servo_id
            entry.position = (target if servo_id == GRIPPER_ID
                              else held[servo_id])
            message.position.append(entry)
        # One command only: each extra message re-baselines the interpolation.
        self.publisher.publish(message)

    def settle(self, duration: float) -> tuple[int, float]:
        """Wait for the register to stop changing; return (value, elapsed)."""
        start = time.monotonic()
        last = None
        stable = 0
        while time.monotonic() - start < duration + 2.5:
            sample = self.read()
            value = sample[GRIPPER_ID] if sample else None
            settled = (value is not None and last is not None
                       and abs(value - last) <= 1)
            if settled:
                stable += 1
                if stable >= 2:
                    return value, time.monotonic() - start
            else:
                stable = 0
            last = value
            time.sleep(0.25)
        return (last if last is not None else -1), time.monotonic() - start


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--start', type=int, default=700)
    parser.add_argument('--end', type=int, default=200)
    parser.add_argument('--step', type=int, default=40)
    parser.add_argument('--duration', type=float, default=1.2)
    parser.add_argument('--record', type=str,
                        default='/tmp/gripper_sweep.json')
    parser.add_argument('--one-way', action='store_true',
                        help='only the close->open sweep (no return pass)')
    parser.add_argument('--yes', action='store_true',
                        help='confirm the gripper is empty and the robot may '
                             'move it (required: the sweep moves the fingers)')
    args = parser.parse_args()
    if not args.yes:
        print(
            'refused: the sweep moves the gripper; empty it and pass --yes',
            file=sys.stderr)
        return 2

    rclpy.init()
    node = GripperSweep()
    rows: list[dict[str, float | int]] = []
    try:
        if not node.client.wait_for_service(timeout_sec=8.0):
            print('ERROR: bus servo state service not available')
            return 2
        held = node.read()
        if held is None:
            print('ERROR: initial read failed')
            return 3
        print(f'held registers: {held}', flush=True)
        print('  cmd  register  settle_s  direction', flush=True)

        targets = list(range(args.start, args.end - 1, -args.step))
        targets += [args.end]
        for target in targets:
            node.command(held, target, args.duration)
            value, elapsed = node.settle(args.duration)
            rows.append({'cmd': target, 'register': value,
                         'settle_s': round(elapsed, 2),
                         'direction': 'close-to-open'})
            print(f'{target:>5} {value:>9} {elapsed:>9.2f}  close->open',
                  flush=True)

        # Return sweep, so a bind can be told apart from a real stop.
        back = [] if args.one_way else (
            list(range(args.end, args.start + 1, args.step)) + [args.start])
        for target in back:
            node.command(held, target, args.duration)
            value, elapsed = node.settle(args.duration)
            rows.append({'cmd': target, 'register': value,
                         'settle_s': round(elapsed, 2),
                         'direction': 'open-to-close'})
            print(f'{target:>5} {value:>9} {elapsed:>9.2f}  open->close',
                  flush=True)

        with open(args.record, 'w') as handle:
            json.dump(rows, handle, indent=2)
        print(f'record: {args.record}')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
