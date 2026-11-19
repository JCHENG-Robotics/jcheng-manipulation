#!/usr/bin/env python3
"""Command bus servos to explicit raw positions and verify tracking.

This is a **supervised** tool for the last few pulses of travel, beyond the
adopted joint limits but inside the measured mechanical / EEPROM bounds. It
does not release torque and does not change any limit. It publishes one
``ServosPosition`` command, waits for the interpolation, reads the positions
back, and reports the tracking error. A joint that stops following its
command (a mechanical stop) is reported as a stall and the tool exits
non-zero.

    python3 pulse_move.py --positions 6=499,5=337,4=934,3=349,2=500,1=500 \
        --duration 3.0 --i-have-clear-workspace

Run inside the vendor container, with the vendor CycloneDDS environment:

    sudo docker exec -it -u ubuntu ArmPiUltra /bin/zsh -c \
      'source ~/.zshrc >/dev/null 2>&1; python3 /tmp/pulse_move.py ...'

Hard bounds are the measured mechanical stops (joint2 low 322) and the servo
EEPROM position limits. Commands outside them are refused. A negative
register read kills the vendor node, so every target and every readback is
checked for the 0..1000 range; all targets here are positive.
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import (
    GetBusServoCmd,
    ServoPosition,
    ServosPosition,
)
from ros_robot_controller_msgs.srv import GetBusServoState

SERVO_IDS = [1, 2, 3, 4, 5, 6]
SERVO_NAMES = {
    1: 'gripper', 2: 'joint5', 3: 'joint4', 4: 'joint3', 5: 'joint2',
    6: 'joint1',
}
# Measured mechanical stop or servo EEPROM position limit, per servo.
HARD_BOUNDS = {
    1: (170, 700),   # gripper open/closed stops
    2: (100, 890),   # joint5 EEPROM
    3: (100, 900),   # joint4 EEPROM
    4: (500, 950),   # joint3 vendor one-sided range / EEPROM
    5: (322, 830),   # joint2 measured mechanical low / EEPROM high
    6: (50, 950),    # joint1 EEPROM
}
# Adopted command limits (20-pulse margin) for warnings only.
ADOPTED_BOUNDS = {
    1: (170, 700),
    2: (120, 870),
    3: (120, 880),
    4: (500, 930),
    5: (342, 810),
    6: (70, 930),
}
STATE_SERVICE = '/ros_robot_controller/bus_servo/get_state'
POSITION_TOPIC = '/ros_robot_controller/bus_servo/set_position'


class PulseMoveNode(Node):
    """Read positions and send one explicit position command."""

    def __init__(self):
        super().__init__('pulse_move')
        self.client = self.create_client(GetBusServoState, STATE_SERVICE)
        self.publisher = self.create_publisher(
            ServosPosition, POSITION_TOPIC, 10)

    def read_state(self):
        """Return ``{servo_id: (position, torque)}``, one servo per request."""
        if not self.client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('bus servo state service not available')
        result = {}
        for servo_id in SERVO_IDS:
            request = GetBusServoState.Request()
            command = GetBusServoCmd()
            command.id = servo_id
            command.get_position = 1
            command.get_torque_state = 1
            request.cmd.append(command)
            future = self.client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
            response = future.result()
            if response is None or not response.success:
                raise RuntimeError(
                    f'bus servo state read failed for id {servo_id}')
            if not response.state:
                raise RuntimeError(f'empty state for id {servo_id}')
            position = response.state[0].position
            torque = response.state[0].enable_torque
            result[servo_id] = (
                int(position[0]) if position else None,
                int(torque[0]) if torque else None,
            )
        return result

    def command(self, positions, duration, servo_ids):
        """Publish one ServosPosition command and wait for the subscriber."""
        message = ServosPosition()
        message.duration = float(duration)
        for servo_id in servo_ids:
            entry = ServoPosition()
            entry.id = servo_id
            entry.position = int(positions[servo_id])
            message.position.append(entry)
        deadline = time.monotonic() + 5.0
        while self.publisher.get_subscription_count() == 0:
            if time.monotonic() > deadline:
                raise RuntimeError('set_position subscriber not found')
            rclpy.spin_once(self, timeout_sec=0.1)
        self.publisher.publish(message)


def parse_positions(text):
    """Parse ``ID=POS`` pairs into a full pose, holding omitted servos."""
    values = {}
    for item in text.split(','):
        if not item:
            continue
        key, value = item.split('=', 1)
        servo_id = int(key)
        if servo_id not in SERVO_IDS:
            raise ValueError(f'unknown servo id {servo_id}')
        values[servo_id] = int(value)
    return values


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--positions',
        help='comma-separated ID=POS pairs; omitted servos hold current',
    )
    parser.add_argument('--duration', type=float, default=3.0)
    parser.add_argument('--settle', type=float, default=0.8)
    parser.add_argument('--max-error-pulses', type=int, default=5)
    parser.add_argument(
        '--i-have-clear-workspace', action='store_true',
        help='operator acknowledgement required to move',
    )
    args = parser.parse_args(argv)
    if not args.i_have_clear_workspace:
        parser.error('--i-have-clear-workspace is required to move')

    rclpy.init(args=[])
    node = PulseMoveNode()
    try:
        current = node.read_state()
        targets = {servo_id: current[servo_id][0] for servo_id in SERVO_IDS}
        if args.positions:
            specified = parse_positions(args.positions)
            targets.update(specified)
            command_ids = sorted(specified)
        else:
            command_ids = list(SERVO_IDS)

        for servo_id, target in targets.items():
            low, high = HARD_BOUNDS[servo_id]
            if not low <= target <= high:
                print(
                    f'error: {SERVO_NAMES[servo_id]} target {target} outside '
                    f'hard bounds {low}..{high}', file=sys.stderr)
                return 2

        for servo_id in (2, 3, 4, 5, 6):
            if current[servo_id][1] != 1:
                print(
                    f'error: {SERVO_NAMES[servo_id]} torque is '
                    f'{current[servo_id][1]}, expected 1; refusing',
                    file=sys.stderr)
                return 2

        print('current: ' + ', '.join(
            f'{SERVO_NAMES[i]}={current[i][0]}' for i in SERVO_IDS))
        print('target : ' + ', '.join(
            f'{SERVO_NAMES[i]}={targets[i]}' for i in command_ids))
        for servo_id in (2, 3, 4, 5, 6):
            low, high = ADOPTED_BOUNDS[servo_id]
            if not low <= targets[servo_id] <= high:
                print(
                    f'warning: {SERVO_NAMES[servo_id]} target '
                    f'{targets[servo_id]} is outside the adopted '
                    f'{low}..{high} range (supervised probe)')
        print(f'duration={args.duration:.2f}s settle={args.settle:.2f}s')
        sys.stdout.flush()

        node.command(targets, args.duration, command_ids)
        time.sleep(args.duration + args.settle)
        after = node.read_state()

        stalled = []
        print('id  joint      target  measured  delta  torque')
        for servo_id in command_ids:
            delta = after[servo_id][0] - targets[servo_id]
            print(
                f'{servo_id:2d}  {SERVO_NAMES[servo_id]:8s} '
                f'{targets[servo_id]:6d}  {after[servo_id][0]:8d} '
                f'{delta:+6d}  {after[servo_id][1]}')
            if abs(delta) > args.max_error_pulses:
                stalled.append(servo_id)
        if stalled:
            print(
                'STALL: ' + ', '.join(SERVO_NAMES[i] for i in stalled)
                + ' did not reach the target; stop here')
            return 3
        print('TRACK_OK: all servos reached their targets')
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
