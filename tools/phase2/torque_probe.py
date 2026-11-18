"""Probe the bus-servo torque command semantics on the empty gripper.

**Result (2026-11-18): the vendor SDK boolean is inverted on this servo
firmware.** ``enable_torque=[1, 1]`` (SDK "enable", command 0x0B) releases
torque and the register reads 0; ``enable_torque=[1, 0]`` (SDK "disable",
0x0C) engages it and the register reads 1. The operator confirmed by hand
that the gripper was free after 0x0B. ``joint_limit_survey.py`` now uses
that mapping; this probe is kept as a re-verification tool.

It tests both commands on the **empty gripper** (servo 1), which does not
carry the arm. It first commands a small gripper move through the same
state topic's ``position`` branch to prove that the vendor callback
executes at all, then tests each torque command. For each step it records
the position and torque registers, asks the operator whether the fingers
are free to move by hand, and restores torque with a slow position hold at
the pre-test positions.

The tool never commands an arm joint to a new position: every hold uses
the positions read before the test.

Run inside the vendor container with the vendor CycloneDDS environment:

    python3 torque_probe.py --output /tmp/torque_probe_<date>.json

Safety: support the arm, keep the gripper empty, and keep the power
switch in reach. If the arm goes limp, do not let go; the tool re-engages
within a few seconds.
"""

import argparse
import json
import sys
import time

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import (
    BusServoState,
    GetBusServoCmd,
    ServoPosition,
    ServosPosition,
    SetBusServoState,
)
from ros_robot_controller_msgs.srv import GetBusServoState

GRIPPER_SERVO = 1
SERVO_IDS = [1, 2, 3, 4, 5, 6]
SERVO_NAMES = {
    1: 'r_joint', 2: 'joint5', 3: 'joint4',
    4: 'joint3', 5: 'joint2', 6: 'joint1',
}
STATE_SERVICE = '/ros_robot_controller/bus_servo/get_state'
STATE_TOPIC = '/ros_robot_controller/bus_servo/set_state'
POSITION_TOPIC = '/ros_robot_controller/bus_servo/set_position'


def utc_now():
    """Return an ISO-8601 UTC timestamp."""
    return time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())


class ProbeNode(Node):
    """Read servo registers and publish the torque/position commands."""

    def __init__(self):
        super().__init__('torque_probe')
        self.client = self.create_client(GetBusServoState, STATE_SERVICE)
        self.state_pub = self.create_publisher(
            SetBusServoState, STATE_TOPIC, 10)
        self.position_pub = self.create_publisher(
            ServosPosition, POSITION_TOPIC, 10)

    def spin_for(self, seconds):
        """Spin the node for a fixed wall-clock interval."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_for_subscription(self, publisher, timeout=10.0):
        """Wait until the vendor node has matched this publisher."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publisher.get_subscription_count() > 0:
                return
            self.spin_for(0.1)
        raise RuntimeError('no subscriber on ' + STATE_TOPIC)

    def read_all(self, timeout=5.0):
        """Return ``{servo_id: {'position': int, 'torque': int}}``."""
        request = GetBusServoState.Request()
        for servo_id in SERVO_IDS:
            command = GetBusServoCmd()
            command.id = servo_id
            command.get_position = 1
            command.get_torque_state = 1
            request.cmd.append(command)
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        if not future.done() or future.result() is None:
            raise RuntimeError('bus servo state read timed out')
        response = future.result()
        if not response.success:
            raise RuntimeError('bus servo state read failed')
        states = {}
        for command, state in zip(request.cmd, response.state):
            states[int(command.id)] = {
                'position': int(state.position[0])
                if state.position else None,
                'torque': int(state.enable_torque[0])
                if state.enable_torque else None,
            }
        return states

    def publish_torque(self, servo_id, value):
        """Publish ``enable_torque=[1, value]`` for one servo."""
        message = SetBusServoState()
        state = BusServoState()
        state.present_id = [1, int(servo_id)]
        state.enable_torque = [1, int(value)]
        message.state.append(state)
        message.duration = 0.0
        self.state_pub.publish(message)
        self.spin_for(0.3)

    def publish_position(self, servo_id, position, duration=0.6):
        """Publish ``position=[1, position]`` through the state topic."""
        message = SetBusServoState()
        state = BusServoState()
        state.present_id = [1, int(servo_id)]
        state.position = [1, int(position)]
        message.state.append(state)
        message.duration = float(duration)
        self.state_pub.publish(message)
        self.spin_for(duration + 0.4)

    def hold(self, positions, duration=1.0, repeats=3):
        """Re-engage torque with a slow hold at the given positions."""
        self.wait_for_subscription(self.position_pub)
        for _ in range(max(1, repeats)):
            message = ServosPosition()
            message.duration = float(duration)
            for servo_id in SERVO_IDS:
                entry = ServoPosition()
                entry.id = servo_id
                entry.position = int(positions[servo_id])
                message.position.append(entry)
            self.position_pub.publish(message)
            self.spin_for(duration + 0.2)


def summarize(states):
    """Return a compact printable one-line summary of the registers."""
    return ', '.join(
        '{}={}/t{}'.format(
            SERVO_NAMES[servo_id],
            states[servo_id]['position'],
            states[servo_id]['torque'])
        for servo_id in SERVO_IDS)


def ask_free(prompt):
    """Ask the operator whether the gripper is free; return bool or None."""
    answer = input(prompt).strip().lower()
    if answer in ('y', 'yes', 'free', '1'):
        return True
    if answer in ('n', 'no', 'stiff', '0'):
        return False
    return None


def position_branch_check(node, pre_positions, delta=8):
    """Prove the SetBusServoState callback runs, using the position branch.

    A small move of the empty gripper is commanded through the same state
    topic. ``bus_servo_set_position`` is known to work, so if the register
    changes the callback is executing; if it does not, the message is not
    reaching the callback at all.
    """
    print('\n--- state-topic callback check (position branch) ---')
    node.wait_for_subscription(node.state_pub)
    target = pre_positions[GRIPPER_SERVO] + delta
    input(
        'Gripper EMPTY, arm supported. Press Enter to command a {}-count '
        'gripper move: '.format(delta))
    node.publish_position(GRIPPER_SERVO, target)
    after = node.read_all()
    moved = abs(after[GRIPPER_SERVO]['position'] - target) <= 3
    print('target={} read={} moved={}'.format(
        target, after[GRIPPER_SERVO]['position'], moved))
    record = {
        'target': target,
        'delta': delta,
        'moved': moved,
        'read_after': after,
    }
    print('Restoring the gripper with a position hold ...')
    node.hold(pre_positions)
    record['after_hold'] = node.read_all()
    print('registers after hold: ' + summarize(record['after_hold']))
    return record


def test_command(node, label, value, pre_positions):
    """Run one torque-command test on the gripper and return its record."""
    print('\n--- {} (enable_torque=[1, {}]) ---'.format(label, value))
    input(
        'Gripper EMPTY, arm supported, hand on the gripper. '
        'Press Enter to send the command: ')
    node.publish_torque(GRIPPER_SERVO, value)
    print('Command sent. Try to open/close the gripper by hand now.')
    record = {'command': label, 'enable_torque': value}
    try:
        record['free'] = ask_free(
            'Is it FREE to move? [y/n]: ')
        record['read_while_free'] = node.read_all()
        print('registers now: ' + summarize(record['read_while_free']))
    finally:
        print('Restoring torque by position hold ...')
        node.hold(pre_positions)
        record['after_hold'] = node.read_all()
        print('registers after hold: ' + summarize(record['after_hold']))
    return record


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--output', required=True,
        help='path for the JSON evidence record')
    parser.add_argument(
        '--i-have-support', action='store_true',
        help='confirm the arm is supported and the gripper is empty')
    args = parser.parse_args()
    if not args.i_have_support:
        parser.error('refusing to run without --i-have-support')
    if not sys.stdin.isatty():
        parser.error('an interactive TTY is required')

    print(__doc__)
    print('Safety: arm supported, gripper empty, power switch in reach.')
    print('If the arm goes limp, do NOT let go; the hold runs in seconds.')

    rclpy.init()
    node = ProbeNode()
    record = {'schema_version': 1, 'captured_at': utc_now()}
    try:
        if not node.client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('bus servo state service not available')
        pre = node.read_all()
        record['pre'] = pre
        pre_positions = {
            servo_id: pre[servo_id]['position'] for servo_id in SERVO_IDS}
        print('pre: ' + summarize(pre))
        missing = [
            SERVO_NAMES[servo_id] for servo_id in SERVO_IDS
            if pre[servo_id]['torque'] != 1]
        if missing:
            raise RuntimeError(
                'preflight: torque is not enabled on: '
                + ', '.join(missing))
        branch = position_branch_check(node, pre_positions)
        record['position_branch'] = branch
        if not branch['moved']:
            record['callback_not_running'] = True
            print('\nThe state topic did not move the gripper, so the '
                  'vendor callback is not executing. Skipping the torque '
                  'tests.')
        else:
            record['callback_not_running'] = False
            record['tests'] = [
                test_command(
                    node, 'torque_release_0x0B', 1, pre_positions),
                test_command(
                    node, 'torque_engage_0x0C', 0, pre_positions),
            ]
        final = node.read_all()
        record['final'] = final
        print('final: ' + summarize(final))
    finally:
        try:
            latest = node.read_all()
            node.hold({
                servo_id: latest[servo_id]['position']
                for servo_id in SERVO_IDS})
        except Exception as exc:  # noqa: BLE001 - never hide final state
            print('ERROR: final hold failed: {}'.format(exc),
                  file=sys.stderr)
        node.destroy_node()
        rclpy.shutdown()

    with open(args.output, 'w') as handle:
        json.dump(record, handle, indent=2)
        handle.write('\n')
    print('\nwrote ' + args.output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
