#!/usr/bin/env python3
"""Read bus-servo state, or measure mechanical joint limits with torque off.

Run inside the robot's vendor container (ROS 2 Humble, vendor stack running):

    sudo docker cp tools/phase2/joint_limit_survey.py ArmPiUltra:/tmp/
    sudo docker exec -it -u ubuntu ArmPiUltra /bin/zsh -c \
      'source ~/.zshrc >/dev/null 2>&1; \
       python3 /tmp/joint_limit_survey.py --mode read'

Source `~/.zshrc`: it sets `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` and
`CYCLONEDDS_URI`, matching the vendor nodes. With the default RMW (Fast DDS)
the battery topic may still arrive, but **services do not respond across RMW
implementations** and the bus-servo read will fail.

``--mode read`` is safe: it subscribes to the battery topic and reads the
bus-servo registers (position and torque state). It sends nothing.

``--mode survey`` disables torque on one joint at a time. **The arm will be
free to move under gravity.** You must support it by hand or with a stand, and
the tool refuses to run without ``--i-have-support`` and a TTY. After each
joint the torque is restored, including on Ctrl-C or error. The tool only
publishes ``SetBusServoState`` for torque enable/disable; it never sends a
position command.

``--mode restore`` re-enables torque on all five arm servos.

Copy the JSON output into the private calibration area and commit only the
summary you intend to publish. Raw stops are useful evidence but may include
workspace details.
"""

import argparse
import json
import os
import statistics
import sys
import time
from datetime import datetime, timezone

import rclpy
from rclpy.node import Node
from ros_robot_controller_msgs.msg import (
    BusServoState,
    GetBusServoCmd,
    SetBusServoState,
)
from ros_robot_controller_msgs.srv import GetBusServoState
from std_msgs.msg import UInt16

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
# Vendor servo_controller.yaml: joint1..joint5 = IDs 6,5,4,3,2.
SERVO_IDS = {
    'joint1': 6,
    'joint2': 5,
    'joint3': 4,
    'joint4': 3,
    'joint5': 2,
}
SERVO_INIT_PULSE = 500
RADIANS_PER_TICK = 240.0 / 360.0 * (2.0 * 3.141592653589793) / 1000.0
# Provisional software limits, computed with the vendor convention.
PROVISIONAL_RAD = {
    'joint1': (-2.0943951023931953, 2.0943951023931953),
    'joint2': (-2.0943951023931953, 2.0943951023931953),
    'joint3': (-2.0943951023931953, 0.0),
    'joint4': (-2.0943951023931953, 2.0943951023931953),
    'joint5': (-2.0943951023931953, 2.0943951023931953),
}
BATTERY_TOPIC = '/ros_robot_controller/battery'
SERVO_STATE_SERVICE = '/ros_robot_controller/bus_servo/get_state'
SERVO_STATE_TOPIC = '/ros_robot_controller/bus_servo/set_state'


def utc_now():
    """Return an ISO-8601 UTC timestamp."""
    return datetime.now(timezone.utc).isoformat(timespec='seconds')


def pulse_to_rad(joint, pulse):
    """Convert a raw bus-servo pulse to joint radians (vendor convention)."""
    if joint == 'joint3':
        return (SERVO_INIT_PULSE - pulse) * RADIANS_PER_TICK
    return (pulse - SERVO_INIT_PULSE) * RADIANS_PER_TICK


class SurveyNode(Node):
    """Minimal node for battery, bus-servo reads and torque enable/disable."""

    def __init__(self, servo_reads=5):
        """Create the service client and publisher."""
        super().__init__('joint_limit_survey')
        self.servo_reads = max(1, int(servo_reads))
        self.battery_mv = None
        self.create_subscription(
            UInt16, BATTERY_TOPIC, self._battery_cb, 10)
        self.servo_client = self.create_client(
            GetBusServoState, SERVO_STATE_SERVICE)
        self.torque_pub = self.create_publisher(
            SetBusServoState, SERVO_STATE_TOPIC, 10)

    def _battery_cb(self, msg):
        self.battery_mv = int(msg.data)

    def spin_for(self, seconds):
        """Spin callbacks for a bounded duration."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_battery(self, timeout=5.0):
        """Wait for a battery sample and return millivolts."""
        deadline = time.monotonic() + timeout
        while self.battery_mv is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.battery_mv is None:
            raise RuntimeError(
                'no battery sample on {}; is the vendor stack running?'.format(
                    BATTERY_TOPIC))
        return self.battery_mv

    def _read(self, servo_ids, get_position=False, get_torque=False):
        if not self.servo_client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('bus servo state service not available')
        request = GetBusServoState.Request()
        for servo_id in servo_ids:
            cmd = GetBusServoCmd()
            cmd.id = int(servo_id)
            cmd.get_position = 1 if get_position else 0
            cmd.get_torque_state = 1 if get_torque else 0
            request.cmd.append(cmd)
        future = self.servo_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=3.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('bus servo state read failed')
        out = {}
        for cmd, state in zip(request.cmd, response.state):
            out[int(cmd.id)] = {
                'position': int(state.position[0]) if state.position else None,
                'torque': int(state.enable_torque[0])
                if state.enable_torque else None,
            }
        return out

    def read_pulses(self):
        """Read all arm servo positions once."""
        states = self._read(sorted(SERVO_IDS.values()), get_position=True)
        pulses = {}
        for name, servo_id in SERVO_IDS.items():
            value = states.get(servo_id, {}).get('position')
            if value is None:
                raise RuntimeError(
                    'no position for {} (servo {})'.format(name, servo_id))
            pulses[name] = value
        return pulses

    def read_torque(self):
        """Read torque state for all arm servos."""
        states = self._read(sorted(SERVO_IDS.values()), get_torque=True)
        return {
            name: states.get(servo_id, {}).get('torque')
            for name, servo_id in SERVO_IDS.items()
        }

    def sample_pulses(self):
        """Read several times; return median pulses and per-joint spread."""
        readings = [self.read_pulses() for _ in range(self.servo_reads)]
        pulses = {}
        spread = {}
        for name in ARM_JOINTS:
            values = [reading[name] for reading in readings]
            pulses[name] = int(statistics.median(values))
            spread[name] = max(values) - min(values)
        return pulses, spread

    def set_torque(self, servo_id, enabled, timeout=3.0):
        """Enable or disable torque on one servo and verify the register."""
        msg = SetBusServoState()
        state = BusServoState()
        # Vendor convention for SetBusServoState arrays: [flag, value].
        # The flag must be non-zero; the servo ID is the second element.
        state.present_id = [1, int(servo_id)]
        state.enable_torque = [1, 1 if enabled else 0]
        msg.state.append(state)
        msg.duration = 0.0
        self.torque_pub.publish(msg)
        deadline = time.monotonic() + timeout
        expected = 1 if enabled else 0
        while time.monotonic() < deadline:
            self.spin_for(0.1)
            states = self._read([servo_id], get_torque=True)
            actual = states.get(servo_id, {}).get('torque')
            if actual == expected:
                return
        raise RuntimeError(
            'servo {} torque did not become {}'.format(servo_id, expected))

    def restore_all_torque(self):
        """Best-effort torque enable for all arm servos."""
        for name, servo_id in SERVO_IDS.items():
            try:
                self.set_torque(servo_id, True)
                print('restored torque: {} (servo {})'.format(name, servo_id))
            except Exception as exc:  # noqa: BLE001 - best effort on exit
                print('WARNING: could not restore torque on {}: {}'.format(
                    name, exc), file=sys.stderr)


def read_payload(node):
    """Build the read-only payload."""
    battery_mv = node.wait_battery()
    pulses = node.read_pulses()
    torque = node.read_torque()
    joints = []
    for name in ARM_JOINTS:
        provisional = PROVISIONAL_RAD[name]
        rad = pulse_to_rad(name, pulses[name])
        joints.append({
            'joint': name,
            'servo_id': SERVO_IDS[name],
            'pulse': pulses[name],
            'joint_rad': round(rad, 6),
            'torque_enabled': torque[name],
            'provisional_min_rad': round(provisional[0], 6),
            'provisional_max_rad': round(provisional[1], 6),
            'within_provisional': provisional[0] <= rad <= provisional[1],
        })
    return {
        'schema_version': 1,
        'captured_at': utc_now(),
        'source': 'joint_limit_survey.py --mode read',
        'battery_mv': battery_mv,
        'joints': joints,
        'notes': [
            'Position is the bus-servo register, not an independent encoder.',
            'Provisional limits come from the vendor software range.',
        ],
    }


def survey_one(node, name):
    """Measure one joint and return its record."""
    servo_id = SERVO_IDS[name]
    provisional = PROVISIONAL_RAD[name]
    print('\n=== {} (servo {}) ==='.format(name, servo_id))
    print('provisional range: {:.3f} .. {:.3f} rad'.format(*provisional))
    print('Torque will be OFF for this joint only. Support the arm now.')
    answer = input(
        "Type 'ready' to disable torque on {} (anything else skips): ".format(
            name))
    if answer.strip().lower() != 'ready':
        print('skipped {}'.format(name))
        return None

    node.set_torque(servo_id, False)
    first = second = None
    first_spread = second_spread = None
    try:
        input('Move {} slowly to one mechanical stop, then press Enter: '.format(
            name))
        first, first_spread = node.sample_pulses()
        input('Move {} to the other mechanical stop, then press Enter: '.format(
            name))
        second, second_spread = node.sample_pulses()
    finally:
        node.set_torque(servo_id, True)
        print('torque restored on {}'.format(name))

    pulse_a = first[name]
    pulse_b = second[name]
    low_pulse, high_pulse = sorted((pulse_a, pulse_b))
    rad_a = pulse_to_rad(name, pulse_a)
    rad_b = pulse_to_rad(name, pulse_b)
    low_rad, high_rad = sorted((rad_a, rad_b))
    spread = max(first_spread[name], second_spread[name])
    inside = (
        low_pulse >= 0 and high_pulse <= 1000
        and provisional[0] <= low_rad and high_rad <= provisional[1]
    )
    record = {
        'joint': name,
        'servo_id': servo_id,
        'stop_a_pulse': pulse_a,
        'stop_b_pulse': pulse_b,
        'low_stop_pulse': low_pulse,
        'high_stop_pulse': high_pulse,
        'low_stop_rad': round(low_rad, 6),
        'high_stop_rad': round(high_rad, 6),
        'span_rad': round(high_rad - low_rad, 6),
        'repeat_spread_pulses': spread,
        'provisional_min_rad': round(provisional[0], 6),
        'provisional_max_rad': round(provisional[1], 6),
        'inside_provisional': inside,
    }
    print(
        'measured: raw {}..{} -> {:.3f}..{:.3f} rad (span {:.3f}); '
        'repeat spread {} pulse(s); inside provisional={}'.format(
            low_pulse, high_pulse, low_rad, high_rad, high_rad - low_rad,
            spread, inside))
    if spread > 5:
        print('WARNING: repeat spread > 5 pulses; repeat the measurement.')
    if high_rad - low_rad < 0.2:
        print('WARNING: span < 0.2 rad; the stop may not have been reached.')
    return record


def survey_payload(node, args):
    """Run the interactive survey and build the JSON payload."""
    if not args.i_have_support:
        raise SystemExit(
            'ERROR: --mode survey requires --i-have-support; the arm can fall '
            'when torque is disabled')
    if not sys.stdin.isatty():
        raise SystemExit(
            'ERROR: --mode survey needs an interactive TTY; run it with '
            'docker exec -it from your terminal')
    battery_mv = node.wait_battery()
    if battery_mv < args.min_voltage_mv:
        raise SystemExit(
            'ERROR: battery {} mV is below --min-voltage-mv {}'.format(
                battery_mv, args.min_voltage_mv))
    print('battery: {} mV'.format(battery_mv))
    print('initial pulses: {}'.format(node.read_pulses()))
    print('initial torque: {}'.format(node.read_torque()))

    selected = [name.strip() for name in args.joints.split(',') if name.strip()]
    unknown = [name for name in selected if name not in ARM_JOINTS]
    if unknown:
        raise SystemExit('ERROR: unknown joints: {}'.format(', '.join(unknown)))

    records = []
    try:
        for name in selected:
            record = survey_one(node, name)
            if record is not None:
                records.append(record)
    finally:
        node.restore_all_torque()

    missing = [name for name in ARM_JOINTS if name not in {
        record['joint'] for record in records}]
    return {
        'schema_version': 1,
        'captured_at': utc_now(),
        'source': 'joint_limit_survey.py --mode survey',
        'battery_mv': battery_mv,
        'joints': records,
        'not_measured': missing,
        'notes': [
            'Mechanical stops were found by hand with servo torque off.',
            'Position is the bus-servo register, not an independent encoder.',
            'Servo direction/offset is the vendor convention; hand-eye '
            'calibration independently validated the mapping signs.',
            'Adopting these limits into configs/joint_mapping.yaml is a '
            'separate reviewed change.',
        ],
    }


def parse_args(argv=None):
    """Parse command-line arguments."""
    parser = argparse.ArgumentParser(
        description='Read bus servos or survey mechanical joint limits')
    parser.add_argument(
        '--mode', choices=['read', 'survey', 'restore'], default='read',
        help='read (safe), survey (torque off, interactive), restore torque')
    parser.add_argument(
        '--output', default=None,
        help='write JSON here (default: /tmp/joint_limits_<timestamp>.json)')
    parser.add_argument('--servo-reads', type=int, default=5)
    parser.add_argument('--min-voltage-mv', type=int, default=6800)
    parser.add_argument(
        '--joints', default=','.join(ARM_JOINTS),
        help='comma-separated joints for survey')
    parser.add_argument(
        '--i-have-support', action='store_true',
        help='acknowledge that the arm is supported before torque is disabled')
    return parser.parse_args(argv)


def write_output(payload, path):
    """Write the JSON payload and print a confirmation."""
    path = path or '/tmp/joint_limits_{}.json'.format(
        datetime.now().strftime('%Y%m%d_%H%M%S'))
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(payload, handle, indent=2, ensure_ascii=False)
        handle.write('\n')
    print('\nwrote {}'.format(path))
    return path


def check_rmw():
    """Warn when the shell does not match the vendor CycloneDDS setup."""
    if os.environ.get('RMW_IMPLEMENTATION') != 'rmw_cyclonedds_cpp':
        print(
            'WARNING: RMW_IMPLEMENTATION is not rmw_cyclonedds_cpp; vendor '
            'services may not respond. Run with '
            "zsh -c 'source ~/.zshrc; python3 ...'.",
            file=sys.stderr)
    if not os.environ.get('CYCLONEDDS_URI'):
        print(
            'WARNING: CYCLONEDDS_URI is unset; the vendor nodes use '
            '/home/ubuntu/cyclonedds.xml.', file=sys.stderr)


def main(argv=None):
    """Entry point."""
    args = parse_args(argv)
    check_rmw()
    rclpy.init()
    node = SurveyNode(servo_reads=args.servo_reads)
    try:
        if args.mode == 'read':
            payload = read_payload(node)
            print(json.dumps(payload, indent=2, ensure_ascii=False))
            write_output(payload, args.output)
        elif args.mode == 'restore':
            node.restore_all_torque()
            print('torque state: {}'.format(node.read_torque()))
        else:
            payload = survey_payload(node, args)
            print(json.dumps(payload, indent=2, ensure_ascii=False))
            write_output(payload, args.output)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
