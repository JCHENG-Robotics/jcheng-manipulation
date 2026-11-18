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

``--mode survey`` moves one joint at a time with torque released. A
selective release was **demonstrated and hand-verified on 2026-11-18** on
the empty gripper (evidence ``onsite_20261118/step6_torque_mapping.md``).
The vendor SDK's boolean is inverted on this servo firmware: bit 1 in
``enable_torque=[1, 1]`` (SDK "enable", command 0x0B) **releases** torque,
and ``enable_torque=[1, 0]`` (SDK "disable", 0x0C) **engages** it. The
torque register read reflects the real state.

The tool waits for the state-topic subscription to match before the first
publish: a single message sent right after startup can be lost to DDS
discovery. The recovery path is a **position hold** at the current
positions; ``--mode restore`` uses it and a running survey preflights it.

``--mode read`` is safe: it subscribes to the battery topic and reads the
bus-servo registers (position, torque, EEPROM limits/offset). It sends nothing.

``--mode restore`` re-engages all five arm servos.

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
    ServoPosition,
    ServosPosition,
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
SERVO_TO_JOINT = {servo_id: name for name, servo_id in SERVO_IDS.items()}
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
SERVO_POSITION_TOPIC = '/ros_robot_controller/bus_servo/set_position'


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
        self.position_pub = self.create_publisher(
            ServosPosition, SERVO_POSITION_TOPIC, 10)

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
            raise RuntimeError(
                'the vendor ros_robot_controller node is not available; '
                'support the arm and restart that node before continuing')
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

    def read_joint_pulse(self, name):
        """Read one joint's position.

        One servo per request keeps the shared bus-servo reply queue from
        carrying late replies into the next request. A five-servo read is
        what preceded the 2026-11-18 node crash.
        """
        servo_id = SERVO_IDS[name]
        states = self._read([servo_id], get_position=True)
        value = states.get(servo_id, {}).get('position')
        if value is None:
            raise RuntimeError(
                'no position for {} (servo {})'.format(name, servo_id))
        return value

    def sample_joint(self, name):
        """Read one joint several times with a hard range guard.

        A position outside 0..1000 means the joint was back-driven past the
        servo's reporting range; the 2026-11-18 survey crashed the vendor
        node on the read after such a sample. Abort before the next read.
        """
        values = []
        for _ in range(self.servo_reads):
            pulse = self.read_joint_pulse(name)
            if pulse < 0 or pulse > 1000:
                raise RuntimeError(
                    '{} is outside the servo range (pulse {}); stopping '
                    'before the next read and restoring torque'.format(
                        name, pulse))
            values.append(pulse)
            self.spin_for(0.1)
        return int(statistics.median(values)), max(values) - min(values)

    def read_torque(self):
        """Read torque state for all arm servos."""
        states = self._read(sorted(SERVO_IDS.values()), get_torque=True)
        return {
            name: states.get(servo_id, {}).get('torque')
            for name, servo_id in SERVO_IDS.items()
        }

    def hold_positions(self, duration=1.0, repeats=3):
        """Re-engage torque by commanding the current arm positions.

        This is the recovery path: a slow position command at the current
        register values loads torque and holds the pose. It is used after
        every survey joint and as a fallback if the 0x0C load command does
        not come back in the register.
        """
        pulses = self.read_pulses()
        self._wait_for_subscription(self.position_pub)
        for _ in range(max(1, repeats)):
            message = ServosPosition()
            message.duration = float(duration)
            for name in ARM_JOINTS:
                entry = ServoPosition()
                entry.id = SERVO_IDS[name]
                entry.position = int(pulses[name])
                message.position.append(entry)
            self.position_pub.publish(message)
            self.spin_for(duration + 0.2)
        torque = self.read_torque()
        missing = [name for name, value in torque.items() if value != 1]
        if missing:
            raise RuntimeError(
                'torque not restored on: {}'.format(', '.join(missing)))
        return pulses, torque

    def _wait_for_subscription(self, publisher, timeout=10.0):
        """Wait until a vendor subscription is matched with this publisher.

        A message published before DDS discovery completes can be dropped;
        a single gripper command was lost that way on 2026-11-18.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if publisher.get_subscription_count() > 0:
                return
            self.spin_for(0.1)
        raise RuntimeError(
            'no vendor subscription matched on {}'.format(
                SERVO_STATE_TOPIC))

    def _publish_torque(self, servo_id, value):
        """Publish ``enable_torque=[1, value]`` three times.

        The vendor convention for the arrays is ``[flag, value]``. Repeat
        the publish to cover a lost sample; the command is idempotent.
        """
        for _ in range(3):
            message = SetBusServoState()
            state = BusServoState()
            state.present_id = [1, int(servo_id)]
            state.enable_torque = [1, int(value)]
            message.state.append(state)
            message.duration = 0.0
            self.torque_pub.publish(message)
            self.spin_for(0.1)

    def set_torque(self, servo_id, enabled, timeout=5.0):
        """Set one servo's torque and verify it through the register.

        The vendor SDK boolean is **inverted on this servo firmware**
        (measured 2026-11-18, empty gripper): ``enable_torque=[1, 1]``
        (SDK "enable", 0x0B) releases torque and ``enable_torque=[1, 0]``
        (SDK "disable", 0x0C) engages it.

        A disable is only accepted if the target servo reads 0 and every
        other arm servo stays at 1. Any collateral torque-off is treated as
        a failed selective disable; the caller's recovery hold re-engages
        all servos.
        """
        self._wait_for_subscription(self.torque_pub)
        joint = SERVO_TO_JOINT.get(servo_id)
        if joint is None:
            raise RuntimeError('servo {} is not an arm joint'.format(servo_id))
        if enabled:
            self._publish_torque(servo_id, 0)
            self.spin_for(1.0)
            states = self.read_torque()
            if states.get(joint) == 1:
                return
            # 0x0C did not take; fall back to the position-hold recovery.
            self.hold_positions()
            states = self.read_torque()
            if states.get(joint) != 1:
                raise RuntimeError(
                    'servo {} torque did not become 1'.format(servo_id))
            return

        attempts = max(1, int(round(timeout / 1.5)))
        last = None
        for _ in range(attempts):
            self._publish_torque(servo_id, 1)
            self.spin_for(1.5)
            last = self.read_torque()
            if last.get(joint) == 0:
                collateral = [
                    name for name, value in last.items()
                    if name != joint and value != 1]
                if collateral:
                    raise RuntimeError(
                        'collateral torque-off on {}; expected only servo {} '
                        'to be off (state: {})'.format(
                            ', '.join(collateral), servo_id, last))
                return
        raise RuntimeError(
            'servo {} torque did not become 0 (last state: {})'.format(
                servo_id, last))

    def restore_all_torque(self):
        """Re-engage all arm servos through the verified position hold."""
        try:
            pulses, torque = self.hold_positions()
            print('restored torque by position hold at {}: {}'.format(
                pulses, torque))
        except Exception as exc:  # noqa: BLE001 - report, do not hide
            print(
                'ERROR: could not restore torque by position hold: {}'.format(
                    exc), file=sys.stderr)
            raise


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
        input(
            'Move {} slowly to one mechanical stop, then press Enter: '.format(
                name))
        first, first_spread = node.sample_joint(name)
        input(
            'Move {} to the other mechanical stop, then press Enter: '.format(
                name))
        second, second_spread = node.sample_joint(name)
    finally:
        try:
            node.hold_positions()
            print('torque restored on all arm servos after {}'.format(name))
        except Exception as exc:  # noqa: BLE001 - report and continue
            print(
                'ERROR: torque restore failed after {}: {}\n'
                'If the vendor node died, keep supporting the arm, restart '
                'the node, then run --mode restore.'.format(name, exc),
                file=sys.stderr)

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
    print('preflight: holding the current pose to verify the recovery path...')
    pulses, torque = node.hold_positions()
    print('preflight OK: positions {} torque {}'.format(pulses, torque))

    selected = [
        name.strip() for name in args.joints.split(',') if name.strip()]
    unknown = [name for name in selected if name not in ARM_JOINTS]
    if unknown:
        raise SystemExit(
            'ERROR: unknown joints: {}'.format(', '.join(unknown)))

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
