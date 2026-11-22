#!/usr/bin/env python3
"""Fast, smooth arm moves on the vendor trajectory path.

The vendor stack executes one bus-servo command per trajectory point with the
segment duration, so a **dense** waypoint list is a real velocity profile: the
servos follow the S-curve from :mod:`motion_profile` instead of doing one
constant-velocity move with hard start/stop steps. This tool

* reads the measured bus-servo pose (``/joint_states`` is only a command echo),
* plans a synchronised jerk-limited move from that pose to a commanded pose,
* sends it to ``/arm_controller/follow_joint_trajectory``,
* waits for the bus-servo registers to stop moving (measured settle, not a
  fixed sleep), and
* reports the planned vs wall time, the settle time and the post-move
  excursion, optionally tracing the registers during the move.

It also drives the gripper with one raw ``ServosPosition`` per step on
``/ros_robot_controller/bus_servo/set_position`` (the gripper action clamps
``r_joint`` to +-1.257 rad on the robot, far short of the measured open stop).

Run inside the robot container with the vendor environment (CycloneDDS)::

    python3 /tmp/fast_motion.py --target 0,0.2574,-1.18773,-1.70,-0.01676 \
        --vmax 1.2 --amax 6.0 --i-have-clear-workspace

``--bench N`` sends ``N`` points at the *current commanded* pose, i.e. no
motion, and reports the wall time against the planned duration. That is the
per-segment overhead of the vendor controller, measured without moving the
arm.
"""

import argparse
import json
import sys
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from ros_robot_controller_msgs.msg import (
    GetBusServoCmd,
    ServoPosition,
    ServosPosition,
)
from ros_robot_controller_msgs.srv import GetBusServoState
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

import motion_profile
from vendor_servo_map import (
    ARM_JOINTS,
    GRIPPER_CLOSED_PULSE,
    GRIPPER_OPEN_PULSE,
    SERVO_IDS,
    check_waypoints,
    pulse_to_rad,
)

ARM_ACTION = '/arm_controller/follow_joint_trajectory'
POSITION_TOPIC = '/ros_robot_controller/bus_servo/set_position'
STATE_SERVICE = '/ros_robot_controller/bus_servo/get_state'


def stamp_duration(seconds):
    """Build a ``builtin_interfaces/Duration`` without an overflow nanosec."""
    nanoseconds = int(round(seconds * 1e9))
    return Duration(sec=nanoseconds // 1000000000,
                    nanosec=nanoseconds % 1000000000)


def _stable(samples, count, tol, window):
    """Return True when no register moved more than ``tol`` in ``window``."""
    if len(samples) < 3:
        return False
    if samples[-1][0] - samples[0][0] < window:
        return False
    cutoff = samples[-1][0] - window
    tail = [sample for sample in samples if sample[0] >= cutoff]
    return all(max(sample[index + 1] for sample in tail)
               - min(sample[index + 1] for sample in tail) <= tol
               for index in range(count))


def _peak_to_peak(samples, names, tail_fraction=0.0):
    """Return ``{joint_name: max - min}`` over the register samples.

    ``samples`` rows are ``[t, value, ...]`` in ``names`` order. With
    ``tail_fraction`` only the last fraction of the time window is used, which
    separates a decaying stop from a steady limit cycle.
    """
    if not samples:
        return {}
    if tail_fraction > 0.0:
        cutoff = samples[-1][0] * (1.0 - tail_fraction)
        tail = [sample for sample in samples if sample[0] >= cutoff]
        samples = tail if len(tail) > 1 else samples[-1:]
    return {name: max(sample[index + 1] for sample in samples)
            - min(sample[index + 1] for sample in samples)
            for index, name in enumerate(names)}


class FastMover(Node):
    """Arm trajectory client, bus-servo readback and gripper publisher."""

    def __init__(self, lead=0.06):
        """Create the clients. ``lead`` is the trajectory start delay."""
        super().__init__('fast_motion')
        self.lead = float(lead)
        self.arm = ActionClient(self, FollowJointTrajectory, ARM_ACTION)
        self.servo = self.create_client(GetBusServoState, STATE_SERVICE)
        self.position_pub = self.create_publisher(
            ServosPosition, POSITION_TOPIC, 1)
        qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                         durability=DurabilityPolicy.VOLATILE)
        self.joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState, '/controller_manager/joint_states',
            self._joint_state_callback, qos)

    def _joint_state_callback(self, msg):
        self.joint_state = msg

    # ---------------------------------------------------------------- reads
    def read_pulses(self, names=None):
        """Read the requested servo registers as ``{joint_name: pulse}``."""
        if not self.servo.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('bus servo state service unavailable')
        names = names or ARM_JOINTS
        request = GetBusServoState.Request()
        for name in names:
            command = GetBusServoCmd()
            command.id = int(SERVO_IDS[name])
            command.get_position = 1
            request.cmd.append(command)
        future = self.servo.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('bus servo read failed')
        return {name: int(state.position[0]) for name, state
                in zip(names, response.state) if state.position}

    def measured_joints(self):
        """Return the measured five-joint pose in commanded radians."""
        pulses = self.read_pulses()
        return [pulse_to_rad(name, pulses[name]) for name in ARM_JOINTS]

    def commanded_joints(self, timeout=2.0):
        """Return the last commanded pose from ``/controller_manager``."""
        deadline = time.monotonic() + timeout
        while self.joint_state is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.joint_state is None:
            raise RuntimeError('no /controller_manager/joint_states message')
        by_name = dict(zip(self.joint_state.name, self.joint_state.position))
        missing = [name for name in ARM_JOINTS if name not in by_name]
        if missing:
            raise RuntimeError('joint_states missing {}'.format(missing))
        return [float(by_name[name]) for name in ARM_JOINTS]

    # --------------------------------------------------------------- writes
    def send_single_point(self, client, names, values, duration):
        """Send one single-point trajectory and wait for its result."""
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('action server not available')
        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in values]
        point.velocities = [0.0] * len(values)
        point.time_from_start = stamp_duration(duration)
        trajectory.points = [point]
        goal.trajectory = trajectory
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('goal rejected')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result, timeout_sec=duration + 15.0)
        outcome = result.result()
        if outcome is None:
            raise RuntimeError('trajectory result timeout')
        code = getattr(outcome.result, 'error_code', 0)
        if code not in (0, FollowJointTrajectory.Result.SUCCESSFUL):
            raise RuntimeError('trajectory failed: {} {}'.format(
                code, getattr(outcome.result, 'error_string', '')))

    def slow_move(self, values, duration=6.0, settle=0.6):
        """One single-point arm move (the pre-2026-11-20 behaviour)."""
        started = time.monotonic()
        self.send_single_point(self.arm, ARM_JOINTS, values, duration)
        time.sleep(settle)
        return {'planned_s': duration, 'segments': 1, 'mode': 'slow',
                'wall_s': round(time.monotonic() - started, 4)}

    def arm_move(self, target, vmax, amax, dt=0.05, trace_hz=0.0,
                 settle=True, settle_tol=3, settle_timeout=2.0,
                 hold_trace=0.0):
        """Plan and execute one S-curve move; return a report dict."""
        start = self.measured_joints()
        times, points, info = motion_profile.plan_move(
            start, target, vmax, amax, dt)
        warnings = check_waypoints(points, strict_adopted=('joint2',))
        if not self.arm.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('arm action server unavailable')

        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = list(ARM_JOINTS)
        for t, point in zip(times, points):
            waypoint = JointTrajectoryPoint()
            waypoint.positions = [float(value) for value in point]
            waypoint.velocities = [0.0] * len(point)
            waypoint.time_from_start = stamp_duration(t)
            trajectory.points.append(waypoint)
        trajectory.header.stamp = (
            self.get_clock().now()
            + rclpy.duration.Duration(seconds=self.lead)).to_msg()
        goal.trajectory = trajectory

        steps = sorted(range(len(ARM_JOINTS)),
                       key=lambda index: -abs(target[index] - start[index]))
        trace_names = [ARM_JOINTS[index] for index in steps[:2]]

        started = time.monotonic()
        future = self.arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('arm goal rejected')
        result_future = handle.get_result_async()
        trace = []
        next_trace = started + self.lead
        while not result_future.done():
            rclpy.spin_once(self, timeout_sec=0.002)
            if trace_hz > 0.0 and time.monotonic() >= next_trace:
                sample = self.read_pulses(trace_names)
                trace.append([round(time.monotonic() - started, 4)]
                             + [sample[name] for name in trace_names])
                next_trace += 1.0 / trace_hz
        wall = time.monotonic() - started
        outcome = result_future.result()
        code = getattr(outcome.result, 'error_code', 0) if outcome else -1
        text = getattr(outcome.result, 'error_string', '') if outcome else ''

        report = {
            'planned_s': round(info['total'], 4),
            'lead_s': self.lead,
            'wall_s': round(wall, 4),
            'segments': info['segments'],
            'v_peak': round(info['v_peak'], 4),
            't_accel': round(info['t_accel'], 4),
            't_cruise': round(info['t_cruise'], 4),
            'peak_joint_velocity': [round(v, 4)
                                    for v in info['peak_velocity']],
            'target': [round(value, 5) for value in target],
            'start': [round(value, 5) for value in start],
            'result_code': int(code),
            'result_text': text,
            'warnings': warnings,
            'trace_names': trace_names,
            'trace': trace,
        }
        if settle:
            settle_started = time.monotonic()
            settled, samples = self.wait_settled(
                trace_names, tol=settle_tol, timeout=settle_timeout)
            report['settled'] = settled
            report['settle_s'] = round(time.monotonic() - settle_started, 4)
            report['settle_samples'] = [
                [round(sample[0] - settle_started, 4)] + sample[1:]
                for sample in samples]
            report['pulses_after'] = self.read_pulses()
            if samples:
                final = samples[-1][1:]
                report['settle_excursion_pulses'] = {
                    name: max(abs(sample[index + 1] - final[index])
                              for sample in samples)
                    for index, name in enumerate(trace_names)}
        if hold_trace > 0.0:
            report['hold_samples'] = self.hold_trace(
                trace_names, hold_trace)
            report['hold_p2p_pulses'] = _peak_to_peak(
                report['hold_samples'], trace_names)
            report['hold_tail_p2p_pulses'] = _peak_to_peak(
                report['hold_samples'], trace_names, tail_fraction=0.4)
        return report

    def hold_trace(self, names, duration):
        """Sample the registers for ``duration`` seconds after the move.

        This is the vibration probe: a settled arm repeats its register to
        +-1 count, an arm that rings after a fast stop shows a larger spread.
        """
        samples = []
        started = time.monotonic()
        while time.monotonic() - started < duration:
            sample = self.read_pulses(names)
            samples.append([round(time.monotonic() - started, 4)]
                           + [sample[name] for name in names])
        return samples

    def wait_settled(self, names, tol=3, window=0.12, timeout=2.0,
                     min_time=0.0):
        """Poll until every register moved at most ``tol`` over ``window``.

        A per-read tolerance is too weak: at the ~60 Hz read rate two counts
        per read is 120 counts/s, which a slowly creeping joint or the gripper
        still interpolating passes. The window criterion asks whether the
        register *saw* more than ``tol`` counts of travel in ``window``
        seconds. ``min_time`` delays the first possible pass, which matters for
        the gripper: its firmware queues a command and starts ~0.1 s later.

        Returns ``(settled, samples)`` with absolute monotonic timestamps.
        """
        started = time.monotonic()
        deadline = started + timeout
        samples = []
        while time.monotonic() < deadline:
            sample = self.read_pulses(names)
            samples.append([time.monotonic()]
                           + [sample[name] for name in names])
            if time.monotonic() - started >= min_time and _stable(
                    samples, len(names), tol, window):
                return True, samples
        return False, samples

    def gripper_send(self, pulse, duration):
        """Publish one raw gripper command and return immediately."""
        pulse = int(pulse)
        if not GRIPPER_OPEN_PULSE <= pulse <= GRIPPER_CLOSED_PULSE:
            raise ValueError('gripper pulse {} outside {}..{}'.format(
                pulse, GRIPPER_OPEN_PULSE, GRIPPER_CLOSED_PULSE))
        message = ServosPosition()
        message.duration = float(duration)
        message.position = [ServoPosition(id=1, position=pulse)]
        self.position_pub.publish(message)

    def wait_for_gripper(self, timeout=2.0):
        """Wait until at least one subscriber is matched on the raw topic."""
        deadline = time.monotonic() + timeout
        while self.position_pub.get_subscription_count() < 1:
            if time.monotonic() >= deadline:
                return False
            rclpy.spin_once(self, timeout_sec=0.02)
        return True

    def gripper(self, pulse, duration):
        """Send one raw gripper command (pulse) and return when it is due."""
        self.gripper_send(pulse, duration)
        time.sleep(duration)

    def gripper_watch(self, pulse, duration, window=2.0):
        """Command one gripper pulse and sample its register for ``window``.

        The gripper servo is slower than the arm joints; this records what it
        actually did instead of inferring it from the commanded duration.
        """
        started = time.monotonic()
        self.gripper_send(pulse, duration)
        samples = []
        while time.monotonic() - started < window:
            sample = self.read_pulses(['gripper'])
            samples.append([round(time.monotonic() - started, 4),
                            sample['gripper']])
        return {'command': pulse, 'duration_s': duration,
                'samples': samples,
                'final': samples[-1][1] if samples else None}

    def gripper_wait(self, pulse, duration, tol=3, timeout=1.5):
        """Send one gripper command and wait until the register stops.

        The timeout must cover the commanded interpolation: with a long
        duration the vendor servo tracks the ramp slowly (measured about
        100 pulses/s, so a full open from the gripper grip takes ~3 s). The
        old fixed 1.5 s gave up halfway and reported a partial open as if it
        were the final position (the 2026-11-21 p015 release stall).
        """
        started = time.monotonic()
        timeout = max(timeout, duration + 1.0)
        self.gripper_send(pulse, duration)
        settled, samples = self.wait_settled(
            ['gripper'], tol=tol, timeout=timeout,
            min_time=duration + 0.15)
        final = samples[-1][1] if samples else None
        return {'command': pulse, 'duration': round(duration, 3),
                'wall_s': round(time.monotonic() - started, 3),
                'settled': settled, 'register': final,
                'error_pulses': None if final is None else final - pulse,
                'samples': [[round(sample[0] - started, 4)] + sample[1:]
                            for sample in samples]}


def parse_pose(text):
    """Parse a comma-separated five-joint pose."""
    values = [float(value) for value in text.split(',')]
    if len(values) != 5:
        raise SystemExit('pose needs 5 values, got {}'.format(len(values)))
    return values


def run_bench(node, count, total=2.0):
    """Send ``count`` points at the current commanded pose (no motion)."""
    pose = node.commanded_joints()
    goal = FollowJointTrajectory.Goal()
    trajectory = JointTrajectory()
    trajectory.joint_names = list(ARM_JOINTS)
    for index in range(1, count + 1):
        t = total * index / count
        waypoint = JointTrajectoryPoint()
        waypoint.positions = [float(value) for value in pose]
        waypoint.velocities = [0.0] * len(pose)
        waypoint.time_from_start = stamp_duration(t)
        trajectory.points.append(waypoint)
    trajectory.header.stamp = (
        node.get_clock().now()
        + rclpy.duration.Duration(seconds=node.lead)).to_msg()
    goal.trajectory = trajectory
    if not node.arm.wait_for_server(timeout_sec=10.0):
        raise RuntimeError('arm action server unavailable')
    started = time.monotonic()
    future = node.arm.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
    handle = future.result()
    if handle is None or not handle.accepted:
        raise RuntimeError('bench goal rejected')
    result = handle.get_result_async()
    rclpy.spin_until_future_complete(
        node, result, timeout_sec=total + node.lead + 15.0)
    wall = time.monotonic() - started
    outcome = result.result()
    code = getattr(outcome.result, 'error_code', 0) if outcome else -1
    return {'points': count, 'planned_s': total, 'wall_s': round(wall, 4),
            'overhead_s': round(wall - total - node.lead, 4),
            'per_segment_ms': round(
                1000.0 * (wall - total - node.lead) / count, 2),
            'lead_s': node.lead, 'result_code': int(code),
            'commanded_pose': [round(value, 5) for value in pose]}


def _sweep_report(node, vmax, target, args, label):
    """Run one sweep move and tag it with the speed setting."""
    report = node.arm_move(
        target, vmax, args.amax, args.dt, trace_hz=args.trace_hz,
        settle=not args.no_settle, settle_timeout=args.settle_timeout,
        hold_trace=args.hold_trace)
    report['vmax'] = vmax
    report['label'] = label
    return report


def main():
    """Command-line entry point."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--target', help='five commanded joint radians')
    parser.add_argument('--vmax', type=float, default=4.0,
                        help='peak joint velocity (rad/s), default 4.0 '
                             '(the measured 2026-11-20 ceiling)')
    parser.add_argument('--amax', type=float, default=20.0,
                        help='peak joint acceleration (rad/s^2), default 20')
    parser.add_argument('--dt', type=float,
                        default=motion_profile.MIN_SEGMENT_S,
                        help='maximum segment length (s)')
    parser.add_argument('--lead', type=float, default=0.06)
    parser.add_argument('--trace-hz', type=float, default=0.0,
                        help='sample two moving joints during the move')
    parser.add_argument('--no-settle', action='store_true')
    parser.add_argument('--settle-timeout', type=float, default=2.0)
    parser.add_argument('--bench', type=int, default=0,
                        help='send N points at the current pose (no motion)')
    parser.add_argument('--bench-total', type=float, default=2.0)
    parser.add_argument('--hold-trace', type=float, default=0.0,
                        help='sample the settled arm for this many seconds')
    parser.add_argument('--sweep',
                        help='comma-separated vmax values; move to --target '
                             'and back for each one')
    parser.add_argument('--gripper', type=int,
                        help='raw gripper pulse to command (1..700 stop)')
    parser.add_argument('--gripper-duration', type=float, default=0.5)
    parser.add_argument('--gripper-watch', type=float, default=0.0,
                        help='sample the gripper register for this long')
    parser.add_argument('--output')
    parser.add_argument('--i-have-clear-workspace', action='store_true')
    parser.add_argument('--plan-only', action='store_true')
    args = parser.parse_args()

    if args.plan_only:
        start = [0.0] * 5
        target = parse_pose(args.target)
        times, points, info = motion_profile.plan_move(
            start, target, args.vmax, args.amax, args.dt)
        print(json.dumps({'info': info, 'times': [round(t, 4) for t in times],
                          'warnings': check_waypoints(points)}, indent=2))
        return 0

    if not args.i_have_clear_workspace:
        raise SystemExit('pass --i-have-clear-workspace to command a servo')

    rclpy.init()
    node = FastMover(lead=args.lead)
    report = {'ok': False}
    try:
        if args.sweep:
            target = parse_pose(args.target)
            home = node.measured_joints()
            values = [float(value) for value in args.sweep.split(',')]
            moves = []
            for vmax in values:
                print('SWEEP vmax={:.2f} -> target'.format(vmax), flush=True)
                moves.append(_sweep_report(
                    node, vmax, target, args, 'target'))
                print('SWEEP vmax={:.2f} -> home'.format(vmax), flush=True)
                moves.append(_sweep_report(node, vmax, home, args, 'home'))
            report = {'mode': 'sweep', 'home': [round(value, 5)
                                                for value in home],
                      'target': [round(value, 5) for value in target],
                      'moves': moves}
            report['ok'] = all(move['result_code'] == 0 for move in moves)
        elif args.bench:
            report = run_bench(node, args.bench, args.bench_total)
            report['mode'] = 'bench'
        elif args.gripper is not None:
            if args.gripper_watch > 0.0:
                report = node.gripper_watch(
                    args.gripper, args.gripper_duration,
                    window=args.gripper_watch)
            else:
                report = node.gripper_wait(
                    args.gripper, args.gripper_duration,
                    timeout=args.settle_timeout)
            report['mode'] = 'gripper'
        else:
            if not args.target:
                raise SystemExit('need --target, --bench, --sweep or '
                                 '--gripper')
            target = parse_pose(args.target)
            report = node.arm_move(
                target, args.vmax, args.amax, args.dt,
                trace_hz=args.trace_hz, settle=not args.no_settle,
                settle_timeout=args.settle_timeout,
                hold_trace=args.hold_trace)
            report['mode'] = 'move'
        if 'ok' not in report:
            report['ok'] = True
    except Exception as exc:  # noqa: BLE001 - report and keep the log
        report['error'] = str(exc)
        print('FAILED:', exc, file=sys.stderr, flush=True)
    finally:
        if args.output:
            with open(args.output, 'w', encoding='utf-8') as handle:
                json.dump(report, handle, indent=2)
        node.destroy_node()
        rclpy.shutdown()
    print(json.dumps(report, indent=2), flush=True)
    return 0 if report.get('ok') else 1


if __name__ == '__main__':
    sys.exit(main())
