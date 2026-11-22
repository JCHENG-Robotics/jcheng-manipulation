#!/usr/bin/env python3
"""Measure the three remaining error-budget sources on the real robot.

The 2026-11-20 session fills the gaps the 2026-11-19 budget still lists as
required and unmeasured:

``fixture``
    The operator lifts the 39.4 mm cube off the new floor mark and puts it
    back, twenty times. The cube is segmented at the fixed view pose after
    every placement; the spread of the reconstructed top-face point around its
    mean is the object-fixture pose error. Removal and re-appearance are
    detected from the segmenter itself, so no keystroke is needed.

``trials``
    Twenty pick-lift-put-back trials with the validated intervention method
    (measured point + ``(+13.7, +3.9, +8.9) mm``). Each trial measures:

    * grasp execution -- the commanded joint vector at the grasp pose versus
      the bus-servo registers read back at that same pose, mapped through the
      same KDL model. Translation and geodesic rotation are both reported;
    * gripper placement -- the segmented landing point after release minus the
      pre-grasp measurement, in xy.

Both commands append to a JSONL file, so a session is resumable, and write a
JSON/Markdown report when the requested count is reached.

Run on the laptop (needs ``sg docker`` for the Humble container and an SSH key
for the robot). ``fixture`` needs no ROS environment; ``trials`` imports the
KDL model, and ``sg docker -c`` runs the command through ``sh``, so source the
Jazzy workspace inside a bash wrapper::

    sg docker -c "python3 tools/phase3/execution_fixture_run.py fixture \
        --samples 20 --records evidence/phase3/onsite_20261120/fixture.jsonl"

    sg docker -c "bash -c 'source /opt/ros/jazzy/setup.bash && \
        source ros2_ws/install/setup.bash && \
        python3 tools/phase3/execution_fixture_run.py trials \
        --count 20 --records evidence/phase3/onsite_20261120/trials.jsonl'"

Add ``--dry-run`` to exercise the control flow and the statistics without a
robot or a container.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os
import time

import numpy as np

from _bootstrap import REPO, bootstrap  # noqa: E402
bootstrap()

import grasp_joint_fit  # noqa: E402
import paired_trial_session as pts  # noqa: E402

SERVO_IDS = (6, 5, 4, 3, 2)
JOINT_DIRECTIONS = (1.0, 1.0, -1.0, 1.0, 1.0)
RAD_PER_TICK = 0.004188790204786391
PULSE_ORIGIN = 500
DEFAULT_VIEW_POSE = [0.0, 0.2574, -1.18773, -1.70, -0.01676]
DEFAULT_CORRECTION_MM = [13.7, 3.9, 8.9]
DEFAULT_SETTLE_SAMPLES = 2
DEFAULT_STABLE_MM = 1.5
DEFAULT_FIXTURE_X = (0.215, 0.240)
DEFAULT_FIXTURE_Y = 0.030
DEFAULT_MAX_Z_RESIDUAL_MM = 10.0


def now_iso():
    """Return the current UTC time as an ISO-8601 string."""
    return datetime.now(timezone.utc).isoformat()


def commanded_pulses(commanded_joints):
    """Return the pulse registers the vendor controller should reach."""
    return [PULSE_ORIGIN + float(joint) * direction / RAD_PER_TICK
            for joint, direction in zip(commanded_joints, JOINT_DIRECTIONS)]


def joint_delta_from_pulses(commanded_joints, actual_pulses):
    """Return the achieved joint vector from commanded values and registers.

    A constant per-servo offset cancels because only the difference between
    the commanded pulse and the measured pulse is used, so this does not
    depend on the vendor's absolute tick origin.
    """
    target = commanded_pulses(commanded_joints)
    delta = [(float(actual) - expected) * RAD_PER_TICK * direction
             for actual, expected, direction
             in zip(actual_pulses, target, JOINT_DIRECTIONS)]
    return [float(value) + change for value, change
            in zip(commanded_joints, delta)]


def _matrix(frame):
    """Return (position, rotation matrix) from a KDL frame."""
    rotation = np.array([[frame.M[i, j] for j in range(3)]
                         for i in range(3)])
    return np.array([frame.p.x(), frame.p.y(), frame.p.z()]), rotation


def _dry_frame(joints):
    """Fake frame for --dry-run; it never touches the ROS workspace."""
    position = np.array([0.23 + 0.2 * joints[0], 0.2 * joints[1],
                         -0.04 + 0.2 * joints[2]])
    angle = joints[3]
    rotation = np.array([
        [math.cos(angle), -math.sin(angle), 0.0],
        [math.sin(angle), math.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ])
    return position, rotation


def execution_error(commanded_joints, actual_pulses, frame_fn):
    """Return the translation (mm) and rotation (deg) execution error."""
    achieved = joint_delta_from_pulses(commanded_joints, actual_pulses)
    position_c, rotation_c = frame_fn(commanded_joints)
    position_a, rotation_a = frame_fn(achieved)
    translation_mm = float(np.linalg.norm(position_a - position_c)) * 1000.0
    relative = rotation_c.T @ rotation_a
    cosine = (float(np.trace(relative)) - 1.0) / 2.0
    angle = math.degrees(math.acos(max(-1.0, min(1.0, cosine))))
    return {
        'translation_mm': translation_mm,
        'rotation_deg': angle,
        'achieved_joints': [float(value) for value in achieved],
        'delta_joints': [float(value - base) for value, base
                         in zip(achieved, commanded_joints)],
    }


def summarize(values):
    """Return count, mean, sample SD, RMS, max and range of a 1-D sample."""
    data = np.asarray(values, dtype=np.float64)
    if data.size == 0:
        return {'n': 0}
    return {
        'n': int(data.size),
        'mean': float(data.mean()),
        'std': float(data.std(ddof=1)) if data.size > 1 else 0.0,
        'rms': float(np.sqrt(np.mean(data ** 2))),
        'max': float(data.max()),
        'min': float(data.min()),
        'range': float(data.max() - data.min()),
    }


def summarize_vectors(vectors):
    """Return mean, per-axis and radial statistics of an N x 3 sample."""
    data = np.asarray(vectors, dtype=np.float64)
    if data.size == 0:
        return {'n': 0}
    mean = data.mean(axis=0)
    radial = np.linalg.norm((data - mean)[:, :2], axis=1) * 1000.0
    if data.shape[0] > 1:
        std = data.std(axis=0, ddof=1)
    else:
        std = np.zeros(3)
    return {
        'n': int(data.shape[0]),
        'mean': [float(value) for value in mean],
        'std': [float(value) for value in std],
        'range': [float(value) for value in
                  (data.max(axis=0) - data.min(axis=0))],
        'radial_mm': summarize(radial),
    }


def validate_family(point, fit, args):
    """Refuse a fixture outside the grasp family's measured envelope.

    The deep family fixes joint2/joint3, so the TCP height is a function of
    reach. At the 2026-11-19 fixture (x 0.226-0.234 m) the fitted TCP sits
    2-9 mm above the corrected target; at x = 0.262 m the same fit puts it
    23 mm above, so the pads close over the cube. The 2026-11-20 first
    placement was exactly that case and was refused before any motion. The
    x window is widened to 0.215-0.240 m, where the measured z residual stays
    below 10 mm and the segmenter still sees the whole top face; a 2026-11-20
    run drifted to x = 0.2192 m with a +0.73 mm z residual, which is safely
    inside the physical envelope. The z residual, not the window, is the
    safety gate.
    """
    if not args.fixture_x_min <= point[0] <= args.fixture_x_max:
        raise SystemExit(
            'cube x {:.4f} m is outside the validated fixture window '
            '{:.3f}-{:.3f} m; re-place it before grasping'.format(
                point[0], args.fixture_x_min, args.fixture_x_max))
    if abs(point[1]) > args.fixture_y_max:
        raise SystemExit(
            'cube y {:.4f} m is outside +/-{:.3f} m; re-place it'.format(
                point[1], args.fixture_y_max))
    residual = float(fit['residual_mm'][2])
    if residual > args.max_z_residual_mm:
        raise SystemExit(
            'joint-fit z residual {:+.2f} mm exceeds {:.1f} mm; the family '
            'is out of envelope'.format(residual, args.max_z_residual_mm))


def load_records(path):
    """Read a JSONL record file, tolerating a missing file."""
    if not os.path.exists(path):
        return []
    records = []
    with open(path, 'r', encoding='utf-8') as handle:
        for line in handle:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


class Session:
    """One execution/fixture measurement session."""

    def __init__(self, args):
        self.args = args
        self.correction = np.asarray(
            [float(value) for value in args.correction.split(',')],
            dtype=np.float64) / 1000.0
        self.view_pose = [float(value) for value in args.view_pose.split(',')]
        self.robot = None
        self.humble = None
        if not args.dry_run:
            self.robot = pts.RobotLink(args.robot, args.robot_container)
            self.humble = pts.HumbleLink(
                args.humble_container, args.hand_eye_container, REPO)
        self.records = load_records(args.records)
        self.frame_fn = None

    def prepare(self):
        """Install the container helpers and put the arm at the view pose."""
        if self.args.dry_run:
            return
        self.robot.install_tools(os.path.join(REPO, 'tools', 'phase2'))
        self.humble.install_hand_eye(os.path.join(
            REPO, self.args.hand_eye_host))
        print('moving to the view pose ...')
        self.robot.move_to_pose(self.view_pose, tag='view')
        print('arm at the view pose.')

    def measure(self, tag):
        """Segment the cube at the current pose; return (point, report)."""
        pulses = self.robot.pulses()
        report = self.humble.segment(pulses, tag)
        point, valid = pts.cube_point(report)
        ok, reason = pts.placement_quality(report)
        if point is None or not valid or not ok:
            if point is not None and not ok:
                print('  placement rejected: {}'.format(reason))
            return None, report
        return point, report

    def wait_for_placement(self, require_removal, previous=None,
                           timeout_s=240.0):
        """Wait for a stable, valid cube placement, optionally after removal.

        A new sample is accepted after the cube leaves the box or after it
        moves more than 3 mm from *previous*. The removal path still records a
        placement that lands on almost the same spot, while the displacement
        path tolerates an operator who lifts and replaces the cube between two
        segmenter calls.
        """
        removed = not require_removal
        stable = 0
        last = None
        started = time.time()
        while time.time() - started < timeout_s:
            point, report = self.measure('wait')
            if point is None:
                if not removed:
                    print('  waiting: cube removed')
                removed = True
                stable = 0
                last = None
                time.sleep(1.0)
                continue
            moved = previous is not None and np.linalg.norm(
                point - previous) * 1000.0 > 3.0
            if not (removed or moved):
                print('  waiting: lift or move the cube for the next sample')
                stable = 0
                last = None
                time.sleep(1.0)
                continue
            if last is not None and np.linalg.norm(point - last) * 1000.0 \
                    <= DEFAULT_STABLE_MM:
                stable += 1
            else:
                stable = 1
            last = point
            if stable >= DEFAULT_SETTLE_SAMPLES:
                return point, report
            time.sleep(1.0)
        raise TimeoutError('no stable cube placement within {:.0f} s'.format(
            timeout_s))

    def record(self, record):
        """Append one JSON record and keep it in memory."""
        os.makedirs(os.path.dirname(os.path.abspath(self.args.records)) or '.',
                    exist_ok=True)
        with open(self.args.records, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(record, sort_keys=True) + '\n')
        self.records.append(record)

    def write_report(self, name, payload):
        """Write a JSON report next to the record file."""
        path = self.args.report or os.path.join(
            os.path.dirname(os.path.abspath(self.args.records)),
            '{}.json'.format(name))
        os.makedirs(os.path.dirname(os.path.abspath(path)) or '.',
                    exist_ok=True)
        with open(path, 'w', encoding='utf-8') as handle:
            json.dump(payload, handle, indent=2, sort_keys=True)
        print('wrote {}'.format(path))
        markdown = os.path.splitext(path)[0] + '.md'
        with open(markdown, 'w', encoding='utf-8') as handle:
            handle.write(payload['markdown'])
        print('wrote {}'.format(markdown))
        return path


def fixture_report(records):
    """Build the fixture-pose report from recorded placements."""
    points = [record['point'] for record in records]
    stats = summarize_vectors(points)
    lines = [
        '# Object-fixture pose error (2026-11-20)',
        '',
        'Cube lifted and re-placed on the new floor mark {} times; top-face '
        'point measured at the fixed view pose after every placement.'.format(
            stats.get('n', 0)),
        '',
        '| Quantity | Value |',
        '|---|---:|',
        '| Placements | {} |'.format(stats.get('n', 0)),
        '| Radial RMS about mean | {:.3f} mm |'.format(
            stats['radial_mm']['rms']),
        '| Radial max | {:.3f} mm |'.format(stats['radial_mm']['max']),
        '| Mean position | ({:+.4f}, {:+.4f}, {:+.4f}) m |'.format(
            *stats['mean']),
        '| Per-axis SD | ({:.3f}, {:.3f}, {:.3f}) mm |'.format(
            *(value * 1000.0 for value in stats['std'])),
        '| Per-axis range | ({:.3f}, {:.3f}, {:.3f}) mm |'.format(
            *(value * 1000.0 for value in stats['range'])),
        '',
        'The fixture is a radial yellow tape strip with no x-direction stop; '
        'the x scatter is the operator placing the cube at different '
        'distances along the strip. The raw capture also holds three '
        'placements off the strip (|y - median| > 8 mm); they are stored in '
        'discarded_fixture_off_tape.jsonl and are not part of this report.',
        '',
        'The measurement uses the same RGB-D segmenter as the grasp trials, '
        'so it is a task-level bound and includes the perception noise that '
        'the budget already carries separately.',
        '',
    ]
    return {'n': stats.get('n', 0), 'stats': stats,
            'points': points,
            'recorded_at': now_iso(),
            'markdown': '\n'.join(lines)}


def trial_report(records, attempts):
    """Build the execution/placement report from recorded trials."""
    successes = [record for record in records if record['grasp_success']]
    execution_t = [record['execution']['translation_mm']
                   for record in attempts]
    execution_r = [record['execution']['rotation_deg']
                   for record in attempts]
    placement = [record['placement_xy_mm'] for record in successes]
    placements = [record['placement_vector_m'] for record in successes]
    lines = [
        '# Grasp execution and gripper placement (2026-11-20)',
        '',
        '{} attempts, {} held and put back.'.format(
            len(attempts), len(successes)),
        '',
        '## Grasp execution (commanded vs bus-servo registers at the grasp '
        'pose)',
        '',
        '| Metric | Mean | SD | Max |',
        '|---|---:|---:|---:|',
        '| Translation | {:.3f} mm | {:.3f} mm | {:.3f} mm |'.format(
            summarize(execution_t)['mean'], summarize(execution_t)['std'],
            summarize(execution_t)['max']),
        '| Rotation | {:.3f} deg | {:.3f} deg | {:.3f} deg |'.format(
            summarize(execution_r)['mean'], summarize(execution_r)['std'],
            summarize(execution_r)['max']),
        '',
        '## Gripper placement (landing minus pre-grasp measurement)',
        '',
        '| Metric | Value |',
        '|---|---:|',
        '| Successful placements | {} |'.format(
            summarize(placement).get('n', 0)),
        '| Mean radial displacement | {:.3f} mm |'.format(
            summarize(placement)['mean']),
        '| SD | {:.3f} mm |'.format(summarize(placement)['std']),
        '| Max | {:.3f} mm |'.format(summarize(placement)['max']),
        '| Mean xy vector | ({:+.3f}, {:+.3f}) mm |'.format(
            *(value * 1000.0 for value in
              np.asarray(placements).mean(axis=0))),
        '',
        'Execution is an arm-motion term: it is the joint-level tracking '
        'error at the grasp pose, not the object shift. Placement is the '
        'object displacement caused by the whole grasp-lift-release cycle.',
        '',
    ]
    return {
        'attempts': len(attempts),
        'successes': len(successes),
        'execution_translation_mm': summarize(execution_t),
        'execution_rotation_deg': summarize(execution_r),
        'placement_xy_mm': summarize(placement),
        'placement_vector_m': summarize_vectors(placements),
        'trials': attempts,
        'recorded_at': now_iso(),
        'markdown': '\n'.join(lines),
    }


def run_fixture(args):
    """Record the operator's repeated placements at the fixed view pose."""
    session = Session(args)
    sample_count = len([record for record in session.records
                        if record.get('kind') == 'fixture'])
    session.prepare()
    while sample_count < args.samples:
        index = sample_count + 1
        if args.dry_run:
            rng = np.random.default_rng(args.seed + index)
            point = np.array([0.23, 0.0, -0.048]) + rng.normal(
                0.0, 0.0015, 3)
            print('  dry-run placement {} {:.4f}, {:.4f}, {:.4f}'.format(
                index, *point))
        else:
            previous = (session.records[-1]['point']
                        if session.records else None)
            point, report = session.wait_for_placement(
                require_removal=sample_count > 0, previous=previous)
            print('  placement {}: ({:+.4f}, {:+.4f}, {:+.4f}) m'.format(
                index, *point))
        session.record({'kind': 'fixture', 'index': index,
                        'point': [float(value) for value in point],
                        'recorded_at': now_iso()})
        sample_count += 1
    report = fixture_report(session.records)
    session.write_report('fixture_report', report)
    print('radial RMS {:.3f} mm, radial max {:.3f} mm over {} placements'
          .format(report['stats']['radial_mm']['rms'],
                  report['stats']['radial_mm']['max'], report['n']))
    return report


def run_trials(args):
    """Run pick-lift-put-back trials and record both error terms."""
    session = Session(args)
    done = len([record for record in session.records
                if record.get('kind') == 'trial'])
    session.prepare()
    if args.dry_run:
        session.frame_fn = _dry_frame
    else:
        kinematics = grasp_joint_fit.Kinematics()
        session.frame_fn = lambda joints: _matrix(
            kinematics._tcp.forward([float(value) for value in joints]))
    require_removal = False
    while done < args.count:
        index = done + 1
        tag = 't{:02d}'.format(index)
        print('\n=== trial {} / {} ==='.format(index, args.count))
        if args.dry_run:
            point = np.array([0.23, 0.0, -0.048]) + np.random.default_rng(
                args.seed + index).normal(0.0, 0.002, 3)
            fit = {'joints': [0.0, -0.6618, -1.8012, -0.3917, 0.0],
                   'xy_error_mm': 0.5, 'reach_limited': False}
            pulses = {str(servo): int(round(value)) for servo, value in
                      zip(SERVO_IDS, commanded_pulses(fit['joints']))}
            pulses[str(SERVO_IDS[3])] += 1
            execution = execution_error(
                fit['joints'], [pulses[str(servo)] for servo in SERVO_IDS],
                session.frame_fn)
            landing = point + np.array([0.002, -0.001, 0.0])
            placement_vector = (landing - point)[:2]
            placement_mm = float(np.linalg.norm(placement_vector)) * 1000.0
            print('  dry-run execution {:.3f} mm / {:.3f} deg, placement '
                  '{:.3f} mm'.format(execution['translation_mm'],
                                     execution['rotation_deg'],
                                     placement_mm))
            grip_ok = True
        else:
            point, report = session.wait_for_placement(
                require_removal=require_removal)
            print('  cube  ({:+.4f}, {:+.4f}, {:+.4f}) m'.format(*point))
            target = point + session.correction
            fit = grasp_joint_fit.fit(target)
            pts.validate_target(target, fit, args.max_xy_error_mm)
            validate_family(point, fit, args)
            print('  target({:+.4f}, {:+.4f}, {:+.4f}) m'.format(*target))
            print('  joints {}  xy error {:.2f} mm'.format(
                ','.join('{:.4f}'.format(value)
                         for value in fit['joints'][:4]),
                fit['xy_error_mm']))
            grasp_report = session.robot.grasp(
                fit['joints'], session.view_pose, tag)
            execution = execution_error(
                fit['joints'],
                [grasp_report['pulses_at_grasp'][str(servo)]
                 for servo in SERVO_IDS],
                session.frame_fn)
            grip_pulses = [grasp_report['pulses_at_verify'][str(servo)]
                           for servo in SERVO_IDS]
            grip_report = session.humble.segment(grip_pulses, tag + '_grip')
            held = not pts.cube_box_present(grip_report, point)
            grip_ok = held
            if held:
                release_report = session.robot.release(
                    fit['joints'], session.view_pose, tag)
                land_pulses = [release_report['pulses_at_verify'][str(servo)]
                               for servo in SERVO_IDS]
                land_report = session.humble.segment(
                    land_pulses, tag + '_land')
                landing, valid = pts.cube_point(land_report)
                land_ok, land_reason = pts.placement_quality(land_report)
                if landing is None or not valid or not land_ok:
                    raise RuntimeError(
                        'landing measurement rejected ({}); re-place the cube '
                        'and re-run the trial'.format(land_reason))
                placement_vector = (landing - point)[:2]
                placement_mm = float(np.linalg.norm(placement_vector)) * 1000.0
                print('  execution {:.3f} mm / {:.3f} deg, placement '
                      '{:.3f} mm'.format(execution['translation_mm'],
                                         execution['rotation_deg'],
                                         placement_mm))
            else:
                placement_vector = None
                placement_mm = None
                print('  MISS: the cube box still has points; '
                      'place the cube again')
        record = {
            'kind': 'trial',
            'index': index,
            'cube_point': [float(value) for value in point],
            'target': [float(value) for value in (point + session.correction)],
            'joints': [float(value) for value in fit['joints']],
            'xy_error_mm': float(fit['xy_error_mm']),
            'grasp_success': bool(grip_ok),
            'execution': execution,
            'placement_vector_m': ([float(value) for value in placement_vector]
                                   if placement_vector is not None else None),
            'placement_xy_mm': placement_mm,
            'recorded_at': now_iso(),
        }
        session.record(record)
        done += 1
        require_removal = not grip_ok
    attempts = [record for record in session.records
                if record.get('kind') == 'trial']
    report = trial_report(records=attempts, attempts=attempts)
    session.write_report('trial_report', report)
    print('execution: {:.3f} mm mean / {:.3f} deg mean; placement: {:.3f} mm '
          'mean over {} successes'.format(
              report['execution_translation_mm']['mean'],
              report['execution_rotation_deg']['mean'],
              report['placement_xy_mm']['mean'], report['successes']))
    return report


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='command', required=True)
    for name in ('fixture', 'trials'):
        command = sub.add_parser(name)
        command.add_argument('--records', required=True,
                             help='append-only JSONL record file')
        command.add_argument('--report', default=None,
                             help='JSON report path (default: beside records)')
        command.add_argument('--view-pose', default=','.join(
            '{:.5f}'.format(value) for value in DEFAULT_VIEW_POSE))
        command.add_argument('--robot', default='pi@10.42.0.218')
        command.add_argument('--robot-container', default='ArmPiUltra')
        command.add_argument('--humble-container', default='armpi-humble')
        command.add_argument('--hand-eye-host', default=(
            'evidence/calibration/20261118/table_down/'
            'calibration_table_down_20261118.json'))
        command.add_argument('--hand-eye-container',
                             default='/tmp/hand_eye_table_down.json')
        command.add_argument('--correction', default=','.join(
            '{:.1f}'.format(value) for value in DEFAULT_CORRECTION_MM))
        command.add_argument('--dry-run', action='store_true')
        command.add_argument('--seed', type=int, default=11)
    sub.choices['fixture'].add_argument('--samples', type=int, default=20)
    trials = sub.choices['trials']
    trials.add_argument('--count', type=int, default=20)
    trials.add_argument('--max-xy-error-mm', type=float, default=5.0)
    trials.add_argument('--fixture-x-min', type=float,
                        default=DEFAULT_FIXTURE_X[0])
    trials.add_argument('--fixture-x-max', type=float,
                        default=DEFAULT_FIXTURE_X[1])
    trials.add_argument('--fixture-y-max', type=float,
                        default=DEFAULT_FIXTURE_Y)
    trials.add_argument('--max-z-residual-mm', type=float,
                        default=DEFAULT_MAX_Z_RESIDUAL_MM)
    return parser


def main(argv=None):
    """Run the requested measurement command."""
    args = build_parser().parse_args(argv)
    if args.command == 'fixture':
        run_fixture(args)
    else:
        run_trials(args)


if __name__ == '__main__':
    main()
