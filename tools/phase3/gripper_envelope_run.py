#!/usr/bin/env python3
"""Run the gripper tolerance-envelope grid on the real robot.

The cube stays at the fixture; the commanded target is deliberately offset
from the measured top-face centre along one base axis at a time:

* ``x`` — along the finger length (the arm's pointing direction);
* ``y`` — across the gripper opening, the grip-critical axis.

Each offset is attempted twice, in a seeded random order, and every trial
records the grasp outcome and the release landing. The aggregated result is
fed to ``measurement_protocol.py gripper_envelope --mode analyze``, which
reports the largest single-axis offset that still grasps reliably. That turns
the provisional 5 mm tolerance into a measured one.

The deep-family joint fit and the grasp sequence are the validated ones from
the paired trial. A placement whose top face is more than 10 deg off the arm's
azimuth is refused: a square cube then presents its ~50 mm diagonal to a
~40 mm opening and the trial would measure geometry, not tolerance.

    python3 tools/phase3/gripper_envelope_run.py \
        --trials evidence/phase3/onsite_20261119/envelope/trials.jsonl
"""

import argparse
import json
import math
import os
import random
from datetime import datetime, timezone

import numpy as np

from _bootstrap import REPO, bootstrap  # noqa: E402
bootstrap()

import grasp_joint_fit  # noqa: E402
import paired_trial_session as pts  # noqa: E402

DEFAULT_CORRECTION_MM = [13.7, 3.9, 8.9]
DEFAULT_OFFSETS_MM = [0, 3, -3, 6, -6, 9, -9]
DEFAULT_Y_OFFSETS_MM = [0, 3, -3, 6, -6, 9, -9, 12, -12, 15, -15]
DEFAULT_Z_OFFSETS_MM = [0, 3, -3, 6, -6, 9, -9]
AXIS_INDEX = {'x': 0, 'y': 1, 'z': 2}
DEFAULT_VIEW_POSE = [0.0, 0.2574, -1.18773, -1.70, -0.01676]
MAX_RELATIVE_YAW_DEG = 10.0


def build_schedule(x_offsets, y_offsets, z_offsets, axis_repeats, seed):
    """Return the randomized trial list [(block, axis, offset_mm), ...].

    ``x`` keeps its zero offset; ``y`` and ``z`` drop theirs because a zero
    offset on any axis is the same central target.
    """
    points = [('x', int(offset)) for offset in x_offsets]
    points += [('y', int(offset)) for offset in y_offsets if offset != 0]
    points += [('z', int(offset)) for offset in z_offsets if offset != 0]
    schedule = []
    for block in range(axis_repeats):
        block_points = list(points)
        random.Random(seed + block).shuffle(block_points)
        schedule.extend((block + 1, axis, offset)
                        for axis, offset in block_points)
    return schedule


def relative_yaw_deg(report, point):
    """Return the cube-face vs arm-azimuth angle, folded to +/-45 deg."""
    top = (report.get('selected') or {}).get('top') or {}
    yaw = float(top.get('yaw_rad', 0.0))
    azimuth = math.atan2(float(point[1]), float(point[0]))
    turn = math.pi / 2.0
    relative = (yaw - azimuth + turn / 2.0) % turn - turn / 2.0
    return math.degrees(relative)


def now_iso():
    """Return the current UTC time as an ISO-8601 string."""
    return datetime.now(timezone.utc).isoformat()


class EnvelopeSession:
    """One gripper-envelope session."""

    def __init__(self, args):
        self.args = args
        self.correction = np.asarray(
            [float(v) for v in args.correction.split(',')],
            dtype=np.float64) / 1000.0
        self.view_pose = [float(v) for v in args.view_pose.split(',')]
        self.offsets = [int(v) for v in args.offsets.split(',')]
        self.y_offsets = [int(v) for v in args.y_offsets.split(',')]
        self.z_offsets = [int(v) for v in args.z_offsets.split(',')]
        self.robot = pts.RobotLink(args.robot, args.robot_container)
        self.humble = pts.HumbleLink(
            args.humble_container, args.hand_eye_container, REPO)
        self.done = self._load_done()

    def _load_done(self):
        if not os.path.exists(self.args.trials):
            return set()
        done = set()
        with open(self.args.trials, 'r', encoding='utf-8') as handle:
            for line in handle:
                record = json.loads(line)
                done.add((record['block'], record['axis'],
                          record['offset_mm']))
        return done

    def prepare(self):
        """Install the helpers and put the arm at the view pose."""
        if self.args.dry_run:
            return
        self.robot.install_tools(os.path.join(REPO, 'tools', 'phase2'))
        self.humble.install_hand_eye(os.path.join(
            REPO, self.args.hand_eye_host))
        self.robot.move_to_pose(self.view_pose, tag='envelope_view')
        print('arm at the view pose.')

    def measure(self, retries=3):
        """Measure the cube and enforce the face-toward-arm gate."""
        for attempt in range(retries):
            pulses = self.robot.pulses()
            report = self.humble.segment(
                pulses, 'env_{}'.format(int(datetime.now().timestamp())))
            point, valid = pts.cube_point(report)
            ok, reason = pts.placement_quality(report)
            if valid and ok:
                relative = relative_yaw_deg(report, point)
                if abs(relative) <= MAX_RELATIVE_YAW_DEG:
                    return point, report, relative
                reason = 'cube face {:.1f} deg off the arm'.format(relative)
            elif not valid:
                reason = 'no valid top face: {}'.format(
                    (report.get('rejection') or {}).get(
                        'reason', 'no usable cluster'))
            print('placement rejected: {}'.format(reason))
            if point is not None:
                print('  measured point ({:+.3f}, {:+.3f}) m'.format(
                    point[0], point[1]))
            if attempt + 1 < retries and not self.args.dry_run:
                input('  place the cube flat, one face toward the arm; '
                      'press Enter to re-measure ... ')
        return None, report, None

    def trial(self, entry):
        """Run one offset trial and append its record."""
        block, axis, offset = entry
        print('\n=== block {} / {} {:+d} mm ==='.format(
            block, axis, offset))
        if self.args.dry_run:
            return self._dry_trial(entry)
        if not self.args.dry_run:
            input('  place the cube flat, one face toward the arm; '
                  'press Enter ... ')
        point, report, relative = self.measure()
        if point is None:
            raise RuntimeError('no usable placement for {} {:+d}'.format(
                axis, offset))
        offset_vector = np.zeros(3)
        offset_vector[AXIS_INDEX[axis]] = offset / 1000.0
        target = point + self.correction + offset_vector
        fit = grasp_joint_fit.fit(target)
        pts.validate_target(target, fit, self.args.max_xy_error_mm)
        print('  cube ({:+.4f}, {:+.4f}) face {:.1f} deg  joints {}'.format(
            point[0], point[1], relative,
            ','.join('{:.4f}'.format(v) for v in fit['joints'][:4])))
        grasp_report = self.robot.grasp(
            fit['joints'], self.view_pose,
            'env_b{}_s{}_{:+d}'.format(block, axis, offset))
        grip_pulses = [
            grasp_report['pulses_at_verify'][str(servo)]
            for servo in (6, 5, 4, 3, 2)
        ]
        grip_report = self.humble.segment(
            grip_pulses, 'env_{}_{}_{:+d}_grip'.format(block, axis, offset))
        in_box = pts.cube_box_present(grip_report, point)
        notes = {
            'cube_point': [float(v) for v in point],
            'target': [float(v) for v in target],
            'joints': fit['joints'],
            'relative_yaw_deg': relative,
            'grip_cube_box': bool(in_box),
            'grip_clusters': grip_report['segmentation']['clusters'],
        }
        error_mm = None
        if not in_box:
            release_report = self.robot.release(
                fit['joints'], self.view_pose,
                'env_b{}_s{}_{:+d}'.format(block, axis, offset))
            land_pulses = [
                release_report['pulses_at_verify'][str(servo)]
                for servo in (6, 5, 4, 3, 2)
            ]
            land_report = self.humble.segment(
                land_pulses,
                'env_{}_{}_{:+d}_land'.format(block, axis, offset))
            landing, valid = pts.cube_point(land_report)
            land_ok, reason = pts.placement_quality(land_report)
            # The primary outcome is the grip. A release can land tilted and
            # shrink the projected top face (a 23 mm y extent was seen on
            # 2026-11-19), so an unusable landing is recorded, not fatal.
            if landing is None or not valid or not land_ok:
                notes['landing_unmeasured'] = (
                    reason if landing is not None else 'no point')
            else:
                error_mm = float(
                    np.linalg.norm((landing - point)[:2]) * 1000.0)
                notes['landing'] = [float(v) for v in landing]
        record = {
            'block': block,
            'axis': axis,
            'offset_mm': int(offset),
            'success': not in_box,
            'error_mm': error_mm,
            'recorded_at': now_iso(),
            'notes': json.dumps(notes, sort_keys=True),
        }
        directory = os.path.dirname(os.path.abspath(self.args.trials))
        os.makedirs(directory, exist_ok=True)
        with open(self.args.trials, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(record) + '\n')
        self.done.add(entry)
        print('  {}  {}'.format(
            'GRIP' if record['success'] else 'MISS',
            '' if error_mm is None else 'landing {:.2f} mm'.format(error_mm)))
        return record['success']

    def _dry_trial(self, entry):
        """Record a synthetic outcome without touching the robot."""
        block, axis, offset = entry
        limit = 9 if axis == 'x' else 6
        success = abs(offset) <= limit
        record = {
            'block': block,
            'axis': axis,
            'offset_mm': int(offset),
            'success': success,
            'error_mm': 2.0 if success else None,
            'recorded_at': now_iso(),
            'notes': json.dumps({'dry_run': True}),
        }
        directory = os.path.dirname(os.path.abspath(self.args.trials))
        os.makedirs(directory, exist_ok=True)
        with open(self.args.trials, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(record) + '\n')
        self.done.add(entry)
        print('  {} (dry)'.format('GRIP' if success else 'MISS'))
        return success

    def run(self):
        """Run every pending trial in schedule order."""
        self.prepare()
        schedule = build_schedule(
            self.offsets, self.y_offsets, self.z_offsets,
            self.args.repeats, self.args.seed)
        pending = [entry for entry in schedule if entry not in self.done]
        print('{} pending of {} trials'.format(len(pending), len(schedule)))
        if self.args.max_trials:
            pending = pending[:self.args.max_trials]
        for entry in pending:
            self.trial(entry)
        self.aggregate()

    def aggregate(self):
        """Write the protocol measurement JSON from the per-trial records."""
        rows = [json.loads(line) for line in open(self.args.trials)]
        groups = {}
        for row in rows:
            key = (row['axis'], row['offset_mm'])
            groups.setdefault(key, []).append(row)
        samples = []
        for (axis, offset), group in sorted(groups.items()):
            vector = [0.0, 0.0, 0.0]
            vector[AXIS_INDEX[axis]] = offset / 1000.0
            samples.append({
                'offset_m': vector,
                'yaw_deg': 0.0,
                'trials': len(group),
                'successes': sum(1 for row in group if row['success']),
            })
        measurement = {
            'schema_version': 1,
            'protocol': 'gripper_envelope',
            'generated_at': now_iso(),
            'samples': samples,
        }
        output = self.args.measurement_out
        os.makedirs(os.path.dirname(os.path.abspath(output)), exist_ok=True)
        with open(output, 'w', encoding='utf-8') as handle:
            json.dump(measurement, handle, indent=2)
        print('wrote', output)


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--trials',
        default='evidence/phase3/onsite_20261119/envelope/trials.jsonl')
    parser.add_argument(
        '--measurement-out',
        default='evidence/phase3/onsite_20261119/envelope/measurement.json')
    parser.add_argument('--offsets', default=','.join(
        str(v) for v in DEFAULT_OFFSETS_MM), help='x offsets')
    parser.add_argument('--y-offsets', default=','.join(
        str(v) for v in DEFAULT_Y_OFFSETS_MM), help='y offsets')
    parser.add_argument('--z-offsets', default=','.join(
        str(v) for v in DEFAULT_Z_OFFSETS_MM), help='z offsets')
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument(
        '--correction',
        default=','.join(str(v) for v in DEFAULT_CORRECTION_MM))
    parser.add_argument(
        '--view-pose', default=','.join(str(v) for v in DEFAULT_VIEW_POSE))
    parser.add_argument(
        '--hand-eye-host',
        default='evidence/calibration/20261118/table_down/'
                'calibration_table_down_20261118.json')
    parser.add_argument(
        '--hand-eye-container', default='/tmp/hand_eye_table_down.json')
    parser.add_argument('--robot', default='pi@10.42.0.218')
    parser.add_argument('--robot-container', default='ArmPiUltra')
    parser.add_argument('--humble-container', default='armpi-humble')
    parser.add_argument('--max-xy-error-mm', type=float, default=3.0)
    parser.add_argument('--max-trials', type=int, default=0)
    parser.add_argument('--dry-run', action='store_true')
    return parser


def main(argv=None):
    """Run the session."""
    args = build_parser().parse_args(argv)
    session = EnvelopeSession(args)
    try:
        session.run()
    except KeyboardInterrupt:
        print('\ninterrupted; completed trials are recorded')
        return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
