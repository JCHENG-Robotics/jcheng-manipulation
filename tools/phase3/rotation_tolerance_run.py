#!/usr/bin/env python3
"""Measure the grasp rotation tolerance with a programmable wrist yaw.

The cube stays at the fixture; the relative yaw between the gripper opening
axis and the cube is commanded on joint5 (the wrist roll) instead of rotating
the object by hand. Joint5 leaves the tool point fixed, so the validated deep
grasp family and the constant correction are unchanged; only the opening axis
turns. Every trial is repeated in seeded blocks and the grip is judged with
the same cube-box classifier as the 2026-11-19 envelope session.

The runner also compensates the cube's own placement yaw: the top-face yaw is
measured at the view pose, and the joint5 command is ``cube_yaw - desired`` so
the achieved relative yaw is the scheduled one. Both the commanded and the
measured (bus-servo register) joint5 angle are recorded, so the analysis can
use the achieved yaw without trusting the command.

    python3 tools/phase3/rotation_tolerance_run.py \
        --trials evidence/phase3/onsite_20261120/rotation/trials.jsonl

The arm motion is a normal pick-lift-put-back sequence. Keep the workspace
clear and require ``--i-have-clear-workspace``. The robot has no e-stop.
"""

import argparse
import json
import math
import os
import random
from datetime import datetime

import numpy as np

from _bootstrap import REPO, bootstrap  # noqa: E402
bootstrap()

import grasp_joint_fit  # noqa: E402
import measurement_protocol  # noqa: E402
import paired_trial_session as pts  # noqa: E402
from gripper_envelope_run import (  # noqa: E402
    DEFAULT_CORRECTION_MM,
    DEFAULT_VIEW_POSE,
    MAX_RELATIVE_YAW_DEG,
    now_iso,
    relative_yaw_deg,
)

DEFAULT_YAW_OFFSETS_DEG = [0, 5, -5, 10, -10, 15, -15, 20, -20, 25, -25,
                           30, -30]
# joint5: raw 120..870 pulse maps to -1.5917..+1.5499 rad, zero at 500.
JOINT5_ZERO_PULSE = 500
# Vendor mapping used by the other tools: 240 deg over 1000 ticks.
JOINT5_RAD_PER_PULSE = 240.0 / 360.0 * (2.0 * math.pi) / 1000.0


def build_schedule(yaw_offsets, repeats, seed):
    """Return the seeded [(block, yaw_deg), ...] schedule."""
    schedule = []
    for block in range(repeats):
        entries = list(yaw_offsets)
        random.Random(seed + block).shuffle(entries)
        schedule.extend((block + 1, int(yaw)) for yaw in entries)
    return schedule


def measured_yaw_deg(pulses):
    """Return the measured joint5 angle in degrees from its register."""
    return math.degrees(
        (int(pulses) - JOINT5_ZERO_PULSE) * JOINT5_RAD_PER_PULSE)


def achieved_relative_deg(cube_relative_deg, measured_j5_deg):
    """Return the cube-vs-gripper relative yaw at the grasp pose.

    ``cube_relative_deg`` is the measured top-face yaw against the approach
    azimuth at j5 = 0 (the envelope gate). Turning the wrist by +theta moves
    the opening axis by +theta, so the relative angle is ``phi - theta``.
    """
    return float(cube_relative_deg) - float(measured_j5_deg)


def group_records(rows):
    """Aggregate trial rows into one record per commanded yaw."""
    groups = {}
    for row in rows:
        groups.setdefault(int(row['yaw_deg']), []).append(row)
    records = []
    for yaw in sorted(groups):
        group = groups[yaw]
        records.append({
            'offset_m': [0.0, 0.0, 0.0],
            'yaw_deg': float(yaw),
            'trials': len(group),
            'successes': sum(1 for row in group if row['success']),
        })
    return records


class RotationSession:
    """One rotation-tolerance session."""

    def __init__(self, args):
        self.args = args
        correction = np.asarray(
            [float(value) for value in args.correction.split(',')],
            dtype=np.float64)
        if correction.size != 3:
            raise SystemExit('--correction needs three millimetres values')
        if np.max(np.abs(correction)) > 50.0:
            raise SystemExit('--correction looks too large for millimetres')
        self.correction = correction / 1000.0
        self.view_pose = [float(value) for value in args.view_pose.split(',')]
        self.yaw_offsets = [
            int(value) for value in args.yaw_offsets.split(',')
        ]
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
                done.add((record['block'], int(record['yaw_deg'])))
        return done

    def prepare(self):
        """Install the helpers and put the arm at the view pose."""
        if self.args.dry_run:
            return
        self.robot.install_tools(os.path.join(REPO, 'tools', 'phase2'))
        self.humble.install_hand_eye(os.path.join(
            REPO, self.args.hand_eye_host))
        self.robot.move_to_pose(self.view_pose, tag='rotation_view')
        print('arm at the view pose.')

    def measure(self, retries=3):
        """Measure the cube and enforce the face-toward-arm gate."""
        report = {}
        for attempt in range(retries):
            pulses = self.robot.pulses()
            report = self.humble.segment(
                pulses, 'rot_{}'.format(int(datetime.now().timestamp())))
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
        """Run one yaw trial and append its record."""
        block, yaw_deg = entry
        desired = math.radians(float(yaw_deg))
        print('\n=== block {} / yaw {:+.0f} deg ==='.format(block, yaw_deg))
        if self.args.dry_run:
            return self._dry_trial(entry)
        input('  place the cube flat, one face toward the arm; '
              'press Enter ... ')
        point, report, relative = self.measure()
        if point is None:
            raise RuntimeError('no usable placement for yaw {:+d}'.format(
                yaw_deg))
        # Compensate the cube's own placement yaw: theta = phi - desired.
        command = math.radians(relative) - desired
        if abs(command) > math.radians(80.0):
            raise RuntimeError('joint5 command {:.1f} deg is too large'.format(
                math.degrees(command)))
        target = point + self.correction
        fit = grasp_joint_fit.fit(target, j5=command)
        pts.validate_target(target, fit, self.args.max_xy_error_mm)
        print('  cube ({:+.4f}, {:+.4f}) face {:+.1f} deg'.format(
            point[0], point[1], relative))
        print('  command j5 {:+.1f} deg, joints {}'.format(
            math.degrees(command),
            ','.join('{:.4f}'.format(v) for v in fit['joints'][:4])))
        grasp_report = self.robot.grasp(
            fit['joints'], self.view_pose,
            'rot_b{}_y{:+.0f}'.format(block, yaw_deg), yaw=command)
        grip_pulses = [
            grasp_report['pulses_at_verify'][str(servo)]
            for servo in (6, 5, 4, 3, 2)
        ]
        grip_report = self.humble.segment(
            grip_pulses, 'rot_{}_{:+d}_grip'.format(block, yaw_deg))
        in_box = pts.cube_box_present(grip_report, point)
        j5_pulse = int(grasp_report['pulses_at_grasp']['2'])
        j5_deg = measured_yaw_deg(j5_pulse)
        achieved = achieved_relative_deg(relative, j5_deg)
        notes = {
            'cube_point': [float(value) for value in point],
            'target': [float(value) for value in target],
            'joints': fit['joints'],
            'cube_relative_yaw_deg': float(relative),
            'commanded_j5_deg': math.degrees(command),
            'measured_j5_pulse': j5_pulse,
            'measured_j5_deg': j5_deg,
            'achieved_relative_yaw_deg': achieved,
            'xy_error_mm': fit['xy_error_mm'],
            'grip_cube_box': bool(in_box),
            'grip_clusters': grip_report['segmentation']['clusters'],
        }
        if not in_box:
            release_report = self.robot.release(
                fit['joints'], self.view_pose,
                'rot_b{}_y{:+d}'.format(block, yaw_deg), yaw=command)
            land_pulses = [
                release_report['pulses_at_verify'][str(servo)]
                for servo in (6, 5, 4, 3, 2)
            ]
            land_report = self.humble.segment(
                land_pulses,
                'rot_{}_{:+d}_land'.format(block, yaw_deg))
            landing, valid = pts.cube_point(land_report)
            land_ok, reason = pts.placement_quality(land_report)
            if landing is None or not valid or not land_ok:
                notes['landing_unmeasured'] = (
                    reason if landing is not None else 'no point')
            else:
                notes['landing'] = [float(value) for value in landing]
                notes['landing_error_mm'] = float(
                    np.linalg.norm((landing - point)[:2]) * 1000.0)
        record = {
            'block': block,
            'yaw_deg': int(yaw_deg),
            'success': not in_box,
            'recorded_at': now_iso(),
            'notes': json.dumps(notes, sort_keys=True),
        }
        directory = os.path.dirname(os.path.abspath(self.args.trials))
        os.makedirs(directory, exist_ok=True)
        with open(self.args.trials, 'a', encoding='utf-8') as handle:
            handle.write(json.dumps(record) + '\n')
        self.done.add(entry)
        print('  {}  achieved relative yaw {:+.1f} deg'.format(
            'GRIP' if record['success'] else 'MISS', achieved))
        return record['success']

    def _dry_trial(self, entry):
        """Record a synthetic outcome without touching the robot."""
        block, yaw_deg = entry
        success = abs(yaw_deg) <= 20
        record = {
            'block': block,
            'yaw_deg': int(yaw_deg),
            'success': success,
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
        if not self.args.dry_run:
            if not self.args.i_have_clear_workspace:
                raise SystemExit(
                    'pass --i-have-clear-workspace to move the arm')
            print('SAFETY: the arm will grasp and lift a cube. The robot has '
                  'no e-stop; stay clear and keep the power switch reachable.')
            if not self.args.auto:
                answer = input('type YES to start: ')
                if answer.strip().upper() != 'YES':
                    raise SystemExit('aborted')
        self.prepare()
        schedule = build_schedule(
            self.yaw_offsets, self.args.repeats, self.args.seed)
        pending = [entry for entry in schedule if entry not in self.done]
        print('{} pending of {} trials'.format(len(pending), len(schedule)))
        if self.args.max_trials:
            pending = pending[:self.args.max_trials]
        for entry in pending:
            self.trial(entry)
        self.aggregate()

    def aggregate(self):
        """Write the protocol measurement and its analysis."""
        rows = [json.loads(line) for line in open(self.args.trials)]
        records = group_records(rows)
        measurement = {
            'schema_version': 1,
            'protocol': 'gripper_envelope',
            'generated_at': now_iso(),
            'samples': records,
        }
        os.makedirs(os.path.dirname(os.path.abspath(
            self.args.measurement_out)), exist_ok=True)
        with open(self.args.measurement_out, 'w', encoding='utf-8') as handle:
            json.dump(measurement, handle, indent=2)
        analysis = measurement_protocol.analyze_gripper_envelope(
            records, self.args.success_threshold)
        analysis['commanded_schedule_deg'] = self.yaw_offsets
        analysis['success_threshold'] = self.args.success_threshold
        analysis_path = os.path.splitext(self.args.measurement_out)[0] + \
            '_analysis.json'
        with open(analysis_path, 'w', encoding='utf-8') as handle:
            json.dump(analysis, handle, indent=2)
        print('wrote', self.args.measurement_out)
        print('wrote', analysis_path)
        print('angular envelope: {}'.format(analysis['envelope_deg']))


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--trials',
        default='evidence/phase3/onsite_20261120/rotation/trials.jsonl')
    parser.add_argument(
        '--measurement-out',
        default='evidence/phase3/onsite_20261120/rotation/measurement.json')
    parser.add_argument('--yaw-offsets', default=','.join(
        str(value) for value in DEFAULT_YAW_OFFSETS_DEG),
        help='joint5 relative-yaw offsets in degrees')
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--success-threshold', type=float, default=0.8)
    parser.add_argument(
        '--correction',
        default=','.join(str(value) for value in DEFAULT_CORRECTION_MM))
    parser.add_argument(
        '--view-pose',
        default=','.join(str(value) for value in DEFAULT_VIEW_POSE))
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
    parser.add_argument('--auto', action='store_true',
                        help='skip the per-trial placement prompt')
    parser.add_argument('--i-have-clear-workspace', action='store_true')
    parser.add_argument('--dry-run', action='store_true')
    return parser


def main(argv=None):
    """Run the session."""
    args = build_parser().parse_args(argv)
    session = RotationSession(args)
    try:
        session.run()
    except KeyboardInterrupt:
        print('\ninterrupted; completed trials are recorded')
        return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
