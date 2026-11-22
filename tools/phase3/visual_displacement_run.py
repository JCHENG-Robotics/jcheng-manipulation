#!/usr/bin/env python3
"""Measure visual displacement against a ruler fixture on the real robot.

The robot stays at the fixed view pose; a 60 mm 36h11 tag is slid on the floor
along two perpendicular steel-ruler arms through a grid of known offsets. For
every position the robot container captures RGB frames without moving the arm;
the laptop detects the AprilTag and reconstructs its centre in ``base_link``
from the measured bus-servo joints, the measured hand--eye and the PnP. The
displacement between each position and the origin is compared with the ruler
displacement.

This is the hardware form of the ``visual_displacement`` protocol. It is the
same reconstruction as the 2026-11-18 three-position tag check
(``tag_displacement_validation``), extended to a repeated grid and analysed by
``measurement_protocol.py``.

The ruler frame's yaw is unknown (the L-square is placed by hand). The runner
fits one 2-D rotation between the nominal grid and the measured displacements
and reports it as a nuisance parameter; the residual after that rotation is the
visual displacement error. The raw residual (no rotation) is also reported.

    # laptop, Jazzy workspace sourced for the detector and KDL model
    source /opt/ros/jazzy/setup.bash
    source ros2_ws/install/setup.bash
    python3 tools/phase3/visual_displacement_run.py \
        --positions-out evidence/phase3/onsite_20261120/visual_displacement

The capture uses ``chessboard_capture.py --check-only``: the arm does not move.
The operator moves only the tag and must step out of the camera view.
"""

import argparse
import json
import math
import os
import random
import shlex
import shutil
import subprocess
import tarfile
from datetime import datetime, timezone

import numpy as np

from _bootstrap import REPO, bootstrap  # noqa: E402
bootstrap()

import measurement_protocol  # noqa: E402

DEFAULT_VIEW_POSE = [0.0, 0.2574, -1.18773, -1.70, -0.01676]
DEFAULT_HAND_EYE = ('evidence/calibration/20261118/table_down/'
                    'calibration_table_down_20261118.json')
CAPTURE_TOOL = 'chessboard_capture.py'


def now_iso():
    """Return the current UTC time as an ISO-8601 string."""
    return datetime.now(timezone.utc).isoformat()


def build_schedule(step_mm, max_mm, repeats, seed):
    """Return the seeded [(block, u_mm, v_mm), ...] grid.

    Each block starts and ends with the origin so the block's own origin mean
    absorbs any slow drift of the tag or the arm.
    """
    steps = list(range(-int(max_mm // step_mm), int(max_mm // step_mm) + 1))
    offsets = []
    for step in steps:
        if step == 0:
            continue
        offsets.append((step * int(step_mm), 0))
        offsets.append((0, step * int(step_mm)))
    schedule = []
    for block in range(repeats):
        entries = list(offsets)
        random.Random(seed + block).shuffle(entries)
        schedule.append((block + 1, 0, 0))
        schedule.extend((block + 1, u, v) for u, v in entries)
        schedule.append((block + 1, 0, 0))
    return schedule


def fit_ruler_rotation(nominal_mm, measured_mm):
    """Return the 2-D rotation (radians) from nominal to measured vectors."""
    nominal = np.asarray(nominal_mm, dtype=np.float64)
    measured = np.asarray(measured_mm, dtype=np.float64)
    cross = float(np.sum(
        nominal[:, 0] * measured[:, 1] - nominal[:, 1] * measured[:, 0]))
    dot = float(np.sum(
        nominal[:, 0] * measured[:, 0] + nominal[:, 1] * measured[:, 1]))
    return math.atan2(cross, dot)


def rotate_2d(vectors_mm, angle):
    """Rotate Nx2 millimetre vectors by *angle* radians."""
    cosine = math.cos(angle)
    sine = math.sin(angle)
    vectors = np.asarray(vectors_mm, dtype=np.float64)
    return np.column_stack((
        cosine * vectors[:, 0] - sine * vectors[:, 1],
        sine * vectors[:, 0] + cosine * vectors[:, 1],
    ))


def rotation_spread_deg(matrices):
    """Return the max/mean geodesic deviation from the mean rotation."""
    matrices = np.asarray(matrices, dtype=np.float64)
    mean = matrices.mean(axis=0)
    u, _, vt = np.linalg.svd(mean)
    mean_rotation = u @ vt
    angles = []
    for matrix in matrices:
        cosine = (np.trace(mean_rotation.T @ matrix) - 1.0) / 2.0
        angles.append(math.degrees(math.acos(np.clip(cosine, -1.0, 1.0))))
    return float(np.max(angles)), float(np.mean(angles))


def install_file(host, container, local, remote_name):
    """Copy one laptop file into the robot container."""
    inner = ('rm -f /tmp/{0}; cat > /tmp/{0}; chmod 644 /tmp/{0}'
             .format(remote_name))
    remote = 'sudo docker exec -i -u root {} /bin/zsh -c {}'.format(
        container, shlex.quote(inner))
    with open(local, 'rb') as handle:
        completed = subprocess.run(
            ['ssh', host, remote], stdin=handle, capture_output=True,
            timeout=30)
    if completed.returncode != 0:
        raise RuntimeError('copy {} failed: {}'.format(
            remote_name, completed.stderr[-500:]))


def run_in_container(host, container, arguments, timeout):
    """Run one command in the robot container and return stdout."""
    inner = 'source ~/.zshrc >/dev/null 2>&1; exec ' + ' '.join(
        shlex.quote(str(value)) for value in arguments)
    remote = 'sudo docker exec -u ubuntu {} /bin/zsh -c {}'.format(
        container, shlex.quote(inner))
    completed = subprocess.run(
        ['ssh', host, remote], capture_output=True, text=True,
        timeout=timeout)
    if completed.returncode != 0:
        raise RuntimeError('container command failed (rc {}):\n{}\n{}'.format(
            completed.returncode, completed.stdout[-1200:],
            completed.stderr[-1200:]))
    return completed.stdout


def fetch_directory(host, container, remote_dir, local_dir):
    """Copy a container output directory to the laptop."""
    parent = os.path.dirname(remote_dir)
    name = os.path.basename(remote_dir)
    archive = local_dir.rstrip('/') + '.tgz'
    local_parent = os.path.dirname(os.path.abspath(local_dir))
    os.makedirs(local_parent, exist_ok=True)
    remote = 'sudo docker exec {} tar czf - -C {} {}'.format(
        container, parent, name)
    with open(archive, 'wb') as handle:
        completed = subprocess.run(
            ['ssh', host, remote], stdout=handle, stderr=subprocess.PIPE,
            timeout=120)
    if completed.returncode != 0:
        raise RuntimeError('fetch {} failed: {}'.format(
            remote_dir, completed.stderr[-500:].decode('utf-8', 'replace')))
    with tarfile.open(archive, 'r:gz') as tar:
        tar.extractall(local_parent, filter='data')
    extracted = os.path.join(local_parent, name)
    if os.path.abspath(extracted) != os.path.abspath(local_dir):
        if os.path.exists(local_dir):
            shutil.rmtree(local_dir)
        shutil.move(extracted, local_dir)
    return local_dir


class VisualDisplacementSession:
    """One visual-displacement session."""

    def __init__(self, args):
        self.args = args
        self.view_pose = [float(value) for value in args.view_pose.split(',')]
        self.rows = []

    def capture(self, block, u_mm, v_mm):
        """Capture one tag position without moving the arm."""
        tag = 'vd_b{}_{:+04d}_{:+04d}'.format(block, u_mm, v_mm)
        remote_dir = '/tmp/{}'.format(tag)
        run_in_container(
            self.args.robot, self.args.robot_container,
            ['python3', '/tmp/{}'.format(CAPTURE_TOOL),
             '--output-dir', remote_dir,
             '--check-only', '--count', str(self.args.frames),
             '--repeats', '1', '--warmup', '2', '--settle', '0.3',
             '--servo-reads', '3'],
            timeout=90)
        local_dir = os.path.join(self.args.work_dir, tag)
        fetch_directory(self.args.robot, self.args.robot_container,
                        remote_dir, local_dir)
        return local_dir

    def measure(self, block, u_mm, v_mm):
        """Capture and reconstruct one position; retry when no tag is found."""
        import validate_hand_eye_tag
        for attempt in range(self.args.retries):
            local_dir = self.capture(block, u_mm, v_mm)
            report, _ = validate_hand_eye_tag.analyze(
                os.path.join(local_dir, 'samples.json'),
                self.args.hand_eye,
                'DICT_APRILTAG_36h11',
                self.args.marker_size_m,
                self.args.marker_id,
                None)
            if report is not None and report.get('frames'):
                return report
            print('  no tag detected at ({:+d}, {:+d}) mm'.format(u_mm, v_mm))
            if attempt + 1 < self.args.retries:
                input('  move the tag so the whole square is visible, then '
                      'press Enter to retry ... ')
        raise RuntimeError('no tag at ({:+d}, {:+d}) mm after {} tries'.format(
            u_mm, v_mm, self.args.retries))

    def position(self, block, u_mm, v_mm):
        """Ask the operator, capture, and append the position record."""
        print('\n=== block {} / u {:+.0f} mm, v {:+.0f} mm ==='.format(
            block, u_mm, v_mm))
        if self.args.dry_run:
            return self._dry_position(block, u_mm, v_mm)
        input('  place the tag with its corner at u={:+.0f} mm, v={:+.0f} mm '
              'on the L-square; step out of the view; press Enter ... '.format(
                  u_mm, v_mm))
        report = self.measure(block, u_mm, v_mm)
        frames = report['frames']
        positions = np.asarray([frame['base_translation_m']
                                for frame in frames])
        rotations = [np.asarray(frame['base_rotation_matrix'])
                     for frame in frames]
        mean = positions.mean(axis=0)
        std = positions.std(axis=0)
        spread, mean_spread = rotation_spread_deg(rotations)
        record = {
            'block': block,
            'u_mm': int(u_mm),
            'v_mm': int(v_mm),
            'frames': len(frames),
            'position_m': mean.tolist(),
            'within_std_mm': (std * 1000.0).tolist(),
            'within_range_mm': (
                (positions.max(axis=0) - positions.min(axis=0)) * 1000.0
            ).tolist(),
            'rotation_spread_deg': spread,
            'rotation_mean_spread_deg': mean_spread,
            'rms_px_max': max(frame['reprojection_error_px']
                              for frame in frames),
            'summary': report['summary'],
            'frame_details': frames,
            'recorded_at': now_iso(),
        }
        self.rows.append(record)
        directory = os.path.dirname(os.path.abspath(self.args.positions_out))
        os.makedirs(directory, exist_ok=True)
        with open(self.args.positions_out, 'w', encoding='utf-8') as handle:
            json.dump(self.rows, handle, indent=2)
        print('  {} frames, ({:+.4f}, {:+.4f}, {:+.4f}) m, std {} mm'.format(
            len(frames), mean[0], mean[1], mean[2],
            ','.join('{:.2f}'.format(value * 1000.0) for value in std)))
        return record

    def _dry_position(self, block, u_mm, v_mm):
        """Record a synthetic position without touching the robot."""
        base = np.array([0.25, -0.01, -0.08])
        angle = math.radians(12.0)  # synthetic ruler mounting yaw
        nominal = rotate_2d([[u_mm, v_mm]], -angle)[0] / 1000.0
        position = base + np.array([nominal[0], nominal[1], 0.0])
        record = {
            'block': block,
            'u_mm': int(u_mm),
            'v_mm': int(v_mm),
            'frames': self.args.frames,
            'position_m': position.tolist(),
            'within_std_mm': [0.2, 0.2, 0.2],
            'within_range_mm': [0.5, 0.5, 0.5],
            'rotation_spread_deg': 0.2,
            'rotation_mean_spread_deg': 0.1,
            'rms_px_max': 0.4,
            'summary': {},
            'frame_details': [],
            'recorded_at': now_iso(),
        }
        self.rows.append(record)
        directory = os.path.dirname(os.path.abspath(self.args.positions_out))
        os.makedirs(directory, exist_ok=True)
        with open(self.args.positions_out, 'w', encoding='utf-8') as handle:
            json.dump(self.rows, handle, indent=2)
        print('  dry position ({:+d}, {:+d}) mm'.format(u_mm, v_mm))
        return record

    def aggregate(self):
        """Fit the ruler yaw and write the protocol measurement + analysis."""
        if not self.rows:
            raise SystemExit('no positions captured')
        blocks = sorted({row['block'] for row in self.rows})
        records = []
        origin_spread = {}
        for block in blocks:
            origin_rows = [row for row in self.rows
                           if row['block'] == block and row['u_mm'] == 0
                           and row['v_mm'] == 0]
            if not origin_rows:
                raise SystemExit('block {} has no origin'.format(block))
            origin = np.mean(
                [row['position_m'] for row in origin_rows], axis=0)
            origin_spread[block] = float(np.linalg.norm(
                np.std([row['position_m'] for row in origin_rows],
                       axis=0)) * 1000.0)
            for row in self.rows:
                if row['block'] != block:
                    continue
                if row['u_mm'] == 0 and row['v_mm'] == 0:
                    continue
                measured = np.asarray(row['position_m']) - origin
                nominal = np.array([row['u_mm'], row['v_mm']]) / 1000.0
                records.append({
                    'block': block,
                    'u_mm': row['u_mm'],
                    'v_mm': row['v_mm'],
                    'nominal_m': [float(nominal[0]), float(nominal[1]), 0.0],
                    'measured_m': measured.tolist(),
                })
        nominal_2d = np.asarray(
            [[record['nominal_m'][0], record['nominal_m'][1]]
             for record in records])
        measured_2d = np.asarray(
            [[record['measured_m'][0], record['measured_m'][1]]
             for record in records])
        # The origin is zero; fit only the ruler-frame rotation.
        angle = fit_ruler_rotation(nominal_2d, measured_2d)
        rotated = rotate_2d(nominal_2d, angle)
        residuals = measured_2d - rotated
        norms = np.linalg.norm(residuals, axis=1)
        raw_residuals = measured_2d - nominal_2d
        raw_norms = np.linalg.norm(raw_residuals, axis=1)
        nominal_radius = np.linalg.norm(rotated, axis=1)
        measured_radius = np.linalg.norm(measured_2d, axis=1)
        radial_error = measured_radius - nominal_radius
        nominal_angle = np.arctan2(rotated[:, 1], rotated[:, 0])
        measured_angle = np.arctan2(measured_2d[:, 1], measured_2d[:, 0])
        angular_error = np.degrees(
            np.arctan2(np.sin(measured_angle - nominal_angle),
                       np.cos(measured_angle - nominal_angle)))
        analysis_records = []
        for record, rotated_nominal, measured in zip(
                records, rotated, measured_2d):
            analysis_records.append({
                'id': 'b{}_{:+d}_{:+d}'.format(
                    record['block'], record['u_mm'], record['v_mm']),
                'nominal_displacement_m': [
                    float(rotated_nominal[0]), float(rotated_nominal[1]), 0.0],
                'measured_displacement_m': [
                    float(measured[0]), float(measured[1]), 0.0],
            })
        analysis = measurement_protocol.analyze_visual_displacement(
            analysis_records)
        analysis['ruler_yaw_deg'] = math.degrees(angle)
        analysis['rotation_fit'] = {
            'angle_deg': math.degrees(angle),
            'raw_rmse_mm': float(np.sqrt(np.mean(raw_norms ** 2))),
            'raw_max_mm': float(raw_norms.max()),
            'residual_rmse_mm': float(np.sqrt(np.mean(norms ** 2))),
            'residual_max_mm': float(norms.max()),
            'radial_bias_mm': float(np.mean(radial_error) * 1000.0),
            'radial_rms_mm': float(np.sqrt(np.mean(radial_error ** 2))
                                   * 1000.0),
            'angular_bias_deg': float(np.mean(angular_error)),
            'angular_rms_deg': float(np.sqrt(np.mean(angular_error ** 2))),
            'angular_max_deg': float(np.max(np.abs(angular_error))),
        }
        analysis['origin'] = {
            'blocks': blocks,
            'origin_spread_mm': [origin_spread[block] for block in blocks],
        }
        analysis['positions'] = records
        analysis['per_position'] = [
            {
                'block': row['block'],
                'u_mm': row['u_mm'],
                'v_mm': row['v_mm'],
                'frames': row['frames'],
                'within_std_mm': row['within_std_mm'],
                'rotation_spread_deg': row['rotation_spread_deg'],
                'rms_px_max': row['rms_px_max'],
            }
            for row in self.rows
        ]
        directory = os.path.dirname(os.path.abspath(self.args.analysis_out))
        os.makedirs(directory, exist_ok=True)
        with open(self.args.analysis_out, 'w', encoding='utf-8') as handle:
            json.dump(analysis, handle, indent=2)
        print('wrote', self.args.analysis_out)
        print('ruler yaw {:.2f} deg; residual RMSE {:.2f} mm, max {:.2f} mm'
              .format(math.degrees(angle),
                      analysis['rotation_fit']['residual_rmse_mm'],
                      analysis['rotation_fit']['residual_max_mm']))

    def run(self):
        """Run the grid in schedule order."""
        if not self.args.dry_run:
            install_file(self.args.robot, self.args.robot_container,
                         os.path.join(REPO, 'tools', 'phase2', CAPTURE_TOOL),
                         CAPTURE_TOOL)
        schedule = build_schedule(
            self.args.step_mm, self.args.max_mm, self.args.repeats,
            self.args.seed)
        if self.args.max_positions:
            schedule = schedule[:self.args.max_positions]
        print('{} positions'.format(len(schedule)))
        for block, u_mm, v_mm in schedule:
            self.position(block, u_mm, v_mm)
        self.aggregate()


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--positions-out',
        default='evidence/phase3/onsite_20261120/visual_displacement/'
                'positions.json')
    parser.add_argument(
        '--analysis-out',
        default='evidence/phase3/onsite_20261120/visual_displacement/'
                'analysis.json')
    parser.add_argument('--step-mm', type=int, default=10)
    parser.add_argument('--max-mm', type=int, default=30)
    parser.add_argument('--repeats', type=int, default=2)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--frames', type=int, default=5)
    parser.add_argument('--retries', type=int, default=3)
    parser.add_argument('--marker-id', type=int, default=0)
    parser.add_argument('--marker-size-m', type=float, default=0.060)
    parser.add_argument('--view-pose', default=','.join(
        str(value) for value in DEFAULT_VIEW_POSE))
    parser.add_argument('--hand-eye', default=DEFAULT_HAND_EYE)
    parser.add_argument('--work-dir', default='/tmp/visual_displacement')
    parser.add_argument('--robot', default='pi@10.42.0.218')
    parser.add_argument('--robot-container', default='ArmPiUltra')
    parser.add_argument('--max-positions', type=int, default=0)
    parser.add_argument('--dry-run', action='store_true')
    return parser


def main(argv=None):
    """Run the session."""
    args = build_parser().parse_args(argv)
    session = VisualDisplacementSession(args)
    try:
        session.run()
    except KeyboardInterrupt:
        print('\ninterrupted; completed positions are recorded')
        return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
