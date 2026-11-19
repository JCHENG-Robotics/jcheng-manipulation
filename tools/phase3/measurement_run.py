#!/usr/bin/env python3
"""Run a Phase-3 measurement protocol on hardware with a fixed AprilTag.

The arm visits one pose many times (``repeatability``) or from two approach
directions (``backlash``); a 60 mm 36h11 tag fixed on the floor is observed at
every visit, and the tag centre in ``base_link`` is reconstructed from the
measured bus-servo joints, the measured hand–eye and the tag PnP. The tag is a
fixed point, so the spread of that reconstruction is the endpoint
repeatability, and the difference between approach directions is the
hysteresis.

Capture runs inside the robot container (it has OpenCV and the vendor action
server); the reconstruction and the analysis run on the laptop. The capture
step is the same tool as the 2026-11-19 low-configuration survey
(``low_config_tag_survey.py``), so the measurement shares that instrument.

    # capture (about 4 min for 30 visits)
    python3 tools/phase3/measurement_run.py capture \
        --protocol repeatability --trials 30 \
        --pose 0,-0.6618,-1.8012,-0.3917,0

    # reconstruct and analyse
    python3 tools/phase3/measurement_run.py analyze \
        --protocol repeatability --dataset /tmp/rep30/samples.json

``--protocol backlash`` runs two captures (``--pre-j4 +0.1`` and ``-0.1``) and
splits the samples into forward/reverse.
"""

import argparse
from datetime import datetime, timezone
import json
import os
import shlex
import subprocess
import sys

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
sys.path.insert(0, os.path.join(REPO, 'tools', 'phase2'))

import low_config_model_fit as model_fit  # noqa: E402

DEFAULT_POSE = [0.0, -0.6618, -1.8012, -0.3917, 0.0]
DEFAULT_HAND_EYE = ('evidence/calibration/20261118/table_down/'
                    'calibration_table_down_20261118.json')
SURVEY_TOOL = 'low_config_tag_survey.py'


def write_poses(path, joints, count, prefix):
    """Write a pose file with one pose repeated *count* times."""
    poses = [
        {'name': '{}_01'.format(prefix) if index == 0
         else '{}_{:02d}'.format(prefix, index + 1),
         'joints': [float(value) for value in joints]}
        for index in range(count)
    ]
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump({'poses': poses}, handle, indent=1)
    return path


def robot_exec(host, container, arguments, timeout):
    """Run one command in the robot container and return stdout."""
    inner = 'source ~/.zshrc >/dev/null 2>&1; exec ' + ' '.join(
        shlex.quote(str(value)) for value in arguments)
    remote = 'sudo docker exec -u ubuntu {} /bin/zsh -c {}'.format(
        container, shlex.quote(inner))
    completed = subprocess.run(
        ['ssh', host, remote], capture_output=True, text=True,
        timeout=timeout)
    if completed.returncode != 0:
        raise RuntimeError('robot command failed (rc {}):\n{}\n{}'.format(
            completed.returncode, completed.stdout[-1500:],
            completed.stderr[-1500:]))
    return completed.stdout


def install_file(host, container, local, remote_name):
    """Copy a host file into the robot container."""
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


def fetch_file(host, container, remote, local):
    """Read a container file to the host (it must be JSON text)."""
    text = robot_exec(host, container, ['cat', remote], timeout=30)
    directory = os.path.dirname(os.path.abspath(local))
    os.makedirs(directory, exist_ok=True)
    with open(local, 'w', encoding='utf-8') as handle:
        handle.write(text)
    return local


def capture(args, tag):
    """Run one survey capture in the robot container."""
    poses_local = '/tmp/poses_{}.json'.format(tag)
    output_dir = '/tmp/{}'.format(tag)
    write_poses(poses_local, args.pose, args.trials, tag)
    install_file(args.robot, args.robot_container,
                 os.path.join(REPO, 'tools', 'phase2', SURVEY_TOOL),
                 SURVEY_TOOL)
    install_file(args.robot, args.robot_container, poses_local,
                 'poses_{}.json'.format(tag))
    arguments = [
        'python3', '/tmp/{}'.format(SURVEY_TOOL),
        '--output-dir', output_dir,
        '--poses-file', '/tmp/poses_{}.json'.format(tag),
        '--marker-id', str(args.marker_id),
        '--marker-size-m', str(args.marker_size_m),
        '--duration', str(args.duration),
        '--settle', str(args.settle),
        '--repeats', str(args.repeats),
        '--servo-reads', str(args.servo_reads),
        '--pre-j1', str(args.pre_j1),
        '--pre-j2', str(args.pre_j2),
        '--pre-j3', str(args.pre_j3),
        '--pre-j4', str(args.pre_j4),
    ]
    print('capture {}: {} visits ...'.format(tag, args.trials))
    robot_exec(args.robot, args.robot_container, arguments,
               timeout=args.trials * 20 + 120)
    local = os.path.join(args.work_dir, '{}.json'.format(tag))
    fetch_file(args.robot, args.robot_container,
               os.path.join(output_dir, 'samples.json'), local)
    print('wrote', local)
    return local


def tag_positions(dataset, hand_eye):
    """Return (name, position_m) for every detected sample."""
    kinematics = model_fit.Kinematics()
    rows = []
    for sample in dataset.get('samples', []):
        tag = sample.get('tag')
        joints = sample.get('measured_joints') or sample.get('joints')
        if tag is None or joints is None:
            continue
        link4 = kinematics.pose(joints, 'link4')
        transform = (link4 @ hand_eye
                     @ model_fit.transform_from_rvec_tvec(
                         tag['rvec'], tag['tvec']))
        rows.append({
            'name': sample.get('name'),
            'position_m': [float(value) for value in transform[:3, 3]],
            'rotation': transform[:3, :3].tolist(),
            'rms_px': float(tag.get('rms_px', 0.0)),
            'joints': [float(value) for value in joints[:5]],
        })
    return rows


def rotation_spread_deg(rows):
    """Return the max geodesic deviation from the mean rotation, in degrees."""
    matrices = np.asarray([row['rotation'] for row in rows])
    mean = matrices.mean(axis=0)
    u, _, vt = np.linalg.svd(mean)
    mean_rotation = u @ vt
    angles = []
    for matrix in matrices:
        cosine = (np.trace(mean_rotation.T @ matrix) - 1.0) / 2.0
        angles.append(np.degrees(np.arccos(np.clip(cosine, -1.0, 1.0))))
    return float(np.max(angles)), float(np.mean(angles))


def build_measurement(protocol, datasets, hand_eye):
    """Build the measurement-protocol input from one or two datasets."""
    groups = []
    for path in datasets:
        with open(path, 'r', encoding='utf-8') as handle:
            dataset = json.load(handle)
        rows = tag_positions(dataset, hand_eye)
        if not rows:
            raise SystemExit('no detected tag samples in {}'.format(path))
        groups.append(rows)
    if protocol == 'repeatability':
        samples = groups[0]
    else:
        if len(groups) != 2:
            raise SystemExit('backlash needs two datasets (forward, reverse)')
        samples = {'forward': groups[0], 'reverse': groups[1]}
    nominal = None
    flat = samples if isinstance(samples, list) else (
        samples['forward'] + samples['reverse'])
    nominal = np.mean(
        [row['position_m'] for row in flat], axis=0).tolist()
    return {
        'schema_version': 1,
        'protocol': protocol,
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'nominal': nominal,
        'samples': samples,
    }


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('capture', 'analyze'))
    parser.add_argument('--protocol',
                        choices=('repeatability', 'backlash'),
                        default='repeatability')
    parser.add_argument('--trials', type=int, default=30)
    parser.add_argument(
        '--pose', default=','.join(str(value) for value in DEFAULT_POSE),
        help='five joints visited by every trial')
    parser.add_argument(
        '--tag', default=None,
        help='capture name (default: the protocol); use one per run')
    parser.add_argument('--marker-id', type=int, default=0)
    parser.add_argument('--marker-size-m', type=float, default=0.060)
    parser.add_argument('--duration', type=float, default=4.0)
    parser.add_argument('--settle', type=float, default=0.5)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--servo-reads', type=int, default=3)
    parser.add_argument('--pre-j1', type=float, default=0.08,
                        help='the survey applies target - offset on joint1')
    parser.add_argument('--pre-j4-delta', type=float, default=0.1)
    parser.add_argument('--pre-j2', type=float, default=0.05)
    parser.add_argument('--pre-j3', type=float, default=0.05)
    parser.add_argument('--pre-j4', type=float, default=0.0)
    parser.add_argument('--robot', default='pi@10.42.0.218')
    parser.add_argument('--robot-container', default='ArmPiUltra')
    parser.add_argument('--work-dir', default='/tmp/phase3_measurement')
    parser.add_argument('--dataset', action='append', default=None,
                        help='dataset path for analyze (repeatable)')
    parser.add_argument('--hand-eye', default=DEFAULT_HAND_EYE)
    parser.add_argument(
        '--measurement-out', default=None,
        help='where to write the reconstructed measurement JSON')
    parser.add_argument('--json-out', default=None,
                        help='where to write the protocol analysis report')
    parser.add_argument('--markdown-out', default=None)
    return parser


def main(argv=None):
    """Run a capture or an analysis."""
    args = build_parser().parse_args(argv)
    args.pose = [float(value) for value in args.pose.split(',')]
    if len(args.pose) != 5:
        raise SystemExit('--pose needs five joints')
    if args.command == 'capture':
        os.makedirs(args.work_dir, exist_ok=True)
        if args.tag or args.protocol == 'repeatability':
            capture(args, args.tag or 'repeatability')
        else:
            forward = dict(args.__dict__)
            forward['pre_j4'] = args.pre_j4 + args.pre_j4_delta
            capture(argparse.Namespace(**forward), 'backlash_forward')
            reverse = dict(args.__dict__)
            reverse['pre_j4'] = args.pre_j4 - args.pre_j4_delta
            capture(argparse.Namespace(**reverse), 'backlash_reverse')
        return 0
    datasets = args.dataset
    if not datasets:
        if args.protocol == 'repeatability':
            datasets = [os.path.join(args.work_dir, 'repeatability.json')]
        else:
            datasets = [
                os.path.join(args.work_dir, 'backlash_forward.json'),
                os.path.join(args.work_dir, 'backlash_reverse.json'),
            ]
    hand_eye = model_fit.load_hand_eye(os.path.join(REPO, args.hand_eye))
    measurement = build_measurement(args.protocol, datasets, hand_eye)
    output = args.measurement_out or os.path.join(
        args.work_dir, 'measurement_{}.json'.format(args.protocol))
    with open(output, 'w', encoding='utf-8') as handle:
        json.dump(measurement, handle, indent=2)
    print('wrote', output)
    command = [
        sys.executable, os.path.join(REPO, 'tools', 'phase2',
                                     'measurement_protocol.py'),
        args.protocol, '--mode', 'analyze', '--input', output,
    ]
    if args.json_out:
        command += ['--json-out', args.json_out]
    if args.markdown_out:
        command += ['--markdown-out', args.markdown_out]
    subprocess.run(command, check=True)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
