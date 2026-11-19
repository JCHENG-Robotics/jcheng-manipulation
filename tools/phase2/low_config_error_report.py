#!/usr/bin/env python3
"""Report the ArmPi model error over the low (grasp) configuration.

Run on the laptop with the Jazzy workspace sourced (PyKDL/urdf_parser_py/xacro
are used by ``ArmKinematics``)::

    source /opt/ros/jazzy/setup.bash
    source ros2_ws/install/setup.bash
    python3 tools/phase2/low_config_error_report.py \
        --samples /tmp/low_tag_survey_20261119/samples.json \
        --calibration evidence/calibration/20261118/table_down/\
calibration_table_down_20261118.json \
        --output evidence/calibration/20261119/low_config_error.json

The tag is fixed in ``base_link``, so its predicted centre should be constant
across poses. The reference poses sit inside the validated table-down volume,
where the model is accurate to 1-3 mm; the offset of every other pose from the
reference mean is the model error at that configuration. The spread of the
error across the low poses says whether one constant local correction is
enough (small spread) or a configuration-dependent model is required (large
spread).
"""

import argparse
from datetime import datetime, timezone
import json
import math

import cv2
import numpy as np

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
ARM_MASS_FREE_LINK = 'link4'


def make_transform(rotation, translation):
    """Build a 4x4 homogeneous transform."""
    out = np.eye(4)
    out[:3, :3] = np.asarray(rotation, dtype=np.float64)
    out[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return out


def transform_from_rvec_tvec(rvec, tvec):
    """Build T_cam_tag from an OpenCV rotation vector and translation."""
    vector = np.asarray(rvec, dtype=np.float64).reshape(3, 1)
    rotation, _ = cv2.Rodrigues(vector)
    translation = np.asarray(tvec, dtype=np.float64).reshape(3)
    return make_transform(rotation, translation)


def link4_transform(joints):
    """Return T_base_link4 for one measured joint vector."""
    from armpi_grasp_planner.reachability import ArmKinematics

    kinematics = ArmKinematics(tip_link=ARM_MASS_FREE_LINK)
    frame = kinematics.forward([float(value) for value in joints[:4]])
    rotation = np.array([
        [frame.M[row, column] for column in range(3)]
        for row in range(3)
    ])
    return make_transform(rotation, [frame.p.x(), frame.p.y(), frame.p.z()])


def load_hand_eye(path):
    """Load T_link4_camera from a calibration report (TSAI preferred)."""
    with open(path, 'r', encoding='utf-8') as handle:
        report = json.load(handle)
    hand_eye = report.get('hand_eye', report)
    methods = hand_eye.get('methods', {})
    if 'TSAI' in methods:
        return np.asarray(methods['TSAI']['T_link4_camera'], dtype=np.float64)
    return np.asarray(hand_eye['T_link4_camera'], dtype=np.float64)


def tag_pose_in_base(sample, t_link4_camera):
    """Return T_base_tag for one sample using its measured joints."""
    measured = sample.get('measured_joints') or sample.get('joints')
    if measured is None:
        raise RuntimeError('sample {} has no joints'.format(sample['index']))
    t_base_link4 = link4_transform(measured)
    tag = sample['tag']
    t_cam_tag = transform_from_rvec_tvec(tag['rvec'], tag['tvec'])
    return t_base_link4 @ t_link4_camera @ t_cam_tag


def rotation_angle_deg(rotation):
    """Return the geodesic angle of a rotation matrix in degrees."""
    cosine = (np.trace(rotation) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


def spread(points):
    """Return (mean, std) of an Nx3 point array."""
    array = np.asarray(points, dtype=np.float64)
    return array.mean(axis=0), array.std(axis=0)


def analyse(samples, t_link4_camera, reference_names):
    """Compute the per-pose tag centre, error field and residuals."""
    entries = []
    for sample in samples:
        if sample.get('tag') is None:
            entries.append({
                'name': sample['name'],
                'index': sample['index'],
                'detected': False,
            })
            continue
        transform = tag_pose_in_base(sample, t_link4_camera)
        entries.append({
            'name': sample['name'],
            'index': sample['index'],
            'detected': True,
            'centre': [float(value) for value in transform[:3, 3]],
            'tag_z_axis': [float(value) for value in transform[:3, 2]],
            'rotation': transform[:3, :3].tolist(),
            'distance_m': float(np.linalg.norm(sample['tag']['tvec'])),
            'rms_px': float(sample['tag']['rms_px']),
            'translation_spread_m': float(
                sample['tag'].get('translation_spread_m', 0.0)),
            'servo_shift_pulses': int(sample.get('servo_shift_pulses', -1)),
        })
    detected = [entry for entry in entries if entry['detected']]
    references = [
        entry for entry in detected if entry['name'] in reference_names
    ]
    if len(references) < 1:
        raise RuntimeError(
            'no reference pose detected; expected one of {}'.format(
                sorted(reference_names)))
    reference_centres = [entry['centre'] for entry in references]
    truth, reference_std = spread(reference_centres)
    reference_pairwise = 0.0
    for i in range(len(reference_centres)):
        for j in range(i + 1, len(reference_centres)):
            reference_pairwise = max(
                reference_pairwise,
                float(np.linalg.norm(
                    np.asarray(reference_centres[i])
                    - np.asarray(reference_centres[j]))))

    for entry in detected:
        error = np.asarray(entry['centre'], dtype=np.float64) - truth
        entry['error_m'] = [float(value) for value in error]
        entry['error_mm'] = [float(1000.0 * value) for value in error]
        entry['error_norm_mm'] = float(1000.0 * np.linalg.norm(error))
        entry['is_reference'] = entry['name'] in reference_names

    grasp = [entry for entry in detected if not entry['is_reference']]
    summary = {}
    if grasp:
        errors = np.asarray([entry['error_mm'] for entry in grasp],
                            dtype=np.float64)
        mean_error = errors.mean(axis=0)
        residual = errors - mean_error
        summary = {
            'grasp_poses': len(grasp),
            'mean_error_mm': [float(value) for value in mean_error],
            'error_std_mm': [float(value) for value in errors.std(axis=0)],
            'error_norm_mean_mm': float(np.linalg.norm(mean_error)),
            'error_norm_max_mm': float(
                np.linalg.norm(errors, axis=1).max()),
            'residual_after_constant_mm': {
                'std': [float(value) for value in residual.std(axis=0)],
                'max_norm': float(
                    np.linalg.norm(residual, axis=1).max()),
            },
        }
    reference_rotations = [
        np.asarray(entry['rotation']) for entry in references
    ]
    rotation_spread = 0.0
    for rotation in reference_rotations[1:]:
        rotation_spread = max(
            rotation_spread,
            rotation_angle_deg(
                rotation @ reference_rotations[0].T))
    return {
        'reference_names': sorted(reference_names),
        'reference_count': len(references),
        'truth_centre': [float(value) for value in truth],
        'reference_std_mm': [float(1000.0 * value) for value in reference_std],
        'reference_max_pairwise_mm': float(1000.0 * reference_pairwise),
        'reference_rotation_spread_deg': float(rotation_spread),
        'samples': entries,
        'summary': summary,
    }


def print_report(report, source):
    """Print a compact human-readable report."""
    print('samples:', source)
    print('reference poses:', ', '.join(report['reference_names']))
    print('truth tag centre (base_link):', [round(v, 5)
                                            for v in report['truth_centre']])
    print('reference std (mm):', [round(v, 2)
                                  for v in report['reference_std_mm']],
          ' max pairwise (mm):',
          round(report['reference_max_pairwise_mm'], 2),
          ' rotation spread (deg):',
          round(report['reference_rotation_spread_deg'], 2))
    print('')
    print('{:<10} {:>4} {:>7} {:>8} {:>8} {:>8} {:>8} {:>9}'.format(
        'pose', 'rms', 'dist', 'ex_mm', 'ey_mm', 'ez_mm', '|e|_mm',
        'servo_p'))
    for entry in report['samples']:
        if not entry['detected']:
            print('{:<10} not detected'.format(entry['name']))
            continue
        error = entry['error_mm']
        print(('{:<10} {:5.2f} {:7.3f} {:8.2f} {:8.2f} {:8.2f} {:8.2f} '
               '{:9d}').format(
                   entry['name'], entry['rms_px'], entry['distance_m'],
                   error[0], error[1], error[2], entry['error_norm_mm'],
                   entry['servo_shift_pulses']))
    summary = report['summary']
    if summary:
        print('')
        print('grasp-family poses:', summary['grasp_poses'])
        print('mean model error (mm):', [round(v, 2)
                                         for v in summary['mean_error_mm']],
              ' |e| =', round(summary['error_norm_mean_mm'], 2))
        print('error std across poses (mm):',
              [round(v, 2) for v in summary['error_std_mm']])
        residual = summary['residual_after_constant_mm']
        print('residual after one constant correction: std (mm)',
              [round(v, 2) for v in residual['std']],
              ' max |e| =', round(residual['max_norm'], 2))


def main():
    """Load the survey dataset, compute the error field and write a report."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--samples', required=True)
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument(
        '--reference', action='append', default=None,
        help='sample name used as the table-down reference (repeatable); '
             'defaults to any name starting with "R_"')
    args = parser.parse_args()

    with open(args.samples, 'r', encoding='utf-8') as handle:
        dataset = json.load(handle)
    samples = dataset['samples']
    t_link4_camera = load_hand_eye(args.calibration)
    if args.reference:
        reference_names = set(args.reference)
    else:
        reference_names = {
            sample['name'] for sample in samples
            if sample['name'].startswith('R_')
        }
    if not reference_names:
        raise RuntimeError(
            'no reference poses found; pass --reference NAME')

    report = analyse(samples, t_link4_camera, reference_names)
    report.update({
        'generated_utc': datetime.now(timezone.utc).isoformat(),
        'samples_file': args.samples,
        'calibration_file': args.calibration,
        'marker_id': dataset.get('marker_id'),
        'marker_size_m': dataset.get('marker_size_m'),
        'pre_pose_offsets_rad': dataset.get('pre_pose_offsets_rad'),
        'T_link4_camera': t_link4_camera.tolist(),
    })
    print_report(report, args.samples)
    import os
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, 'w', encoding='utf-8') as handle:
        json.dump(report, handle, indent=2)
    print('')
    print('WROTE', args.output)


if __name__ == '__main__':
    main()
