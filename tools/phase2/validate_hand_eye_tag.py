#!/usr/bin/env python3
"""Validate the calibrated hand-eye against a surveyed AprilTag.

Runs on the laptop with the Jazzy workspace sourced. Reads the
``samples.json`` written by ``chessboard_capture.py --check-only`` (RGB image
names, camera intrinsics, cached and measured bus-servo joints), detects an
AprilTag with ``FiducialDetector`` and composes

    T_base_marker = T_base_link4(q_measured) @ T_link4_camera @ T_cam_marker

The optional ``--ground-truth`` is a surveyed marker centre in base_link in
metres. The script reports per-frame poses, the frame-to-frame spread and the
translation error against the survey. It does not move the robot.

Example:
    python3 tools/phase2/validate_hand_eye_tag.py \
        --samples /tmp/tag_sticker_A/samples.json \
        --calibration evidence/calibration/20261117/calibration_servo_20261117.json \
        --marker-size 0.0249 --marker-id 3 \
        --output evidence/phase2/onsite_20261118/tag_displacement_validation/validate_A.json
"""

import argparse
import json
import math
import os

import cv2
import numpy as np

from armpi_fiducial_localizer.detector import (
    FiducialDetector,
    rotation_to_quaternion,
)
from armpi_grasp_planner.reachability import ArmKinematics

DEFAULT_DICTIONARY = 'DICT_APRILTAG_36h11'
DEFAULT_MARKER_SIZE_M = 0.06
DEFAULT_MARKER_ID = 0


def load_calibration(path):
    """Load T_link4_camera from a chessboard solver report."""
    with open(path, encoding='utf-8') as handle:
        report = json.load(handle)
    transform = np.asarray(
        report['hand_eye']['T_link4_camera'], dtype=np.float64)
    if transform.shape != (4, 4):
        raise RuntimeError('calibration T_link4_camera is not 4x4')
    return transform, report


def sample_joints(sample):
    """Return measured joints when present, otherwise the cached commands."""
    measured = sample.get('measured_joints')
    if measured and len(measured) >= 4:
        return measured[:4]
    return sample['joints'][:4]


def base_link4(kinematics, joints):
    """Return T_base_link4 for a joint vector."""
    frame = kinematics.forward(joints)
    rotation = np.array([
        [frame.M[i, j] for j in range(3)] for i in range(3)
    ])
    translation = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = translation
    return matrix


def rotation_angle_deg(rotation_a, rotation_b):
    """Return the geodesic angle between two rotation matrices."""
    relative = rotation_a.T @ rotation_b
    cosine = max(-1.0, min(1.0, 0.5 * (np.trace(relative) - 1.0)))
    return math.degrees(math.acos(cosine))


def analyze(samples_path, calibration_path, marker_dictionary,
            marker_size_m, marker_id, ground_truth):
    """Detect the marker in each frame and compose its base_link pose."""
    with open(samples_path, encoding='utf-8') as handle:
        dataset = json.load(handle)
    t_link4_camera, calibration = load_calibration(calibration_path)
    kinematics = ArmKinematics(tip_link='link4')
    detector = FiducialDetector(
        dictionary=marker_dictionary,
        marker_size_m=marker_size_m,
        max_reprojection_error_px=3.0)
    frames_dir = os.path.dirname(os.path.abspath(samples_path))
    frames = []
    for sample in dataset['samples']:
        image_name = sample.get('image')
        if not image_name:
            continue
        image_path = os.path.join(frames_dir, image_name)
        image = cv2.imread(image_path)
        if image is None:
            print(f'frame {sample["index"]}: cannot read {image_path}')
            continue
        camera_matrix = np.asarray(
            sample['camera_matrix'], dtype=np.float64).reshape(3, 3)
        dist_coeffs = np.asarray(
            sample['dist_coeffs'], dtype=np.float64).reshape(-1)
        detections = [
            detection for detection in detector.detect(
                image, camera_matrix, dist_coeffs)
            if detection.marker_id == marker_id
        ]
        if not detections:
            print(f'frame {sample["index"]}: marker {marker_id} not detected')
            continue
        detection = detections[0]
        t_cam_marker = np.eye(4)
        t_cam_marker[:3, :3] = np.asarray(
            detection.rotation_matrix, dtype=np.float64)
        t_cam_marker[:3, 3] = np.asarray(
            detection.tvec, dtype=np.float64)
        joints = sample_joints(sample)
        t_base_marker = (
            base_link4(kinematics, joints) @ t_link4_camera @ t_cam_marker)
        frames.append({
            'index': sample['index'],
            'image': image_name,
            'joints_used': [float(value) for value in joints],
            'marker_id': detection.marker_id,
            'reprojection_error_px': float(
                detection.reprojection_error_px),
            'marker_side_px': float(detection.marker_side_px),
            'camera_translation_m': [
                float(value) for value in detection.tvec
            ],
            'camera_distance_m': float(np.linalg.norm(detection.tvec)),
            'base_translation_m': [
                float(value) for value in t_base_marker[:3, 3]
            ],
            'base_quaternion_xyzw': [
                float(value) for value in rotation_to_quaternion(
                    t_base_marker[:3, :3])
            ],
            'base_rotation_matrix': t_base_marker[:3, :3].tolist(),
        })
        print(
            'frame {index}: id={id} dist={dist:.3f}m '
            'reproj={reproj:.2f}px base=({x:.4f}, {y:.4f}, {z:.4f})'
            .format(
                index=sample['index'], id=detection.marker_id,
                dist=frames[-1]['camera_distance_m'],
                reproj=frames[-1]['reprojection_error_px'],
                x=frames[-1]['base_translation_m'][0],
                y=frames[-1]['base_translation_m'][1],
                z=frames[-1]['base_translation_m'][2]))
    if not frames:
        return None, calibration

    translations = np.asarray(
        [frame['base_translation_m'] for frame in frames])
    rotations = [
        np.asarray(frame['base_rotation_matrix']) for frame in frames]
    reference = rotations[0]
    rotation_errors = [
        rotation_angle_deg(reference, rotation) for rotation in rotations]
    mean = translations.mean(axis=0)
    summary = {
        'frames': len(frames),
        'base_translation_mean_m': mean.tolist(),
        'base_translation_std_mm': (
            translations.std(axis=0) * 1000.0).tolist(),
        'base_translation_range_mm': (
            (translations.max(axis=0) - translations.min(axis=0))
            * 1000.0).tolist(),
        'base_rotation_spread_deg': float(max(rotation_errors)),
        'camera_distance_mean_m': float(np.mean(
            [frame['camera_distance_m'] for frame in frames])),
        'max_reprojection_error_px': float(max(
            frame['reprojection_error_px'] for frame in frames)),
    }
    residual = None
    if ground_truth is not None:
        truth = np.asarray(ground_truth, dtype=np.float64)
        errors = translations - truth
        norms = np.linalg.norm(errors, axis=1)
        residual = {
            'ground_truth_m': truth.tolist(),
            'mean_error_m': (mean - truth).tolist(),
            'mean_error_norm_m': float(np.linalg.norm(mean - truth)),
            'per_frame_error_norm_m': [float(value) for value in norms],
            'max_error_norm_m': float(norms.max()),
        }
    return {
        'samples': os.path.basename(samples_path),
        'calibration': os.path.basename(calibration_path),
        'calibration_method': calibration['hand_eye']['selected'],
        'marker': {
            'dictionary': marker_dictionary,
            'size_m': marker_size_m,
            'id': marker_id,
        },
        'frames': frames,
        'summary': summary,
        'residual': residual,
    }, calibration


def main():
    """Run the tag validation and write a JSON report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--samples', required=True)
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--marker-dictionary', default=DEFAULT_DICTIONARY)
    parser.add_argument('--marker-size', type=float,
                        default=DEFAULT_MARKER_SIZE_M)
    parser.add_argument('--marker-id', type=int, default=DEFAULT_MARKER_ID)
    parser.add_argument(
        '--ground-truth', type=float, nargs=3, default=None,
        metavar=('X', 'Y', 'Z'),
        help='surveyed marker centre in base_link [m]')
    parser.add_argument('--output', required=True)
    args = parser.parse_args()

    report, _ = analyze(
        args.samples, args.calibration, args.marker_dictionary,
        args.marker_size, args.marker_id, args.ground_truth)
    if report is None:
        print('no frames detected')
        return 1
    os.makedirs(
        os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, 'w', encoding='utf-8') as handle:
        json.dump(report, handle, indent=2)
    summary = report['summary']
    print(
        'base mean=({:.4f}, {:.4f}, {:.4f}) m'.format(
            *summary['base_translation_mean_m']))
    print(
        'spread std={} mm range={} mm rotation={:.3f} deg'.format(
            [round(value, 3) for value in
             summary['base_translation_std_mm']],
            [round(value, 3) for value in
             summary['base_translation_range_mm']],
            summary['base_rotation_spread_deg']))
    if report['residual'] is not None:
        residual = report['residual']
        print(
            'ground truth error=({:.4f}, {:.4f}, {:.4f}) m norm={:.4f} m'
            .format(
                *residual['mean_error_m'],
                residual['mean_error_norm_m']))
    print('WROTE', args.output)
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
