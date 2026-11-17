#!/usr/bin/env python3
"""Solve camera intrinsics and link4 -> camera hand-eye from chessboard samples.

Run on the laptop with the Jazzy workspace sourced (PyKDL/urdf_parser_py/xacro
are used by ArmKinematics):

    source /opt/ros/jazzy/setup.bash
    source ros2_ws/install/setup.bash
    python3 tools/phase2/chessboard_hand_eye.py \
        --samples ~/jcheng/calibration/20261117/chessboard_calib_servo_20261117/samples.json \
        --output evidence/calibration/20261117/calibration_servo_20261117.json

The camera is rigidly mounted on link4; the hand-eye result is
``T_link4_camera`` (camera frame in link4). The board is fixed in the world, so
the per-sample board pose in base_link should be constant; its spread is the
main validation metric. When the dataset has ``measured_joints`` (bus-servo
readback), FK uses those instead of the cached joint_states commands.

The vendor static TF ``link4 -> rgb_camera_link`` is a **body** frame while PnP
returns the camera **optical** frame, so the static TF comparison is
informational (a ~90 deg roll difference is expected) and is not a pass gate.
A ``final_base`` sample, when present, is held out of the fit and checked
separately. This is a calibration result, not an absolute metrology claim.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os

import cv2
import numpy as np

from armpi_grasp_planner.reachability import ArmKinematics

BOARD_COLS = 11
BOARD_ROWS = 8
SQUARE_M = 0.020
IMAGE_SIZE = (640, 400)
MAX_BOARD_RMS_PX = 1.0

# Live tf2_echo link4 -> rgb_camera_link on 2026-11-17 (3-decimal printout).
STATIC_TF_LINK4_RGB = np.array([
    [1.000, 0.008, 0.010, -0.050],
    [-0.008, 1.000, -0.003, -0.019],
    [-0.010, 0.003, 1.000, 0.087],
    [0.000, 0.000, 0.000, 1.000],
])

HAND_EYE_METHODS = {
    'TSAI': cv2.CALIB_HAND_EYE_TSAI,
    'PARK': cv2.CALIB_HAND_EYE_PARK,
    'HORAUD': cv2.CALIB_HAND_EYE_HORAUD,
    'ANDREFF': cv2.CALIB_HAND_EYE_ANDREFF,
    'DANIILIDIS': cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def make_transform(rotation, translation):
    """Build a 4x4 homogeneous transform."""
    out = np.eye(4)
    out[:3, :3] = np.asarray(rotation, dtype=np.float64)
    out[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return out


def rotation_angle_deg(rotation):
    """Return the geodesic angle of a rotation matrix in degrees."""
    cosine = (np.trace(rotation) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


def mean_transform(transforms):
    """Average translations and project the average rotation to SO(3)."""
    translation = np.mean([t[:3, 3] for t in transforms], axis=0)
    rotation_sum = np.mean([t[:3, :3] for t in transforms], axis=0)
    u, _, vt = np.linalg.svd(rotation_sum)
    rotation = u @ vt
    if np.linalg.det(rotation) < 0.0:
        u[:, -1] *= -1.0
        rotation = u @ vt
    return make_transform(rotation, translation)


def board_pose_stats(transforms):
    """Return board-in-base translation/rotation spread around the mean."""
    mean = mean_transform(transforms)
    translation_errors = [
        np.linalg.norm(t[:3, 3] - mean[:3, 3]) for t in transforms
    ]
    rotation_errors = [
        rotation_angle_deg(mean[:3, :3].T @ t[:3, :3]) for t in transforms
    ]
    return {
        'mean_transform': mean.tolist(),
        'translation_std_m': float(np.sqrt(np.mean(
            np.square(translation_errors)))),
        'max_translation_error_m': float(max(translation_errors)),
        'rotation_std_deg': float(np.sqrt(np.mean(
            np.square(rotation_errors)))),
        'max_rotation_error_deg': float(max(rotation_errors)),
    }


def sample_joints(sample):
    """Return the joint angles used for FK.

    Datasets captured after 2026-11-17 include ``measured_joints`` read back
    from the bus servos. They are the physical servo positions and are more
    accurate than the controller's cached commands in ``joints``.
    """
    measured = sample.get('measured_joints')
    if measured and len(measured) >= 4:
        return measured
    return sample['joints']


def link4_pose(kinematics, joints):
    """Return T_base_link4 for one joint vector."""
    frame = kinematics.forward(joints[:4])
    rotation = np.array([
        [frame.M[i, j] for j in range(3)] for i in range(3)
    ])
    translation = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
    return make_transform(rotation, translation)


def link4_poses(samples):
    """Compute T_base_link4 for each sample from its arm joints."""
    kinematics = ArmKinematics(tip_link='link4')
    if kinematics.joint_names != ['joint1', 'joint2', 'joint3', 'joint4']:
        raise RuntimeError(
            'unexpected link4 chain joints: {}'.format(
                kinematics.joint_names))
    return [
        link4_pose(kinematics, sample_joints(sample))
        for sample in samples
    ]


def board_poses(samples):
    """Return T_cam_board for each sample."""
    poses = []
    for sample in samples:
        rotation = cv2.Rodrigues(
            np.asarray(sample['board']['rvec'], dtype=np.float64))[0]
        translation = np.asarray(
            sample['board']['tvec'], dtype=np.float64)
        poses.append(make_transform(rotation, translation))
    return poses


def refine_intrinsics(samples):
    """Run a chessboard intrinsic calibration as an independent check."""
    object_points = np.zeros(
        (BOARD_COLS * BOARD_ROWS, 3), np.float32)
    object_points[:, :2] = np.mgrid[
        0:BOARD_COLS, 0:BOARD_ROWS].T.reshape(-1, 2) * SQUARE_M
    image_points = [
        np.asarray(sample['board']['corners'], dtype=np.float32).reshape(
            -1, 1, 2)
        for sample in samples
    ]
    camera_matrix = np.asarray(
        samples[0]['camera_matrix'], dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.asarray(
        samples[0]['dist_coeffs'], dtype=np.float64).reshape(-1)
    rms, refined_k, refined_d, _, _ = cv2.calibrateCamera(
        [object_points] * len(image_points), image_points, IMAGE_SIZE,
        camera_matrix.copy(), dist_coeffs.copy(),
        flags=cv2.CALIB_USE_INTRINSIC_GUESS)
    return {
        'driver_k': camera_matrix.tolist(),
        'driver_d': dist_coeffs.tolist(),
        'refined_k': refined_k.tolist(),
        'refined_d': refined_d.reshape(-1).tolist(),
        'rms_px': float(rms),
        'focal_delta_px': {
            'fx': float(refined_k[0, 0] - camera_matrix[0, 0]),
            'fy': float(refined_k[1, 1] - camera_matrix[1, 1]),
        },
    }


def heldout_final_base(samples, t_link4_camera, mean_board,
                       max_rms=MAX_BOARD_RMS_PX):
    """Check final-base samples that were excluded from the hand-eye fit."""
    held = [
        sample for sample in samples
        if sample.get('final_base') and sample.get('board') is not None
        and sample['board']['rms_px'] <= max_rms
    ]
    if not held:
        return None
    kinematics = ArmKinematics(tip_link='link4')
    translation_errors = []
    rotation_errors = []
    for sample in held:
        t_base_board = (
            link4_pose(kinematics, sample_joints(sample))
            @ t_link4_camera
            @ board_poses([sample])[0])
        translation_errors.append(float(np.linalg.norm(
            t_base_board[:3, 3] - mean_board[:3, 3])))
        rotation_errors.append(rotation_angle_deg(
            mean_board[:3, :3].T @ t_base_board[:3, :3]))
    return {
        'sample_indices': [sample['index'] for sample in held],
        'translation_error_m': max(translation_errors),
        'rotation_error_deg': max(rotation_errors),
    }


def main():
    """Run the solver and write a JSON report."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--samples', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--max-board-rms', type=float,
                        default=MAX_BOARD_RMS_PX)
    args = parser.parse_args()

    with open(args.samples, 'r', encoding='utf-8') as handle:
        dataset = json.load(handle)
    usable = [
        sample for sample in dataset['samples']
        if sample.get('board') is not None
        and not sample.get('final_base')
        and sample['board']['rms_px'] <= args.max_board_rms
    ]
    if len(usable) < 5:
        raise RuntimeError(
            'only {} usable samples (need >= 5)'.format(len(usable)))

    measured_samples = [
        sample for sample in usable if sample.get('measured_joints')
    ]
    joint_source = {
        'measured_servo_samples': len(measured_samples),
        'total_samples': len(usable),
        'command_vs_measured_max_deg': None,
        'command_vs_measured_rms_deg': None,
    }
    if measured_samples:
        differences = np.degrees(np.abs(np.array([
            np.asarray(sample['measured_joints'][:4], dtype=np.float64)
            - np.asarray(sample['joints'][:4], dtype=np.float64)
            for sample in measured_samples
        ])))
        joint_source['command_vs_measured_max_deg'] = float(
            differences.max())
        joint_source['command_vs_measured_rms_deg'] = float(
            np.sqrt(np.mean(np.square(differences))))

    t_base_link4 = link4_poses(usable)
    t_cam_board = board_poses(usable)
    r_gripper2base = [t[:3, :3] for t in t_base_link4]
    t_gripper2base = [t[:3, 3].reshape(3, 1) for t in t_base_link4]
    r_target2cam = [t[:3, :3] for t in t_cam_board]
    t_target2cam = [t[:3, 3].reshape(3, 1) for t in t_cam_board]

    method_results = {}
    for name, method in HAND_EYE_METHODS.items():
        try:
            rotation, translation = cv2.calibrateHandEye(
                r_gripper2base, t_gripper2base,
                r_target2cam, t_target2cam, method=method)
        except cv2.error as exc:
            method_results[name] = {'error': str(exc)}
            continue
        t_link4_cam = make_transform(rotation, translation)
        t_base_board = [
            t_base_link4[i] @ t_link4_cam @ t_cam_board[i]
            for i in range(len(usable))
        ]
        stats = board_pose_stats(t_base_board)
        difference = np.linalg.inv(
            STATIC_TF_LINK4_RGB) @ t_link4_cam
        method_results[name] = {
            'T_link4_camera': t_link4_cam.tolist(),
            'translation_m': t_link4_cam[:3, 3].tolist(),
            'rpy_deg': cv2.RQDecomp3x3(t_link4_cam[:3, :3])[0],
            'board_translation_std_m': stats['translation_std_m'],
            'board_rotation_std_deg': stats['rotation_std_deg'],
            'board_max_translation_error_m':
                stats['max_translation_error_m'],
            'board_max_rotation_error_deg':
                stats['max_rotation_error_deg'],
            'board_mean_transform': stats['mean_transform'],
            'difference_from_static_translation_m': float(
                np.linalg.norm(difference[:3, 3])),
            'difference_from_static_rotation_deg': rotation_angle_deg(
                difference[:3, :3]),
        }

    valid_methods = {
        name: result for name, result in method_results.items()
        if 'error' not in result
    }
    if not valid_methods:
        raise RuntimeError('all hand-eye methods failed')
    selected = min(
        valid_methods,
        key=lambda name: (
            valid_methods[name]['board_translation_std_m']
            + 0.002 * valid_methods[name]['board_rotation_std_deg']))
    selected_result = valid_methods[selected]

    intrinsics = refine_intrinsics(usable)
    heldout = heldout_final_base(
        dataset['samples'],
        np.asarray(selected_result['T_link4_camera'], dtype=np.float64),
        np.asarray(
            selected_result['board_mean_transform'], dtype=np.float64))
    success = (
        selected_result['board_translation_std_m'] <= 0.005
        and selected_result['board_rotation_std_deg'] <= 2.0
        and intrinsics['rms_px'] <= 1.0
        and (
            heldout is None
            or (
                heldout['translation_error_m'] <= 0.005
                and heldout['rotation_error_deg'] <= 2.0
            )
        )
    )

    report = {
        'schema_version': 1,
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'mode': 'real_chessboard_hand_eye',
        'board': {
            'cols': BOARD_COLS,
            'rows': BOARD_ROWS,
            'square_m': SQUARE_M,
        },
        'samples_total': len(dataset['samples']),
        'samples_used': [sample['index'] for sample in usable],
        'joint_source': joint_source,
        'sample_rms_px': {
            str(sample['index']): sample['board']['rms_px']
            for sample in usable
        },
        'hand_eye': {
            'convention': 'T_link4_rgb_camera (camera in link4)',
            'selected': selected,
            'methods': method_results,
            'T_link4_camera': selected_result['T_link4_camera'],
        },
        'static_tf_link4_rgb': {
            'T': STATIC_TF_LINK4_RGB.tolist(),
            'role': (
                'informational: the vendor static TF is a body frame, while '
                'the PnP pose is in the camera optical frame; a ~90 deg '
                'rotation difference is expected'),
            'difference_translation_m': selected_result[
                'difference_from_static_translation_m'],
            'difference_rotation_deg': selected_result[
                'difference_from_static_rotation_deg'],
        },
        'intrinsics': intrinsics,
        'validation': {
            'board_translation_std_m':
                selected_result['board_translation_std_m'],
            'board_rotation_std_deg':
                selected_result['board_rotation_std_deg'],
            'board_max_translation_error_m':
                selected_result['board_max_translation_error_m'],
            'board_max_rotation_error_deg':
                selected_result['board_max_rotation_error_deg'],
            'heldout_final_base': heldout,
            'success': bool(success),
            'criteria': {
                'board_translation_std_m': 0.005,
                'board_rotation_std_deg': 2.0,
                'intrinsics_rms_px': 1.0,
                'heldout_final_base_translation_m': 0.005,
                'heldout_final_base_rotation_deg': 2.0,
            },
        },
    }

    output_dir = os.path.dirname(os.path.abspath(args.output))
    os.makedirs(output_dir, exist_ok=True)
    with open(args.output, 'w', encoding='utf-8') as handle:
        json.dump(report, handle, indent=2)
    print(
        'HAND_EYE selected={} t={} rpy_deg={}'.format(
            selected,
            [round(value, 4) for value in
             selected_result['translation_m']],
            [round(value, 3) for value in selected_result['rpy_deg']]))
    print(
        'BOARD std: dt={:.4f} m dr={:.3f} deg max_dt={:.4f} max_dr={:.3f}'
        .format(
            selected_result['board_translation_std_m'],
            selected_result['board_rotation_std_deg'],
            selected_result['board_max_translation_error_m'],
            selected_result['board_max_rotation_error_deg']))
    print(
        'STATIC diff (informational, body vs optical): '
        'dt={:.4f} m dr={:.3f} deg'.format(
            selected_result['difference_from_static_translation_m'],
            selected_result['difference_from_static_rotation_deg']))
    if heldout is not None:
        print(
            'HELDOUT final base: dt={:.4f} m dr={:.3f} deg'.format(
                heldout['translation_error_m'],
                heldout['rotation_error_deg']))
    print(
        'INTRINSICS driver fx={:.3f} fy={:.3f}; refined fx={:.3f} '
        'fy={:.3f}; rms={:.3f}px'.format(
            intrinsics['driver_k'][0][0], intrinsics['driver_k'][1][1],
            intrinsics['refined_k'][0][0], intrinsics['refined_k'][1][1],
            intrinsics['rms_px']))
    print('RESULT {}'.format('PASS' if success else 'FAIL'))
    print('WROTE', args.output)
    return 0 if success else 1


if __name__ == '__main__':
    raise SystemExit(main())
