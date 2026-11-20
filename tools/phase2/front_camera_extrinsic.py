#!/usr/bin/env python3
"""Solve ``base_link -> front camera`` from a board seen by both cameras.

The off-board D435i is rigidly mounted on the chassis front, below the arm
base.  Its extrinsic is solved by chaining two observations of the *same*
chessboard lying on the table:

1. the on-arm Aurora camera (validated ``T_link4_camera`` hand-eye plus FK from
   the measured bus-servo joints) gives ``T_base_board``;
2. the front camera gives ``T_front_board`` with ``solvePnP``;
3. ``T_base_front = T_base_board @ inv(T_front_board)``.

The board must not move between the two captures.  The arm is folded away for
the front capture, so the front view is unoccluded.

The wrist half runs with the robot stack sourced (FK needs the URDF):

    source /opt/ros/jazzy/setup.bash
    source ros2_ws/install/setup.bash
    python3 tools/phase2/front_camera_extrinsic.py \
        --wrist-samples /tmp/chessboard_front_20261120/samples.json \
        --wrist-calibration \
            evidence/calibration/20261117/calibration_servo_20261117.json \
        --front-frames /tmp/front_board_20261120 \
        --front-calibration \
            evidence/calibration/20261120/uvc_color_intrinsics_aspect.json \
        --output evidence/calibration/20261120/front_camera_extrinsic.json

This is a calibration result, not an absolute metrology claim: the residual
board-plane and pitch checks bound how far it can be wrong, they do not prove
millimetre truth.
"""

import argparse
import glob
import json
import math
import os

import cv2
import numpy as np

import chessboard_hand_eye as hand_eye

BOARD_COLS = 11
BOARD_ROWS = 8
SQUARE_M = 0.020
# Board poses are averaged (not fitted), so a 1.5 px corner gate is enough:
# 1.5 px at 0.45 m is ~1.6 mm of corner error and averages out over samples.
MAX_BOARD_RMS_PX = 1.5


def load_matrix(path):
    with open(path) as handle:
        return json.load(handle)


def wrist_board_poses(samples, t_link4_camera, kinematics):
    """Return ``T_base_board`` for every usable wrist sample."""
    poses = []
    for sample in samples:
        board = sample.get('board')
        if board is None or board.get('rms_px', 9.0) > MAX_BOARD_RMS_PX:
            continue
        joints = sample.get('measured_joints') or sample.get('joints')
        if joints is None:
            continue
        frame = kinematics.forward(joints[:4])
        rotation = np.array([[frame.M[i, j] for j in range(3)]
                             for i in range(3)])
        t_base_link4 = hand_eye.make_transform(
            rotation, [frame.p.x(), frame.p.y(), frame.p.z()])
        t_cam_board = hand_eye.make_transform(
            cv2.Rodrigues(np.asarray(board['rvec'], np.float64))[0],
            board['tvec'])
        poses.append(t_base_link4 @ t_link4_camera @ t_cam_board)
    return poses


def board_object_points(cols=BOARD_COLS, rows=BOARD_ROWS, square_m=SQUARE_M):
    points = np.zeros((cols * rows, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    return points


def front_board_pose(image, camera_matrix, dist_coeffs):
    """Detect the board in a front-camera frame and solve its pose."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    if hasattr(cv2, 'findChessboardCornersSB'):
        found, corners = cv2.findChessboardCornersSB(
            gray, (BOARD_COLS, BOARD_ROWS),
            flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    else:
        found, corners = cv2.findChessboardCorners(
            gray, (BOARD_COLS, BOARD_ROWS))
    if not found or corners is None:
        return None
    corners = corners.reshape(-1, 2)
    object_points = board_object_points()
    # A planar target has two mirror solutions.  With solvePnP's convention a
    # board whose printed face is visible has its +z axis pointing away from
    # the camera, so the right solution has a positive normal z.  The mirror
    # solution would mean looking at the board's blank back.
    ok, rvecs, tvecs, errors = cv2.solvePnPGeneric(
        object_points, corners, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE)
    if not ok or rvecs is None:
        return None
    candidates = []
    for rvec, tvec, error in zip(rvecs, tvecs, errors):
        rotation = cv2.Rodrigues(rvec)[0]
        normal_z = float(rotation[2, 2])
        candidates.append((normal_z > 0.1, float(np.ravel(error)[0]),
                           rvec, tvec))
    facing = [c for c in candidates if c[0]]
    if not facing:
        return None
    _, _, rvec, tvec = min(facing, key=lambda item: item[1])
    rvec, tvec = cv2.solvePnPRefineLM(
        object_points, corners, camera_matrix, dist_coeffs, rvec, tvec)
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, dist_coeffs)
    rms = float(np.sqrt(np.mean(np.square(
        np.linalg.norm(corners - projected.reshape(-1, 2), axis=1)))))
    if rms > MAX_BOARD_RMS_PX:
        return None
    return {
        'transform': hand_eye.make_transform(
            cv2.Rodrigues(rvec)[0], tvec.reshape(3)),
        'rms_px': rms,
    }


def _board_flip():
    """180 deg relabelling of the corner grid (a symmetric board's twin)."""
    half_x = 0.5 * (BOARD_COLS - 1) * SQUARE_M
    half_y = 0.5 * (BOARD_ROWS - 1) * SQUARE_M
    return hand_eye.make_transform(
        cv2.Rodrigues(np.array([0.0, 0.0, math.pi]))[0],
        [2.0 * half_x, 2.0 * half_y, 0.0])


def compose_extrinsic(t_base_board, t_front_board):
    """Return ``T_base_front`` from two observations of the same board."""
    return t_base_board @ np.linalg.inv(t_front_board)


def plausible_camera_prior(t_base_front):
    """Score how well a solve matches the known chassis-front mount.

    The board is symmetric, so the front observation is ambiguous up to the
    180 deg grid relabelling.  The camera is bolted to the chassis front, so
    the right solve puts it a little forward of base_link and keeps the yaw
    near zero; the flipped solve throws it to the far side of the board.
    """
    translation = t_base_front[:3, 3]
    axis = t_base_front[:3, 2]
    yaw = abs(math.degrees(math.atan2(axis[1], axis[0])))
    return float(
        abs(translation[0] - 0.10) + abs(translation[1]) * 2.0
        + abs(translation[2] - 0.13) + 0.02 * yaw)


def axis_report(t_base_cam, board_height_m=None):
    """Describe the solved camera pose in base_link in human terms."""
    rotation = t_base_cam[:3, :3]
    optical_axis = rotation[:, 2]
    translation = t_base_cam[:3, 3]
    report = {
        'translation_m': translation.tolist(),
        'optical_axis_base': optical_axis.tolist(),
        'pitch_below_horizontal_deg': float(
            math.degrees(math.asin(float(np.clip(-optical_axis[2], -1, 1))))),
        'yaw_deg': float(math.degrees(math.atan2(
            float(optical_axis[1]), float(optical_axis[0])))),
        'roll_deg': float(math.degrees(math.atan2(
            float(rotation[1, 0]), float(rotation[0, 0])))),
    }
    if board_height_m is not None:
        report['height_above_board_m'] = float(translation[2] - board_height_m)
    return report


def board_plane_tilt_deg(t_base_board):
    """Angle between the board normal and base_link +z (0 for a flat board)."""
    normal = t_base_board[:3, 2]
    cosine = float(np.clip(abs(normal[2]), 0.0, 1.0))
    return float(math.degrees(math.acos(cosine)))


def build_report(wrist_poses, front_poses):
    """Assemble the extrinsic result from the two board pose lists."""
    if not wrist_poses:
        raise RuntimeError('no usable wrist-camera board poses')
    if not front_poses:
        raise RuntimeError('no usable front-camera board poses')
    wrist_stats = hand_eye.board_pose_stats(wrist_poses)
    front_stats = hand_eye.board_pose_stats(front_poses)
    t_base_board = np.asarray(wrist_stats['mean_transform'])
    t_front_board = np.asarray(front_stats['mean_transform'])
    candidates = [t_front_board, t_front_board @ _board_flip()]
    solved = [(plausible_camera_prior(compose_extrinsic(t_base_board, c)),
               c) for c in candidates]
    solved.sort(key=lambda item: item[0])
    score, t_front_board = solved[0]
    t_base_front = compose_extrinsic(t_base_board, t_front_board)
    return {
        'T_base_board': t_base_board.tolist(),
        'T_front_board': t_front_board.tolist(),
        'T_base_front': t_base_front.tolist(),
        'wrist_observations': len(wrist_poses),
        'front_observations': len(front_poses),
        'wrist_board_spread': {
            'translation_m': wrist_stats['translation_std_m'],
            'rotation_deg': wrist_stats['rotation_std_deg'],
        },
        'front_board_spread': {
            'translation_m': front_stats['translation_std_m'],
            'rotation_deg': front_stats['rotation_std_deg'],
        },
        'board_plane_tilt_deg': board_plane_tilt_deg(t_base_board),
        'grid_flip_used': bool(np.allclose(t_front_board,
                                           candidates[1])),
        'prior_score': float(score),
        'prior_scores': [float(item[0]) for item in solved],
        'camera': axis_report(t_base_front, board_height_m=t_base_board[2, 3]),
    }


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Solve base_link -> front camera from a shared board.')
    parser.add_argument('--wrist-samples', required=True,
                        help='samples.json from chessboard_capture.py')
    parser.add_argument('--wrist-calibration', required=True,
                        help='hand-eye JSON with hand_eye.T_link4_camera')
    parser.add_argument('--front-frames', required=True,
                        help='directory of front-camera board frames')
    parser.add_argument('--front-pattern', default='*.jpg')
    parser.add_argument('--front-calibration', required=True,
                        help='intrinsics JSON from uvc_chessboard_intrinsics')
    parser.add_argument('--output', required=True)
    args = parser.parse_args(argv)

    from armpi_grasp_planner.reachability import ArmKinematics

    wrist_calibration = load_matrix(args.wrist_calibration)
    t_link4_camera = np.asarray(
        wrist_calibration['hand_eye']['T_link4_camera'], np.float64)
    samples = load_matrix(args.wrist_samples)
    if isinstance(samples, dict):
        samples = samples.get('samples', [])
    kinematics = ArmKinematics(tip_link='link4')
    wrist_poses = wrist_board_poses(samples, t_link4_camera, kinematics)

    front_calibration = load_matrix(args.front_calibration)
    camera_matrix = np.asarray(
        front_calibration['camera_matrix'], np.float64)
    dist_coeffs = np.asarray(front_calibration['dist_coeffs'], np.float64)
    frames = sorted(glob.glob(
        os.path.join(args.front_frames, args.front_pattern)))
    if not frames:
        raise SystemExit('no front frames matched {}'.format(
            os.path.join(args.front_frames, args.front_pattern)))
    front_poses = []
    per_frame = []
    for path in frames:
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            continue
        detection = front_board_pose(image, camera_matrix, dist_coeffs)
        if detection is None:
            per_frame.append({'frame': os.path.basename(path),
                              'rms_px': None})
            continue
        front_poses.append(detection['transform'])
        per_frame.append({'frame': os.path.basename(path),
                          'rms_px': detection['rms_px']})

    report = build_report(wrist_poses, front_poses)
    report['per_frame'] = per_frame
    report['wrist_board_rms_max_px'] = MAX_BOARD_RMS_PX
    report['notes'] = [
        'T_base_front maps front-camera optical coordinates into base_link.',
        'wrist half uses measured bus-servo joints and the validated '
        'T_link4_camera; the board must not move between captures.',
    ]
    output_dir = os.path.dirname(os.path.abspath(args.output))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    with open(args.output, 'w') as handle:
        json.dump(report, handle, indent=2)
        handle.write('\n')

    camera = report['camera']
    print('wrist observations {}  front observations {}'.format(
        report['wrist_observations'], report['front_observations']))
    print('board spread: wrist {:.1f} mm / {:.2f} deg  front {:.1f} mm / '
          '{:.2f} deg'.format(
              1000.0 * report['wrist_board_spread']['translation_m'],
              report['wrist_board_spread']['rotation_deg'],
              1000.0 * report['front_board_spread']['translation_m'],
              report['front_board_spread']['rotation_deg']))
    print('T_base_front: x {:.3f} y {:.3f} z {:.3f} m'.format(
        *camera['translation_m']))
    print('pitch {:.1f} deg  yaw {:.1f} deg  height above board {:.0f} mm  '
          'board tilt {:.1f} deg'.format(
              camera['pitch_below_horizontal_deg'], camera['yaw_deg'],
              1000.0 * camera['height_above_board_m'],
              report['board_plane_tilt_deg']))
    print('wrote {}'.format(args.output))
    return report


if __name__ == '__main__':
    main()
