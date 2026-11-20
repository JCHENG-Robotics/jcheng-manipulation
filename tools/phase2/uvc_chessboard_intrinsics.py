#!/usr/bin/env python3
"""Chessboard intrinsic calibration for a UVC camera captured outside ROS.

The on-board Aurora 930 driver only runs inside the robot container, so its
intrinsics come from ``chessboard_capture.py`` samples.  The off-board
RealSense D435i is a plain UVC device: frames are captured with ``ffmpeg`` on
the Pi and solved here on the laptop, which keeps OpenCV off the robot.

The script takes a directory of frames from a hand-held board sweep, keeps one
well-spread view per (position, size, tilt) bucket, runs
``cv2.calibrateCamera`` and reports the model in the same pinhole + plumb-bob
form ``camera_info`` expects.  A two-fold split gives the parameter
uncertainty without a second dataset.

    python3 tools/phase2/uvc_chessboard_intrinsics.py \
        --frames-dir /tmp/uvc_intr_20261120 \
        --output evidence/calibration/20261120/uvc_color_intrinsics.json
"""

import argparse
import glob
import json
import math
import os

import cv2
import numpy as np

BOARD_COLS = 11
BOARD_ROWS = 8
SQUARE_M = 0.020
MAX_CORNER_RMS_PX = 1.5
EDGE_MARGIN_PX = 10
DEFAULT_MAX_VIEWS = 30


def board_object_points(cols=BOARD_COLS, rows=BOARD_ROWS, square_m=SQUARE_M):
    """Return the board's inner-corner grid in board coordinates (metres)."""
    points = np.zeros((cols * rows, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    return points


def detect_board(image, cols=BOARD_COLS, rows=BOARD_ROWS, accuracy=False):
    """Detect an inner-corner grid and return sub-pixel corners plus quality.

    ``findChessboardCornersSB`` is preferred because it refines internally and
    is more stable on a hand-held sweep; the classic detector is the fallback.
    ``accuracy=True`` enables the exhaustive search, which is ~4x slower, so it
    is only used to refine the selected calibration views.
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    flags = 0
    if accuracy and hasattr(cv2, 'CALIB_CB_ACCURACY'):
        flags = cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY
    if hasattr(cv2, 'findChessboardCornersSB'):
        found, corners = cv2.findChessboardCornersSB(
            gray, (cols, rows), flags=flags)
    else:
        found, corners = cv2.findChessboardCorners(
            gray, (cols, rows),
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE)
        corners = None
    if not found or corners is None:
        return None
    corners = corners.reshape(-1, 2).astype(np.float64)
    if corners.shape[0] != cols * rows:
        return None
    # Hold the classic detector to the same sub-pixel standard.
    if not hasattr(cv2, 'findChessboardCornersSB'):
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                    30, 0.01)
        corners = cv2.cornerSubPix(
            gray, corners.reshape(-1, 1, 2).astype(np.float32),
            (5, 5), (-1, -1), criteria).reshape(-1, 2).astype(np.float64)
    grid = corners.reshape(rows, cols, 2)
    # Squares must be convex and similarly sized; a folded sweep frame is
    # useless for calibration and would bias the model.
    horiz = np.linalg.norm(np.diff(grid, axis=1), axis=2)
    vert = np.linalg.norm(np.diff(grid, axis=0), axis=2)
    if np.median(horiz) < 6.0 or np.median(vert) < 6.0:
        return None
    return {
        'corners': corners,
        'bbox': cv2.boundingRect(corners.astype(np.float32)),
        'horiz_px': float(np.median(horiz)),
        'vert_px': float(np.median(vert)),
    }


def view_features(corners, image_size, rows=BOARD_ROWS, cols=BOARD_COLS):
    """Return a normalised description of one board view.

    The vector combines where the board sits, how large it is, how oblique it
    is and how it is rotated in the image plane.  ``select_diverse`` uses it to
    spread the calibration set; a hand sweep otherwise collapses into runs of
    nearly identical frames.
    """
    width, height = image_size
    grid = np.asarray(corners).reshape(rows, cols, 2)
    x, y, w, h = cv2.boundingRect(grid.reshape(-1, 2).astype(np.float32))
    center_x = (x + w / 2.0) / width
    center_y = (y + h / 2.0) / height
    area = float(w * h) / float(width * height)
    horizontal_edges = np.diff(grid, axis=1)
    vertical_edges = np.diff(grid, axis=0)
    horiz = float(np.median(np.linalg.norm(horizontal_edges, axis=2)))
    vert = float(np.median(np.linalg.norm(vertical_edges, axis=2)))
    # A tilted board foreshortens one grid direction; the log ratio separates
    # near-frontal from strongly oblique views.
    tilt = abs(math.log(max(horiz, 1e-6) / max(vert, 1e-6)))
    direction = np.median(horizontal_edges.reshape(-1, 2), axis=0)
    angle = math.atan2(direction[1], direction[0])
    return {
        'center': (center_x, center_y),
        'area': area,
        'tilt': tilt,
        'angle': angle,
    }


def feature_vector(features):
    """Map view features into a metric space used for diverse selection."""
    center_x, center_y = features['center']
    return np.array([
        center_x,
        center_y,
        0.25 * math.log(max(features['area'], 1e-6)),
        features['tilt'] / 0.5,
        0.3 * math.cos(2.0 * features['angle']),
        0.3 * math.sin(2.0 * features['angle']),
    ])


def select_diverse(views, max_views=DEFAULT_MAX_VIEWS, min_distance=0.06):
    """Pick a well-spread subset with farthest-point sampling.

    A hand sweep produces runs of nearly identical frames, so a bucket grid
    either keeps too many near-duplicates or collapses to a handful of bins.
    Farthest-point sampling adapts to whatever spread the sweep actually has.
    """
    if not views:
        return []
    vectors = np.array([
        feature_vector(view['features']) for view in views])
    start = int(np.argmax([view['area'] for view in views]))
    selected = [start]
    distances = np.linalg.norm(vectors - vectors[start], axis=1)
    while len(selected) < min(max_views, len(views)):
        candidate = int(np.argmax(distances))
        if distances[candidate] < min_distance:
            break
        selected.append(candidate)
        distances = np.minimum(
            distances,
            np.linalg.norm(vectors - vectors[candidate], axis=1))
    return [views[index] for index in selected]


def calibrate(views, image_size, object_points, flags=0):
    """Run ``cv2.calibrateCamera`` on the selected views."""
    image_points = [
        v['corners'].reshape(-1, 1, 2).astype(np.float32) for v in views]
    objp = np.asarray(object_points, np.float32)
    rms, matrix, dist, rvecs, tvecs = cv2.calibrateCamera(
        [objp] * len(image_points), image_points, image_size,
        None, None, flags=flags)
    per_view = []
    for view, rvec, tvec in zip(views, rvecs, tvecs):
        projected, _ = cv2.projectPoints(
            object_points, rvec, tvec, matrix, dist)
        error = np.linalg.norm(
            view['corners'] - projected.reshape(-1, 2), axis=1)
        per_view.append({
            'frame': os.path.basename(view['frame']),
            'rms_px': float(np.sqrt(np.mean(error ** 2))),
            'max_px': float(np.max(error)),
        })
    return {
        'rms_px': float(rms),
        'camera_matrix': matrix,
        'dist_coeffs': dist.reshape(-1),
        'per_view': per_view,
    }


def split_cross_check(views, image_size, object_points, flags=0):
    """Calibrate on interleaved halves to bound the parameter uncertainty."""
    if len(views) < 8:
        return None
    halves = [views[0::2], views[1::2]]
    results = [calibrate(half, image_size, object_points, flags=flags)
               for half in halves]
    fx = [r['camera_matrix'][0, 0] for r in results]
    fy = [r['camera_matrix'][1, 1] for r in results]
    cx = [r['camera_matrix'][0, 2] for r in results]
    cy = [r['camera_matrix'][1, 2] for r in results]
    return {
        'fx_spread_px': float(abs(fx[0] - fx[1])),
        'fy_spread_px': float(abs(fy[0] - fy[1])),
        'cx_spread_px': float(abs(cx[0] - cx[1])),
        'cy_spread_px': float(abs(cy[0] - cy[1])),
        'half_rms_px': [r['rms_px'] for r in results],
    }


def build_report(frames, image_size, max_views=DEFAULT_MAX_VIEWS, stride=1,
                 fix_aspect=False):
    """Detect, select, calibrate and assemble the result dictionary.

    Detection runs on every ``stride``-th frame; the selected views are then
    re-detected with the exhaustive search before the calibration.
    """
    object_points = board_object_points()
    views = []
    rejected = []
    for path in frames[::max(1, stride)]:
        image = cv2.imread(path, cv2.IMREAD_COLOR)
        if image is None:
            rejected.append({'frame': os.path.basename(path),
                             'reason': 'unreadable'})
            continue
        if (image.shape[1], image.shape[0]) != tuple(image_size):
            rejected.append({'frame': os.path.basename(path),
                             'reason': 'size {}'.format(image.shape[:2])})
            continue
        detection = detect_board(image)
        if detection is None:
            rejected.append({'frame': os.path.basename(path),
                             'reason': 'no board'})
            continue
        x, y, w, h = detection['bbox']
        if (x < EDGE_MARGIN_PX or y < EDGE_MARGIN_PX
                or x + w > image_size[0] - EDGE_MARGIN_PX
                or y + h > image_size[1] - EDGE_MARGIN_PX):
            rejected.append({'frame': os.path.basename(path),
                             'reason': 'board touches the border'})
            continue
        features = view_features(detection['corners'], image_size)
        views.append({
            'frame': path,
            'corners': detection['corners'],
            'features': features,
            'area': features['area'],
        })
    selected = select_diverse(views, max_views=max_views)
    if len(selected) < 4:
        raise RuntimeError(
            'need at least 4 usable board views, found {}'.format(
                len(selected)))
    refined = 0
    for view in selected:
        image = cv2.imread(view['frame'], cv2.IMREAD_COLOR)
        if image is None:
            continue
        accurate = detect_board(image, accuracy=True)
        if accurate is not None:
            view['corners'] = accurate['corners']
            refined += 1
    flags = cv2.CALIB_FIX_ASPECT_RATIO if fix_aspect else 0
    result = calibrate(selected, image_size, object_points, flags=flags)
    cross = split_cross_check(selected, image_size, object_points, flags=flags)
    matrix = result['camera_matrix']
    dist = result['dist_coeffs']
    width, height = image_size
    aspect_error = abs(matrix[0, 0] - matrix[1, 1]) / matrix[0, 0]
    warnings = []
    if aspect_error > 0.02:
        warnings.append(
            'fx and fy differ by {:.1f}%%; the sweep does not constrain the '
            'vertical focal length. Re-sweep with the board smaller in frame '
            'and tilted about both axes, or pass --fix-aspect.'.format(
                100.0 * aspect_error))
    if max(abs(matrix[0, 2] - width / 2.0), abs(matrix[1, 2] - height / 2.0)) \
            > 0.05 * max(width, height):
        warnings.append('principal point is far from the image centre.')
    report = {
        'tool': 'uvc_chessboard_intrinsics',
        'board': {'cols': BOARD_COLS, 'rows': BOARD_ROWS,
                  'square_m': SQUARE_M},
        'image_size': [int(width), int(height)],
        'model': 'plumb_bob',
        'frame_count': len(frames),
        'detected_views': len(views),
        'selected_views': len(selected),
        'refined_views': refined,
        'detection_stride': int(stride),
        'rejected': rejected[:40],
        'rms_px': result['rms_px'],
        'camera_matrix': matrix.tolist(),
        'dist_coeffs': dist.tolist(),
        'focal_px': {'fx': float(matrix[0, 0]), 'fy': float(matrix[1, 1])},
        'principal_point_px': {'cx': float(matrix[0, 2]),
                               'cy': float(matrix[1, 2])},
        'fov_deg': {
            'horizontal': float(math.degrees(
                2.0 * math.atan(width / (2.0 * matrix[0, 0])))),
            'vertical': float(math.degrees(
                2.0 * math.atan(height / (2.0 * matrix[1, 1])))),
        },
        'per_view': result['per_view'],
        'fix_aspect': bool(fix_aspect),
        'fx_fy_relative_error': float(aspect_error),
        'warnings': warnings,
        'cross_check': cross,
        'notes': [
            'Frames are captured with ffmpeg on the Pi (UVC), not in ROS.',
            'Distortion is the 5-parameter plumb-bob model reported by the '
            'driver for the on-board camera.',
        ],
    }
    return report


def main(argv=None):
    parser = argparse.ArgumentParser(
        description='Chessboard intrinsics for a UVC camera.')
    parser.add_argument('--frames-dir', required=True,
                        help='directory with the captured frames')
    parser.add_argument('--pattern', default='*.jpg',
                        help='glob inside --frames-dir (default *.jpg)')
    parser.add_argument('--max-views', type=int, default=DEFAULT_MAX_VIEWS)
    parser.add_argument('--stride', type=int, default=1,
                        help='detect every Nth frame (default 1)')
    parser.add_argument('--fix-aspect', action='store_true',
                        help='force fx == fy (square pixels)')
    parser.add_argument('--output', required=True,
                        help='JSON report path')
    args = parser.parse_args(argv)
    frames = sorted(glob.glob(os.path.join(args.frames_dir, args.pattern)))
    if not frames:
        raise SystemExit('no frames matched {}'.format(
            os.path.join(args.frames_dir, args.pattern)))
    first = cv2.imread(frames[0], cv2.IMREAD_COLOR)
    if first is None:
        raise SystemExit('cannot read {}'.format(frames[0]))
    image_size = (first.shape[1], first.shape[0])
    report = build_report(frames, image_size, max_views=args.max_views,
                          stride=args.stride, fix_aspect=args.fix_aspect)
    output_dir = os.path.dirname(os.path.abspath(args.output))
    if output_dir:
        os.makedirs(output_dir, exist_ok=True)
    with open(args.output, 'w') as handle:
        json.dump(report, handle, indent=2)
        handle.write('\n')
    matrix = np.asarray(report['camera_matrix'])
    print('views: {} detected, {} selected of {} frames'.format(
        report['detected_views'], report['selected_views'],
        report['frame_count']))
    print('rms: {:.3f} px  fx {:.1f}  fy {:.1f}  cx {:.1f}  cy {:.1f}'.format(
        report['rms_px'], matrix[0, 0], matrix[1, 1], matrix[0, 2],
        matrix[1, 2]))
    print('fov: {:.1f} x {:.1f} deg'.format(
        report['fov_deg']['horizontal'], report['fov_deg']['vertical']))
    if report['cross_check']:
        cross = report['cross_check']
        print('split-half spread: fx {:.1f} px  fy {:.1f} px  '
              'cx {:.1f} px  cy {:.1f} px'.format(
                  cross['fx_spread_px'], cross['fy_spread_px'],
                  cross['cx_spread_px'], cross['cy_spread_px']))
    for warning in report['warnings']:
        print('WARNING: {}'.format(warning))
    print('wrote {}'.format(args.output))
    return report


if __name__ == '__main__':
    main()
