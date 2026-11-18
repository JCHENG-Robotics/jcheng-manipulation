#!/usr/bin/env python3
"""
Same-scene RGB-D vs chessboard check.

Reads the snapshot produced by ``rgbd_snapshot.py`` (same-stamp RGB image,
registered point cloud, joints) and compares three planes in ``base_link``:

1. the chessboard **top** plane from RGB PnP with the validated table-view
   TSAI hand-eye and the refined intrinsics;
2. the chessboard **top** plane from the registered depth cloud, selected by
   projecting the cloud into the detected board polygon;
3. the **table** plane from the same cloud outside the board.

The key comparison is (1) vs (2): both are the same physical surface, so a
non-zero gap is an RGB/depth inconsistency, not board thickness. The old
"board top is 6.1 mm above the table" number is the sum of that gap and the
board thickness measured by (2) vs (3). Pass the caliper thickness with
``--board-thickness-mm`` to close the loop.

Run on the host with the Jazzy workspace sourced (PyKDL/xacro/OpenCV), e.g.:

    source /opt/ros/jazzy/setup.bash
    source ros2_ws/install/setup.bash
    python3 tools/phase2/rgbd_board_check.py \
        --snapshot /tmp/rgbd_board_20261118 \
        --calibration \
        evidence/calibration/20261118/table_down/\
        calibration_table_down_20261118.json \
        --output evidence/phase2/onsite_20261118/rgbd_board_check/result.json \
        --board-thickness-mm 4.0

The tool only reads files; it never connects to ROS.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os

import cv2
import numpy as np

BOARD_COLS = 11
BOARD_ROWS = 8
SQUARE_M = 0.020
MIN_RANGE_M = 0.15
MAX_RANGE_M = 0.60
RANSAC_THRESHOLD_M = 0.004
RANSAC_ITERATIONS = 800


def board_object_points(cols, rows, square_m):
    """Return the planar chessboard object points in metres."""
    points = np.zeros((cols * rows, 3), np.float32)
    points[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square_m
    return points


def detect_board(image, camera_matrix, dist_coeffs, cols, rows, square_m):
    """Detect the chessboard and solve its pose in the camera frame."""
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    found = False
    corners = None
    if hasattr(cv2, 'findChessboardCornersSB'):
        found, corners = cv2.findChessboardCornersSB(
            gray, (cols, rows),
            cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    if not found:
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
        found, corners = cv2.findChessboardCorners(
            gray, (cols, rows), flags)
        if found:
            criteria = (
                cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
                50, 1e-3)
            corners = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1), criteria)
    if not found or corners is None:
        return None
    object_points = board_object_points(cols, rows, square_m)
    solved, rvec, tvec = cv2.solvePnP(
        object_points, corners, camera_matrix, dist_coeffs,
        flags=cv2.SOLVEPNP_IPPE)
    if not solved:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(
        object_points, corners, camera_matrix, dist_coeffs, rvec, tvec)
    projected, _ = cv2.projectPoints(
        object_points, rvec, tvec, camera_matrix, dist_coeffs)
    error = projected.reshape(-1, 2) - corners.reshape(-1, 2)
    rms = float(np.sqrt(np.mean(np.sum(error * error, axis=1))))
    return {
        'rms_px': rms,
        'rvec': [float(value) for value in rvec.reshape(3)],
        'tvec': [float(value) for value in tvec.reshape(3)],
        'corners_px': corners.reshape(-1, 2).astype(float).tolist(),
    }


def make_transform(rotation, translation):
    """Build a 4x4 homogeneous transform."""
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = translation
    return transform


def board_transform(detection):
    """Return T_cam_board from the PnP result."""
    rotation = cv2.Rodrigues(
        np.asarray(detection['rvec'], dtype=np.float64))[0]
    translation = np.asarray(detection['tvec'], dtype=np.float64)
    return make_transform(rotation, translation)


def plane_from_points(points):
    """Fit a plane by SVD and return normal, offset and RMS."""
    centroid = np.mean(points, axis=0)
    _, _, vh = np.linalg.svd(points - centroid, full_matrices=False)
    normal = vh[-1]
    normal = normal / np.linalg.norm(normal)
    offset = -float(normal @ centroid)
    distances = points @ normal + offset
    rms = float(np.sqrt(np.mean(distances * distances)))
    return normal, offset, rms


def ransac_plane(points, threshold, iterations, rng):
    """Return ``(normal, offset, inlier_mask)`` for the dominant plane."""
    best = None
    for _ in range(iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        length = np.linalg.norm(normal)
        if length < 1e-9:
            continue
        normal = normal / length
        offset = -float(normal @ sample[0])
        distances = np.abs(points @ normal + offset)
        mask = distances < threshold
        count = int(np.count_nonzero(mask))
        if best is None or count > best[0]:
            best = (count, normal, offset, mask)
    if best is None:
        raise ValueError('RANSAC could not fit a plane')
    return best[1], best[2], best[3]


def plane_to_base(normal, offset, transform):
    """Transform a camera-frame plane to base_link."""
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    normal_base = rotation @ normal
    offset_base = offset - float(normal_base @ translation)
    if normal_base[2] < 0.0:
        normal_base = -normal_base
        offset_base = -offset_base
    return normal_base, float(offset_base)


def z_at_plane(normal, offset, x, y):
    """Return the plane z at a base_link (x, y)."""
    return -(offset + normal[0] * x + normal[1] * y) / normal[2]


def angle_between(normal_a, normal_b):
    """Return the unsigned angle between two plane normals, degrees."""
    cosine = abs(float(normal_a @ normal_b))
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def polygon_mask(shape, corners, erode_px=0, dilate_px=0):
    """Return a uint8 mask for the convex hull of the board corners."""
    mask = np.zeros(shape[:2], np.uint8)
    points = np.asarray(corners, dtype=np.int32).reshape(-1, 2)
    # The detector returns the inner corners in row-major order; connecting
    # them directly would draw a zigzag, not the board outline.
    hull = cv2.convexHull(points).reshape(-1, 1, 2)
    cv2.fillPoly(mask, [hull], 255)
    if erode_px > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * erode_px + 1, 2 * erode_px + 1))
        mask = cv2.erode(mask, kernel)
    if dilate_px > 0:
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, (2 * dilate_px + 1, 2 * dilate_px + 1))
        mask = cv2.dilate(mask, kernel)
    return mask


def project_cloud(points, camera_matrix):
    """Project camera-frame points into pixels; return uv and validity."""
    z = points[:, 2]
    valid = z > 1e-6
    uv = np.full((len(points), 2), -1.0)
    uv[valid, 0] = (
        camera_matrix[0, 0] * points[valid, 0] / z[valid]
        + camera_matrix[0, 2]
    )
    uv[valid, 1] = (
        camera_matrix[1, 1] * points[valid, 1] / z[valid]
        + camera_matrix[1, 2]
    )
    return uv, valid


def select_points(points, uv, valid, mask):
    """Return the finite, in-range points whose pixels are inside *mask*."""
    height, width = mask.shape
    inside = (
        valid
        & (uv[:, 0] >= 0.0)
        & (uv[:, 0] < width)
        & (uv[:, 1] >= 0.0)
        & (uv[:, 1] < height)
    )
    selected = np.zeros(len(points), bool)
    if np.any(inside):
        columns = np.clip(uv[inside, 0].astype(int), 0, width - 1)
        rows = np.clip(uv[inside, 1].astype(int), 0, height - 1)
        selected[np.where(inside)[0]] = mask[rows, columns] > 0
    finite = np.isfinite(points).all(axis=1)
    ranges = np.linalg.norm(points, axis=1)
    in_range = (ranges >= MIN_RANGE_M) & (ranges <= MAX_RANGE_M)
    return points[selected & finite & in_range]


def t_base_link4(joints, joint_names):
    """Compute T_base_link4 from measured joints using the project FK."""
    from armpi_grasp_planner.reachability import ArmKinematics

    kinematics = ArmKinematics(tip_link='link4')
    by_name = {name: float(value)
               for name, value in zip(joint_names, joints)}
    values = [by_name[name] for name in kinematics.joint_names]
    frame = kinematics.forward(values)
    rotation = np.array(
        [[frame.M[row, column] for column in range(3)]
         for row in range(3)])
    translation = np.array([frame.p.x(), frame.p.y(), frame.p.z()])
    return make_transform(rotation, translation)


def analyse(snapshot_dir, calibration_path, cols, rows, square_m,
            erode_px, dilate_px, caliper_mm, seed):
    """Run the same-frame comparison and return a report dictionary."""
    metadata = json.load(open(
        os.path.join(snapshot_dir, 'metadata.json'), encoding='utf-8'))
    image = np.load(os.path.join(snapshot_dir, 'rgb.npy'))
    if metadata['rgb']['encoding'] == 'rgb8':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    points = np.load(os.path.join(snapshot_dir, 'points.npy'))
    if len(points) == 0:
        raise ValueError('snapshot has no cloud points')

    calibration = json.load(open(calibration_path, encoding='utf-8'))
    hand_eye = np.array(
        calibration['hand_eye']['methods']['TSAI']['T_link4_camera'])
    refined_k = np.array(calibration['intrinsics']['refined_k'])
    refined_d = np.array(calibration['intrinsics']['refined_d'])
    driver_k = np.array(metadata['rgb']['camera_info']['k']).reshape(3, 3)
    driver_d = np.array(metadata['rgb']['camera_info']['d'])

    detection = detect_board(
        image, refined_k, refined_d, cols, rows, square_m)
    if detection is None:
        raise ValueError('chessboard not found in the snapshot RGB image')

    board_center_obj = np.array([
        (cols - 1) * square_m / 2.0,
        (rows - 1) * square_m / 2.0,
        0.0,
        1.0,
    ])
    T_cam_board = board_transform(detection)
    T_base_link4 = t_base_link4(
        metadata['joints']['positions'], metadata['joints']['names'])
    T_base_cam = T_base_link4 @ hand_eye
    T_base_board = T_base_cam @ T_cam_board
    board_center_base = (T_base_board @ board_center_obj)[:3]

    rgb_normal = T_base_board[:3, :3] @ np.array([0.0, 0.0, 1.0])
    rgb_normal = rgb_normal / np.linalg.norm(rgb_normal)
    if rgb_normal[2] < 0.0:
        rgb_normal = -rgb_normal
    rgb_offset = -float(rgb_normal @ T_base_board[:3, 3])
    rgb_center_z = z_at_plane(
        rgb_normal, rgb_offset, board_center_base[0], board_center_base[1])

    uv, valid = project_cloud(points, driver_k)
    board_mask = polygon_mask(
        image.shape, detection['corners_px'], erode_px=erode_px)
    board_points = select_points(points, uv, valid, board_mask)
    if len(board_points) < 500:
        raise ValueError(
            'only {} board points; check the polygon/depth'.format(
                len(board_points)))
    rng = np.random.default_rng(seed)
    subset = board_points
    if len(subset) > 40000:
        subset = subset[
            rng.choice(len(subset), 40000, replace=False)]
    normal, offset, inliers = ransac_plane(
        subset, RANSAC_THRESHOLD_M, RANSAC_ITERATIONS, rng)
    board_inliers = np.count_nonzero(
        np.abs(board_points @ normal + offset) < RANSAC_THRESHOLD_M)
    if board_inliers < 500:
        raise ValueError('board RANSAC found too few inliers')
    normal, offset, board_rms = plane_from_points(
        board_points[np.abs(board_points @ normal + offset)
                     < RANSAC_THRESHOLD_M])
    depth_board_normal, depth_board_offset = plane_to_base(
        normal, offset, T_base_cam)
    depth_board_center = (
        T_base_cam @ np.append(board_points.mean(axis=0), 1.0))[:3]
    depth_board_z = z_at_plane(
        depth_board_normal, depth_board_offset,
        board_center_base[0], board_center_base[1])

    table_mask = polygon_mask(
        image.shape, detection['corners_px'],
        erode_px=0, dilate_px=dilate_px)
    table_points = select_points(points, uv, valid, table_mask)
    if len(table_points) < 500:
        raise ValueError(
            'only {} table points; check the polygon/depth'.format(
                len(table_points)))
    subset = table_points
    if len(subset) > 60000:
        subset = subset[
            rng.choice(len(subset), 60000, replace=False)]
    normal, offset, _ = ransac_plane(
        subset, RANSAC_THRESHOLD_M, RANSAC_ITERATIONS, rng)
    table_inliers = np.count_nonzero(
        np.abs(table_points @ normal + offset) < RANSAC_THRESHOLD_M)
    normal, offset, table_rms = plane_from_points(
        table_points[np.abs(table_points @ normal + offset)
                     < RANSAC_THRESHOLD_M])
    table_normal, table_offset = plane_to_base(
        normal, offset, T_base_cam)
    table_z = z_at_plane(
        table_normal, table_offset,
        board_center_base[0], board_center_base[1])

    report = {
        'schema_version': 1,
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'mode': 'same_scene_rgbd_chessboard_check',
        'motion': 'none',
        'snapshot': {
            'directory': snapshot_dir,
            'rgb_stamp': metadata['rgb']['stamp'],
            'cloud_stamp': metadata['cloud']['stamp'],
            'rgb_size': [metadata['rgb']['width'],
                         metadata['rgb']['height']],
            'cloud_points': metadata['cloud']['points'],
            'joints': metadata['joints']['positions'],
            'joint_names': metadata['joints']['names'],
        },
        'board': {
            'cols': cols,
            'rows': rows,
            'square_m': square_m,
            'rms_px': detection['rms_px'],
            'corners_px': detection['corners_px'],
            'rvec_camera': detection['rvec'],
            'tvec_camera': detection['tvec'],
        },
        'hand_eye': {
            'source': calibration_path,
            'method': 'TSAI',
            'T_link4_camera': hand_eye.tolist(),
        },
        'intrinsics': {
            'used': 'refined',
            'k': refined_k.tolist(),
            'd': refined_d.tolist(),
            'driver_k': driver_k.tolist(),
            'driver_d': driver_d.tolist(),
        },
        'rgb_board_plane_base': {
            'normal': rgb_normal.tolist(),
            'offset': rgb_offset,
            'z_at_board_center': float(rgb_center_z),
            'board_center_base': board_center_base.tolist(),
        },
        'depth_board_plane_base': {
            'normal': depth_board_normal.tolist(),
            'offset': depth_board_offset,
            'z_at_board_center': float(depth_board_z),
            'center_base': depth_board_center.tolist(),
            'points_used': int(len(board_points)),
            'inliers': int(board_inliers),
            'rms_m': board_rms,
        },
        'depth_table_plane_base': {
            'normal': table_normal.tolist(),
            'offset': table_offset,
            'z_at_board_center': float(table_z),
            'points_used': int(len(table_points)),
            'inliers': int(table_inliers),
            'rms_m': table_rms,
        },
        'comparisons': {
            'rgb_board_vs_depth_board_mm': float(
                (rgb_center_z - depth_board_z) * 1000.0),
            'rgb_board_vs_depth_board_angle_deg': angle_between(
                rgb_normal, depth_board_normal),
            'depth_board_vs_table_mm': float(
                (depth_board_z - table_z) * 1000.0),
            'rgb_board_vs_table_mm': float(
                (rgb_center_z - table_z) * 1000.0),
            'caliper_board_thickness_mm': (
                float(caliper_mm) if caliper_mm is not None else None),
            'caliper_minus_depth_board_mm': (
                float(caliper_mm - (depth_board_z - table_z) * 1000.0)
                if caliper_mm is not None else None),
        },
    }
    return report, image, detection, board_mask, table_mask


def write_diagnostics(path, image, detection, board_mask, table_mask,
                      report):
    """Draw the board polygon and the key numbers onto the RGB frame."""
    canvas = image.copy()
    points = np.asarray(
        detection['corners_px'], dtype=np.int32).reshape(-1, 2)
    hull = cv2.convexHull(points).reshape(-1, 1, 2)
    cv2.polylines(canvas, [hull], True, (0, 255, 0), 2)
    contours, _ = cv2.findContours(
        board_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(canvas, contours, -1, (0, 255, 255), 1)
    contours, _ = cv2.findContours(
        table_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    cv2.drawContours(canvas, contours, -1, (255, 0, 255), 1)
    comparisons = report['comparisons']
    lines = [
        'board RMS {:.2f} px'.format(report['board']['rms_px']),
        'RGB board vs depth board {:+.2f} mm'.format(
            comparisons['rgb_board_vs_depth_board_mm']),
        'depth board vs table {:+.2f} mm'.format(
            comparisons['depth_board_vs_table_mm']),
        'RGB board vs table {:+.2f} mm'.format(
            comparisons['rgb_board_vs_table_mm']),
    ]
    for index, line in enumerate(lines):
        cv2.putText(
            canvas, line, (10, 20 + 18 * index),
            cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 0, 255), 1)
    directory = os.path.dirname(path)
    if directory:
        os.makedirs(directory, exist_ok=True)
    cv2.imwrite(path, canvas)


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Compare a same-frame RGB board plane and depth planes.')
    parser.add_argument('--snapshot', required=True)
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--diagnostics', default=None)
    parser.add_argument('--board-cols', type=int, default=BOARD_COLS)
    parser.add_argument('--board-rows', type=int, default=BOARD_ROWS)
    parser.add_argument('--square-m', type=float, default=SQUARE_M)
    parser.add_argument('--erode-px', type=int, default=20)
    parser.add_argument('--dilate-px', type=int, default=80)
    parser.add_argument('--board-thickness-mm', type=float, default=None)
    parser.add_argument('--seed', type=int, default=0)
    return parser.parse_args(argv)


def main(argv=None):
    """Run the analysis. Returns a process exit code."""
    args = _parse_args(argv)
    report, image, detection, board_mask, table_mask = analyse(
        args.snapshot,
        args.calibration,
        args.board_cols,
        args.board_rows,
        args.square_m,
        args.erode_px,
        args.dilate_px,
        args.board_thickness_mm,
        args.seed,
    )
    directory = os.path.dirname(args.output)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(args.output, 'w', encoding='utf-8') as handle:
        json.dump(report, handle, indent=2)
    diagnostics = args.diagnostics or (args.output + '.png')
    write_diagnostics(
        diagnostics, image, detection, board_mask, table_mask, report)
    comparisons = report['comparisons']
    print('RGB board vs depth board: {:+.2f} mm ({:.2f} deg)'.format(
        comparisons['rgb_board_vs_depth_board_mm'],
        comparisons['rgb_board_vs_depth_board_angle_deg']))
    print('depth board vs table:     {:+.2f} mm'.format(
        comparisons['depth_board_vs_table_mm']))
    print('RGB board vs table:       {:+.2f} mm'.format(
        comparisons['rgb_board_vs_table_mm']))
    if args.board_thickness_mm is not None:
        print('caliper minus depth:      {:+.2f} mm'.format(
            comparisons['caliper_minus_depth_board_mm']))
    print('report: {}'.format(args.output))
    print('diagnostics: {}'.format(diagnostics))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
