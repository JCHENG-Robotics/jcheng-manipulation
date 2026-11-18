#!/usr/bin/env python3
"""
Same-scene RGB-D vs a matte AprilTag target.

The chessboard test showed no depth step at the board: the high-contrast,
glossy board returned no usable depth and the driver filled the hole with the
table plane. This tool uses a **uniform white matte paper** target with a
printed tag instead, so the depth sees the paper surface and the RGB sees the
tag on that same surface.

It reads the snapshot from ``rgbd_snapshot.py`` and reports three planes in
``base_link``:

1. the paper/tag plane from RGB tag PnP (table-view TSAI hand-eye, refined
   intrinsics);
2. the paper plane from the registered cloud in a ring just outside the tag
   (the white margin);
3. the table plane from the cloud outside the target/board.

The key check is (1) vs (2): both are the paper top, so the gap must be small.
(2) vs (3) is the physical stack height (board + paper), which should match the
caliper measurement. Run on the host with the Jazzy workspace sourced:

    source /opt/ros/jazzy/setup.bash
    source ros2_ws/install/setup.bash
    export PYTHONPATH=$PWD/ros2_ws/src/armpi_fiducial_localizer:$PYTHONPATH
    python3 tools/phase2/rgbd_tag_check.py \
        --snapshot /tmp/rgbd_tag_20261118 \
        --calibration \
        evidence/calibration/20261118/table_down/\
        calibration_table_down_20261118.json \
        --output evidence/phase2/onsite_20261118/rgbd_tag_check/result.json \
        --board-thickness-mm 11.0

The tool only reads files; it never connects to ROS.
"""

import argparse
from datetime import datetime, timezone
import json
import os

import cv2
import numpy as np

from rgbd_board_check import (
    angle_between,
    make_transform,
    plane_from_points,
    plane_to_base,
    polygon_mask,
    project_cloud,
    ransac_plane,
    select_points,
    t_base_link4,
    z_at_plane,
)

MARKER_SIZE_M = 0.06
RANSAC_THRESHOLD_M = 0.004
RANSAC_ITERATIONS = 800
RING_DILATE_PX = 24
TABLE_INNER_PX = 220


def load_hand_eye(calibration_path):
    """Load T_link4_camera from either calibration report layout."""
    with open(calibration_path, encoding='utf-8') as handle:
        report = json.load(handle)
    hand_eye = report['hand_eye']
    if 'methods' in hand_eye:
        transform = hand_eye['methods']['TSAI']['T_link4_camera']
    else:
        transform = hand_eye['T_link4_camera']
    return np.asarray(transform, dtype=np.float64), report


def tag_transform(detection):
    """Return T_cam_tag from a FiducialDetection (z out of the paper)."""
    rotation = np.asarray(detection.rotation_matrix, dtype=np.float64)
    translation = np.asarray(detection.tvec, dtype=np.float64)
    return make_transform(rotation, translation)


def ring_mask(shape, corners_px, dilate_px):
    """Return the white-margin ring between the tag and its dilation."""
    tag = polygon_mask(shape, corners_px)
    outer = polygon_mask(
        shape, corners_px, erode_px=0, dilate_px=dilate_px)
    ring = outer.copy()
    ring[tag > 0] = 0
    return ring


def outside_radius_mask(shape, center_px, radius_px):
    """Return a mask outside a circle, used for the surrounding table."""
    mask = np.full(shape[:2], 255, np.uint8)
    center = (int(round(center_px[0])), int(round(center_px[1])))
    cv2.circle(mask, center, int(radius_px), 0, -1)
    return mask


def analyse(snapshot_dir, calibration_path, caliper_board_mm,
            paper_mm, ring_dilate_px, table_inner_px, seed,
            marker_size_m=MARKER_SIZE_M):
    """Run the same-scene tag comparison and return the report."""
    from armpi_fiducial_localizer.detector import FiducialDetector

    metadata = json.load(open(
        os.path.join(snapshot_dir, 'metadata.json'), encoding='utf-8'))
    image = np.load(os.path.join(snapshot_dir, 'rgb.npy'))
    if metadata['rgb']['encoding'] == 'rgb8':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    points = np.load(os.path.join(snapshot_dir, 'points.npy'))

    hand_eye, calibration = load_hand_eye(calibration_path)
    refined_k = np.array(calibration['intrinsics']['refined_k'])
    refined_d = np.array(calibration['intrinsics']['refined_d'])
    driver_k = np.array(
        metadata['rgb']['camera_info']['k']).reshape(3, 3)

    detections = FiducialDetector(
        marker_size_m=marker_size_m).detect(image, refined_k, refined_d)
    if not detections:
        raise ValueError('AprilTag not found in the snapshot RGB image')
    detection = detections[0]

    T_base_link4 = t_base_link4(
        metadata['joints']['positions'], metadata['joints']['names'])
    T_base_cam = T_base_link4 @ hand_eye
    T_base_tag = T_base_cam @ tag_transform(detection)
    tag_center_base = T_base_tag[:3, 3]
    tag_normal = T_base_tag[:3, :3] @ np.array([0.0, 0.0, 1.0])
    tag_normal = tag_normal / np.linalg.norm(tag_normal)
    if tag_normal[2] < 0.0:
        tag_normal = -tag_normal
    tag_offset = -float(tag_normal @ T_base_tag[:3, 3])
    tag_z = z_at_plane(
        tag_normal, tag_offset, tag_center_base[0], tag_center_base[1])

    uv, valid = project_cloud(points, driver_k)
    ring = ring_mask(image.shape, detection.corners_px, ring_dilate_px)
    paper_points = select_points(points, uv, valid, ring)
    if len(paper_points) < 300:
        raise ValueError(
            'only {} paper points; check the tag ring/depth'.format(
                len(paper_points)))
    rng = np.random.default_rng(seed)
    subset = paper_points
    if len(subset) > 40000:
        subset = subset[rng.choice(len(subset), 40000, replace=False)]
    normal, offset, _ = ransac_plane(
        subset, RANSAC_THRESHOLD_M, RANSAC_ITERATIONS, rng)
    paper_inliers = np.count_nonzero(
        np.abs(paper_points @ normal + offset) < RANSAC_THRESHOLD_M)
    normal, offset, paper_rms = plane_from_points(
        paper_points[np.abs(paper_points @ normal + offset)
                     < RANSAC_THRESHOLD_M])
    paper_normal, paper_offset = plane_to_base(
        normal, offset, T_base_cam)
    paper_z = z_at_plane(
        paper_normal, paper_offset,
        tag_center_base[0], tag_center_base[1])

    corners = np.asarray(
        detection.corners_px, dtype=np.float64).reshape(-1, 2)
    tag_center_px = corners.mean(axis=0)
    table_mask = outside_radius_mask(
        image.shape, tag_center_px, table_inner_px)
    table_points = select_points(points, uv, valid, table_mask)
    if len(table_points) < 1000:
        raise ValueError(
            'only {} table points; check the mask/range'.format(
                len(table_points)))
    subset = table_points
    if len(subset) > 60000:
        subset = subset[rng.choice(len(subset), 60000, replace=False)]
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
        tag_center_base[0], tag_center_base[1])

    caliper_stack_mm = None
    if caliper_board_mm is not None:
        caliper_stack_mm = float(caliper_board_mm)
        if paper_mm is not None:
            caliper_stack_mm += float(paper_mm)
    paper_minus_table_mm = (paper_z - table_z) * 1000.0
    return {
        'schema_version': 1,
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'mode': 'same_scene_rgbd_apriltag_check',
        'motion': 'none',
        'snapshot': {
            'directory': snapshot_dir,
            'rgb_stamp': metadata['rgb']['stamp'],
            'cloud_stamp': metadata['cloud']['stamp'],
            'cloud_points': metadata['cloud']['points'],
            'joints': metadata['joints']['positions'],
        },
        'tag': {
            'marker_id': detection.marker_id,
            'marker_size_m': float(marker_size_m),
            'reprojection_error_px': detection.reprojection_error_px,
            'marker_side_px': detection.marker_side_px,
            'corners_px': [list(point) for point in detection.corners_px],
            'tvec_camera': list(detection.tvec),
            'rvec_camera': list(detection.rvec),
        },
        'hand_eye': {
            'source': calibration_path,
            'T_link4_camera': hand_eye.tolist(),
        },
        'intrinsics': {
            'used': 'refined',
            'k': refined_k.tolist(),
            'd': refined_d.tolist(),
            'driver_k': driver_k.tolist(),
        },
        'rgb_tag_plane_base': {
            'normal': tag_normal.tolist(),
            'offset': tag_offset,
            'z_at_tag_center': float(tag_z),
            'tag_center_base': tag_center_base.tolist(),
        },
        'depth_paper_plane_base': {
            'normal': paper_normal.tolist(),
            'offset': paper_offset,
            'z_at_tag_center': float(paper_z),
            'points_used': int(len(paper_points)),
            'inliers': int(paper_inliers),
            'rms_m': paper_rms,
        },
        'depth_table_plane_base': {
            'normal': table_normal.tolist(),
            'offset': table_offset,
            'z_at_tag_center': float(table_z),
            'points_used': int(len(table_points)),
            'inliers': int(table_inliers),
            'rms_m': table_rms,
        },
        'comparisons': {
            'rgb_tag_vs_depth_paper_mm': float((tag_z - paper_z) * 1000.0),
            'rgb_tag_vs_depth_paper_angle_deg': angle_between(
                tag_normal, paper_normal),
            'depth_paper_vs_table_mm': float(paper_minus_table_mm),
            'rgb_tag_vs_table_mm': float((tag_z - table_z) * 1000.0),
            'caliper_board_mm': (
                None if caliper_board_mm is None
                else float(caliper_board_mm)),
            'caliper_paper_mm': (
                None if paper_mm is None else float(paper_mm)),
            'caliper_stack_mm': caliper_stack_mm,
            'caliper_minus_depth_paper_mm': (
                None if caliper_stack_mm is None
                else float(caliper_stack_mm - paper_minus_table_mm)),
        },
    }


def write_diagnostics(path, image, detection, report):
    """Draw the tag, ring and key numbers onto the RGB frame."""
    canvas = image.copy()
    corners = np.asarray(
        detection.corners_px, dtype=np.int32).reshape(-1, 1, 2)
    cv2.polylines(canvas, [corners], True, (0, 255, 0), 2)
    center = tuple(
        int(round(value))
        for value in np.asarray(detection.corners_px).reshape(-1, 2).mean(
            axis=0))
    cv2.circle(canvas, center, RING_DILATE_PX, (0, 255, 255), 1)
    comparisons = report['comparisons']
    lines = [
        'tag id {} err {:.2f} px'.format(
            report['tag']['marker_id'],
            report['tag']['reprojection_error_px']),
        'RGB tag vs depth paper {:+.2f} mm'.format(
            comparisons['rgb_tag_vs_depth_paper_mm']),
        'depth paper vs table {:+.2f} mm'.format(
            comparisons['depth_paper_vs_table_mm']),
        'RGB tag vs table {:+.2f} mm'.format(
            comparisons['rgb_tag_vs_table_mm']),
    ]
    if comparisons['caliper_stack_mm'] is not None:
        lines.append('caliper stack {:.2f} mm'.format(
            comparisons['caliper_stack_mm']))
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
        description='Compare a same-frame RGB tag plane and depth planes.')
    parser.add_argument('--snapshot', required=True)
    parser.add_argument('--calibration', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--diagnostics', default=None)
    parser.add_argument('--marker-size-m', type=float, default=MARKER_SIZE_M)
    parser.add_argument('--ring-dilate-px', type=int, default=RING_DILATE_PX)
    parser.add_argument(
        '--table-inner-px', type=int, default=TABLE_INNER_PX)
    parser.add_argument('--board-thickness-mm', type=float, default=None)
    parser.add_argument('--paper-mm', type=float, default=None)
    parser.add_argument('--seed', type=int, default=0)
    return parser.parse_args(argv)


def main(argv=None):
    """Run the analysis. Returns a process exit code."""
    args = _parse_args(argv)
    report = analyse(
        args.snapshot,
        args.calibration,
        args.board_thickness_mm,
        args.paper_mm,
        args.ring_dilate_px,
        args.table_inner_px,
        args.seed,
        args.marker_size_m,
    )
    directory = os.path.dirname(args.output)
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(args.output, 'w', encoding='utf-8') as handle:
        json.dump(report, handle, indent=2)
    diagnostics = args.diagnostics or (args.output + '.png')
    metadata = json.load(open(
        os.path.join(args.snapshot, 'metadata.json'), encoding='utf-8'))
    image = np.load(os.path.join(args.snapshot, 'rgb.npy'))
    if metadata['rgb']['encoding'] == 'rgb8':
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    from armpi_fiducial_localizer.detector import FiducialDetector
    refined_k = np.array(report['intrinsics']['k'])
    refined_d = np.array(report['intrinsics']['d'])
    detection = FiducialDetector(
        marker_size_m=args.marker_size_m).detect(
            image, refined_k, refined_d)[0]
    write_diagnostics(diagnostics, image, detection, report)
    comparisons = report['comparisons']
    print('RGB tag vs depth paper:  {:+.2f} mm ({:.2f} deg)'.format(
        comparisons['rgb_tag_vs_depth_paper_mm'],
        comparisons['rgb_tag_vs_depth_paper_angle_deg']))
    print('depth paper vs table:    {:+.2f} mm'.format(
        comparisons['depth_paper_vs_table_mm']))
    print('RGB tag vs table:        {:+.2f} mm'.format(
        comparisons['rgb_tag_vs_table_mm']))
    if comparisons['caliper_stack_mm'] is not None:
        print('caliper stack:           {:+.2f} mm'.format(
            comparisons['caliper_stack_mm']))
        print('caliper minus depth:     {:+.2f} mm'.format(
            comparisons['caliper_minus_depth_paper_mm']))
    print('report: {}'.format(args.output))
    print('diagnostics: {}'.format(diagnostics))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
