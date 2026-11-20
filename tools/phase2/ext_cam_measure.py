#!/usr/bin/env python3
"""Measure a cube's top-face centre in ``base_link`` from the D435i.

The off-board front camera is rigid to the robot base, so its measurement does
not inherit the wrist camera's arm-pose hand-eye/FK error. The chain is:

1. fit the support plane from the D435i depth frame (pose independent);
2. lift it by the measured cube edge to get the top-face plane;
3. rectify the colour image onto that plane (a true homography after
   undistortion) so the top face becomes a metric square;
4. segment the saturated, top-face-bright blob and take its centroid in the
   plane. The centroid is the base-frame top-face point.

Run on the laptop with OpenCV, after ``ext_cam_capture.py`` has saved a frame::

    CAL=evidence/calibration/20261120
    python3 tools/phase2/ext_cam_measure.py \
        --capture evidence/phase2/onsite_20261120/dual_grasp/frame_000 \
        --intrinsics $CAL/uvc_color_intrinsics_refined.json \
        --extrinsic $CAL/front_camera_extrinsic.json \
        --depth-to-color $CAL/depth_to_color_extrinsics.json \
        --output evidence/phase2/onsite_20261120/dual_grasp/measure.json \
        --overlay evidence/phase2/onsite_20261120/dual_grasp/measure.png

The tool only reads files; it never talks to the robot.
"""

import argparse
import json
import os

import cv2
import numpy as np

DEFAULT_CUBE_EDGE_M = 0.0394
DEFAULT_SCALE_M = 0.0005
DEFAULT_WINDOW_MM = (120.0, 90.0)
# Saturation separates the wooden cube from the light table; the top face is
# brighter than the front/side faces, so a high value threshold removes most of
# the side-face projection that the plane rectification would otherwise pull
# into the blob.
SATURATION_MIN = 70
VALUE_MIN = 68
VALUE_MAX = 150


def load_json(path):
    """Return a JSON file's contents."""
    with open(path, 'r', encoding='utf-8') as handle:
        return json.load(handle)


def jsonable(value):
    """Convert numpy scalars to Python types for json.dump."""
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError('not JSON serializable: {!r}'.format(value))


def camera_matrix(intrinsics):
    """Return the 3x3 matrix and distortion vector of a calibration report."""
    return (np.asarray(intrinsics['camera_matrix'], dtype=np.float64),
            np.asarray(intrinsics['dist_coeffs'], dtype=np.float64).ravel())


def fit_plane(points, iterations=400, threshold=0.003, seed=0):
    """Return ``(normal, offset, inlier_mask, rms)`` for a RANSAC plane.

    The plane is ``normal . p + offset = 0`` with ``|normal| = 1``.
    """
    rng = np.random.default_rng(seed)
    best_mask = None
    best_normal = None
    for _ in range(iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        a, b, c = sample
        normal = np.cross(b - a, c - a)
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal = normal / norm
        offset = -float(normal @ a)
        mask = np.abs(points @ normal + offset) < threshold
        if best_mask is None or mask.sum() > best_mask.sum():
            best_mask = mask
            best_normal = normal
    cloud = points[best_mask]
    centre = cloud.mean(axis=0)
    _, _, right = np.linalg.svd(cloud - centre, full_matrices=False)
    normal = right[-1]
    normal = normal / np.linalg.norm(normal)
    offset = -float(normal @ centre)
    if normal @ best_normal < 0:
        normal, offset = -normal, -offset
    residual = points[best_mask] @ normal + offset
    return normal, offset, best_mask, float(np.sqrt(np.mean(residual ** 2)))


def depth_points(depth, info):
    """Return valid depth points in the depth optical frame (metres)."""
    scale = float(info.get('scale_m_per_unit', 0.001))
    k = info['camera_info']['k']
    fx, fy, cx, cy = k[0], k[4], k[2], k[5]
    z = depth.astype(np.float64) * scale
    height, width = z.shape
    u, v = np.meshgrid(np.arange(width), np.arange(height))
    valid = (z > 0.10) & (z < 1.00)
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    return np.stack([x[valid], y[valid], z[valid]], axis=1)


def plane_to_base(normal, offset, depth_to_color, base_from_color):
    """Transform a plane from the depth frame to ``base_link``."""
    rotation_dc = np.asarray(depth_to_color['R'], dtype=np.float64)
    translation_dc = np.asarray(depth_to_color['t_m'], dtype=np.float64)
    rotation = np.asarray(base_from_color[:3, :3], dtype=np.float64)
    translation = np.asarray(base_from_color[:3, 3], dtype=np.float64)
    point_depth = -offset * normal
    point_color = rotation_dc @ point_depth + translation_dc
    normal_color = rotation_dc @ normal
    point_base = rotation @ point_color + translation
    normal_base = rotation @ normal_color
    offset_base = -float(normal_base @ point_base)
    return normal_base, offset_base, point_base


def project(point, camera_matrix_value, distortion, base_from_camera):
    """Project a base-frame point into the undistorted colour image."""
    point = np.asarray(point, dtype=np.float64)
    rotation = base_from_camera[:3, :3]
    translation = base_from_camera[:3, 3]
    camera = rotation.T @ (point - translation)
    if camera[2] <= 0:
        return None
    u = (camera_matrix_value[0, 0] * camera[0] / camera[2]
         + camera_matrix_value[0, 2])
    v = (camera_matrix_value[1, 1] * camera[1] / camera[2]
         + camera_matrix_value[1, 2])
    return np.array([u, v])


def plane_ray_hit(origin, direction, normal, constant):
    """Intersect a ray with ``normal . p = constant``."""
    denominator = float(normal @ direction)
    if abs(denominator) < 1e-9:
        return None
    distance = (constant - float(normal @ origin)) / denominator
    if distance <= 0:
        return None
    return origin + distance * direction


def rectify(color, camera_matrix_value, distortion, base_from_camera, normal,
            constant, anchor, scale, window):
    """Return the rectified image, its homography and the plane basis."""
    width = int(round(window[0] * 0.001 / scale))
    height = int(round(window[1] * 0.001 / scale))
    up = np.asarray([1.0, 0.0, 0.0])
    axis_u = up - (up @ normal) * normal
    axis_u = axis_u / np.linalg.norm(axis_u)
    axis_v = np.cross(normal, axis_u)
    axis_v = axis_v / np.linalg.norm(axis_v)
    us = (np.arange(width) - width / 2.0) * scale
    vs = (np.arange(height) - height / 2.0) * scale
    grid_u, grid_v = np.meshgrid(us, vs)
    points = (anchor[None, None, :]
              + grid_u[..., None] * axis_u[None, None, :]
              + grid_v[..., None] * axis_v[None, None, :])
    points = points.reshape(-1, 3)
    rotation = base_from_camera[:3, :3]
    translation = base_from_camera[:3, 3]
    camera = (points - translation) @ rotation
    projected = np.stack([
        camera_matrix_value[0, 0] * camera[:, 0] / camera[:, 2]
        + camera_matrix_value[0, 2],
        camera_matrix_value[1, 1] * camera[:, 1] / camera[:, 2]
        + camera_matrix_value[1, 2],
    ], axis=1)
    homography, _ = cv2.findHomography(
        np.stack([grid_u.ravel(), grid_v.ravel()], axis=1), projected, 0)
    pixel_to_plane = np.array([
        [scale, 0.0, -width / 2.0 * scale],
        [0.0, scale, -height / 2.0 * scale],
        [0.0, 0.0, 1.0],
    ])
    undistorted = cv2.undistort(color, camera_matrix_value, distortion)
    rectified = cv2.warpPerspective(
        undistorted, homography @ pixel_to_plane, (width, height),
        flags=cv2.INTER_LINEAR | cv2.WARP_INVERSE_MAP)
    return rectified, pixel_to_plane, axis_u, axis_v


def segment(rectified):
    """Return the largest top-face blob as ``(mask, centroid_px, rect)``."""
    hsv = cv2.cvtColor(rectified, cv2.COLOR_BGR2HSV)
    saturation = hsv[:, :, 1]
    value = hsv[:, :, 2]
    mask = ((saturation > SATURATION_MIN) & (value > VALUE_MIN)
            & (value < VALUE_MAX)).astype(np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                            np.ones((3, 3), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE,
                            np.ones((9, 9), np.uint8))
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
    if count < 2:
        return None, None, None
    index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
    mask = (labels == index).astype(np.uint8)
    contours, _ = cv2.findContours(
        mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contour = max(contours, key=cv2.contourArea)
    moments = cv2.moments(mask)
    centroid = np.array([moments['m10'] / moments['m00'],
                         moments['m01'] / moments['m00']])
    return mask, centroid, cv2.minAreaRect(contour)


def base_point(anchor, axis_u, axis_v, scale, centre_px, width, height):
    """Map a rectified pixel back to ``base_link``."""
    u = (centre_px[0] - width / 2.0) * scale
    v = (centre_px[1] - height / 2.0) * scale
    return anchor + u * axis_u + v * axis_v


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture', required=True,
                        help='dir with color.npy, depth.npy, metadata.json')
    parser.add_argument('--intrinsics', required=True,
                        help='refined colour intrinsics JSON')
    parser.add_argument('--extrinsic', required=True,
                        help='T_base_front report JSON')
    parser.add_argument('--depth-to-color', required=True,
                        help='depth->colour extrinsics JSON')
    parser.add_argument('--cube-edge', type=float, default=DEFAULT_CUBE_EDGE_M)
    parser.add_argument('--anchor', default=None,
                        help='optional base x,y of the expected object')
    parser.add_argument('--scale', type=float, default=DEFAULT_SCALE_M,
                        help='rectified metres per pixel')
    parser.add_argument('--window-mm', default='120,90')
    parser.add_argument('--x-range', default='0.15,0.32')
    parser.add_argument('--y-range', type=float, default=0.09)
    parser.add_argument('--min-plane-inliers', type=float, default=0.30)
    parser.add_argument('--max-plane-rms-m', type=float, default=0.003)
    parser.add_argument('--output', required=True)
    parser.add_argument('--overlay', default=None)
    return parser


def main(argv=None):
    """Measure and write the report."""
    args = build_parser().parse_args(argv)
    color = np.load(os.path.join(args.capture, 'color.npy'))
    depth = np.load(os.path.join(args.capture, 'depth.npy'))
    metadata = load_json(os.path.join(args.capture, 'metadata.json'))
    intrinsics = load_json(args.intrinsics)
    extrinsic = load_json(args.extrinsic)
    depth_to_color = load_json(args.depth_to_color)
    matrix, distortion = camera_matrix(intrinsics)
    base_from_camera = np.asarray(extrinsic['T_base_front'], dtype=np.float64)
    if tuple(intrinsics['image_size']) != color.shape[1::-1]:
        raise SystemExit('intrinsics image_size {} does not match colour {}'
                         .format(intrinsics['image_size'], color.shape[1::-1]))
    # 1. support plane from depth
    points = depth_points(depth, metadata['depth'])
    if len(points) < 1000:
        raise SystemExit('too few depth points: {}'.format(len(points)))
    normal_d, offset_d, mask, rms = fit_plane(points)
    normal, offset, plane_point = plane_to_base(
        normal_d, offset_d, depth_to_color, base_from_camera)
    if normal[2] < 0:
        normal, offset = -normal, -offset
    # 2. top-face plane
    plane_constant = -offset + args.cube_edge
    # 3. anchor: auto from the unrectified cube centroid, or the CLI value
    if args.anchor:
        x, y = (float(value) for value in args.anchor.split(','))
        anchor = np.array([x, y, 0.0])
        anchor[2] = ((plane_constant - normal[0] * x - normal[1] * y)
                     / normal[2])
    else:
        hsv = cv2.cvtColor(color, cv2.COLOR_BGR2HSV)
        cube = ((hsv[:, :, 1] > 80) & (hsv[:, :, 2] < VALUE_MAX)).astype(
            np.uint8)
        count, labels, stats, centres = cv2.connectedComponentsWithStats(cube)
        if count < 2:
            raise SystemExit('no saturated cube blob for the anchor')
        index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        centroid = centres[index]
        undistorted_points = cv2.undistortPoints(
            np.array([[centroid]], dtype=np.float64), matrix, distortion,
            P=matrix).reshape(2)
        direction = np.array([
            (undistorted_points[0] - matrix[0, 2]) / matrix[0, 0],
            (undistorted_points[1] - matrix[1, 2]) / matrix[1, 1],
            1.0,
        ])
        rotation = base_from_camera[:3, :3]
        direction_base = rotation @ direction
        direction_base = direction_base / np.linalg.norm(direction_base)
        anchor = plane_ray_hit(base_from_camera[:3, 3], direction_base,
                               normal, plane_constant)
        if anchor is None:
            raise SystemExit('anchor ray does not hit the top plane')
    # 4. rectified segmentation
    window = tuple(float(value) for value in args.window_mm.split(','))
    rectified, pixel_to_plane, axis_u, axis_v = rectify(
        color, matrix, distortion, base_from_camera, normal, plane_constant,
        anchor, args.scale, window)
    mask, centroid, rectangle = segment(rectified)
    if mask is None:
        raise SystemExit('no top-face blob in the rectified image')
    height, width = mask.shape
    point = base_point(anchor, axis_u, axis_v, args.scale, centroid,
                       width, height)
    extent_mm = None
    if rectangle is not None:
        extent_mm = sorted(
            value * args.scale * 1000.0 for value in rectangle[1])
    # 5. gates and report
    x_range = tuple(float(value) for value in args.x_range.split(','))
    blob_area_mm2 = float(mask.sum()) * args.scale ** 2 * 1e6
    plane_fraction = float((np.abs(points @ normal_d + offset_d)
                            < 0.003).mean())
    gates = {
        'plane_inlier_fraction': {
            'value': plane_fraction,
            'ok': plane_fraction >= args.min_plane_inliers,
        },
        'plane_rms_m': {
            'value': rms,
            'ok': rms <= args.max_plane_rms_m,
        },
        'x_in_range': {
            'value': float(point[0]),
            'ok': x_range[0] <= point[0] <= x_range[1],
        },
        'y_in_range': {
            'value': float(point[1]),
            'ok': abs(point[1]) <= args.y_range,
        },
        'rect_extent_mm': {
            'value': extent_mm,
            # The min side is the top-face edge; the max side absorbs the
            # front/side-face projection that rectification pulls just outside
            # the near edge.
            'ok': extent_mm is not None
            and 30.0 <= extent_mm[0] <= 55.0
            and extent_mm[1] <= 70.0,
        },
    }
    report = {
        'schema_version': 1,
        'mode': 'front_camera_top_face',
        'capture': os.path.abspath(args.capture),
        'cube_edge_m': args.cube_edge,
        'depth_plane': {
            'normal_base': [float(value) for value in normal],
            'rms_m': rms,
            'inlier_fraction': plane_fraction,
            'point_base': [float(value) for value in plane_point],
        },
        'top_plane': {
            'normal_base': [float(value) for value in normal],
            'height_m': float(plane_point[2] + args.cube_edge),
        },
        'anchor_base': [float(value) for value in anchor],
        'point_base': [float(value) for value in point],
        'image_position': [float(value) for value in centroid],
        'rectified': {
            'scale_m_per_px': args.scale,
            'window_mm': list(window),
            'blob_area_mm2': blob_area_mm2,
            'extent_mm': extent_mm,
            'angle_deg': float(rectangle[2]) if rectangle else None,
        },
        'gates': gates,
        'success': all(gate['ok'] for gate in gates.values()),
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.output)) or '.',
                exist_ok=True)
    with open(args.output, 'w', encoding='utf-8') as handle:
        json.dump(report, handle, indent=2, default=jsonable)
    if args.overlay:
        image = cv2.undistort(color, matrix, distortion)
        centre = project(point, matrix, distortion, base_from_camera)
        half_u = 0.5 * args.cube_edge * axis_u
        half_v = 0.5 * args.cube_edge * axis_v
        square = np.array([
            point + half_u + half_v,
            point - half_u + half_v,
            point - half_u - half_v,
            point + half_u - half_v,
        ])
        projected = [project(corner, matrix, distortion, base_from_camera)
                     for corner in square]
        if centre is not None:
            cv2.drawMarker(image, tuple(np.round(centre).astype(int)),
                           (0, 0, 255), cv2.MARKER_CROSS, 18, 2)
        if all(corner is not None for corner in projected):
            outline = np.round(projected).astype(np.int32)
            cv2.polylines(image, [outline.reshape(-1, 1, 2)],
                          True, (0, 255, 0), 2)
        cv2.imwrite(args.overlay, image)
    print('front top-face point (base): '
          '({:+.5f}, {:+.5f}, {:+.5f}) m'.format(*point))
    print('plane rms {:.2f} mm, inliers {:.1%}, extent {} mm, success {}'.
          format(rms * 1000, plane_fraction,
                 ['{:.1f}'.format(value) for value in (extent_mm or [])],
                 report['success']))
    print('WROTE', args.output)


if __name__ == '__main__':
    main()
