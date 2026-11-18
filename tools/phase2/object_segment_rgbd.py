#!/usr/bin/env python3
"""
Markerless RGB-D object segmentation for a floor/table grasp.

Pipeline (all in ``base_link``, no fiducials):

1. capture one registered point cloud and the measured bus-servo pulses;
2. transform the cloud with the measured hand-eye (optical convention);
3. RANSAC the support plane and remove it;
4. voxel-grid Euclidean connected components on the remaining points;
5. select the cluster that passes a size/height gate near the expected spot;
6. take the highest slab (the top face) and fit it, giving the grasp point.

The tool only subscribes; it never moves the robot. It runs inside the Humble
container (numpy + PyKDL + the description package), like
``depth_plane_fit.py``:

    source /opt/ros/humble/setup.bash
    source /workspace/install/setup.bash
    python3 /repo_tools/phase2/object_segment_rgbd.py \
        --hand-eye evidence/calibration/20261118/table_down/calibration.json \
        --joints-pulses 498,569,772,131,494 \
        --output /tmp/object_segment.json \
        --cloud-out /tmp/grasp_cloud.npz
"""

import argparse
import json
import math
import os

import numpy as np


def voxel_labels(points, pitch):
    """Return per-point voxel coordinates as an (N, 3) int array."""
    return np.floor(points / pitch).astype(np.int64)


def connected_components(points, pitch, min_points=20):
    """
    Euclidean components on a voxel grid with 26-neighbour connectivity.

    Returns ``(labels, clusters)`` where ``labels`` is -1 for ignored points
    and ``clusters`` is a list of ``{'label', 'indices', 'points', ...}``.
    Two points closer than one voxel are connected; this is the standard
    voxel-grid approximation of Euclidean clustering and is deterministic.
    """
    if len(points) == 0:
        return np.full(0, -1, dtype=np.int64), []
    voxels = voxel_labels(points, pitch)
    keyed = {}
    for index, voxel in enumerate(map(tuple, voxels)):
        keyed.setdefault(voxel, []).append(index)

    # union-find over occupied voxels
    parent = {voxel: voxel for voxel in keyed}

    def find(item):
        while parent[item] != item:
            parent[item] = parent[parent[item]]
            item = parent[item]
        return item

    def union(a, b):
        root_a, root_b = find(a), find(b)
        if root_a != root_b:
            parent[root_b] = root_a

    neighbours = [
        (dx, dy, dz)
        for dx in (-1, 0, 1)
        for dy in (-1, 0, 1)
        for dz in (-1, 0, 1)
        if (dx, dy, dz) != (0, 0, 0)
    ]
    for voxel in keyed:
        for delta in neighbours:
            other = (voxel[0] + delta[0], voxel[1] + delta[1],
                     voxel[2] + delta[2])
            if other in keyed:
                union(voxel, other)

    groups = {}
    for voxel in keyed:
        groups.setdefault(find(voxel), []).extend(keyed[voxel])

    clusters = []
    labels = np.full(len(points), -1, dtype=np.int64)
    for number, (_, indices) in enumerate(
        sorted(groups.items(), key=lambda item: -len(item[1]))
    ):
        if len(indices) < min_points:
            continue
        index_array = np.array(sorted(indices), dtype=np.int64)
        clusters.append({
            'label': number,
            'indices': index_array,
            'points': points[index_array],
        })
        labels[index_array] = number
    return labels, clusters


def min_area_rectangle(xy, step_deg=1.0):
    """
    Return the minimum-area axis-aligned box over yaw angles.

    Returns ``(yaw_rad, width_m, depth_m, area_m2)``; ``width`` is along the
    yaw direction.  Pure numpy so the container needs no OpenCV.
    """
    if len(xy) == 0:
        return 0.0, 0.0, 0.0, 0.0
    angles = np.deg2rad(np.arange(0.0, 90.0 + step_deg, step_deg))
    cosine = np.cos(angles)[:, None]
    sine = np.sin(angles)[:, None]
    rotated_x = xy[None, :, 0] * cosine - xy[None, :, 1] * sine
    rotated_y = xy[None, :, 0] * sine + xy[None, :, 1] * cosine
    widths = rotated_x.max(axis=1) - rotated_x.min(axis=1)
    depths = rotated_y.max(axis=1) - rotated_y.min(axis=1)
    areas = widths * depths
    best = int(np.argmin(areas))
    return (float(angles[best]), float(widths[best]), float(depths[best]),
            float(areas[best]))


def fit_slab(points, slab_m=0.005):
    """
    Fit the highest slab of a cluster.

    The top face is the set of points within ``slab_m`` of the cluster's
    maximum height.  Returns the centroid, the plane tilt and the extents.
    """
    if len(points) == 0:
        raise ValueError('empty cluster')
    z_max = float(points[:, 2].max())
    slab = points[points[:, 2] >= z_max - slab_m]
    centroid = slab.mean(axis=0)
    # SVD plane normal; report the angle to base_link +z.
    centred = slab - centroid
    _, _, vh = np.linalg.svd(centred, full_matrices=False)
    normal = vh[-1]
    if normal[2] < 0:
        normal = -normal
    tilt_deg = math.degrees(math.acos(max(-1.0, min(1.0, normal[2]))))
    yaw, width, depth, area = min_area_rectangle(slab[:, :2])
    return {
        'slab_points': int(len(slab)),
        'slab_thickness_m': slab_m,
        'centroid': [float(value) for value in centroid],
        'normal': [float(value) for value in normal],
        'tilt_deg': tilt_deg,
        'yaw_rad': yaw,
        'extent_m': [width, depth],
        'area_m2': area,
        'z_max': z_max,
    }


DEFAULT_MIN_TOP_EXTENT_M = 0.035


def top_view_gate(top, min_extent_m):
    """
    Return ``(ok, reason, min_extent)`` for a fitted top face.

    A top face clipped by the image border has a minimum-area extent below the
    object's real size, and its slab centroid is biased toward the visible
    part. This caused the 2026-11-18 first-grasp miss: the last pre-grasp
    capture had 37.8 x 31.9 mm instead of 39.4 x 39.4 mm and the plan came out
    several millimetres off. Refuse to output a grasp point from such a view.
    """
    extent = min(float(value) for value in top['extent_m'])
    if extent < min_extent_m:
        return False, 'partial_top_view', extent
    return True, None, extent


def describe_cluster(points):
    """Return bounds and height statistics for a cluster."""
    minimum = points.min(axis=0)
    maximum = points.max(axis=0)
    return {
        'points': int(len(points)),
        'min': [float(value) for value in minimum],
        'max': [float(value) for value in maximum],
        'size': [float(value) for value in (maximum - minimum)],
        'centroid': [float(value) for value in points.mean(axis=0)],
    }


def select_cluster(clusters, floor_z, gates):
    """
    Pick the cluster that looks like the expected object.

    ``gates`` holds ``x_range``, ``y_range``, ``max_xy_m``, ``min_height_m``,
    ``max_height_m`` and ``min_points``.  Returns ``(cluster, reasons)``; the
    cluster is ``None`` when every candidate is rejected.
    """
    reasons = []
    best = None
    for cluster in clusters:
        points = cluster['points']
        bounds = describe_cluster(points)
        size = bounds['size']
        centroid = bounds['centroid']
        height = bounds['max'][2] - floor_z
        ok = True
        why = []
        if len(points) < gates['min_points']:
            ok = False
            why.append('too_few_points')
        if max(size[0], size[1]) > gates['max_xy_m']:
            ok = False
            why.append('too_wide')
        if not gates['min_height_m'] <= height <= gates['max_height_m']:
            ok = False
            why.append('height_%.3f' % height)
        if not gates['x_range'][0] <= centroid[0] <= gates['x_range'][1]:
            ok = False
            why.append('x_out_of_region')
        if abs(centroid[1]) > gates['y_range']:
            ok = False
            why.append('y_out_of_region')
        reasons.append({'label': cluster['label'], 'bounds': bounds,
                        'height_m': float(height), 'ok': ok,
                        'why': why})
        if ok and (best is None or len(points) > len(best['points'])):
            best = cluster
    return best, reasons


def main(argv=None):
    """Capture, segment and write a JSON report. Returns an exit code."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--topic', default='/depth_cam/depth_registered/points')
    parser.add_argument('--output', required=True)
    parser.add_argument(
        '--hand-eye',
        required=True,
        help='calibration JSON with hand_eye.methods.TSAI.T_link4_camera')
    parser.add_argument(
        '--joints-pulses', required=True,
        help='measured joint1..joint5 bus-servo pulses, comma separated')
    parser.add_argument('--min-range', type=float, default=0.15)
    parser.add_argument('--max-range', type=float, default=0.60)
    parser.add_argument('--floor-threshold', type=float, default=0.004)
    parser.add_argument('--floor-margin', type=float, default=0.006,
                        help='extra distance above the plane removed as floor')
    parser.add_argument('--voxel', type=float, default=0.005)
    parser.add_argument('--min-cluster-points', type=int, default=30)
    parser.add_argument('--slab', type=float, default=0.005)
    parser.add_argument(
        '--min-top-extent', type=float, default=DEFAULT_MIN_TOP_EXTENT_M,
        help='reject a top face whose minimum-area extent is below this '
             'length (a clipped view biases the centroid)')
    parser.add_argument('--x-range', default='0.15,0.32')
    parser.add_argument('--y-range', type=float, default=0.09)
    parser.add_argument('--max-xy', type=float, default=0.09)
    parser.add_argument('--min-height', type=float, default=0.020)
    parser.add_argument('--max-height', type=float, default=0.070)
    parser.add_argument('--ransac-iterations', type=int, default=800)
    parser.add_argument('--ransac-points', type=int, default=40000)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--cloud-out', default=None)
    args = parser.parse_args(argv)

    pulses = [float(value) for value in args.joints_pulses.split(',')]
    if len(pulses) != 5:
        raise SystemExit('--joints-pulses needs five values')
    import depth_plane_fit as plane_tools

    joints = plane_tools.pulses_to_radians(pulses)

    from armpi_grasp_planner.reachability import ArmKinematics

    kinematics = ArmKinematics(tip_link='link4')
    frame = kinematics.forward(joints[:4])
    T_base_link4 = np.eye(4)
    T_base_link4[:3, :3] = np.array(
        [[frame.M[i, j] for j in range(3)] for i in range(3)])
    T_base_link4[:3, 3] = [frame.p.x(), frame.p.y(), frame.p.z()]

    with open(args.hand_eye, 'r', encoding='utf-8') as handle:
        calibration = json.load(handle)
    T_link4_camera = np.array(
        calibration['hand_eye']['methods']['TSAI']['T_link4_camera'])
    T_base_cloud = T_base_link4 @ T_link4_camera

    import rclpy

    rclpy.init()
    node = None
    try:
        node = plane_tools.CloudCapture(args.topic)
        message = node.wait_for_cloud(15.0)
        if message is None:
            raise SystemExit(
                'no point cloud received on {}'.format(args.topic))
        points = plane_tools.cloud_to_array(message)
        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        ranges = np.linalg.norm(points, axis=1)
        keep = (ranges >= args.min_range) & (ranges <= args.max_range)
        points = points[keep]

        rng = np.random.default_rng(args.seed)
        subset = points
        if len(subset) > args.ransac_points:
            subset = subset[
                rng.choice(len(subset), args.ransac_points, replace=False)]
        normal, offset, _, _ = plane_tools.ransac_plane(
            subset, args.floor_threshold, args.ransac_iterations, rng)
        distances = points @ normal + offset
        inliers = np.abs(distances) < args.floor_threshold
        normal, offset, rms = plane_tools.plane_from_points(points[inliers])

        rotation = T_base_cloud[:3, :3]
        translation = T_base_cloud[:3, 3]
        normal_base = rotation @ normal
        offset_base = offset - float(normal_base @ translation)
        if normal_base[2] < 0:
            normal_base = -normal_base
            offset_base = -offset_base

        points_base = (rotation @ points.T).T + translation
        signed = points_base @ normal_base + offset_base
        # the object sits above the plane; remove the floor and anything below
        above = signed > args.floor_margin
        candidates = points_base[above]

        labels, clusters = connected_components(
            candidates, args.voxel, args.min_cluster_points)
        floor_z = plane_tools.plane_z_at(
            normal_base, offset_base, 0.24, 0.0)
        gates = {
            'min_points': args.min_cluster_points,
            'max_xy_m': args.max_xy,
            'min_height_m': args.min_height,
            'max_height_m': args.max_height,
            'x_range': tuple(float(v) for v in args.x_range.split(',')),
            'y_range': args.y_range,
        }
        selected, reasons = select_cluster(clusters, floor_z, gates)

        report = {
            'schema_version': 1,
            'mode': 'markerless_rgbd_segment',
            'cloud': {
                'topic': args.topic,
                'frame_id': message.header.frame_id,
                'point_step': int(message.point_step),
                'points_total': int(len(finite)),
                'points_in_range': int(len(points)),
            },
            'joints': {
                'source': 'measured_bus_servo',
                'pulses': pulses,
                'radians': joints,
                'fk_tip': 'link4',
            },
            'hand_eye': {
                'calibration': args.hand_eye,
                'method': 'TSAI',
                'T_base_cloud': T_base_cloud.tolist(),
            },
            'floor': {
                'normal_base': [float(v) for v in normal_base],
                'offset_base': float(offset_base),
                'rms_m': float(rms),
                'inliers': int(inliers.sum()),
                'z_at_0p24': float(floor_z),
                'angle_to_z_deg': math.degrees(math.acos(
                    max(-1.0, min(1.0, float(normal_base[2]))))),
            },
            'segmentation': {
                'voxel_m': args.voxel,
                'points_above_floor': int(len(candidates)),
                'clusters': len(clusters),
                'cluster_details': reasons,
            },
        }
        if selected is not None:
            top = fit_slab(selected['points'], args.slab)
            target = top['centroid']
            view_ok, view_reason, view_extent = top_view_gate(
                top, args.min_top_extent)
            report['selected'] = {
                'points': int(len(selected['points'])),
                'bounds': describe_cluster(selected['points']),
                'top': top,
                'height_above_floor_m': float(
                    target[2] - plane_tools.plane_z_at(
                        normal_base, offset_base, target[0], target[1])),
            }
            if view_ok:
                report['selected']['grasp_point_base'] = [
                    float(value) for value in target]
            else:
                # A clipped top face biases the slab centroid toward the
                # visible part; the point is reported for diagnosis only.
                report['selected']['rejected_point_base'] = [
                    float(value) for value in target]
                report['rejection'] = {
                    'reason': view_reason,
                    'min_extent_m': view_extent,
                    'threshold_m': args.min_top_extent,
                }
            if args.cloud_out:
                np.savez_compressed(
                    args.cloud_out,
                    points=selected['points'].astype(np.float32),
                    all_points=points_base.astype(np.float32))
            if not view_ok:
                selected = None
        report['success'] = selected is not None
        os.makedirs(os.path.dirname(os.path.abspath(args.output)) or '.',
                    exist_ok=True)
        with open(args.output, 'w', encoding='utf-8') as handle:
            json.dump(report, handle, indent=2)
        print('selected=%s clusters=%d target=%s' % (
            selected is not None, len(clusters),
            report.get('selected', {}).get('grasp_point_base')))
        return 0 if selected is not None else 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
