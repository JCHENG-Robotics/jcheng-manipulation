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
import pathlib
import sys

_SHARED = pathlib.Path(__file__).resolve().parents[2] / "packages" / "jcheng_core"
if _SHARED.is_dir() and str(_SHARED) not in sys.path:
    sys.path.insert(0, str(_SHARED))

from jcheng_vision import (  # noqa: E402
    connected_components,
    describe_cluster,
    fit_slab,
    min_area_rectangle,
    select_cluster,
    top_view_gate,
)

__all__ = [
    "connected_components",
    "describe_cluster",
    "fit_slab",
    "min_area_rectangle",
    "select_cluster",
    "top_view_gate",
]










DEFAULT_MIN_TOP_EXTENT_M = 0.035








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
