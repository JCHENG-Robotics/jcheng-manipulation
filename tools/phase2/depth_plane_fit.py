#!/usr/bin/env python3
"""Fit the table plane from the registered depth point cloud.

Runs in the Humble container (it needs ``numpy``, ``tf2_ros`` and the
``armpi_grasp_planner`` FK helper). The camera pose is composed from the
**measured bus-servo joints** and the live static
``link4 -> depth_camera_link`` transform, not from the cached
``/joint_states`` command echo.

The tool is read-only: it subscribes to the point cloud and TF, fits one
dominant plane with RANSAC + SVD, transforms it to ``base_link`` and writes a
JSON report. It never publishes a servo command.

Frame convention: the Aurora driver labels
``/depth_cam/depth_registered/points`` with
``frame_id: depth_camera_link``, but the point layout is the OpenCV
**optical** frame (x right, y down, z forward), the same convention as the RGB
PnP. Use ``--convention optical`` (default) with the validated hand-eye
``T_link4_camera``; ``--convention body`` is kept for a driver whose cloud is
really in the body frame. The frame mismatch is recorded in the report.

Example (inside the Humble container)::

    python3 /repo_tools/phase2/depth_plane_fit.py \
        --output /tmp/depth_plane_20261118.json \
        --joints-pulses 497,565,774,122,493

The JSON contains the plane in the cloud frame and in ``base_link``, the
composed camera transform, inlier/RMS statistics, the table height at chosen
``base_link`` reference points, and a downsampled point sample for offline
recomputation.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os
import sys

import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from tf2_ros import Buffer, TransformListener

PULSE_SCALE = 0.00418879
SERVO_IDS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5')
SERVO_ID_ORDER = (6, 5, 4, 3, 2)


def utc_now():
    """Return an ISO-8601 UTC timestamp."""
    return datetime.now(timezone.utc).isoformat()


def pulses_to_radians(pulses):
    """Convert joint1..joint5 bus-servo pulses to radians.

    The vendor mapping is ``(pulse - 500) * 0.00418879`` for joints 1, 2, 4
    and 5; joint3 is inverted (``(500 - pulse) * 0.00418879``).
    """
    values = []
    for index, pulse in enumerate(pulses):
        if index == 2:
            values.append((500.0 - float(pulse)) * PULSE_SCALE)
        else:
            values.append((float(pulse) - 500.0) * PULSE_SCALE)
    return values


def transform_to_matrix(transform):
    """Convert a tf2 TransformStamped to a 4x4 matrix."""
    t = transform.transform.translation
    q = transform.transform.rotation
    x, y, z, w = q.x, q.y, q.z, q.w
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),
         2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z),
         2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w),
         1 - 2 * (x * x + y * y)],
    ])
    matrix = np.eye(4)
    matrix[:3, :3] = rotation
    matrix[:3, 3] = [t.x, t.y, t.z]
    return matrix


def plane_from_points(points):
    """Return ``(normal, offset, rms)`` for the SVD plane of the points."""
    centroid = points.mean(axis=0)
    centered = points - centroid
    _, _, vh = np.linalg.svd(centered, full_matrices=False)
    normal = vh[-1]
    offset = -float(normal @ centroid)
    distances = points @ normal + offset
    rms = float(np.sqrt(np.mean(distances ** 2)))
    return normal, offset, rms


def ransac_plane(points, threshold, iterations, rng):
    """Return ``(normal, offset, inlier_mask, inlier_count)`` for RANSAC."""
    if len(points) < 3:
        raise RuntimeError('not enough points for a plane fit')
    best = None
    for _ in range(iterations):
        sample = points[rng.choice(len(points), 3, replace=False)]
        normal = np.cross(sample[1] - sample[0], sample[2] - sample[0])
        norm = np.linalg.norm(normal)
        if norm < 1e-9:
            continue
        normal /= norm
        offset = -float(normal @ sample[0])
        distances = np.abs(points @ normal + offset)
        mask = distances < threshold
        count = int(mask.sum())
        if best is None or count > best[3]:
            best = (normal, offset, mask, count)
    if best is None:
        raise RuntimeError('RANSAC found no plane')
    return best


def cloud_to_array(message):
    """Convert a PointCloud2 to an Nx3 float array using its x/y/z fields."""
    fields = {field.name: field for field in message.fields}
    if not all(name in fields for name in ('x', 'y', 'z')):
        raise RuntimeError('point cloud has no x/y/z fields')
    point_step = int(message.point_step)
    data = np.frombuffer(message.data, dtype=np.uint8)
    count = len(data) // point_step
    data = data[:count * point_step].reshape(count, point_step)
    columns = []
    for name in ('x', 'y', 'z'):
        offset = int(fields[name].offset)
        columns.append(
            data[:, offset:offset + 4].copy().view(np.float32).ravel())
    return np.column_stack(columns).astype(np.float64)


def plane_z_at(normal, offset, x, y):
    """Return the plane z at ``(x, y)``; None when the plane is vertical."""
    if abs(normal[2]) < 1e-9:
        return None
    return -(normal[0] * x + normal[1] * y + offset) / normal[2]


class CloudCapture(Node):
    """Take one registered point cloud and the required transforms."""

    def __init__(self, topic):
        super().__init__('depth_plane_fit')
        self.topic = topic
        self.message = None
        self.create_subscription(
            PointCloud2, topic, self._callback, qos_profile_sensor_data)
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)

    def _callback(self, message):
        self.message = message

    def wait_for_cloud(self, timeout):
        """Wait for one message; return it or None."""
        deadline = self.get_clock().now().nanoseconds + int(timeout * 1e9)
        while self.message is None:
            if self.get_clock().now().nanoseconds > deadline:
                return None
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.message

    def lookup(self, target, source, stamp):
        """Look up a transform, falling back to the latest available."""
        try:
            return self.buffer.lookup_transform(
                target, source, stamp, timeout=Duration(seconds=1.0))
        except Exception:
            return self.buffer.lookup_transform(
                target, source, Time(),
                timeout=Duration(seconds=1.0))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--topic', default='/depth_cam/depth_registered/points')
    parser.add_argument('--output', required=True)
    parser.add_argument(
        '--hand-eye',
        default='',
        help='hand-eye JSON with hand_eye.methods.TSAI.T_link4_camera '
             '(required for --convention optical)')
    parser.add_argument(
        '--convention',
        choices=('optical', 'body'),
        default='optical',
        help='point layout convention; see the module docstring')
    parser.add_argument(
        '--joints-pulses',
        required=True,
        help='measured joint1..joint5 bus-servo pulses, comma separated')
    parser.add_argument('--min-range', type=float, default=0.15)
    parser.add_argument('--max-range', type=float, default=0.60)
    parser.add_argument('--inlier-threshold', type=float, default=0.004)
    parser.add_argument('--ransac-iterations', type=int, default=800)
    parser.add_argument('--ransac-points', type=int, default=40000)
    parser.add_argument('--sample-points', type=int, default=2000)
    parser.add_argument('--seed', type=int, default=0)
    args = parser.parse_args()

    pulses = [float(value) for value in args.joints_pulses.split(',')]
    if len(pulses) != 5:
        raise SystemExit('--joints-pulses needs five values')
    joints = pulses_to_radians(pulses)

    from armpi_grasp_planner.reachability import ArmKinematics

    kinematics = ArmKinematics(tip_link='link4')
    link4_frame = kinematics.forward(joints[:4])
    T_base_link4 = np.eye(4)
    T_base_link4[:3, :3] = np.array(
        [[link4_frame.M[i, j] for j in range(3)] for i in range(3)])
    T_base_link4[:3, 3] = [
        link4_frame.p.x(), link4_frame.p.y(), link4_frame.p.z()]

    if args.convention == 'optical':
        if not args.hand_eye:
            raise SystemExit(
                '--hand-eye is required with --convention optical')
        with open(args.hand_eye, 'r', encoding='utf-8') as handle:
            calibration = json.load(handle)
        T_link4_camera = np.array(
            calibration['hand_eye']['methods']['TSAI']['T_link4_camera'])
    else:
        T_link4_camera = None

    rclpy.init()
    node = None
    try:
        node = CloudCapture(args.topic)
        message = node.wait_for_cloud(15.0)
        if message is None:
            raise SystemExit(
                'no point cloud received on {}'.format(args.topic))
        frame = message.header.frame_id
        stamp = message.header.stamp
        transform = node.lookup('link4', frame, stamp)
        T_link4_cloud = transform_to_matrix(transform)
        if args.convention == 'optical':
            T_base_cloud = T_base_link4 @ T_link4_camera
            T_link4_cloud_used = T_link4_camera
        else:
            T_base_cloud = T_base_link4 @ T_link4_cloud
            T_link4_cloud_used = T_link4_cloud

        points = cloud_to_array(message)
        finite = np.isfinite(points).all(axis=1)
        points = points[finite]
        ranges = np.linalg.norm(points, axis=1)
        keep = (ranges >= args.min_range) & (ranges <= args.max_range)
        points = points[keep]
        ranges = ranges[keep]
        if len(points) < 1000:
            raise SystemExit('too few points after the range filter')

        rng = np.random.default_rng(args.seed)
        subset = points
        if len(subset) > args.ransac_points:
            subset = subset[
                rng.choice(len(subset), args.ransac_points, replace=False)]
        normal, offset, mask, count = ransac_plane(
            subset, args.inlier_threshold, args.ransac_iterations, rng)
        distances = np.abs(points @ normal + offset)
        inliers = distances < args.inlier_threshold
        normal, offset, rms = plane_from_points(points[inliers])

        # Transform the plane to base_link: n_b = R n_c,
        # d_b = d_c - n_b . t.
        rotation = T_base_cloud[:3, :3]
        translation = T_base_cloud[:3, 3]
        normal_base = rotation @ normal
        offset_base = offset - float(normal_base @ translation)
        if normal_base[2] < 0:
            normal_base = -normal_base
            offset_base = -offset_base
        angle = math.degrees(math.acos(
            max(-1.0, min(1.0, float(normal_base[2])))))

        camera_base = T_base_cloud[:3, 3]
        references = {
            'under_camera': {
                'x': float(camera_base[0]),
                'y': float(camera_base[1]),
            },
            'base_center': {'x': 0.34, 'y': 0.0},
        }
        for item in references.values():
            item['z'] = plane_z_at(
                normal_base, offset_base, item['x'], item['y'])

        sample = points
        if len(sample) > args.sample_points:
            sample = sample[
                rng.choice(len(sample), args.sample_points, replace=False)]
        sample_base = (rotation @ sample.T).T + translation

        report = {
            'schema_version': 1,
            'generated_at': utc_now(),
            'mode': 'depth_table_plane_fit',
            'cloud': {
                'topic': args.topic,
                'frame_id': frame,
                'width': int(message.width),
                'height': int(message.height),
                'point_step': int(message.point_step),
                'is_dense': bool(message.is_dense),
                'stamp_s': float(stamp.sec) + float(stamp.nanosec) * 1e-9,
            },
            'filter': {
                'min_range_m': args.min_range,
                'max_range_m': args.max_range,
                'inlier_threshold_m': args.inlier_threshold,
                'points_total': int(len(finite)),
                'points_in_range': int(len(points)),
            },
            'joints': {
                'source': 'measured_bus_servo',
                'pulses': pulses,
                'radians': joints,
                'fk_tip': 'link4',
            },
            'transform': {
                'base_frame': 'base_link',
                'cloud_frame': frame,
                'convention': args.convention,
                'hand_eye': args.hand_eye if args.convention == 'optical'
                else None,
                'frame_label_note': (
                    'driver label is depth_camera_link; the point layout is '
                    'the optical convention'
                    if args.convention == 'optical' else 'body frame'
                ),
                'T_base_link4': T_base_link4.tolist(),
                'T_link4_cloud_tf': T_link4_cloud.tolist(),
                'T_link4_cloud_used': T_link4_cloud_used.tolist(),
                'T_base_cloud': T_base_cloud.tolist(),
                'tf_available': True,
            },
            'plane_cloud_frame': {
                'normal': normal.tolist(),
                'offset': float(offset),
                'inliers': int(inliers.sum()),
                'ransac_inliers_subset': int(count),
                'rms_m': rms,
            },
            'plane_base': {
                'normal': normal_base.tolist(),
                'offset': float(offset_base),
                'normal_angle_to_z_deg': angle,
                'z_at': references,
            },
            'depth_stats': {
                'median_range_m': float(np.median(ranges)),
                'p10_range_m': float(np.percentile(ranges, 10)),
                'p90_range_m': float(np.percentile(ranges, 90)),
                'inlier_range_median_m': float(
                    np.median(ranges[inliers])),
            },
            'sample_points_base': sample_base.tolist(),
        }
        target = os.path.abspath(args.output)
        os.makedirs(os.path.dirname(target), exist_ok=True)
        with open(target, 'w', encoding='utf-8') as handle:
            json.dump(report, handle, indent=2)
            handle.write('\n')

        print('cloud {} {}x{} frame={}'.format(
            args.topic, message.width, message.height, frame))
        print('points in range: {}'.format(len(points)))
        print('plane inliers: {} rms={:.4f} m'.format(
            int(inliers.sum()), rms))
        print('plane base normal: [{}]'.format(
            ', '.join('{:.5f}'.format(v) for v in normal_base)))
        print('plane angle to +z: {:.2f} deg'.format(angle))
        print('table z under camera: {:.4f} m'.format(
            references['under_camera']['z']))
        print('table z at base_center: {:.4f} m'.format(
            references['base_center']['z']))
        print('depth median: {:.4f} m'.format(
            report['depth_stats']['median_range_m']))
        print('json_out={}'.format(target))
        sys.stdout.flush()
        os._exit(0)
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
