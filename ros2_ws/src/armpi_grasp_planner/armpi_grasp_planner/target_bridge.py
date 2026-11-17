"""
Fiducial target -> grasp search bridge.

One-shot ROS 2 node:

    target_pose (PoseStamped)
    -> TF to base_link
    -> optional marker-frame grasp offset
    -> grasp_candidate_check (joint sampler, MoveIt plan-only, safety veto)

Nothing is executed. The bridge exists so the grasp search consumes a live
perception topic instead of a hand-typed point, and so the frame/age/orientation
contract is checked explicitly.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os
import sys
import time

from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
import tf2_geometry_msgs  # noqa: F401 - registers PoseStamped for TF2
from tf2_ros import Buffer, TransformListener

from .cli import main as search_main
from .geometry import clamp, matrix_from_quaternion


def stamp_seconds(stamp):
    """Return a ROS timestamp as float seconds."""
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def pose_age_s(now_s, stamp_s):
    """Return the non-negative age of a pose stamp."""
    if stamp_s <= 0.0:
        return 0.0
    return max(0.0, float(now_s) - float(stamp_s))


def marker_tilt_deg(quaternion):
    """Return the marker +z tilt from base_link +z, in degrees."""
    rotation = matrix_from_quaternion(quaternion)
    return math.degrees(math.acos(clamp(rotation[2][2], -1.0, 1.0)))


def grasp_point_from_pose(pose, marker_offset_m):
    """
    Return the grasp point in the pose frame plus a marker-frame offset.

    The offset is applied in the marker frame, so ``z`` is along the marker
    normal. A zero offset means the marker origin is the grasp point.
    """
    rotation = matrix_from_quaternion(
        (
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
    )
    offset = tuple(float(value) for value in marker_offset_m)
    return tuple(
        float(getattr(pose.position, axis))
        + sum(
            rotation[index][column] * offset[column]
            for column in range(3)
        )
        for index, axis in enumerate(('x', 'y', 'z'))
    )


class TargetCaptureNode(Node):
    """Capture one valid fiducial target pose in the output frame."""

    def __init__(
        self,
        target_topic,
        output_frame,
        max_age_s,
        tf_timeout_s,
        marker_offset_m,
        require_marker_up,
        max_marker_tilt_deg,
    ):
        """Create the subscription and TF listener."""
        super().__init__('armpi_grasp_target_bridge')
        self.target_topic = target_topic
        self.output_frame = output_frame
        self.max_age_s = float(max_age_s)
        self.tf_timeout_s = float(tf_timeout_s)
        self.marker_offset_m = tuple(marker_offset_m)
        self.require_marker_up = bool(require_marker_up)
        self.max_marker_tilt_deg = float(max_marker_tilt_deg)
        self.capture = None
        self.rejections = []
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.subscription = self.create_subscription(
            PoseStamped, target_topic, self.on_pose, 10
        )
        self.get_logger().info(
            'waiting for {} -> {} (max_age={:.2f}s, offset={})'.format(
                target_topic,
                output_frame or '(same frame)',
                self.max_age_s,
                self.marker_offset_m,
            )
        )

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _reject(self, reason, detail):
        self.rejections.append(
            {'reason': reason, 'detail': str(detail), 'at_s': self._now()}
        )
        self.get_logger().warn(
            'target rejected ({}): {}'.format(reason, detail),
            throttle_duration_sec=2.0,
        )

    def on_pose(self, message):
        """Validate and store the first usable target pose."""
        if self.capture is not None:
            return
        stamp = stamp_seconds(message.header.stamp)
        age = pose_age_s(self._now(), stamp)
        if self.max_age_s > 0.0 and age > self.max_age_s:
            self._reject(
                'stale',
                'pose age {:.3f}s > {:.3f}s'.format(
                    age, self.max_age_s
                ),
            )
            return
        pose = message
        if (
            self.output_frame
            and message.header.frame_id
            and message.header.frame_id != self.output_frame
        ):
            try:
                pose = self.tf_buffer.transform(
                    message,
                    self.output_frame,
                    timeout=Duration(seconds=self.tf_timeout_s),
                )
            except Exception as exc:  # noqa: BLE001 - report TF failure
                self._reject(
                    'tf',
                    '{} -> {} failed: {}'.format(
                        message.header.frame_id,
                        self.output_frame,
                        exc,
                    ),
                )
                return
        quaternion = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )
        norm = math.sqrt(sum(value * value for value in quaternion))
        if abs(norm - 1.0) > 1e-3:
            self._reject(
                'orientation',
                'quaternion is not normalised (norm={:.6f})'.format(norm),
            )
            return
        tilt = marker_tilt_deg(quaternion)
        if self.require_marker_up and tilt > self.max_marker_tilt_deg:
            self._reject(
                'tilt',
                'marker +z is {:.2f} deg from base +z > {:.2f} deg'.format(
                    tilt, self.max_marker_tilt_deg
                ),
            )
            return
        point = grasp_point_from_pose(pose.pose, self.marker_offset_m)
        self.capture = {
            'source_frame': message.header.frame_id,
            'output_frame': pose.header.frame_id or self.output_frame,
            'pose_age_s': age,
            'marker_tilt_deg': tilt,
            'marker_position_m': [
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ],
            'marker_quaternion_xyzw': list(quaternion),
            'marker_offset_m': list(self.marker_offset_m),
            'grasp_point_m': list(point),
            'stamp_s': stamp,
        }
        self.get_logger().info(
            'target captured in {}: point=({:.4f}, {:.4f}, {:.4f}) '
            'age={:.3f}s tilt={:.2f}deg'.format(
                self.capture['output_frame'],
                point[0],
                point[1],
                point[2],
                age,
                tilt,
            )
        )

    def wait_for_target(self, timeout_s):
        """Spin until a target is captured or the timeout expires."""
        deadline = time.monotonic() + float(timeout_s)
        while rclpy.ok() and self.capture is None:
            if time.monotonic() >= deadline:
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.capture


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            'Capture a fiducial target and run the 5-DoF grasp search.'
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('--target-topic', default='target_pose')
    parser.add_argument(
        '--output-frame',
        default='base_link',
        help='frame consumed by the grasp search; empty keeps the input frame',
    )
    parser.add_argument(
        '--max-age-s',
        type=float,
        default=0.5,
        help='reject poses older than this; 0 disables the age check',
    )
    parser.add_argument('--tf-timeout-s', type=float, default=0.2)
    parser.add_argument(
        '--capture-timeout-s',
        type=float,
        default=15.0,
        help='give up when no usable target arrives',
    )
    parser.add_argument(
        '--marker-offset',
        nargs=3,
        type=float,
        default=(0.0, 0.0, 0.0),
        metavar=('X', 'Y', 'Z'),
        help='grasp-point offset in the marker frame, metres',
    )
    parser.add_argument(
        '--max-marker-tilt-deg',
        type=float,
        default=45.0,
        help='reject markers whose +z is tilted more than this from +z',
    )
    parser.add_argument(
        '--allow-tilted-marker',
        action='store_true',
        help='disable the marker-up check',
    )
    parser.add_argument('--max-plan', type=int, default=3)
    parser.add_argument(
        '--sampler', choices=('joint',), default='joint'
    )
    parser.add_argument('--max-sampled', type=int, default=80)
    parser.add_argument('--sampling-radius', type=float, default=0.02)
    parser.add_argument('--position-step', type=float, default=0.2)
    parser.add_argument('--roll-step', type=float, default=0.3)
    parser.add_argument('--no-refined', action='store_true')
    parser.add_argument(
        '--safety-service',
        default='',
        help='arm_interfaces/ValidateTrajectory service to veto the plan',
    )
    parser.add_argument(
        '--max-start-state-distance', type=float, default=0.0
    )
    parser.add_argument('--include-trajectory', action='store_true')
    parser.add_argument('--service-timeout', type=float, default=10.0)
    parser.add_argument('--planning-timeout', type=float, default=10.0)
    parser.add_argument(
        '--json-out',
        default=None,
        help='combined bridge report; the raw search JSON is written beside it',
    )
    parser.add_argument(
        '--search-json-out',
        default=None,
        help='raw grasp_candidate_check report path',
    )
    parser.add_argument('--quiet', action='store_true')
    return parser.parse_args(argv)


def _search_argv(args, search_json):
    argv = [
        '--target-point',
        str(args.grasp_point[0]),
        str(args.grasp_point[1]),
        str(args.grasp_point[2]),
        '--sampler',
        args.sampler,
        '--max-plan',
        str(args.max_plan),
        '--max-sampled',
        str(args.max_sampled),
        '--sampling-radius',
        str(args.sampling_radius),
        '--position-step',
        str(args.position_step),
        '--roll-step',
        str(args.roll_step),
        '--service-timeout',
        str(args.service_timeout),
        '--planning-timeout',
        str(args.planning_timeout),
        '--json-out',
        search_json,
    ]
    if args.no_refined:
        argv.append('--no-refined')
    if args.safety_service:
        argv.extend(['--safety-service', args.safety_service])
    if args.max_start_state_distance > 0.0:
        argv.extend(
            [
                '--max-start-state-distance',
                str(args.max_start_state_distance),
            ]
        )
    if args.include_trajectory:
        argv.append('--include-trajectory')
    if args.quiet:
        argv.append('--quiet')
    return argv


def _trim_search(summary):
    trimmed = dict(summary)
    trimmed.pop('candidates', None)
    return trimmed


def main(argv=None):
    """Run the one-shot bridge. Returns a process exit code."""
    args = _parse_args(argv)
    if args.json_out is None and args.search_json_out is None:
        args.search_json_out = (
            '/tmp/armpi_grasp_target_bridge_search.json'
        )
    if args.search_json_out is None:
        base, extension = os.path.splitext(args.json_out)
        args.search_json_out = base + '.search' + (extension or '.json')

    report = {
        'schema_version': 1,
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'success': False,
        'bridge': {
            'target_topic': args.target_topic,
            'output_frame': args.output_frame,
            'capture_timeout_s': args.capture_timeout_s,
            'max_age_s': args.max_age_s,
            'marker_offset_m': list(args.marker_offset),
        },
    }
    rclpy.init(args=[])
    node = None
    try:
        node = TargetCaptureNode(
            target_topic=args.target_topic,
            output_frame=args.output_frame,
            max_age_s=args.max_age_s,
            tf_timeout_s=args.tf_timeout_s,
            marker_offset_m=args.marker_offset,
            require_marker_up=not args.allow_tilted_marker,
            max_marker_tilt_deg=args.max_marker_tilt_deg,
        )
        capture = node.wait_for_target(args.capture_timeout_s)
        report['bridge']['capture'] = capture
        report['bridge']['rejections'] = list(node.rejections)
        if capture is None:
            report['error'] = 'no usable target within {:.1f}s'.format(
                args.capture_timeout_s
            )
            print(
                'BRIDGE_RESULT FAIL: {}'.format(report['error']),
                file=sys.stderr,
            )
            return 2
        args.grasp_point = tuple(capture['grasp_point_m'])
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    search_argv = _search_argv(args, args.search_json_out)
    print(
        'BRIDGE_TARGET frame={} point=({:.4f}, {:.4f}, {:.4f}) '
        'age={:.3f}s tilt={:.2f}deg'.format(
            capture['output_frame'],
            args.grasp_point[0],
            args.grasp_point[1],
            args.grasp_point[2],
            capture['pose_age_s'],
            capture['marker_tilt_deg'],
        )
    )
    search_status = search_main(search_argv)
    report['search_json'] = args.search_json_out
    report['search_exit_code'] = search_status
    report['success'] = search_status == 0
    try:
        with open(args.search_json_out, 'r', encoding='utf-8') as handle:
            search_summary = json.load(handle)
        report['search_summary'] = _trim_search(search_summary)
    except (OSError, ValueError) as exc:
        report['search_summary_error'] = str(exc)
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as handle:
            json.dump(report, handle, indent=2)
    print(
        'BRIDGE_RESULT {}'.format(
            'PASS' if report['success'] else 'FAIL'
        )
    )
    return 0 if report['success'] else 1


if __name__ == '__main__':
    sys.exit(main())
