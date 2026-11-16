#!/usr/bin/env python3
"""
Check one published target pose against an expected pose.

Used for hardware-free integration checks of the fiducial localizer. It
subscribes to a ``PoseStamped`` topic, waits for one message, and writes a JSON
comparison. It does not move the robot.
"""

import argparse
import json
import math
import sys
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node


def _normalised(values):
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1e-15:
        raise ValueError('cannot normalise a zero quaternion')
    return tuple(value / norm for value in values)


def _angle_deg(quaternion_a, quaternion_b):
    a = _normalised(quaternion_a)
    b = _normalised(quaternion_b)
    dot = abs(sum(x * y for x, y in zip(a, b)))
    dot = max(-1.0, min(1.0, dot))
    return math.degrees(2.0 * math.acos(dot))


class PoseCheck(Node):
    """Capture the first pose on a topic."""

    def __init__(self, topic):
        super().__init__('fiducial_pose_check')
        self.message = None
        self.subscription = self.create_subscription(
            PoseStamped, topic, self._callback, 10
        )

    def _callback(self, message):
        if self.message is None:
            self.message = message


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Compare one target pose with an expected pose.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('--topic', default='/target_pose')
    parser.add_argument(
        '--expected-position',
        nargs=3,
        type=float,
        default=[0.0, 0.0, 0.9],
        metavar=('X', 'Y', 'Z'),
    )
    parser.add_argument(
        '--expected-quaternion',
        nargs=4,
        type=float,
        default=[1.0, 0.0, 0.0, 0.0],
        metavar=('X', 'Y', 'Z', 'W'),
    )
    parser.add_argument('--position-tolerance-mm', type=float, default=2.0)
    parser.add_argument('--angle-tolerance-deg', type=float, default=1.0)
    parser.add_argument('--timeout', type=float, default=10.0)
    parser.add_argument('--json-out', default=None)
    return parser.parse_args(argv)


def main(argv=None):
    """Run the pose comparison. Returns a process exit code."""
    args = _parse_args(argv)
    rclpy.init(args=[])
    node = PoseCheck(args.topic)
    summary = {
        'schema_version': 1,
        'topic': args.topic,
        'expected_position': args.expected_position,
        'expected_orientation': args.expected_quaternion,
        'success': False,
    }
    try:
        deadline = time.monotonic() + args.timeout
        while (
            node.message is None
            and time.monotonic() < deadline
            and rclpy.ok()
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
        if node.message is None:
            summary['error'] = 'no PoseStamped received'
            print('POSE_CHECK FAIL: no PoseStamped received')
        else:
            message = node.message
            position = (
                message.pose.position.x,
                message.pose.position.y,
                message.pose.position.z,
            )
            orientation = (
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            )
            position_error_mm = 1000.0 * math.sqrt(
                sum(
                    (position[index] - args.expected_position[index]) ** 2
                    for index in range(3)
                )
            )
            angle_error = _angle_deg(
                orientation, args.expected_quaternion
            )
            success = (
                position_error_mm <= args.position_tolerance_mm
                and angle_error <= args.angle_tolerance_deg
            )
            summary.update(
                {
                    'frame_id': message.header.frame_id,
                    'stamp': {
                        'sec': message.header.stamp.sec,
                        'nanosec': message.header.stamp.nanosec,
                    },
                    'received_position': list(position),
                    'received_orientation': list(orientation),
                    'position_error_mm': position_error_mm,
                    'angle_error_deg': angle_error,
                    'success': success,
                }
            )
            print(
                'POSE_CHECK frame={} dt={:.3f}mm dr={:.3f}deg '
                'RESULT {}'.format(
                    message.header.frame_id,
                    position_error_mm,
                    angle_error,
                    'PASS' if success else 'FAIL',
                )
            )
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as handle:
            json.dump(summary, handle, indent=2)
    return 0 if summary['success'] else 1


if __name__ == '__main__':
    sys.exit(main())
