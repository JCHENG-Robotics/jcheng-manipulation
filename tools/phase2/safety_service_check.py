#!/usr/bin/env python3
"""
Exercise the safety service with a valid plan and three invalid variants.

The bridge report must contain the full planned trajectory
(``search_summary.best.plan.trajectory``), which is why the E2E runner passes
``--include-trajectory``. This tool never executes a trajectory.
"""

import argparse
import copy
import json
import sys

from arm_interfaces.srv import ValidateTrajectory
from builtin_interfaces.msg import Duration
from moveit_msgs.msg import RobotTrajectory
import rclpy
from trajectory_msgs.msg import JointTrajectoryPoint


def trajectory_message(data):
    """Build a RobotTrajectory message from the bridge JSON schema."""
    message = RobotTrajectory()
    trajectory = message.joint_trajectory
    trajectory.joint_names = list(data['joint_names'])
    for point in data['points']:
        message_point = JointTrajectoryPoint()
        message_point.positions = [
            float(value) for value in point['positions']
        ]
        message_point.velocities = [
            float(value) for value in point.get('velocities', [])
        ]
        message_point.accelerations = [
            float(value) for value in point.get('accelerations', [])
        ]
        seconds = float(point.get('time_from_start_s', 0.0))
        message_point.time_from_start = Duration(
            sec=int(seconds), nanosec=int((seconds % 1.0) * 1e9)
        )
        trajectory.points.append(message_point)
    return message


def load_trajectory(path):
    """Read the best planned trajectory from a bridge or search report."""
    with open(path, 'r', encoding='utf-8') as handle:
        report = json.load(handle)
    if 'search_summary' in report:
        best = report['search_summary'].get('best') or {}
    else:
        best = report.get('best') or {}
    trajectory = (best.get('plan') or {}).get('trajectory')
    if trajectory is None:
        raise ValueError(
            '{} has no best.plan.trajectory; rerun with '
            '--include-trajectory'.format(path)
        )
    return trajectory


def mutate_joint_limit(trajectory):
    data = copy.deepcopy(trajectory)
    data['points'][0]['positions'][0] = 3.0
    return data


def mutate_velocity(trajectory):
    data = copy.deepcopy(trajectory)
    point = data['points'][1]
    if len(point.get('velocities', [])) != len(data['joint_names']):
        point['velocities'] = [0.0] * len(data['joint_names'])
    point['velocities'][0] = 5.0
    return data


def mutate_timestamp(trajectory):
    data = copy.deepcopy(trajectory)
    data['points'][1]['time_from_start_s'] = data['points'][0][
        'time_from_start_s'
    ]
    return data


def call_service(node, client, data, timeout_s):
    """Call the safety service and return its response."""
    request = ValidateTrajectory.Request()
    request.trajectory = trajectory_message(data)
    request.max_start_state_distance_rad = 0.0
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout_s)
    if not future.done() or future.result() is None:
        raise RuntimeError('safety service did not respond')
    return future.result()


def main(argv=None):
    """Run the four service cases. Returns a process exit code."""
    parser = argparse.ArgumentParser(
        description='Check the safety service with valid and invalid plans.'
    )
    parser.add_argument('--input', required=True)
    parser.add_argument(
        '--service', default='/armpi_safety/validate_trajectory'
    )
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parser.parse_args(argv)
    trajectory = load_trajectory(args.input)
    cases = (
        ('valid', trajectory, True, 'OK'),
        ('joint_limit', mutate_joint_limit(trajectory), False,
         'JOINT_LIMIT'),
        ('velocity', mutate_velocity(trajectory), False, 'VELOCITY_LIMIT'),
        ('timestamp', mutate_timestamp(trajectory), False, 'TIMESTAMP'),
    )
    rclpy.init(args=[])
    node = rclpy.create_node('safety_service_check')
    client = node.create_client(ValidateTrajectory, args.service)
    try:
        if not client.wait_for_service(timeout_sec=args.timeout):
            print(
                'SAFETY_CHECK FAIL: {} not available'.format(args.service),
                file=sys.stderr,
            )
            return 2
        failed = False
        for name, data, expect_approved, expect_code in cases:
            response = call_service(node, client, data, args.timeout)
            ok = (
                bool(response.approved) == expect_approved
                and response.code == expect_code
            )
            print(
                'SAFETY_CASE {} approved={} code={} expected=({}, {}) '
                'result={}'.format(
                    name,
                    response.approved,
                    response.code,
                    expect_approved,
                    expect_code,
                    'PASS' if ok else 'FAIL',
                )
            )
            failed = failed or not ok
        print('RESULT {}'.format('FAIL' if failed else 'PASS'))
        return 1 if failed else 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
