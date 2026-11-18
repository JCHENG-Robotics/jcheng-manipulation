"""Execute one inspected trajectory from a grasp_candidate_check report.

The plan-only search writes trajectories with ``--include-trajectory``. This
tool sends the exact inspected trajectory to ``/execute_trajectory``. It
refuses unless ``--execute --i-have-clear-workspace`` is passed, revalidates
the trajectory through ``/armpi_safety/validate_trajectory`` and refuses when
the first point no longer matches ``/joint_states`` (start-state drift).

Run inside the Humble container after sourcing the workspace::

    python3 tools/phase2/execute_plan_json.py /tmp/plan.json js_0016
    python3 tools/phase2/execute_plan_json.py /tmp/plan.json js_0016 \
        --execute --i-have-clear-workspace

Nothing is executed by default: the first form is a dry run that prints the
safety result and the start-state delta.
"""

import argparse
import json
import os

import rclpy
from builtin_interfaces.msg import Duration
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import RobotTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState


def _duration(seconds):
    whole = int(seconds)
    return Duration(sec=whole, nanosec=int(round((seconds - whole) * 1e9)))


class PlanExecutor(Node):
    def __init__(self):
        super().__init__('execute_plan_json')
        self.latest = None
        self.create_subscription(JointState, '/joint_states', self._cb, 10)
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory')

    def _cb(self, message):
        self.latest = message

    def trajectory(self, plan):
        message = RobotTrajectory()
        joint_trajectory = JointTrajectory()
        joint_trajectory.joint_names = list(plan['joint_names'])
        for point in plan['points']:
            item = JointTrajectoryPoint()
            item.positions = [float(v) for v in point['positions']]
            item.velocities = [float(v) for v in point.get('velocities', [])]
            item.accelerations = [
                float(v) for v in point.get('accelerations', [])]
            item.time_from_start = _duration(point['time_from_start_s'])
            joint_trajectory.points.append(item)
        message.joint_trajectory = joint_trajectory
        return message

    def validate(self, trajectory):
        from arm_interfaces.srv import ValidateTrajectory

        client = self.create_client(
            ValidateTrajectory, '/armpi_safety/validate_trajectory')
        if not client.wait_for_service(timeout_sec=5.0):
            return False, 'SAFETY_SERVICE_UNAVAILABLE', []
        request = ValidateTrajectory.Request()
        request.trajectory = trajectory
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        if not future.done() or future.result() is None:
            return False, 'SAFETY_SERVICE_TIMEOUT', []
        response = future.result()
        return (
            bool(response.approved),
            str(response.code),
            list(response.violations),
        )

    def execute(self, trajectory):
        if not self.execute_client.wait_for_server(timeout_sec=10.0):
            raise SystemExit('/execute_trajectory not available')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=15.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise SystemExit('execute goal rejected')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result, timeout_sec=60.0)
        wrapped = result.result()
        if wrapped is None:
            raise SystemExit('no execute result')
        print('execute_status=%d' % wrapped.status, flush=True)
        print('execute_error_code=%d' % wrapped.result.error_code.val,
              flush=True)
        return wrapped.result.error_code.val


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('report')
    parser.add_argument('candidate')
    parser.add_argument('--execute', action='store_true')
    parser.add_argument('--i-have-clear-workspace', action='store_true')
    args = parser.parse_args()
    if args.execute and not args.i_have_clear_workspace:
        raise SystemExit('--execute requires --i-have-clear-workspace')

    data = json.load(open(args.report))
    entry = None
    for candidate in data['candidates']:
        if candidate['candidate_id'] == args.candidate:
            entry = candidate
            break
    if entry is None:
        raise SystemExit('candidate %s not found' % args.candidate)
    plan = (entry.get('plan') or {}).get('trajectory')
    if not plan:
        raise SystemExit('candidate has no trajectory')

    rclpy.init()
    node = PlanExecutor()
    trajectory = node.trajectory(plan)
    print(
        'candidate=%s points=%d duration=%.3fs'
        % (
            args.candidate,
            len(plan['points']),
            plan['points'][-1]['time_from_start_s'],
        ),
        flush=True,
    )
    approved, code, violations = node.validate(trajectory)
    print('safety: approved=%s code=%s violations=%s' % (
        approved, code, violations), flush=True)
    if not approved:
        os._exit(1)

    # Start-state check against the live command state.
    deadline = 0
    while node.latest is None and deadline < 50:
        rclpy.spin_once(node, timeout_sec=0.1)
        deadline += 1
    if node.latest is None:
        raise SystemExit('/joint_states not received')
    current = dict(zip(node.latest.name, node.latest.position))
    first = plan['points'][0]['positions']
    names = plan['joint_names']
    deltas = [abs(first[i] - current.get(name, float('nan')))
              for i, name in enumerate(names)]
    print('start_state_max_delta=%.4f rad' % max(deltas), flush=True)
    if max(deltas) > 0.02:
        print('refusing: start state moved since planning', flush=True)
        os._exit(1)

    if not args.execute:
        print('plan-only: no execute goal sent', flush=True)
        os._exit(0)
    code = node.execute(trajectory)
    os._exit(0 if code == 1 else 1)


if __name__ == '__main__':
    main()
