"""Plan and optionally execute an SRDF named state on the real robot.

The plan is produced by ``move_group``'s ``/plan_kinematic_path`` service at
the requested velocity/acceleration scaling. With ``--execute`` the exact
planned trajectory is sent to ``move_group``'s ``/execute_trajectory`` action;
there is no replanning between the inspection and the execution.

Execution is refused unless ``--i-have-clear-workspace`` is also passed, so a
mistyped command cannot move the arm by itself.

Run inside the Humble MoveIt container after sourcing the workspace:

    python3 real_moveit_execute.py camera_forward
    python3 real_moveit_execute.py camera_forward --execute \
        --i-have-clear-workspace
"""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan
from rclpy.action import ActionClient
from rclpy.node import Node


def load_group_state(name, srdf_path=None):
    """Return ``(group, [(joint, value), ...])`` for a named state."""
    if srdf_path is None:
        share = Path(get_package_share_directory('armpi_ultra_moveit_config'))
        srdf_path = share / 'config' / 'armpi_ultra.srdf'
    root = ET.parse(srdf_path).getroot()
    for state in root.findall('group_state'):
        if state.get('name') == name:
            return state.get('group'), [
                (joint.get('name'), float(joint.get('value')))
                for joint in state.findall('joint')
            ]
    raise SystemExit(f'named state {name!r} not found in {srdf_path}')


def print_trajectory(trajectory):
    """Print waypoint count and the first/last joint positions."""
    points = trajectory.joint_trajectory.points
    names = trajectory.joint_trajectory.joint_names
    print(f'waypoints={len(points)}')
    if not points:
        return
    duration = points[-1].time_from_start
    seconds = duration.sec + duration.nanosec / 1e9
    print(f'  duration={seconds:.3f}s')
    print('  start ' + ' '.join(
        f'{name}={value:.6f}'
        for name, value in zip(names, points[0].positions)))
    print('  final ' + ' '.join(
        f'{name}={value:.6f}'
        for name, value in zip(names, points[-1].positions)))


class NamedStatePlan(Node):
    """Plan one joint-constrained goal and optionally execute it."""

    def __init__(self, group, joints, tolerance, velocity, acceleration,
                 timeout):
        super().__init__('named_state_plan_execute')
        self.group = group
        self.joints = joints
        self.tolerance = tolerance
        self.velocity = velocity
        self.acceleration = acceleration
        self.timeout = timeout
        self.plan_client = self.create_client(
            GetMotionPlan, '/plan_kinematic_path')
        if not self.plan_client.wait_for_service(timeout_sec=15.0):
            raise SystemExit(
                '/plan_kinematic_path not available; is move_group up?')
        self.execute_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory')

    def plan(self):
        """Return the MotionPlanResponse, or None on failure."""
        request = GetMotionPlan.Request()
        plan_request = request.motion_plan_request
        plan_request.group_name = self.group
        plan_request.num_planning_attempts = 10
        plan_request.allowed_planning_time = self.timeout
        plan_request.max_velocity_scaling_factor = self.velocity
        plan_request.max_acceleration_scaling_factor = self.acceleration

        constraints = Constraints()
        for name, value in self.joints:
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = value
            joint.tolerance_above = self.tolerance
            joint.tolerance_below = self.tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        plan_request.goal_constraints.append(constraints)

        future = self.plan_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        response = future.result()
        if response is None:
            print('error: no response from /plan_kinematic_path')
            return None

        result = response.motion_plan_response
        print(f'error_code={result.error_code.val}')
        print(f'planning_time={result.planning_time:.3f}s')
        print_trajectory(result.trajectory)
        if result.error_code.val != 1:
            return None
        return result

    def execute(self, trajectory, result_timeout=300.0):
        """Send the planned trajectory to /execute_trajectory."""
        if not self.execute_client.wait_for_server(timeout_sec=15.0):
            raise SystemExit(
                '/execute_trajectory not available; is move_group up?')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        send_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future, timeout_sec=15.0)
        goal_handle = send_future.result()
        if goal_handle is None or not goal_handle.accepted:
            print('error: /execute_trajectory rejected the goal')
            return None
        print(f'execute_goal_id={goal_handle.goal_id.uuid}')
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result_future, timeout_sec=result_timeout)
        wrapped = result_future.result()
        if wrapped is None:
            print('error: no execute result (timeout)')
            return None
        print(f'execute_status={wrapped.status}')
        print(f'execute_error_code={wrapped.result.error_code.val}')
        return wrapped.result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('state', help='SRDF group_state name')
    parser.add_argument(
        '--srdf', help='SRDF path (default: installed package)')
    parser.add_argument('--tolerance', type=float, default=0.01)
    parser.add_argument('--velocity-scaling', type=float, default=0.05)
    parser.add_argument('--acceleration-scaling', type=float, default=0.05)
    parser.add_argument('--timeout', type=float, default=10.0)
    parser.add_argument(
        '--execute', action='store_true',
        help='execute the plan (requires --i-have-clear-workspace)')
    parser.add_argument(
        '--i-have-clear-workspace', action='store_true',
        help='operator acknowledgement required with --execute')
    parser.add_argument(
        '--result-timeout', type=float, default=300.0,
        help='seconds to wait for the execute result')
    args = parser.parse_args()

    if args.execute and not args.i_have_clear_workspace:
        parser.error('--execute requires --i-have-clear-workspace')

    group, joints = load_group_state(args.state, args.srdf)
    print(f'state={args.state} group={group}')
    print('target: ' + ' '.join(f'{name}={value:g}' for name, value in joints))
    print(f'scaling: velocity={args.velocity_scaling:g} '
          f'acceleration={args.acceleration_scaling:g}')

    rclpy.init()
    try:
        node = NamedStatePlan(
            group, joints, args.tolerance, args.velocity_scaling,
            args.acceleration_scaling, args.timeout)
        result = node.plan()
        if result is None:
            return 1
        if not args.execute:
            print('plan-only: no execute goal sent')
            return 0
        print('executing the inspected trajectory')
        executed = node.execute(result.trajectory, args.result_timeout)
        if executed is None:
            return 1
        return 0 if executed.error_code.val == 1 else 1
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
