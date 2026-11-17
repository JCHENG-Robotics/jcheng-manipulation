"""Plan-only check for an ArmPi SRDF named state on the real robot.

Run inside the Humble MoveIt container after sourcing the workspace:

    python3 plan_named_state_check.py camera_forward

It reads the installed ``armpi_ultra.srdf``, turns the requested named state
into joint constraints and calls ``/plan_kinematic_path``. It never sends an
execute goal, so the real arm does not move.
"""

import argparse
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan
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


class PlanCheck(Node):
    """Call the MoveIt plan-only service for one joint-constrained goal."""

    def __init__(self, group, joints, tolerance, velocity, acceleration, timeout):
        super().__init__('named_state_plan_check')
        self.group = group
        self.joints = joints
        self.tolerance = tolerance
        self.velocity = velocity
        self.acceleration = acceleration
        self.timeout = timeout
        self.client = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        if not self.client.wait_for_service(timeout_sec=15.0):
            raise SystemExit('/plan_kinematic_path not available; is move_group up?')

    def run(self):
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

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        response = future.result()
        if response is None:
            print('error: no response from /plan_kinematic_path')
            return 3

        result = response.motion_plan_response
        print(f'error_code={result.error_code.val}')
        print(f'planning_time={result.planning_time:.3f}s')
        trajectory = result.trajectory.joint_trajectory
        print(f'waypoints={len(trajectory.points)}')
        if trajectory.points:
            for name, value in zip(trajectory.joint_names,
                                   trajectory.points[-1].positions):
                print(f'  final {name}={value:.6f}')
        return 0 if result.error_code.val == 1 else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('state', help='SRDF group_state name')
    parser.add_argument('--srdf', help='SRDF path (default: installed package)')
    parser.add_argument('--tolerance', type=float, default=0.01)
    parser.add_argument('--velocity-scaling', type=float, default=0.05)
    parser.add_argument('--acceleration-scaling', type=float, default=0.05)
    parser.add_argument('--timeout', type=float, default=10.0)
    args = parser.parse_args()

    group, joints = load_group_state(args.state, args.srdf)
    print(f'state={args.state} group={group}')
    print('target: ' + ' '.join(f'{name}={value:g}' for name, value in joints))

    rclpy.init()
    try:
        return PlanCheck(
            group, joints, args.tolerance, args.velocity_scaling,
            args.acceleration_scaling, args.timeout,
        ).run()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
