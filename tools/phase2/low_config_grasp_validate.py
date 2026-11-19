#!/usr/bin/env python3
"""Execute one supervised grasp sequence and leave the arm at a verify pose.

Run inside the robot container with the vendor environment (CycloneDDS)::

    python3 low_config_grasp_validate.py \
        --grasp "-0.015,-0.6618,-1.8012,-0.340" \
        --verify-pose "0,0.2574,-1.18773,-1.70,-0.01676" \
        --output /tmp/grasp_validate_run.json

Sequence: open the gripper, approach from above (joint4 to 0), descend to the
grasp joints, close in two steps, then return to the verify pose and report the
bus-servo registers. It does not lift: the grip is judged from a fresh
snapshot at the verify pose (a held cube is no longer on the floor).

The arm and gripper use the vendor FollowJointTrajectory action servers
(``/arm_controller``, ``/gripper_controller``). Only bus-servo registers are
read back; ``/joint_states`` holds cached commands.
"""

import argparse
import json
import time

import rclpy
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from ros_robot_controller_msgs.msg import GetBusServoCmd
from ros_robot_controller_msgs.srv import GetBusServoState
from rclpy.action import ActionClient
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5']
SERVO_IDS = [6, 5, 4, 3, 2]


class GraspRunner(Node):
    """Arm and gripper trajectory clients plus a bus-servo readback."""

    def __init__(self, settle=0.6):
        """Create the action clients and the servo service client."""
        super().__init__('low_config_grasp_validate')
        self.settle = settle
        self.arm = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')
        self.gripper = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory')
        self.servo = self.create_client(
            GetBusServoState, '/ros_robot_controller/bus_servo/get_state')

    def move(self, client, names, positions, duration):
        """Send one trajectory point and wait for the result."""
        if not client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('action server not available')
        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = list(names)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.velocities = [0.0] * len(positions)
        point.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration % 1.0) * 1e9))
        trajectory.points = [point]
        goal.trajectory = trajectory
        send = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('goal rejected')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result, timeout_sec=duration + 15.0)
        outcome = result.result()
        if outcome is None:
            raise RuntimeError('trajectory result timeout')
        code = getattr(outcome.result, 'error_code', 0)
        if code not in (0, FollowJointTrajectory.Result.SUCCESSFUL):
            raise RuntimeError('trajectory failed: {} {}'.format(
                code, getattr(outcome.result, 'error_string', '')))
        time.sleep(self.settle)

    def arm_move(self, positions, duration=6.0):
        """Move the five arm joints."""
        self.move(self.arm, ARM_JOINTS, list(positions) + [0.0], duration)

    def gripper_move(self, r_joint, duration=3.0):
        """Move the gripper joint."""
        self.move(self.gripper, ['r_joint'], [float(r_joint)], duration)

    def pulses(self):
        """Read the five arm servo registers."""
        if not self.servo.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('servo service unavailable')
        request = GetBusServoState.Request()
        for servo_id in SERVO_IDS:
            cmd = GetBusServoCmd()
            cmd.id = int(servo_id)
            cmd.get_position = 1
            request.cmd.append(cmd)
        future = self.servo.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('servo read failed')
        return {int(cmd.id): int(state.position[0]) for cmd, state
                in zip(request.cmd, response.state) if state.position}


def parse_joints(text, count):
    """Parse a comma-separated joint vector."""
    values = [float(value) for value in text.split(',')]
    if len(values) < count:
        raise SystemExit('need {} joints, got {}'.format(count, len(values)))
    return values[:count]


def main():
    """Run the sequence and log every step."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--grasp', required=True,
                        help='joint1..joint4 at the grasp pose')
    parser.add_argument('--verify-pose', required=True,
                        help='joint1..joint5 to return to for the grip check')
    parser.add_argument('--open', type=float, default=-2.22)
    parser.add_argument('--close-step', type=float, default=-1.6)
    parser.add_argument('--close', type=float, default=-0.63)
    parser.add_argument('--output', default='/tmp/grasp_validate_run.json')
    parser.add_argument(
        '--release-only', action='store_true',
        help='lower to --grasp, open the gripper, then go to --verify-pose; '
             'used to put a held object back after a successful grip')
    parser.add_argument('--i-have-clear-workspace', action='store_true')
    args = parser.parse_args()
    if not args.i_have_clear_workspace:
        raise SystemExit('pass --i-have-clear-workspace to move the arm')

    grasp = parse_joints(args.grasp, 4)
    verify = parse_joints(args.verify_pose, 5)
    approach = [grasp[0], grasp[1], grasp[2], 0.0]

    rclpy.init()
    node = GraspRunner()
    log = {'grasp_joints': grasp, 'approach_joints': approach,
           'verify_pose': verify, 'steps': []}
    try:
        log['pulses_start'] = node.pulses()
        print('START pulses', log['pulses_start'], flush=True)

        if args.release_only:
            print('LOWER to grasp', grasp, flush=True)
            node.arm_move(grasp, 9.0)
            print('  pulses', node.pulses(), flush=True)
            print('OPEN gripper', args.open, flush=True)
            node.gripper_move(args.open, 3.0)
            print('  pulses', node.pulses(), flush=True)
            print('RETURN to verify pose', verify, flush=True)
            node.arm_move(verify, 9.0)
            log['pulses_at_verify'] = node.pulses()
            print('  pulses', log['pulses_at_verify'], flush=True)
            log['ok'] = True
        else:
            print('OPEN gripper', args.open, flush=True)
            node.gripper_move(args.open, 4.0)
            log['steps'].append({'open': args.open})
            print('  pulses', node.pulses(), flush=True)

            print('APPROACH (joint4 -> 0)', approach, flush=True)
            node.arm_move(approach, 8.0)
            print('  pulses', node.pulses(), flush=True)

            print('DESCEND to grasp', grasp, flush=True)
            node.arm_move(grasp, 6.0)
            log['pulses_at_grasp'] = node.pulses()
            print('  pulses', log['pulses_at_grasp'], flush=True)

            print('CLOSE step', args.close_step, flush=True)
            node.gripper_move(args.close_step, 3.0)
            print('  pulses', node.pulses(), flush=True)
            print('CLOSE final', args.close, flush=True)
            node.gripper_move(args.close, 3.0)
            log['pulses_after_close'] = node.pulses()
            print('  pulses', log['pulses_after_close'], flush=True)

            print('RETURN to verify pose', verify, flush=True)
            node.arm_move(verify, 9.0)
            log['pulses_at_verify'] = node.pulses()
            print('  pulses', log['pulses_at_verify'], flush=True)
            log['ok'] = True
    except Exception as exc:  # noqa: BLE001 - report and keep the log
        log['ok'] = False
        log['error'] = str(exc)
        print('FAILED:', exc, flush=True)
    finally:
        with open(args.output, 'w', encoding='utf-8') as handle:
            json.dump(log, handle, indent=2)
        node.destroy_node()
        rclpy.shutdown()
        print('WROTE', args.output, flush=True)


if __name__ == '__main__':
    main()
