#!/usr/bin/env python3
"""Move the arm to one five-joint pose and report the bus-servo registers.

Container-side helper for the paired-trial session: it is the only motion that
is not part of a grasp, used to put the arm at the measurement/view pose
before the first trial and to recover after an interrupted one.

Run inside the robot container with the vendor environment (CycloneDDS)::

    python3 /tmp/move_to_pose.py \
        --pose 0,0.2574,-1.18773,-1.70,-0.01676 --i-have-clear-workspace

The pose is a single trajectory point sent to ``/arm_controller``; the vendor
controller interpolates. ``joint4`` may be a few pulses past the adopted
EEPROM limit, which the measurement pose needs and the 2026-11-19 runs used.
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


class Mover(Node):
    """One arm trajectory plus a bus-servo readback."""

    def __init__(self, settle=0.8):
        super().__init__('move_to_pose')
        self.settle = settle
        self.arm = ActionClient(
            self, FollowJointTrajectory,
            '/arm_controller/follow_joint_trajectory')
        self.servo = self.create_client(
            GetBusServoState, '/ros_robot_controller/bus_servo/get_state')

    def move(self, positions, duration):
        """Send one trajectory point and wait for the action result."""
        if not self.arm.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('arm action server unavailable')
        goal = FollowJointTrajectory.Goal()
        trajectory = JointTrajectory()
        trajectory.joint_names = list(ARM_JOINTS)
        point = JointTrajectoryPoint()
        point.positions = [float(value) for value in positions]
        point.velocities = [0.0] * len(positions)
        point.time_from_start = Duration(
            sec=int(duration), nanosec=int((duration % 1.0) * 1e9))
        trajectory.points = [point]
        goal.trajectory = trajectory
        send = self.arm.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send, timeout_sec=10.0)
        handle = send.result()
        if handle is None or not handle.accepted:
            raise RuntimeError('goal rejected')
        result = handle.get_result_async()
        rclpy.spin_until_future_complete(
            self, result, timeout_sec=duration + 20.0)
        outcome = result.result()
        if outcome is None:
            raise RuntimeError('trajectory result timeout')
        code = getattr(outcome.result, 'error_code', 0)
        if code not in (0, FollowJointTrajectory.Result.SUCCESSFUL):
            raise RuntimeError('trajectory failed: {} {}'.format(
                code, getattr(outcome.result, 'error_string', '')))
        time.sleep(self.settle)

    def pulses(self):
        """Read the five arm registers."""
        if not self.servo.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('servo service unavailable')
        request = GetBusServoState.Request()
        for servo_id in SERVO_IDS:
            command = GetBusServoCmd()
            command.id = int(servo_id)
            command.get_position = 1
            request.cmd.append(command)
        future = self.servo.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError('servo read failed')
        return {int(command.id): int(state.position[0])
                for command, state in zip(request.cmd, response.state)
                if state.position}


def main(argv=None):
    """Run the move and write a small report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pose', required=True,
                        help='five comma-separated joints')
    parser.add_argument('--duration', type=float, default=6.0)
    parser.add_argument('--output', default='/tmp/move_to_pose.json')
    parser.add_argument('--i-have-clear-workspace', action='store_true')
    args = parser.parse_args(argv)
    if not args.i_have_clear_workspace:
        raise SystemExit('pass --i-have-clear-workspace to move the arm')
    pose = [float(value) for value in args.pose.split(',')]
    if len(pose) != 5:
        raise SystemExit('--pose needs five joints')
    rclpy.init()
    node = Mover()
    report = {'target': pose, 'ok': False}
    try:
        report['pulses_start'] = node.pulses()
        node.move(pose, args.duration)
        report['pulses_end'] = node.pulses()
        report['ok'] = True
    except Exception as error:  # noqa: BLE001 - report and fail loudly
        report['error'] = str(error)
        raise
    finally:
        with open(args.output, 'w', encoding='utf-8') as handle:
            json.dump(report, handle, indent=2)
        node.destroy_node()
        rclpy.shutdown()
    print('moved to {}'.format(pose))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
