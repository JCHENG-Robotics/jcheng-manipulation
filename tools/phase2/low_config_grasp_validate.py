#!/usr/bin/env python3
"""Execute one supervised grasp (or put-back) sequence, fast and measured.

Run inside the robot container with the vendor environment (CycloneDDS)::

    python3 /tmp/low_config_grasp_validate.py \
        --grasp "-0.015,-0.6618,-1.8012,-0.340" \
        --verify-pose "0,0.2574,-1.18773,-1.70,-0.01676" \
        --output /tmp/grasp_validate_run.json --i-have-clear-workspace

Default (fast) sequence: open the gripper **while** the arm travels to the
approach pose, descend with a jerk-limited profile (fast to a pre-grasp offset,
slow for the last 0.15 rad), close in two raw gripper steps, and return to the
verify pose. Every arm move is a dense S-curve sent through
``/arm_controller/follow_joint_trajectory`` (see ``fast_motion.py``) and every
step ends on a **measured** bus-servo settle instead of a fixed sleep.
``--slow`` reproduces the previous single-point 6-9 s moves for comparison.

The gripper is driven with one raw ``ServosPosition`` per step in the fast
path; ``--slow`` keeps the vendor gripper action.

Only bus-servo registers are read back; ``/joint_states`` holds cached
commands.
"""

import argparse
import json
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient

import fast_motion
from vendor_servo_map import ARM_JOINTS, gripper_pulse

READ_NAMES = ['gripper'] + ARM_JOINTS


class GraspRunner(fast_motion.FastMover):
    """Grasp / put-back sequence on the fast-motion client."""

    def __init__(self, settle=0.6, yaw=0.0, lead=0.06):
        """``yaw`` is joint5 for every arm move of the sequence."""
        super().__init__(lead=lead)
        self.settle = settle
        self.yaw = float(yaw)
        self.gripper_server = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory')

    # ------------------------------------------------------------- one move
    def arm_move(self, positions, duration=None, vmax=1.2, amax=6.0,
                 dt=fast_motion.motion_profile.MIN_SEGMENT_S, slow=False):
        """Move the arm; a four-joint target keeps the configured yaw."""
        values = list(positions)
        if len(values) == 4:
            values.append(self.yaw)
        if slow:
            return self._slow_move(values, duration if duration else 6.0)
        return self._fast_move(values, vmax, amax, dt)

    def _fast_move(self, values, vmax, amax, dt):
        """Fast S-curve move that ends on a measured settle."""
        report = super().arm_move(values, vmax, amax, dt, settle_tol=3,
                                  settle_timeout=1.0)
        if report['result_code'] != 0:
            raise RuntimeError('trajectory failed: {} {}'.format(
                report['result_code'], report['result_text']))
        return report

    def _slow_move(self, values, duration):
        """One single-point arm move (the pre-2026-11-20 behaviour)."""
        return super().slow_move(values, duration, settle=self.settle)

    def _slow_gripper(self, angle, duration):
        """Gripper move through the vendor action (pre-2026-11-20 path)."""
        started = time.monotonic()
        self.send_single_point(
            self.gripper_server, ['r_joint'], [angle], duration)
        time.sleep(self.settle)
        return {'command_rad': angle, 'duration_s': duration, 'mode': 'slow',
                'wall_s': round(time.monotonic() - started, 4)}

    def gripper_step(self, angle, duration, slow=False):
        """Command one gripper angle (rad) and wait for the register."""
        if slow:
            return self._slow_gripper(angle, duration)
        return self.gripper_wait(gripper_pulse(angle), duration)

    def open_gripper_send(self, angle, duration):
        """Start the gripper opening without waiting for it."""
        self.gripper_send(gripper_pulse(angle), duration)
        return {'command_rad': angle, 'pulse': gripper_pulse(angle),
                'duration_s': duration, 'mode': 'raw-async'}

    def pulses(self):
        """Read the gripper and the five arm registers."""
        return self.read_pulses(READ_NAMES)

    # ------------------------------------------------------------- sequence
    def grasp(self, grasp, verify, open_rad=-2.22, preclose_rad=-1.6,
              grip_rad=-0.63, vmax=3.0, amax=12.0,
              descent_vmax=0.6, final_vmax=0.35, pre_gap=0.15,
              dt=fast_motion.motion_profile.MIN_SEGMENT_S, slow=False,
              open_duration=0.8, preclose_duration=0.35,
              grip_duration=0.7):
        """Approach, descend, grip, and return to the verify pose."""
        approach = [grasp[0], grasp[1], grasp[2], 0.0]
        steps = []
        log = {'steps': steps}
        started = time.monotonic()

        if slow:
            print('OPEN gripper (action)', flush=True)
            steps.append(dict(step='open',
                              **self.gripper_step(open_rad, 4.0, slow=True)))
        else:
            self.wait_for_gripper()
            print('OPEN gripper (parallel with the approach)', flush=True)
            steps.append(dict(step='open',
                              **self.open_gripper_send(open_rad,
                                                       open_duration)))

        print('APPROACH (joint4 -> 0)', flush=True)
        steps.append(dict(step='approach', **self.arm_move(
            approach, duration=8.0, vmax=vmax, amax=amax, dt=dt,
            slow=slow)))
        print('  pulses', self.pulses(), flush=True)

        sign = 1.0 if grasp[3] >= approach[3] else -1.0
        pre = list(grasp)
        pre[3] = grasp[3] - sign * pre_gap
        print('DESCEND to pre-grasp', ['{:.4f}'.format(value)
                                       for value in pre], flush=True)
        steps.append(dict(step='descend', **self.arm_move(
            pre, duration=6.0, vmax=descent_vmax, amax=amax, dt=dt,
            slow=slow)))

        print('FINAL DESCENT to grasp', flush=True)
        steps.append(dict(step='final', **self.arm_move(
            grasp, duration=6.0, vmax=final_vmax, amax=amax, dt=dt,
            slow=slow)))
        log['pulses_at_grasp'] = self.pulses()
        print('  pulses', log['pulses_at_grasp'], flush=True)

        print('CLOSE step', preclose_rad, flush=True)
        steps.append(dict(step='close_step', **self.gripper_step(
            preclose_rad, preclose_duration, slow=slow)))
        print('CLOSE final', grip_rad, flush=True)
        steps.append(dict(step='close_final', **self.gripper_step(
            grip_rad, grip_duration, slow=slow)))
        log['pulses_after_close'] = self.pulses()
        print('  pulses', log['pulses_after_close'], flush=True)

        print('RETURN to verify pose', verify, flush=True)
        steps.append(dict(step='return', **self.arm_move(
            verify, duration=9.0, vmax=vmax, amax=amax, dt=dt, slow=slow)))
        log['pulses_at_verify'] = self.pulses()
        print('  pulses', log['pulses_at_verify'], flush=True)
        log['motion_s'] = round(time.monotonic() - started, 3)
        return log

    def release(self, grasp, verify, open_rad=-2.22, vmax=3.0, amax=12.0,
                descent_vmax=0.6, final_vmax=0.35, pre_gap=0.15,
                dt=fast_motion.motion_profile.MIN_SEGMENT_S, slow=False,
                open_duration=0.8):
        """Lower a held cube to the grasp pose, open, and return."""
        approach = [grasp[0], grasp[1], grasp[2], 0.0]
        steps = []
        log = {'steps': steps}
        started = time.monotonic()

        sign = 1.0 if grasp[3] >= approach[3] else -1.0
        pre = list(grasp)
        pre[3] = grasp[3] - sign * pre_gap
        print('DESCEND to pre-grasp', flush=True)
        steps.append(dict(step='descend', **self.arm_move(
            pre, duration=9.0, vmax=descent_vmax, amax=amax, dt=dt,
            slow=slow)))

        print('FINAL DESCENT to grasp', flush=True)
        steps.append(dict(step='final', **self.arm_move(
            grasp, duration=9.0, vmax=final_vmax, amax=amax, dt=dt,
            slow=slow)))
        print('  pulses', self.pulses(), flush=True)

        print('OPEN gripper', open_rad, flush=True)
        steps.append(dict(step='open', **self.gripper_step(
            open_rad, open_duration, slow=slow)))
        print('  pulses', self.pulses(), flush=True)

        print('RETURN to verify pose', verify, flush=True)
        steps.append(dict(step='return', **self.arm_move(
            verify, duration=9.0, vmax=vmax, amax=amax, dt=dt, slow=slow)))
        log['pulses_at_verify'] = self.pulses()
        print('  pulses', log['pulses_at_verify'], flush=True)
        log['motion_s'] = round(time.monotonic() - started, 3)
        return log


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
    parser.add_argument(
        '--yaw', type=float, default=0.0,
        help='joint5 wrist yaw in radians for the approach, grasp and '
             'release moves; the verify pose restores its own joint5')
    parser.add_argument('--vmax', type=float, default=3.0,
                        help='peak joint velocity (rad/s) for the free moves')
    parser.add_argument('--amax', type=float, default=12.0,
                        help='peak joint acceleration (rad/s^2)')
    parser.add_argument('--descent-vmax', type=float, default=0.6,
                        help='peak velocity (rad/s) for the pre-grasp descent')
    parser.add_argument('--final-vmax', type=float, default=0.35,
                        help='peak velocity (rad/s) for the final approach')
    parser.add_argument('--pre-gap', type=float, default=0.15,
                        help='joint4 offset (rad) of the pre-grasp pose')
    parser.add_argument('--dt', type=float,
                        default=fast_motion.motion_profile.MIN_SEGMENT_S,
                        help='maximum trajectory segment (s)')
    parser.add_argument('--slow', action='store_true',
                        help='reproduce the previous single-point 6-9 s moves')
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

    rclpy.init()
    node = GraspRunner(yaw=args.yaw)
    log = {'grasp_joints': grasp,
           'approach_joints': [grasp[0], grasp[1], grasp[2], 0.0],
           'verify_pose': verify, 'yaw_rad': float(args.yaw),
           'mode': 'slow' if args.slow else 'fast',
           'vmax': args.vmax, 'amax': args.amax,
           'descent_vmax': args.descent_vmax,
           'final_vmax': args.final_vmax, 'steps': []}
    try:
        log['pulses_start'] = node.pulses()
        print('START pulses', log['pulses_start'], flush=True)
        if args.release_only:
            sequence = node.release(
                grasp, verify, open_rad=args.open, vmax=args.vmax,
                amax=args.amax, descent_vmax=args.descent_vmax,
                final_vmax=args.final_vmax, pre_gap=args.pre_gap,
                dt=args.dt, slow=args.slow)
        else:
            sequence = node.grasp(
                grasp, verify, open_rad=args.open,
                preclose_rad=args.close_step,
                grip_rad=args.close, vmax=args.vmax, amax=args.amax,
                descent_vmax=args.descent_vmax, final_vmax=args.final_vmax,
                pre_gap=args.pre_gap, dt=args.dt, slow=args.slow)
        log.update(sequence)
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
