#!/usr/bin/env python3
"""Move the arm to one five-joint pose and report the bus-servo registers.

Container-side helper for the measurement / grasp sessions: it is the motion
that is not part of a grasp, used to put the arm at the measurement/view pose
before a trial and to recover after an interrupted one.

Run inside the robot container with the vendor environment (CycloneDDS)::

    python3 /tmp/move_to_pose.py \
        --pose 0,0.2574,-1.18773,-1.70,-0.01676 --i-have-clear-workspace

The default is a dense jerk-limited S-curve (``fast_motion.py``) that ends on a
measured bus-servo settle. ``--slow`` restores the previous single-point move
over ``--duration`` seconds.

``joint4`` may be a few pulses past the adopted EEPROM limit, which the
measurement pose needs and the 2026-11-19 runs used.
"""

import argparse
import json

import rclpy

import fast_motion


def main(argv=None):
    """Run the move and write a small report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pose', required=True,
                        help='five comma-separated joints')
    parser.add_argument('--vmax', type=float, default=3.0,
                        help='peak joint velocity (rad/s)')
    parser.add_argument('--amax', type=float, default=12.0,
                        help='peak joint acceleration (rad/s^2)')
    parser.add_argument('--dt', type=float,
                        default=fast_motion.motion_profile.MIN_SEGMENT_S)
    parser.add_argument('--duration', type=float, default=6.0,
                        help='single-point duration for --slow')
    parser.add_argument('--slow', action='store_true',
                        help='single-point move (pre-2026-11-20 behaviour)')
    parser.add_argument('--output', default='/tmp/move_to_pose.json')
    parser.add_argument('--i-have-clear-workspace', action='store_true')
    args = parser.parse_args(argv)
    if not args.i_have_clear_workspace:
        raise SystemExit('pass --i-have-clear-workspace to move the arm')
    pose = [float(value) for value in args.pose.split(',')]
    if len(pose) != 5:
        raise SystemExit('--pose needs five joints')
    rclpy.init()
    node = fast_motion.FastMover()
    report = {'target': pose, 'ok': False}
    try:
        report['pulses_start'] = node.read_pulses()
        if args.slow:
            report['move'] = node.slow_move(
                pose, args.duration, settle=0.8)
        else:
            report['move'] = node.arm_move(
                pose, args.vmax, args.amax, args.dt)
        report['pulses_end'] = node.read_pulses()
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
