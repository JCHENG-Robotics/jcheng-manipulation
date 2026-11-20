#!/usr/bin/env python3
"""One supervised fast-timing grasp and put-back through the validated chain.

Runs on the laptop with the Jazzy workspace sourced (the joint fit needs the
description package), drives the robot's vendor container over SSH and the
wrist-camera segmentation inside the Humble container::

    source /opt/ros/jazzy/setup.bash
    source ros2_ws/install/setup.bash
    python3 tools/phase3/fast_motion_e2e.py --i-have-clear-workspace \
        --output evidence/phase2/onsite_20261120/fast_motion/e2e_cube.json

Sequence: move to the view pose, measure the cube with the markerless wrist
segmenter, add the validated constant correction ``e``, fit the deep-family
joints, grasp with the fast profile, verify no cube-sized cluster remains in
the cube box, put it back with the fast profile, and measure the landing
displacement. Every step records its own measured time; the vendor report's
``motion_s`` is the pure arm motion, the wall time adds SSH and startup.

The measurement is operator-gated: a cube that fails the placement gate
(height 30-50 mm, extents 30-50 mm) is re-measured after the operator adjusts
it, and the run refuses to grasp when no usable measurement remains.
"""

import argparse
import json
import os
import sys
import time

import numpy as np

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
TOOLS = os.path.join(REPO, 'tools')
sys.path.insert(0, os.path.join(TOOLS, 'phase2'))
sys.path.insert(0, os.path.join(TOOLS, 'phase3'))
sys.path.insert(0, os.path.join(REPO, 'ros2_ws', 'src', 'armpi_analysis'))

import grasp_joint_fit  # noqa: E402
import paired_trial_session as session  # noqa: E402

DEFAULT_VIEW_POSE = '0,0.2574,-1.18773,-1.70,-0.01676'
DEFAULT_CORRECTION_MM = '13.7,3.9,8.9'
SERVOS = (6, 5, 4, 3, 2)


def measure(robot, humble, tag, retries):
    """Segment the cube at the current pose, with an operator retry loop."""
    pulses = robot.pulses()
    last = None
    for attempt in range(retries):
        report = humble.segment(pulses, tag)
        point, valid = session.cube_point(report)
        quality, reason = session.placement_quality(report)
        print('  measurement {}: success={} point={} placement={}'.format(
            attempt + 1, valid,
            None if point is None else [round(float(v), 4) for v in point],
            reason), flush=True)
        if valid and quality:
            return point, report
        last = report
        if attempt + 1 < retries:
            if not sys.stdin.isatty():
                print('  adjust the cube and re-run the measurement')
                break
            input('  adjust the cube and press Enter to re-measure ... ')
            pulses = robot.pulses()
    return None, last


def pulses_at(report, key):
    """Return the five arm registers of one vendor report in servo order."""
    return [report[key][str(servo)] for servo in SERVOS]


def main(argv=None):
    """Run the sequence and write the report."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', default='pi@10.42.0.218')
    parser.add_argument('--robot-container', default='ArmPiUltra')
    parser.add_argument('--humble-container', default='armpi-humble')
    parser.add_argument(
        '--hand-eye',
        default=('evidence/calibration/20261118/table_down/'
                 'calibration_table_down_20261118.json'))
    parser.add_argument('--hand-eye-container',
                        default='/tmp/hand_eye_table_down.json')
    parser.add_argument('--view-pose', default=DEFAULT_VIEW_POSE)
    parser.add_argument('--correction', default=DEFAULT_CORRECTION_MM)
    parser.add_argument('--max-xy-error-mm', type=float, default=8.0)
    parser.add_argument('--retries', type=int, default=3)
    parser.add_argument('--output', default='/tmp/fast_motion_e2e.json')
    parser.add_argument('--i-have-clear-workspace', action='store_true')
    args = parser.parse_args(argv)
    if not args.i_have_clear_workspace:
        raise SystemExit('pass --i-have-clear-workspace to move the arm')

    view = [float(value) for value in args.view_pose.split(',')]
    correction_mm = np.asarray(
        [float(value) for value in args.correction.split(',')],
        dtype=np.float64)
    if correction_mm.size != 3:
        raise SystemExit('--correction needs three millimetres values')
    if np.max(np.abs(correction_mm)) > 50.0:
        raise SystemExit('--correction looks too large for millimetres')
    correction = correction_mm / 1000.0

    robot = session.RobotLink(args.robot, args.robot_container)
    humble = session.HumbleLink(
        args.humble_container, args.hand_eye_container, REPO)
    robot.install_tools(os.path.join(TOOLS, 'phase2'))
    humble.install_hand_eye(os.path.join(REPO, args.hand_eye))

    report = {'ok': False, 'view_pose': view,
              'correction_mm': correction_mm.tolist()}
    try:
        started = time.monotonic()
        robot.move_to_pose(view, tag='e2e_view')
        report['move_to_view_s'] = round(time.monotonic() - started, 3)

        print('MEASURE the cube at the view pose', flush=True)
        point, segment = measure(robot, humble, 'e2e_pre', args.retries)
        if point is None:
            raise RuntimeError('no usable cube measurement')
        target = point + correction
        fit = grasp_joint_fit.fit(target)
        session.validate_target(target, fit, args.max_xy_error_mm)
        report['cube_point'] = [float(v) for v in point]
        report['target'] = [float(v) for v in target]
        report['joints'] = [float(v) for v in fit['joints']]
        report['xy_error_mm'] = float(fit['xy_error_mm'])
        print('  cube {} -> target {}'.format(
            [round(float(v), 4) for v in point],
            [round(float(v), 4) for v in target]), flush=True)
        print('  joints {} (xy error {:.2f} mm)'.format(
            ','.join('{:.4f}'.format(v) for v in fit['joints'][:4]),
            fit['xy_error_mm']), flush=True)

        print('GRASP (fast profile)', flush=True)
        started = time.monotonic()
        grasp = robot.grasp(fit['joints'], view, 'e2e_fast')
        report['grasp_wall_s'] = round(time.monotonic() - started, 3)
        report['grasp_motion_s'] = grasp.get('motion_s')
        report['grasp_steps'] = grasp.get('steps')
        report['grasp_pulses_at_grasp'] = grasp.get('pulses_at_grasp')
        report['grasp_pulses_after_close'] = grasp.get('pulses_after_close')
        report['grasp_pulses_at_verify'] = grasp.get('pulses_at_verify')

        print('VERIFY the grip (floor under the cube box)', flush=True)
        grip = humble.segment(pulses_at(grasp, 'pulses_at_verify'),
                              'e2e_grip')
        in_box = session.cube_box_present(grip, point)
        report['grip'] = {
            'cube_box_present': bool(in_box),
            'clusters': (grip.get('segmentation') or {}).get('clusters', 0),
            'success': grip.get('success'),
        }
        if in_box:
            raise RuntimeError('the cube is still on the floor: MISS')

        print('RELEASE (fast profile)', flush=True)
        started = time.monotonic()
        release = robot.release(fit['joints'], view, 'e2e_fast')
        report['release_wall_s'] = round(time.monotonic() - started, 3)
        report['release_motion_s'] = release.get('motion_s')
        report['release_steps'] = release.get('steps')

        print('MEASURE the landing', flush=True)
        landing, _ = session.cube_point(humble.segment(
            pulses_at(release, 'pulses_at_verify'), 'e2e_land'))
        if landing is None:
            raise RuntimeError('no landing measurement')
        report['landing'] = [float(v) for v in landing]
        report['landing_error_mm'] = float(
            np.linalg.norm((landing - point)[:2]) * 1000.0)
        print('  landing error {:.2f} mm'.format(report['landing_error_mm']),
              flush=True)
        report['ok'] = True
    except Exception as exc:  # noqa: BLE001 - keep the partial report
        report['error'] = str(exc)
        print('FAILED: {}'.format(exc), flush=True)
    finally:
        with open(args.output, 'w', encoding='utf-8') as handle:
            json.dump(report, handle, indent=2)
        print('WROTE {}'.format(args.output), flush=True)
    return 0 if report.get('ok') else 1


if __name__ == '__main__':
    sys.exit(main())
