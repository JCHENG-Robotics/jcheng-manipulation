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
segmenter, drive the base along the cube bearing (up to three closed-loop
duty-20 pulses, re-measured between pulses) until the cube is inside the
0.226-0.234 m measurement band, add the validated constant correction ``e``,
fit the deep-family joints, grasp with the fast profile, verify no cube-sized
cluster remains in the cube box, put it back with the fast profile, and measure
the landing displacement. ``--return-home`` reverses the commanded approach
while the cube is held, so a far cube comes back to the robot's start pose
before the release; the landing is then measured against the fitted target
rather than the cube's original floor site. Every step records its own
measured time; the vendor report's ``motion_s`` is the pure arm motion, the
wall time adds SSH and startup. ``--no-auto-base`` restores the manual base
pulses.

The measurement is operator-gated: a cube that fails the placement gate
(height 30-50 mm, extents 30-50 mm) is re-measured after the operator adjusts
it, and the run refuses to grasp when no usable measurement remains.
"""

import argparse
import json
import math
import os
import sys
import time

import numpy as np

from _bootstrap import TOOLS, REPO, bootstrap  # noqa: E402
bootstrap()

import grasp_joint_fit  # noqa: E402
import paired_trial_session as session  # noqa: E402

DEFAULT_VIEW_POSE = '0,0.2574,-1.18773,-1.70,-0.01676'
DEFAULT_CORRECTION_MM = '13.7,3.9,8.9'
SERVOS = (6, 5, 4, 3, 2)
#: 48-51 mm/s measured at duty 10 in the 2026-11-19 closed-loop approaches.
BASE_SPEED_PER_DUTY = 4.9
BASE_DUTY = 20.0
BASE_BAND = (0.226, 0.234)
BASE_COAST_S = 0.4
MIN_BASE_TRAVEL_M = 0.015
MIN_BASE_PULSE_S = 0.20


def base_pulse_plan(point, band=BASE_BAND, duty=BASE_DUTY,
                    tolerance=MIN_BASE_TRAVEL_M):
    """Return ``(vx, vy, duration)`` to place the cube at ``(r, 0)``.

    The base pulse is ``C - (target_r, 0)`` in the base frame (``C`` is the
    measured cube point), so one pulse both brings the cube into the
    ``band`` radius **and** removes the lateral offset. Aligning to the
    forward axis keeps the grasp in the ``joint1 ~ 0`` family that passed
    24/24: at a 16.5 degree yaw the gripper closes across the cube diagonal
    (~49 mm) instead of the 39.4 mm face, which is the measured failure edge.
    ``None`` means no useful pulse: already aligned, or the correction would
    sit inside the ~0.15 s dead band.
    """
    x, y = float(point[0]), float(point[1])
    radius = math.hypot(x, y)
    if radius <= 0.0:
        raise ValueError('cube at the base origin')
    low, high = band
    # Aim at the band midpoint, not the nearest edge: the ~2 % duty-scale error
    # then has slack on both sides.
    target_r = 0.5 * (low + high)
    dx, dy = x - target_r, y
    travel = math.hypot(dx, dy)
    if travel < tolerance:
        return None
    duration = max(travel * 1000.0 / (duty * BASE_SPEED_PER_DUTY),
                   MIN_BASE_PULSE_S)
    return dx / travel, dy / travel, min(duration, 3.0)


def net_base_travel(steps):
    """Return the net commanded base displacement of the approach pulses.

    The base has no odometry, so the return home can only undo what was
    commanded. A pulse that the vendor watchdog dropped (no matched
    subscriber) did not move the robot and is excluded; a pulse whose cube
    measurement was lost still moved it and counts.
    """
    net = np.zeros(2)
    for step in steps or []:
        matched = step.get('subscribers_matched')
        if matched is not None and int(matched) < 1:
            continue
        travel = (float(step['duty']) * float(step['duration'])
                  * BASE_SPEED_PER_DUTY / 1000.0)
        net += travel * np.asarray(
            [float(step['vx']), float(step['vy'])], dtype=np.float64)
    return net


def return_home(robot, steps, duty=BASE_DUTY):
    """Drive back along the reverse of the commanded approach, holding still.

    The same open-loop reverse-pulse recipe as the 24-site session: the
    commanded travel is undone with the same duration at the same duty, so a
    floor that is faster or slower than the nominal 4.9 mm/s per duty scales
    both directions equally. Pulses are chunked at 3 s, the base caps.
    """
    net = net_base_travel(steps)
    distance = float(np.linalg.norm(net))
    report = {
        'net_travel_m': [float(value) for value in net],
        'distance_m': distance,
        'pulses': [],
        'wall_s': 0.0,
    }
    if distance < 0.005:
        return report
    direction = -net / distance
    remaining = distance
    started = time.monotonic()
    while remaining > 1e-6:
        duration = min(remaining * 1000.0 / (duty * BASE_SPEED_PER_DUTY), 3.0)
        print('  home pulse vx={:+.3f} vy={:+.3f} {:.2f} s'.format(
            direction[0], direction[1], duration), flush=True)
        pulse = robot.base_pulse(float(direction[0]), float(direction[1]),
                                 duty, duration)
        report['pulses'].append({
            'vx': float(direction[0]), 'vy': float(direction[1]),
            'duty': float(duty), 'duration': float(duration),
            'nominal_travel_mm': pulse.get('nominal_travel_mm'),
            'subscribers_matched': pulse.get('subscribers_matched'),
        })
        remaining -= duration * duty * BASE_SPEED_PER_DUTY / 1000.0
        time.sleep(BASE_COAST_S)
    report['wall_s'] = round(time.monotonic() - started, 3)
    return report


#: The segmenter's default region stops at x 0.32 / |y| 0.09, which a cube
#: that still needs a base pulse can sit outside. The base phase is allowed to
#: see it anywhere the arm could eventually reach.
WIDE_REGION = ('--x-range', '0.15,0.60', '--y-range', '0.40',
               '--max-xy', '0.12')


def measure(robot, humble, tag, retries, wide=False):
    """Segment the cube at the current pose, with an operator retry loop."""
    pulses = robot.pulses()
    last = None
    for attempt in range(retries):
        report = humble.segment(pulses, tag,
                                extra_args=WIDE_REGION if wide else None)
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
    parser.add_argument(
        '--no-auto-base', action='store_true',
        help='do not drive the base when the cube is outside the 0.226-0.234 '
             'm measurement band')
    parser.add_argument('--base-duty', type=float, default=BASE_DUTY)
    parser.add_argument('--base-pulses', type=int, default=3,
                        help='maximum closed-loop base pulses')
    parser.add_argument(
        '--return-home', action='store_true',
        help='after a verified grip, reverse the commanded approach so the '
             'cube is put back where the robot started')
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
        point, segment = measure(robot, humble, 'e2e_pre', args.retries,
                                 wide=not args.no_auto_base)
        if point is None:
            raise RuntimeError('no usable cube measurement')

        base_steps = []
        if not args.no_auto_base:
            for attempt in range(args.base_pulses):
                plan = base_pulse_plan(point, duty=args.base_duty)
                if plan is None:
                    break
                vx, vy, duration = plan
                print('  BASE pulse {}: vx={:.3f} vy={:.3f} duty={:.0f} '
                      '{:.2f} s'.format(attempt + 1, vx, vy,
                                        args.base_duty, duration),
                      flush=True)
                started = time.monotonic()
                pulse = robot.base_pulse(vx, vy, args.base_duty, duration)
                time.sleep(BASE_COAST_S)
                point, segment = measure(
                    robot, humble, 'e2e_base{}'.format(attempt), 1, wide=True)
                base_steps.append({
                    'vx': vx, 'vy': vy, 'duty': args.base_duty,
                    'duration': duration,
                    'nominal_travel_mm': pulse.get('nominal_travel_mm'),
                    'subscribers_matched': pulse.get('subscribers_matched'),
                    'wall_s': round(time.monotonic() - started, 3),
                    'point_after': (None if point is None
                                    else [float(v) for v in point]),
                })
                if point is None:
                    raise RuntimeError(
                        'no usable measurement after a base pulse')
                print('  cube now {:.4f} m from the base'.format(
                    float(np.linalg.norm(point[:2]))), flush=True)
            report['base_pulses'] = base_steps

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

        landing_reference = point
        if args.return_home and base_steps:
            print('RETURN HOME (holding the cube)', flush=True)
            home = return_home(robot, base_steps, args.base_duty)
            report['return_home'] = home
            print('  commanded displacement {:.1f} mm undone in {:.2f} s of '
                  'pulses'.format(home['distance_m'] * 1000.0,
                                  sum(pulse['duration']
                                      for pulse in home['pulses'])),
                  flush=True)
            after = humble.segment(pulses_at(grasp, 'pulses_at_verify'),
                                   'e2e_grip_home')
            dropped = session.cube_box_present(after, target)
            report['grip_after_return'] = {
                'cube_box_present': bool(dropped),
                'clusters': (after.get('segmentation') or {}).get(
                    'clusters', 0),
                'success': after.get('success'),
            }
            if dropped:
                raise RuntimeError(
                    'the cube is on the floor after the home return: MISS')
            # The cube now leaves the arm where the base-frame target points
            # from the *home* pose, not at its original floor site.
            landing_reference = target

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
            np.linalg.norm((landing - landing_reference)[:2]) * 1000.0)
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
