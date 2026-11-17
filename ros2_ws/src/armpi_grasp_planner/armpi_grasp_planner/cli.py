"""
Command-line grasp-candidate search and MoveIt 2 screening.

The command never executes a trajectory. By default it plans only, using the
Pilz PTP pipeline, and writes a JSON report when ``--json-out`` is given.

Example (positive control: an FK-generated, model-reachable grasp point)::

    ros2 run armpi_grasp_planner grasp_candidate_check \
        --target-joints 0.3 0.9 -1.8 -1.7 0.4 \
        --json-out /tmp/grasp.json

The target modes are mutually exclusive:

* ``--target-point X Y Z``: a vision-style grasp point in ``base_link``.
* ``--target-joints J1 J2 J3 J4 J5``: a model pose used as a reachable
  positive control. The exact FK candidate is always included and flagged.
"""

import argparse
from datetime import datetime, timezone
import json
import sys

import rclpy

from .candidates import (
    DEFAULT_APPROACH_YAWS_DEG,
    DEFAULT_PITCHES_DEG,
    DEFAULT_ROLLS_DEG,
    DEFAULT_STANDOFFS_M,
    DEFAULT_TCP_OFFSET_M,
    generate_candidates,
    mark_exact_fk,
    sort_candidates,
)
from .evaluator import MoveItGraspEvaluator
from .geometry import (
    decompose_tool_orientation,
    matrix_from_quaternion,
    wrap_degrees,
)

DEFAULT_SEEDS = (
    (0.0, 0.7309, -1.6127, -1.6591, 0.0),  # SRDF init_pose
    (0.0, 0.0, 0.0, 0.0, 0.0),
    (0.0, 0.6, -1.2, -0.6, 0.5),  # Phase 0 planning check pose
)


def parse_float_list(text):
    """Parse a comma- or space-separated list of floats."""
    values = []
    for token in text.replace(',', ' ').split():
        values.append(float(token))
    if not values:
        raise argparse.ArgumentTypeError('expected at least one number')
    return values


def _unique(values):
    result = []
    seen = set()
    for value in values:
        key = round(float(value), 6)
        if key not in seen:
            seen.add(key)
            result.append(float(value))
    return result


def _family_from_fk(decomposition):
    pitch = decomposition['pitch_deg']
    yaw = decomposition['approach_yaw_deg']
    roll = decomposition['roll_deg']
    pitches = _unique(
        max(0.0, min(89.0, pitch + delta)) for delta in (-10.0, 0.0, 10.0)
    )
    yaws = _unique(wrap_degrees(yaw + delta) for delta in (-15.0, 0.0, 15.0))
    rolls = _unique(roll + delta for delta in (-15.0, 0.0, 15.0))
    return pitches, yaws, rolls, [0.0, 0.03]


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Generate and screen 5-DoF grasp candidates with MoveIt 2.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    target = parser.add_mutually_exclusive_group(required=True)
    target.add_argument(
        '--target-point',
        nargs=3,
        type=float,
        metavar=('X', 'Y', 'Z'),
        help='grasp point in base_link, metres',
    )
    target.add_argument(
        '--target-joints',
        nargs=5,
        type=float,
        metavar=('J1', 'J2', 'J3', 'J4', 'J5'),
        help='model pose used as a reachable positive control',
    )
    parser.add_argument(
        '--pitches',
        type=parse_float_list,
        default=None,
        help='approach pitch list, degrees from straight down',
    )
    parser.add_argument(
        '--approach-yaws',
        type=parse_float_list,
        default=None,
        help='approach azimuth list, degrees about base_link +z',
    )
    parser.add_argument(
        '--rolls',
        type=parse_float_list,
        default=None,
        help='tool-roll list about the approach axis, degrees',
    )
    parser.add_argument(
        '--standoffs',
        type=parse_float_list,
        default=None,
        help='pre-grasp stand-off list along the approach axis, metres',
    )
    parser.add_argument(
        '--tcp-offset',
        type=float,
        default=DEFAULT_TCP_OFFSET_M,
        help='gripper_link to tool-point distance along +z, metres',
    )
    parser.add_argument(
        '--max-pitch',
        type=float,
        default=75.0,
        help='reject approaches more horizontal than this, degrees',
    )
    parser.add_argument('--object-width', type=float, default=None)
    parser.add_argument('--gripper-max-opening', type=float, default=None)
    parser.add_argument(
        '--max-plan',
        type=int,
        default=5,
        help='maximum number of candidates sent to the trajectory planner',
    )
    parser.add_argument('--ik-timeout', type=float, default=0.2)
    parser.add_argument(
        '--seed-joints',
        action='append',
        nargs=5,
        type=float,
        default=None,
        metavar=('J1', 'J2', 'J3', 'J4', 'J5'),
        help='IK seed; repeatable. Defaults to the init/zero/ready seeds',
    )
    parser.add_argument(
        '--pipeline', default='pilz_industrial_motion_planner'
    )
    parser.add_argument('--planner', default='PTP')
    parser.add_argument('--planning-timeout', type=float, default=10.0)
    parser.add_argument('--position-tolerance', type=float, default=0.001)
    parser.add_argument(
        '--orientation-tolerance',
        type=float,
        default=0.01,
        help='orientation tolerance in radians',
    )
    parser.add_argument('--fk-position-tolerance', type=float, default=0.001)
    parser.add_argument(
        '--fk-orientation-tolerance', type=float, default=1.0
    )
    parser.add_argument(
        '--sampler',
        choices=('auto', 'cartesian', 'joint'),
        default='auto',
        help=(
            'auto: cartesian for --target-joints, joint for --target-point'
        ),
    )
    parser.add_argument(
        '--sampling-radius',
        type=float,
        default=0.02,
        help='joint-sampler TCP radius around the target, metres',
    )
    parser.add_argument(
        '--position-step',
        type=float,
        default=0.2,
        help='joint-sampler step for joint1..joint4, radians',
    )
    parser.add_argument(
        '--roll-step',
        type=float,
        default=0.3,
        help='joint-sampler step for joint5, radians',
    )
    parser.add_argument(
        '--max-sampled',
        type=int,
        default=80,
        help='maximum number of joint-sampled candidates',
    )
    parser.add_argument(
        '--no-refined',
        action='store_true',
        help='skip exact-target refinements in joint-sampler mode',
    )
    parser.add_argument(
        '--urdf',
        default=None,
        help='URDF/xacro path for joint sampling; defaults to the description',
    )
    parser.add_argument(
        '--safety-service',
        default='',
        help=(
            'optional arm_interfaces/ValidateTrajectory service; planned '
            'candidates are vetoed unless it approves'
        ),
    )
    parser.add_argument(
        '--max-start-state-distance',
        type=float,
        default=0.0,
        help='override the safety start-state threshold, radians (0=config)',
    )
    parser.add_argument(
        '--include-trajectory',
        action='store_true',
        help='include the full planned joint trajectory in the JSON report',
    )
    parser.add_argument('--service-timeout', type=float, default=10.0)
    parser.add_argument('--json-out', default=None)
    parser.add_argument('--quiet', action='store_true')
    return parser.parse_args(argv)


def _target_summary(args, decomposition, tcp_pose):
    summary = {
        'mode': 'fk_joints' if args.target_joints else 'point',
        'joints': list(args.target_joints) if args.target_joints else None,
        'point': None,
        'decomposition': decomposition,
        'tcp_pose': None,
    }
    if args.target_joints:
        summary['point'] = [
            tcp_pose.position.x,
            tcp_pose.position.y,
            tcp_pose.position.z,
        ]
        summary['tcp_pose'] = {
            'position': summary['point'],
            'orientation': [
                tcp_pose.orientation.x,
                tcp_pose.orientation.y,
                tcp_pose.orientation.z,
                tcp_pose.orientation.w,
            ],
        }
    else:
        summary['point'] = list(args.target_point)
    return summary


def _best_entry(results):
    preferred = [
        entry for entry in results
        if entry['status'] == 'PLAN_OK'
        and (entry.get('safety') or {}).get('approved')
    ]
    if preferred:
        results = preferred
    for status in ('PLAN_OK', 'COLLISION_FREE'):
        for entry in results:
            if entry['status'] == status:
                return {
                    'candidate_id': entry['candidate_id'],
                    'status': entry['status'],
                    'is_exact_fk': entry['is_exact_fk'],
                    'ik_joints': (
                        entry.get('ik', {}).get('joints')
                        if entry.get('ik')
                        else None
                    ),
                    'plan': entry.get('plan'),
                    'safety': entry.get('safety'),
                }
    return None


def main(argv=None):
    """Run the candidate search. Returns a process exit code."""
    args = _parse_args(argv)
    rclpy.init(args=[])
    node = None
    summary = {
        'schema_version': 1,
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'success': False,
    }
    try:
        node = MoveItGraspEvaluator(
            service_timeout=args.service_timeout,
            safety_service=args.safety_service,
        )
        node.wait_for_interfaces()

        sampler = args.sampler
        if sampler == 'auto':
            sampler = 'cartesian' if args.target_joints else 'joint'

        decomposition = None
        tcp_pose = None
        pitches = []
        approach_yaws = []
        rolls = []
        standoffs = []
        if args.target_joints:
            fk_poses = node.compute_fk(args.target_joints)
            tcp_pose = fk_poses[node.tcp_link]
            rotation = matrix_from_quaternion(
                (
                    tcp_pose.orientation.x,
                    tcp_pose.orientation.y,
                    tcp_pose.orientation.z,
                    tcp_pose.orientation.w,
                )
            )
            decomposition = decompose_tool_orientation(rotation)
            target_point = (
                tcp_pose.position.x,
                tcp_pose.position.y,
                tcp_pose.position.z,
            )
        else:
            target_point = tuple(args.target_point)

        exact_found = False
        if sampler == 'cartesian':
            if decomposition:
                defaults = _family_from_fk(decomposition)
            else:
                defaults = (
                    list(DEFAULT_PITCHES_DEG),
                    list(DEFAULT_APPROACH_YAWS_DEG),
                    list(DEFAULT_ROLLS_DEG),
                    list(DEFAULT_STANDOFFS_M),
                )
            pitches = (
                list(args.pitches) if args.pitches is not None else defaults[0]
            )
            approach_yaws = (
                list(args.approach_yaws)
                if args.approach_yaws is not None
                else defaults[1]
            )
            rolls = (
                list(args.rolls) if args.rolls is not None else defaults[2]
            )
            standoffs = (
                list(args.standoffs)
                if args.standoffs is not None
                else defaults[3]
            )
            if decomposition:
                pitches = _unique(pitches + [decomposition['pitch_deg']])
                approach_yaws = _unique(
                    approach_yaws + [decomposition['approach_yaw_deg']]
                )
                rolls = _unique(rolls + [decomposition['roll_deg']])
                standoffs = _unique(standoffs + [0.0])
            candidates = generate_candidates(
                target_point,
                pitches_deg=pitches,
                approach_yaws_deg=approach_yaws,
                rolls_deg=rolls,
                standoffs_m=standoffs,
                tcp_offset_m=args.tcp_offset,
            )
            if decomposition:
                candidates, exact_found = mark_exact_fk(
                    candidates,
                    decomposition['pitch_deg'],
                    decomposition['approach_yaw_deg'],
                    decomposition['roll_deg'],
                )
        else:
            from .reachability import ArmKinematics

            kinematics = ArmKinematics(urdf_path=args.urdf)
            candidates = kinematics.sample_candidates(
                target_point,
                tcp_offset_m=args.tcp_offset,
                radius_m=args.sampling_radius,
                position_step_rad=args.position_step,
                roll_step_rad=args.roll_step,
                max_pitch_deg=args.max_pitch,
                max_candidates=args.max_sampled,
                include_refined=not args.no_refined,
            )
        candidates = sort_candidates(candidates)

        seeds = tuple(args.seed_joints) if args.seed_joints else DEFAULT_SEEDS
        results, counts = node.evaluate_all(
            candidates,
            seeds,
            max_plan=args.max_plan,
            max_pitch_deg=args.max_pitch,
            object_width_m=args.object_width,
            gripper_max_opening_m=args.gripper_max_opening,
            pipeline=args.pipeline,
            planner=args.planner,
            allowed_planning_time=args.planning_timeout,
            position_tolerance=args.position_tolerance,
            orientation_tolerance=args.orientation_tolerance,
            fk_position_tolerance=args.fk_position_tolerance,
            fk_orientation_tolerance_deg=args.fk_orientation_tolerance,
            ik_timeout=args.ik_timeout,
            include_trajectory=args.include_trajectory,
            max_start_state_distance_rad=(
                args.max_start_state_distance
            ),
        )

        target_summary = _target_summary(args, decomposition, tcp_pose)
        summary.update(
            {
                'target': target_summary,
                'frame': node.frame,
                'tool_link': node.tool_link,
                'tcp_link': node.tcp_link,
                'parameters': {
                    'sampler': sampler,
                    'pitches_deg': list(pitches),
                    'approach_yaws_deg': list(approach_yaws),
                    'rolls_deg': list(rolls),
                    'standoffs_m': list(standoffs),
                    'tcp_offset_m': args.tcp_offset,
                    'max_pitch_deg': args.max_pitch,
                    'sampling_radius_m': args.sampling_radius,
                    'position_step_rad': args.position_step,
                    'roll_step_rad': args.roll_step,
                    'max_sampled': args.max_sampled,
                    'include_refined': not args.no_refined,
                    'urdf': args.urdf,
                    'object_width_m': args.object_width,
                    'gripper_max_opening_m': args.gripper_max_opening,
                    'max_plan': args.max_plan,
                    'ik_timeout_s': args.ik_timeout,
                    'safety_service': args.safety_service or None,
                    'include_trajectory': args.include_trajectory,
                    'seeds': [list(seed) for seed in seeds],
                    'pipeline': args.pipeline,
                    'planner': args.planner,
                },
                'exact_fk_candidate_found': exact_found,
                'counts': counts,
                'best': _best_entry(results),
                'candidates': results,
            }
        )
        success = (
            counts['plan_ok'] > 0
            if args.max_plan > 0
            else counts['collision_ok'] > 0
        )
        if args.safety_service and counts['plan_ok'] > 0:
            success = success and counts['safety_ok'] > 0
        summary['success'] = success

        if not args.quiet:
            if decomposition:
                print(
                    'TARGET mode=fk_joints sampler={} '
                    'tcp=({:.4f}, {:.4f}, {:.4f}) '
                    'pitch={:.2f} yaw={:.2f} roll={:.2f}'.format(
                        sampler,
                        target_point[0],
                        target_point[1],
                        target_point[2],
                        decomposition['pitch_deg'],
                        decomposition['approach_yaw_deg'],
                        decomposition['roll_deg'],
                    )
                )
            else:
                print(
                    'TARGET mode=point sampler={} '
                    'tcp=({:.4f}, {:.4f}, {:.4f})'.format(
                        sampler, *target_point
                    )
                )
            for entry in results:
                plan = entry.get('plan') or {}
                ik = entry.get('ik') or {}
                seed = ik.get('seed_index')
                print(
                    'CANDIDATE {} status={} source={} pitch={:.1f} '
                    'yaw={:.1f} roll={:+.1f} standoff={:.3f} '
                    'offset={:.1f}mm seed={} points={}'.format(
                        entry['candidate_id'],
                        entry['status'],
                        entry['source'],
                        entry['pitch_deg'],
                        entry['approach_yaw_deg'],
                        entry['roll_deg'],
                        entry['standoff_m'],
                        entry['target_offset_m'] * 1000.0,
                        '-' if seed is None else seed,
                        plan.get('trajectory_points', 0),
                    )
                )
                safety = entry.get('safety')
                if safety:
                    print(
                        'SAFETY {} approved={} code={}'.format(
                            entry['candidate_id'],
                            safety['approved'],
                            safety['code'],
                        )
                    )
            print(
                'GRASP_SEARCH candidates={generated} sanity_ok={sanity_ok} '
                'ik_ok={ik_ok} fk_ok={fk_ok} collision_ok={collision_ok} '
                'plan_ok={plan_ok}'.format(**counts)
            )
            exact = next(
                (entry for entry in results if entry['is_exact_fk']), None
            )
            if exact:
                print(
                    'EXACT_FK candidate={} status={}'.format(
                        exact['candidate_id'], exact['status']
                    )
                )
            print('RESULT {}'.format('PASS' if success else 'FAIL'))
        if args.json_out:
            with open(args.json_out, 'w', encoding='utf-8') as handle:
                json.dump(summary, handle, indent=2)
        return 0 if success else 1
    except Exception as exc:  # noqa: BLE001 - CLI should report, not traceback
        summary['error'] = str(exc)
        print('GRASP_SEARCH FAILED: {}'.format(exc), file=sys.stderr)
        if args.json_out:
            with open(args.json_out, 'w', encoding='utf-8') as handle:
                json.dump(summary, handle, indent=2)
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
