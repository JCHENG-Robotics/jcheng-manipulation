#!/usr/bin/env python3
"""
Measurement-protocol schedules and analysis for Phase 3.

This tool is deliberately split into three modes so that no measurement can
start by accident:

* ``--mode dry-run`` (default): writes the trial schedule and validates the
  target against the provisional joint limits. It does not import ROS and does
  not send any command.
* ``--mode simulate``: generates synthetic samples with known noise/bias and
  runs the same analysis used for hardware data. This is the mock end-to-end
  check of the analysis path.
* ``--mode analyze``: analyzes a measurement JSON produced on hardware.

Protocols:

* ``repeatability`` — 30–50 visits to one target; per-axis and radial spread.
* ``backlash`` — alternating pre-approach directions; forward/reverse offset.
* ``visual_displacement`` — known fixture displacements vs marker measurement.
* ``gripper_envelope`` — offset grid and grasp success; tolerated envelope.

The hardware run itself is intentionally not implemented here. A separate
supervised runner (or the measurement notebook) must execute the schedule only
after the joint-limit survey and safety review.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os
import random
import statistics
import sys

try:
    import yaml
except ImportError:  # pragma: no cover - PyYAML is present in ROS setups
    yaml = None

DEFAULT_TARGET_JOINTS = (0.0, 0.59888, -1.25664, -0.71209, 0.0)
DEFAULT_LIMITS = {
    'joint1': (-2.09, 2.09),
    'joint2': (-2.09, 2.09),
    'joint3': (-2.09, 0.0),
    'joint4': (-2.09, 2.09),
    'joint5': (-2.09, 2.09),
}
JOINTS = tuple(DEFAULT_LIMITS)
LIMITATIONS = [
    'Dry-run/simulate only; the robot is not moved.',
    'Joint limits are provisional URDF/MoveIt values; mechanical limits are '
    'not measured.',
    'Simulation noise is synthetic and does not model servo dynamics, '
    'backlash or camera noise.',
    'A hardware run needs the supervised joint-limit survey and the safety '
    'review first.',
]


def _default_limits_path():
    repository = os.path.dirname(
        os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    )
    return os.path.join(
        repository,
        'ros2_ws',
        'src',
        'armpi_safety',
        'config',
        'safety_limits.yaml',
    )


def load_limits(path):
    """Load joint limits from the safety YAML, falling back to provisional."""
    limits = dict(DEFAULT_LIMITS)
    source = 'built-in provisional limits'
    if path and os.path.exists(path):
        if yaml is None:
            raise RuntimeError('PyYAML is required to read the limits file')
        with open(path, 'r', encoding='utf-8') as handle:
            raw = yaml.safe_load(handle) or {}
        for joint, values in (raw.get('joint_limits') or {}).items():
            if joint in limits:
                limits[joint] = (
                    float(values['lower']),
                    float(values['upper']),
                )
        source = path
    return limits, source


def validate_target_joints(joints, limits):
    """Raise ``ValueError`` when a target is outside the provisional limits."""
    if len(joints) != len(JOINTS):
        raise ValueError('target must have 5 joint values')
    for joint, value in zip(JOINTS, joints):
        lower, upper = limits[joint]
        if not math.isfinite(value):
            raise ValueError('{} target is not finite'.format(joint))
        if value < lower or value > upper:
            raise ValueError(
                '{} target {:.4f} is outside [{:.4f}, {:.4f}]'.format(
                    joint, value, lower, upper
                )
            )


def _mean(values):
    return sum(values) / len(values)


def _std(values):
    return statistics.stdev(values) if len(values) > 1 else 0.0


def _norm(values):
    return math.sqrt(sum(value * value for value in values))


def summarize_values(values, scale=1.0):
    """Summarize one scalar sample in the requested scale."""
    scaled = [value * scale for value in values]
    count = len(scaled)
    mean = _mean(scaled) if scaled else 0.0
    std = _std(scaled)
    return {
        'n': count,
        'mean': mean,
        'std': std,
        'min': min(scaled) if scaled else None,
        'max': max(scaled) if scaled else None,
        'range': (max(scaled) - min(scaled)) if scaled else 0.0,
        'ci95': 1.96 * std / math.sqrt(count) if count > 1 else 0.0,
    }


def summarize_positions(samples, nominal=None):
    """Summarize position samples in millimetres."""
    points = [
        [float(value) for value in sample['position_m']]
        for sample in samples
    ]
    if not points:
        return {
            'n': 0,
            'axes_mm': {},
            'mean_m': None,
            'radial_rms_mm': None,
            'radial_max_mm': None,
            'ci95_radial_mm': None,
            'nominal_error_mm': None,
        }
    columns = list(zip(*points))
    axes = {}
    for index, axis in enumerate('xyz'):
        axes[axis] = summarize_values(columns[index], scale=1000.0)
    mean = [
        sum(column) / len(column) for column in columns
    ]
    radial = [
        _norm(
            [point[index] - mean[index] for index in range(3)]
        ) * 1000.0
        for point in points
    ]
    radial_rms = math.sqrt(
        sum(value * value for value in radial) / len(radial)
    )
    result = {
        'n': len(points),
        'axes_mm': axes,
        'mean_m': mean,
        'radial_rms_mm': radial_rms,
        'radial_max_mm': max(radial),
        'ci95_radial_mm': (
            1.96 * _std(radial) / math.sqrt(len(radial))
            if len(radial) > 1
            else 0.0
        ),
        'nominal_error_mm': None,
    }
    if nominal is not None:
        errors = [
            _norm(
                [point[index] - nominal[index] for index in range(3)]
            ) * 1000.0
            for point in points
        ]
        result['nominal_error_mm'] = {
            'mean': _mean(errors),
            'max': max(errors),
            'std': _std(errors),
        }
    return result


def analyze_repeatability(samples, nominal=None):
    """Analyze repeated visits to one target."""
    summary = summarize_positions(samples, nominal=nominal)
    return {
        'kind': 'repeatability',
        'summary': summary,
        'pass_5mm': (
            summary['radial_rms_mm'] is not None
            and summary['radial_rms_mm'] <= 5.0
        ),
    }


def analyze_backlash(forward_samples, reverse_samples, nominal=None):
    """Analyze the offset between forward and reverse approaches."""
    forward = summarize_positions(forward_samples, nominal=nominal)
    reverse = summarize_positions(reverse_samples, nominal=nominal)
    delta = None
    if forward['mean_m'] is not None and reverse['mean_m'] is not None:
        vector = [
            forward['mean_m'][index] - reverse['mean_m'][index]
            for index in range(3)
        ]
        delta = {
            'vector_mm': [value * 1000.0 for value in vector],
            'magnitude_mm': _norm(vector) * 1000.0,
        }
    return {
        'kind': 'backlash',
        'forward': forward,
        'reverse': reverse,
        'hysteresis': delta,
        'pass_2mm': (
            delta is not None
            and delta['magnitude_mm'] <= 2.0 + 1e-9
        ),
    }


def analyze_visual_displacement(records, nominal=None):
    """
    Analyze measured vs known marker displacements.

    Each record has ``nominal_displacement_m`` and
    ``measured_displacement_m``.
    """
    del nominal
    errors = []
    rows = []
    for record in records:
        nominal_vector = [
            float(value) for value in record['nominal_displacement_m']
        ]
        measured = [
            float(value) for value in record['measured_displacement_m']
        ]
        error = [
            measured[index] - nominal_vector[index] for index in range(3)
        ]
        rows.append(
            {
                'id': record.get('id'),
                'nominal_displacement_m': nominal_vector,
                'measured_displacement_m': measured,
                'error_mm': [value * 1000.0 for value in error],
                'error_norm_mm': _norm(error) * 1000.0,
            }
        )
        errors.append(error)
    columns = list(zip(*errors)) if errors else [[], [], []]
    axes = {
        axis: summarize_values(columns[index], scale=1000.0)
        for index, axis in enumerate('xyz')
    }
    norms = [row['error_norm_mm'] for row in rows]
    return {
        'kind': 'visual_displacement',
        'records': rows,
        'axes_mm': axes,
        'rmse_mm': (
            math.sqrt(sum(value * value for value in norms) / len(norms))
            if norms
            else None
        ),
        'max_error_mm': max(norms) if norms else None,
        'pass_5mm': bool(norms) and max(norms) <= 5.0 + 1e-9,
    }


def analyze_gripper_envelope(records, success_threshold=0.8):
    """
    Find the largest single-axis offset that still grasps reliably.

    Each record has ``offset_m``, ``successes`` and ``trials``.
    """
    rows = []
    for record in records:
        offset = [float(value) for value in record['offset_m']]
        trials = int(record['trials'])
        successes = int(record['successes'])
        rate = successes / trials if trials else 0.0
        rows.append(
            {
                'offset_m': offset,
                'offset_mm': [value * 1000.0 for value in offset],
                'trials': trials,
                'successes': successes,
                'success_rate': rate,
                'pass': rate >= success_threshold,
            }
        )
    envelope = {}
    for index, axis in enumerate('xyz'):
        candidates = [
            abs(row['offset_m'][index])
            for row in rows
            if row['pass']
            and all(
                abs(row['offset_m'][other]) < 1e-12
                for other in range(3)
                if other != index
            )
        ]
        envelope[axis] = (
            max(candidates) * 1000.0 if candidates else None
        )
    return {
        'kind': 'gripper_envelope',
        'records': rows,
        'envelope_mm': envelope,
        'success_threshold': success_threshold,
    }


def _repeatability_schedule(target_joints, trials):
    return {
        'trials': trials,
        'target_joints': list(target_joints),
        'entries': [
            {
                'trial': index + 1,
                'target_joints': list(target_joints),
                'settle_s': 0.5,
                'dwell_s': 0.5,
            }
            for index in range(trials)
        ],
    }


def _backlash_schedule(target_joints, trials, approach_joint, delta):
    index = JOINTS.index(approach_joint)
    entries = []
    for trial in range(trials):
        direction = 'positive' if trial % 2 == 0 else 'negative'
        offset = delta if direction == 'positive' else -delta
        pre = list(target_joints)
        pre[index] = pre[index] + offset
        if pre[index] < -2.09 or pre[index] > 2.09:
            pre[index] = target_joints[index] - offset
            direction = 'negative' if direction == 'positive' else 'positive'
        entries.append(
            {
                'trial': trial + 1,
                'direction': direction,
                'pre_joints': pre,
                'target_joints': list(target_joints),
                'approach_joint': approach_joint,
                'approach_delta_rad': abs(offset),
                'settle_s': 0.5,
            }
        )
    return {
        'trials': trials,
        'target_joints': list(target_joints),
        'approach_joint': approach_joint,
        'approach_delta_rad': delta,
        'entries': entries,
    }


def _visual_schedule(step_mm, max_offset_mm, repeats):
    displacements = []
    for axis in ('x', 'y'):
        for step in range(-int(max_offset_mm // step_mm),
                          int(max_offset_mm // step_mm) + 1):
            if step == 0:
                continue
            vector = [0.0, 0.0, 0.0]
            vector['xy'.index(axis)] = step * step_mm / 1000.0
            displacements.append(
                {
                    'id': '{}_{:+d}mm'.format(axis, step * int(step_mm)),
                    'nominal_displacement_m': vector,
                    'repeats': repeats,
                }
            )
    displacements.append(
        {
            'id': 'origin',
            'nominal_displacement_m': [0.0, 0.0, 0.0],
            'repeats': repeats,
        }
    )
    return {
        'step_mm': step_mm,
        'max_offset_mm': max_offset_mm,
        'repeats': repeats,
        'placements': displacements,
    }


def _gripper_schedule(step_mm, max_offset_mm, trials):
    offsets = []
    for index, axis in enumerate('xyz'):
        for step in range(-int(max_offset_mm // step_mm),
                          int(max_offset_mm // step_mm) + 1):
            offset = [0.0, 0.0, 0.0]
            offset[index] = step * step_mm / 1000.0
            offsets.append(
                {
                    'id': '{}_{:+d}mm'.format(axis, step * int(step_mm)),
                    'offset_m': offset,
                    'trials': trials,
                }
            )
    return {
        'step_mm': step_mm,
        'max_offset_mm': max_offset_mm,
        'trials_per_offset': trials,
        'offsets': offsets,
    }


def _simulate_repeatability(target_point, trials, noise_mm, bias_mm, seed):
    generator = random.Random(seed)
    samples = []
    for index in range(trials):
        position = [
            target_point[axis]
            + (bias_mm / 1000.0 if axis == 0 else 0.0)
            + generator.gauss(0.0, noise_mm / 1000.0)
            for axis in range(3)
        ]
        samples.append(
            {'trial': index + 1, 'position_m': position}
        )
    return samples


def _simulate_visual(step_mm, max_offset_mm, repeats, noise_mm, bias_mm,
                     seed):
    generator = random.Random(seed)
    schedule = _visual_schedule(step_mm, max_offset_mm, repeats)
    records = []
    for placement in schedule['placements']:
        nominal = placement['nominal_displacement_m']
        for repeat in range(repeats):
            measured = [
                nominal[index]
                + (bias_mm / 1000.0 if index == 0 else 0.0)
                + generator.gauss(0.0, noise_mm / 1000.0)
                for index in range(3)
            ]
            records.append(
                {
                    'id': placement['id'],
                    'repeat': repeat + 1,
                    'nominal_displacement_m': nominal,
                    'measured_displacement_m': measured,
                }
            )
    return records


def _simulate_gripper(step_mm, max_offset_mm, trials, envelope_mm, seed):
    generator = random.Random(seed)
    schedule = _gripper_schedule(step_mm, max_offset_mm, trials)
    records = []
    for entry in schedule['offsets']:
        offset = entry['offset_m']
        successes = 0
        for _ in range(trials):
            success = all(
                abs(value) * 1000.0 <= envelope_mm for value in offset
            )
            if success and generator.random() < 0.9:
                successes += 1
            elif not success and generator.random() < 0.05:
                successes += 1
        records.append(
            {
                'id': entry['id'],
                'offset_m': offset,
                'trials': trials,
                'successes': successes,
            }
        )
    return records


def _build_report(args):
    limits, source = load_limits(args.limits_file)
    validate_target_joints(args.target_joints, limits)
    target_point = [0.20, 0.0, 0.10]
    schedule = None
    analysis = None
    samples = None
    if args.protocol == 'repeatability':
        schedule = _repeatability_schedule(
            args.target_joints, args.trials
        )
        if args.mode == 'simulate':
            samples = _simulate_repeatability(
                target_point, args.trials, args.noise_mm,
                args.bias_mm, args.seed,
            )
            analysis = analyze_repeatability(
                samples, nominal=target_point
            )
    elif args.protocol == 'backlash':
        schedule = _backlash_schedule(
            args.target_joints, args.trials,
            args.approach_joint, args.approach_delta,
        )
        if args.mode == 'simulate':
            forward = _simulate_repeatability(
                target_point, args.trials // 2, args.noise_mm,
                0.0, args.seed,
            )
            reverse = _simulate_repeatability(
                target_point, args.trials - args.trials // 2,
                args.noise_mm, -args.bias_mm, args.seed + 1,
            )
            analysis = analyze_backlash(
                forward, reverse, nominal=target_point
            )
    elif args.protocol == 'visual_displacement':
        schedule = _visual_schedule(
            args.step_mm, args.max_offset_mm, args.repeats
        )
        if args.mode == 'simulate':
            records = _simulate_visual(
                args.step_mm, args.max_offset_mm, args.repeats,
                args.noise_mm, args.bias_mm, args.seed,
            )
            analysis = analyze_visual_displacement(records)
    elif args.protocol == 'gripper_envelope':
        schedule = _gripper_schedule(
            args.step_mm, args.max_offset_mm, args.repeats
        )
        if args.mode == 'simulate':
            records = _simulate_gripper(
                args.step_mm, args.max_offset_mm, args.repeats,
                args.gripper_envelope_mm, args.seed,
            )
            analysis = analyze_gripper_envelope(
                records, success_threshold=args.success_threshold
            )
    else:  # pragma: no cover - argparse choices
        raise ValueError('unknown protocol {}'.format(args.protocol))

    if args.mode == 'analyze':
        if not args.input:
            raise ValueError('--mode analyze requires --input')
        with open(args.input, 'r', encoding='utf-8') as handle:
            measurement = json.load(handle)
        samples = measurement.get('samples') or measurement.get('records')
        if args.protocol == 'repeatability':
            analysis = analyze_repeatability(
                samples or [], nominal=measurement.get('nominal')
            )
        elif args.protocol == 'backlash':
            analysis = analyze_backlash(
                (samples or {}).get('forward', []),
                (samples or {}).get('reverse', []),
                nominal=measurement.get('nominal'),
            )
        elif args.protocol == 'visual_displacement':
            analysis = analyze_visual_displacement(samples or [])
        elif args.protocol == 'gripper_envelope':
            analysis = analyze_gripper_envelope(
                samples or [],
                success_threshold=args.success_threshold,
            )

    report = {
        'schema_version': 1,
        'protocol': args.protocol,
        'mode': args.mode,
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'dry_run': args.mode == 'dry-run',
        'no_hardware': True,
        'limits_source': source,
        'limits': {
            joint: list(values) for joint, values in limits.items()
        },
        'target_joints': list(args.target_joints),
        'target_point_m': target_point,
        'schedule': schedule,
        'samples': samples,
        'analysis': analysis,
        'limitations': LIMITATIONS,
    }
    return report


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            'Build Phase 3 measurement schedules and analyze results. '
            'The default dry-run never moves the robot.'
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        'protocol',
        choices=(
            'repeatability',
            'backlash',
            'visual_displacement',
            'gripper_envelope',
        ),
    )
    parser.add_argument(
        '--mode',
        choices=('dry-run', 'simulate', 'analyze'),
        default='dry-run',
    )
    parser.add_argument('--input', default=None)
    parser.add_argument('--json-out', default=None)
    parser.add_argument(
        '--limits-file', default=_default_limits_path()
    )
    parser.add_argument(
        '--target-joints',
        nargs=5,
        type=float,
        default=list(DEFAULT_TARGET_JOINTS),
        metavar=('J1', 'J2', 'J3', 'J4', 'J5'),
    )
    parser.add_argument('--trials', type=int, default=30)
    parser.add_argument('--repeats', type=int, default=3)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--noise-mm', type=float, default=0.3)
    parser.add_argument('--bias-mm', type=float, default=1.0)
    parser.add_argument('--step-mm', type=float, default=2.0)
    parser.add_argument('--max-offset-mm', type=float, default=10.0)
    parser.add_argument(
        '--approach-joint', choices=JOINTS, default='joint4'
    )
    parser.add_argument('--approach-delta', type=float, default=0.1)
    parser.add_argument(
        '--gripper-envelope-mm', type=float, default=6.0
    )
    parser.add_argument('--success-threshold', type=float, default=0.8)
    return parser.parse_args(argv)


def main(argv=None):
    """Build a report and write it when requested."""
    args = _parse_args(argv)
    if args.trials < 2:
        print('error: --trials must be at least 2', file=sys.stderr)
        return 2
    try:
        report = _build_report(args)
    except (OSError, ValueError) as exc:
        print('error: {}'.format(exc), file=sys.stderr)
        return 2
    if args.json_out:
        with open(args.json_out, 'w', encoding='utf-8') as handle:
            json.dump(report, handle, indent=2)
    print(
        'MEASUREMENT protocol={} mode={} dry_run={}'.format(
            report['protocol'], report['mode'], report['dry_run']
        )
    )
    if report['protocol'] == 'repeatability':
        entries = len(report['schedule']['entries'])
        print('SCHEDULE repeatability trials={}'.format(entries))
        if report['analysis']:
            summary = report['analysis']['summary']
            print(
                'ANALYSIS radial_rms_mm={:.3f} radial_max_mm={:.3f} '
                'nominal_error_mean_mm={:.3f} pass_5mm={}'.format(
                    summary['radial_rms_mm'],
                    summary['radial_max_mm'],
                    summary['nominal_error_mm']['mean'],
                    report['analysis']['pass_5mm'],
                )
            )
    elif report['protocol'] == 'backlash':
        print(
            'SCHEDULE backlash trials={}'.format(
                len(report['schedule']['entries'])
            )
        )
        if report['analysis'] and report['analysis']['hysteresis']:
            print(
                'ANALYSIS hysteresis_mm={:.3f} pass_2mm={}'.format(
                    report['analysis']['hysteresis']['magnitude_mm'],
                    report['analysis']['pass_2mm'],
                )
            )
    elif report['protocol'] == 'visual_displacement':
        print(
            'SCHEDULE placements={}'.format(
                len(report['schedule']['placements'])
            )
        )
        if report['analysis']:
            print(
                'ANALYSIS rmse_mm={:.3f} max_error_mm={:.3f} '
                'pass_5mm={}'.format(
                    report['analysis']['rmse_mm'],
                    report['analysis']['max_error_mm'],
                    report['analysis']['pass_5mm'],
                )
            )
    elif report['protocol'] == 'gripper_envelope':
        print(
            'SCHEDULE offsets={}'.format(
                len(report['schedule']['offsets'])
            )
        )
        if report['analysis']:
            print(
                'ANALYSIS envelope_mm={}'.format(
                    report['analysis']['envelope_mm']
                )
            )
    print('RESULT PASS')
    return 0


if __name__ == '__main__':
    sys.exit(main())
