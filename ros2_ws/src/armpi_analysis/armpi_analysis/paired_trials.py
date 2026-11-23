#!/usr/bin/env python3
"""
Paired-trial planning, recording and analysis for grasp experiments.

The experiment unit is a matched pair: the same object fixture is attempted
once with ``baseline`` and once with ``intervention``. The order of the two
methods is randomized within each pair, and the pair order is randomized too,
so time drift does not align with the method.

Records are append-only JSON Lines. ``continuous_error_mm`` is the primary
continuous outcome; ``grasp_success`` and ``failure_code`` are the binary
outcome. A failed grasp may have a null continuous error if no displacement
was measured; the paired continuous analysis only uses pairs where both
values exist.

The analysis uses the shared ``jcheng_stats`` package (stdlib only, no SciPy):
a seeded bootstrap interval for the mean paired difference, an exact sign
test, and an exact McNemar test for the paired success table.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os
import random
import sys
from jcheng_stats import (
    bootstrap_mean_ci as _bootstrap_mean_ci,
    exact_binomial_two_sided as _exact_binomial_two_sided,
    summary as _summary,
)

SCHEMA_VERSION = 1
METHODS = ('baseline', 'intervention')
REQUIRED_FIELDS = (
    'pair_id',
    'trial_order',
    'method',
    'object_pose_fixture',
    'continuous_error_mm',
    'grasp_success',
    'failure_code',
)
LIMITATIONS = [
    'Analysis is only as good as the recorded fixture and measurement.',
    'The exact sign test ignores effect size; report it with the mean '
    'difference and interval.',
    'McNemar tests paired success only; it does not explain a failure.',
    'Order-effect estimates are observational, not a randomized crossover '
    'of time.',
    'A fixture is assumed geometrically identical across its pairs.',
]


def _is_number(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def validate_record(record):
    """Validate one trial record and return a normalized copy."""
    if not isinstance(record, dict):
        raise ValueError('record must be an object')
    missing = [field for field in REQUIRED_FIELDS if field not in record]
    if missing:
        raise ValueError(
            'record is missing field(s): {}'.format(', '.join(missing))
        )
    normalized = dict(record)
    pair_id = record['pair_id']
    if not isinstance(pair_id, str) or not pair_id:
        raise ValueError('pair_id must be a non-empty string')
    trial_order = record['trial_order']
    if (
        not isinstance(trial_order, int)
        or isinstance(trial_order, bool)
        or trial_order < 1
    ):
        raise ValueError('trial_order must be an integer >= 1')
    method = record['method']
    if method not in METHODS:
        raise ValueError(
            'method must be one of {}'.format(METHODS)
        )
    fixture = record['object_pose_fixture']
    if not isinstance(fixture, str) or not fixture:
        raise ValueError('object_pose_fixture must be a non-empty string')
    success = record['grasp_success']
    if not isinstance(success, bool):
        raise ValueError('grasp_success must be a boolean')
    error = record['continuous_error_mm']
    if error is not None:
        if not _is_number(error) or float(error) < 0.0:
            raise ValueError(
                'continuous_error_mm must be null or finite and '
                'non-negative'
            )
        error = float(error)
    elif success:
        raise ValueError(
            'a successful grasp needs continuous_error_mm'
        )
    failure_code = record['failure_code']
    if success:
        if failure_code not in (None, '', 'NONE'):
            raise ValueError(
                'a successful grasp must not have a failure_code'
            )
        failure_code = None
    else:
        if not isinstance(failure_code, str) or not failure_code:
            raise ValueError(
                'a failed grasp needs a non-empty failure_code'
            )
        if failure_code == 'NONE':
            raise ValueError('failure_code NONE contradicts grasp_success')
    normalized['pair_id'] = pair_id
    normalized['trial_order'] = trial_order
    normalized['method'] = method
    normalized['object_pose_fixture'] = fixture
    normalized['continuous_error_mm'] = error
    normalized['grasp_success'] = success
    normalized['failure_code'] = failure_code
    normalized['schema_version'] = SCHEMA_VERSION
    return normalized


def _as_record_list(records):
    if isinstance(records, dict):
        for key in ('records', 'trials'):
            if isinstance(records.get(key), list):
                return records[key]
        raise ValueError('JSON object has no records/trials list')
    if not isinstance(records, list):
        raise ValueError('records must be a list or an object')
    return records


def validate_dataset(records):
    """Validate a record list and group it into complete/incomplete pairs."""
    raw_records = _as_record_list(records)
    valid = []
    invalid = []
    seen_method = set()
    seen_order = set()
    for index, raw_record in enumerate(raw_records):
        try:
            record = validate_record(raw_record)
        except ValueError as exc:
            invalid.append({'index': index, 'error': str(exc)})
            continue
        method_key = (record['pair_id'], record['method'])
        order_key = (record['pair_id'], record['trial_order'])
        if method_key in seen_method:
            invalid.append(
                {
                    'index': index,
                    'error': 'duplicate pair/method {}'.format(
                        method_key
                    ),
                }
            )
            continue
        if order_key in seen_order:
            invalid.append(
                {
                    'index': index,
                    'error': 'duplicate pair/trial_order {}'.format(
                        order_key
                    ),
                }
            )
            continue
        seen_method.add(method_key)
        seen_order.add(order_key)
        valid.append(record)
    groups = {}
    for record in valid:
        groups.setdefault(record['pair_id'], []).append(record)
    pairs = {}
    complete_pairs = []
    incomplete_pairs = []
    for pair_id, group in groups.items():
        methods = {record['method'] for record in group}
        complete = methods == set(METHODS)
        pairs[pair_id] = {
            'pair_id': pair_id,
            'complete': complete,
            'methods': sorted(methods),
            'n_records': len(group),
        }
        if complete:
            complete_pairs.append(pair_id)
        else:
            incomplete_pairs.append(
                {
                    'pair_id': pair_id,
                    'methods': sorted(methods),
                    'n_records': len(group),
                }
            )
    return {
        'schema_version': SCHEMA_VERSION,
        'n_input': len(raw_records),
        'n_valid': len(valid),
        'n_invalid': len(invalid),
        'invalid': invalid,
        'complete_pairs': sorted(complete_pairs),
        'incomplete_pairs': incomplete_pairs,
        'pairs': pairs,
        'records': valid,
    }


def build_plan(pair_ids, fixtures=None, seed=7):
    """
    Build a randomized paired schedule.

    The pair order is shuffled, and each pair's method order is shuffled
    independently. The same seed reproduces the schedule.
    """
    if not isinstance(pair_ids, list) or not pair_ids:
        raise ValueError('pair_ids must be a non-empty list')
    if len(set(pair_ids)) != len(pair_ids):
        raise ValueError('pair_ids must be unique')
    for pair_id in pair_ids:
        if not isinstance(pair_id, str) or not pair_id:
            raise ValueError('each pair_id must be a non-empty string')
    if fixtures is None:
        fixture_list = [
            'fixture_{:03d}'.format(index + 1)
            for index in range(len(pair_ids))
        ]
    else:
        if not isinstance(fixtures, list) or not fixtures:
            raise ValueError('fixtures must be a non-empty list')
        if len(fixtures) == 1:
            fixture_list = [fixtures[0]] * len(pair_ids)
        elif len(fixtures) == len(pair_ids):
            fixture_list = list(fixtures)
        else:
            raise ValueError(
                'fixtures must have one entry or one per pair'
            )
    generator = random.Random(seed)
    pair_order = list(range(len(pair_ids)))
    generator.shuffle(pair_order)
    entries = []
    pairs = []
    sequence = 0
    for index in pair_order:
        pair_id = pair_ids[index]
        fixture = fixture_list[index]
        method_order = list(METHODS)
        generator.shuffle(method_order)
        pairs.append(
            {
                'pair_id': pair_id,
                'object_pose_fixture': fixture,
                'order': method_order,
            }
        )
        for trial_order, method in enumerate(method_order, start=1):
            sequence += 1
            entries.append(
                {
                    'sequence': sequence,
                    'pair_id': pair_id,
                    'trial_order': trial_order,
                    'method': method,
                    'object_pose_fixture': fixture,
                }
            )
    return {
        'schema_version': SCHEMA_VERSION,
        'created_at': datetime.now(timezone.utc).isoformat(),
        'seed': seed,
        'n_pairs': len(pair_ids),
        'pairs': pairs,
        'entries': entries,
    }


def load_records(path):
    """Load JSON Lines or a JSON list/object of records."""
    with open(path, 'r', encoding='utf-8') as handle:
        text = handle.read().strip()
    if not text:
        return []
    if path.endswith('.jsonl'):
        records = []
        for line_number, line in enumerate(text.splitlines(), start=1):
            if not line.strip():
                continue
            try:
                records.append(json.loads(line))
            except json.JSONDecodeError as exc:
                raise ValueError(
                    '{}:{}: {}'.format(path, line_number, exc)
                )
        return records
    return _as_record_list(json.loads(text))


def _find_plan_entry(plan, pair_id, method):
    for entry in plan.get('entries') or []:
        if entry.get('pair_id') == pair_id and entry.get('method') == method:
            return entry
    return None


def append_record(path, record, plan=None):
    """Validate and append one record to an append-only JSONL file."""
    normalized = validate_record(record)
    if plan is not None:
        entry = _find_plan_entry(
            plan, normalized['pair_id'], normalized['method']
        )
        if entry is None:
            raise ValueError(
                '{} / {} is not in the randomization plan'.format(
                    normalized['pair_id'], normalized['method']
                )
            )
        if normalized['trial_order'] != entry['trial_order']:
            raise ValueError(
                'trial_order {} does not match the plan ({})'.format(
                    normalized['trial_order'], entry['trial_order']
                )
            )
        if (
            normalized['object_pose_fixture']
            != entry['object_pose_fixture']
        ):
            raise ValueError(
                'object_pose_fixture does not match the plan'
            )
    existing = load_records(path) if os.path.exists(path) else []
    for old in existing:
        if (
            old.get('pair_id') == normalized['pair_id']
            and old.get('method') == normalized['method']
        ):
            raise ValueError(
                '{} / {} is already recorded'.format(
                    normalized['pair_id'], normalized['method']
                )
            )
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, 'a', encoding='utf-8') as handle:
        handle.write(json.dumps(normalized) + '\n')
    return normalized


def _count_failures(records):
    counts = {}
    for record in records:
        code = record['failure_code']
        if code is not None:
            counts[code] = counts.get(code, 0) + 1
    return counts


def _method_summary(records):
    errors = [
        record['continuous_error_mm']
        for record in records
        if record['continuous_error_mm'] is not None
    ]
    successes = sum(1 for record in records if record['grasp_success'])
    return {
        'n': len(records),
        'error': _summary(errors),
        'successes': successes,
        'success_rate': successes / len(records) if records else None,
        'failure_codes': _count_failures(records),
    }


def analyze(records, seed=7, bootstrap_samples=10000, confidence=0.95):
    """Analyze complete matched pairs and return a JSON report."""
    if bootstrap_samples < 100:
        raise ValueError('bootstrap_samples must be at least 100')
    validation = validate_dataset(records)
    complete = []
    for pair_id in validation['complete_pairs']:
        group = [
            record
            for record in validation['records']
            if record['pair_id'] == pair_id
        ]
        by_method = {record['method']: record for record in group}
        complete.append(
            {
                'pair_id': pair_id,
                'baseline': by_method['baseline'],
                'intervention': by_method['intervention'],
            }
        )
    if not complete:
        raise ValueError(
            'no complete pairs; record one baseline and one intervention '
            'for at least one pair'
        )
    baseline_records = [pair['baseline'] for pair in complete]
    intervention_records = [pair['intervention'] for pair in complete]
    paired = []
    for pair in complete:
        baseline_error = pair['baseline']['continuous_error_mm']
        intervention_error = pair['intervention']['continuous_error_mm']
        if baseline_error is None or intervention_error is None:
            continue
        difference = intervention_error - baseline_error
        paired.append(
            {
                'pair_id': pair['pair_id'],
                'difference_mm': difference,
                'baseline_first': (
                    pair['baseline']['trial_order']
                    < pair['intervention']['trial_order']
                ),
            }
        )
    differences = [item['difference_mm'] for item in paired]
    difference_summary = _summary(differences)
    improvement = (
        -difference_summary['mean']
        if difference_summary['mean'] is not None
        else None
    )
    interval = _bootstrap_mean_ci(
        differences, bootstrap_samples, seed, confidence
    )
    wins = sum(1 for value in differences if value < -1e-12)
    losses = sum(1 for value in differences if value > 1e-12)
    ties = len(differences) - wins - losses
    sign_p = _exact_binomial_two_sided(wins, losses)
    effect_size = None
    if (
        difference_summary['std'] is not None
        and difference_summary['std'] > 0.0
    ):
        effect_size = (
            difference_summary['mean'] / difference_summary['std']
        )
    baseline_summary = _method_summary(baseline_records)
    intervention_summary = _method_summary(intervention_records)
    baseline_success = baseline_summary['success_rate']
    intervention_success = intervention_summary['success_rate']
    baseline_only = sum(
        1
        for pair in complete
        if pair['baseline']['grasp_success']
        and not pair['intervention']['grasp_success']
    )
    intervention_only = sum(
        1
        for pair in complete
        if not pair['baseline']['grasp_success']
        and pair['intervention']['grasp_success']
    )
    mcnemar_p = _exact_binomial_two_sided(
        intervention_only, baseline_only
    )
    order_groups = {
        'baseline_first': [
            item['difference_mm']
            for item in paired
            if item['baseline_first']
        ],
        'intervention_first': [
            item['difference_mm']
            for item in paired
            if not item['baseline_first']
        ],
    }
    order_effect = {}
    for name, values in order_groups.items():
        order_effect[name] = _summary(values)
    order_difference = None
    if (
        order_effect['baseline_first']['mean'] is not None
        and order_effect['intervention_first']['mean'] is not None
    ):
        order_difference = (
            order_effect['intervention_first']['mean']
            - order_effect['baseline_first']['mean']
        )
    baseline_first_count = sum(
        1
        for pair in complete
        if pair['baseline']['trial_order']
        < pair['intervention']['trial_order']
    )
    return {
        'schema_version': SCHEMA_VERSION,
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'seed': seed,
        'bootstrap_samples': bootstrap_samples,
        'confidence': confidence,
        'validation': {
            'n_input': validation['n_input'],
            'n_valid': validation['n_valid'],
            'n_invalid': validation['n_invalid'],
            'invalid': validation['invalid'],
            'n_complete_pairs': len(complete),
            'incomplete_pairs': validation['incomplete_pairs'],
        },
        'methods': {
            'baseline': baseline_summary,
            'intervention': intervention_summary,
        },
        'paired': {
            'n_pairs': len(complete),
            'n_pairs_with_error': len(paired),
            'difference_mm': difference_summary,
            'improvement_mm': improvement,
            'ci95_difference_mm': interval,
            'effect_size_dz': effect_size,
            'sign_test': {
                'wins': wins,
                'losses': losses,
                'ties': ties,
                'p_value': sign_p,
            },
        },
        'success': {
            'baseline_rate': baseline_success,
            'intervention_rate': intervention_success,
            'difference': (
                intervention_success - baseline_success
                if baseline_success is not None
                and intervention_success is not None
                else None
            ),
            'discordant': {
                'baseline_only': baseline_only,
                'intervention_only': intervention_only,
            },
            'mcnemar_exact_p': mcnemar_p,
        },
        'randomization': {
            'baseline_first': baseline_first_count,
            'intervention_first': len(complete) - baseline_first_count,
        },
        'order_effect': {
            'groups': order_effect,
            'difference_mm': order_difference,
            'note': 'observational; not a randomized crossover of time',
        },
        'limitations': LIMITATIONS,
    }


def render_markdown(report):
    """Render a paired-trial analysis as Markdown."""
    validation = report['validation']
    paired = report['paired']
    methods = report['methods']
    lines = [
        '# Paired Trial Analysis',
        '',
        '- Generated: `{}`'.format(report['generated_at']),
        '- Seed: {}; bootstrap draws: {}'.format(
            report['seed'], report['bootstrap_samples']
        ),
        '- Complete pairs: {}; input records: {}; invalid: {}'.format(
            validation['n_complete_pairs'],
            validation['n_input'],
            validation['n_invalid'],
        ),
        '',
        '## Method summary',
        '',
        '| Method | n | Error mean (mm) | Error SD (mm) | Success rate |',
        '|---|---:|---:|---:|---:|',
    ]
    for method in METHODS:
        summary = methods[method]
        error = summary['error']
        lines.append(
            '| {} | {} | {} | {} | {} |'.format(
                method,
                summary['n'],
                _format_number(error['mean']),
                _format_number(error['std']),
                _format_rate(summary['success_rate']),
            )
        )
    lines.extend(
        [
            '',
            '## Paired continuous error',
            '',
            '- Pairs with both errors: {}'.format(
                paired['n_pairs_with_error']
            ),
            '- Mean paired difference (intervention - baseline): {} mm'
            .format(_format_number(paired['difference_mm']['mean'])),
            '- Mean improvement (baseline - intervention): {} mm'.format(
                _format_number(paired['improvement_mm'])
            ),
            '- 95% CI of the difference: [{}, {}] mm'.format(
                _format_number(paired['ci95_difference_mm']['low']),
                _format_number(paired['ci95_difference_mm']['high']),
            ),
            '- Effect size (dz): {}'.format(
                _format_number(paired['effect_size_dz'])
            ),
            '- Sign test: {} wins / {} losses / {} ties; p = {}'.format(
                paired['sign_test']['wins'],
                paired['sign_test']['losses'],
                paired['sign_test']['ties'],
                _format_p(paired['sign_test']['p_value']),
            ),
            '',
            '## Paired success',
            '',
            '- Baseline: {}'.format(
                _format_rate(report['success']['baseline_rate'])
            ),
            '- Intervention: {}'.format(
                _format_rate(report['success']['intervention_rate'])
            ),
            '- Discordant: baseline-only {}; intervention-only {}'.format(
                report['success']['discordant']['baseline_only'],
                report['success']['discordant']['intervention_only'],
            ),
            '- McNemar exact p: {}'.format(
                _format_p(report['success']['mcnemar_exact_p'])
            ),
            '',
            '## Randomization and order',
            '',
            '- Baseline first: {}; intervention first: {}'.format(
                report['randomization']['baseline_first'],
                report['randomization']['intervention_first'],
            ),
            '- Order difference (intervention-first minus baseline-first): '
            '{} mm'.format(
                _format_number(report['order_effect']['difference_mm'])
            ),
            '- {}'.format(report['order_effect']['note']),
            '',
            '## Failure codes',
            '',
            '| Method | Codes |',
            '|---|---|',
        ]
    )
    for method in METHODS:
        codes = methods[method]['failure_codes']
        text = ', '.join(
            '{}={}'.format(code, count)
            for code, count in sorted(codes.items())
        )
        lines.append('| {} | {} |'.format(method, text or '-'))
    incomplete = validation['incomplete_pairs']
    if incomplete:
        lines.extend(
            [
                '',
                '## Incomplete pairs',
                '',
            ]
        )
        for item in incomplete:
            lines.append(
                '- `{}` (methods: {})'.format(
                    item['pair_id'], ', '.join(item['methods'])
                )
            )
    lines.extend(['', '## Limitations', ''])
    for item in report['limitations']:
        lines.append('- {}'.format(item))
    lines.append('')
    return '\n'.join(lines)


def _format_number(value, digits=3):
    if value is None:
        return 'n/a'
    return '{:.{digits}f}'.format(value, digits=digits)


def _format_rate(value):
    if value is None:
        return 'n/a'
    return '{:.1f}%'.format(value * 100.0)


def _format_p(value):
    if value is None:
        return 'n/a'
    if value < 1e-4:
        return '{:.2e}'.format(value)
    return '{:.4f}'.format(value)


def _write_json(path, payload):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as handle:
        json.dump(payload, handle, indent=2)
        handle.write('\n')


def _write_text(path, text):
    directory = os.path.dirname(os.path.abspath(path))
    if directory:
        os.makedirs(directory, exist_ok=True)
    with open(path, 'w', encoding='utf-8') as handle:
        handle.write(text)


def _print_analysis(report):
    validation = report['validation']
    paired = report['paired']
    print(
        'PAIRED_TRIALS pairs={} records={} invalid={}'.format(
            validation['n_complete_pairs'],
            validation['n_input'],
            validation['n_invalid'],
        )
    )
    print(
        'CONTINUOUS n={} mean_diff_mm={} improvement_mm={} ci95=[{}, {}] '
        'sign_p={}'.format(
            paired['n_pairs_with_error'],
            _format_number(paired['difference_mm']['mean']),
            _format_number(paired['improvement_mm']),
            _format_number(paired['ci95_difference_mm']['low']),
            _format_number(paired['ci95_difference_mm']['high']),
            _format_p(paired['sign_test']['p_value']),
        )
    )
    print(
        'SUCCESS baseline={} intervention={} mcnemar_p={}'.format(
            _format_rate(report['success']['baseline_rate']),
            _format_rate(report['success']['intervention_rate']),
            _format_p(report['success']['mcnemar_exact_p']),
        )
    )
    if validation['n_invalid']:
        print('RESULT FAIL')
        return 1
    if validation['incomplete_pairs']:
        print('RESULT PARTIAL')
        return 0
    print('RESULT PASS')
    return 0


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            'Plan, record and analyze matched baseline/intervention '
            'grasp trials. Recording is append-only.'
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    subparsers = parser.add_subparsers(dest='command', required=True)
    plan = subparsers.add_parser('plan')
    plan.add_argument('--pairs', type=int, default=20)
    plan.add_argument('--seed', type=int, default=7)
    plan.add_argument(
        '--fixtures',
        default=None,
        help='comma-separated fixture names (one or one per pair)',
    )
    plan.add_argument('--json-out', default=None)
    record = subparsers.add_parser('record')
    record.add_argument('--jsonl-out', required=True)
    record.add_argument('--plan', default=None)
    record.add_argument('--pair-id', required=True)
    record.add_argument('--method', required=True, choices=METHODS)
    record.add_argument('--trial-order', type=int, default=None)
    record.add_argument('--fixture', default=None)
    record.add_argument('--error-mm', type=float, default=None)
    outcome = record.add_mutually_exclusive_group(required=True)
    outcome.add_argument('--success', action='store_true')
    outcome.add_argument('--failure-code', default=None)
    record.add_argument('--notes', default=None)
    validate = subparsers.add_parser('validate')
    validate.add_argument('--input', required=True)
    validate.add_argument('--json-out', default=None)
    analyze_parser = subparsers.add_parser('analyze')
    analyze_parser.add_argument('--input', required=True)
    analyze_parser.add_argument('--json-out', default=None)
    analyze_parser.add_argument('--markdown-out', default=None)
    analyze_parser.add_argument('--seed', type=int, default=7)
    analyze_parser.add_argument('--bootstrap', type=int, default=10000)
    analyze_parser.add_argument('--confidence', type=float, default=0.95)
    simulate = subparsers.add_parser('simulate')
    simulate.add_argument('--pairs', type=int, default=20)
    simulate.add_argument('--effect-mm', type=float, default=1.0)
    simulate.add_argument('--noise-mm', type=float, default=0.5)
    simulate.add_argument('--baseline-error-mm', type=float, default=2.0)
    simulate.add_argument('--baseline-success', type=float, default=0.6)
    simulate.add_argument('--success-gain', type=float, default=0.2)
    simulate.add_argument('--seed', type=int, default=7)
    simulate.add_argument('--fixtures', default=None)
    simulate.add_argument('--json-out', default=None)
    simulate.add_argument('--jsonl-out', default=None)
    simulate.add_argument('--markdown-out', default=None)
    return parser.parse_args(argv)


def _fixtures_from_args(text):
    if text is None:
        return None
    return [item.strip() for item in text.split(',') if item.strip()]


def _simulate(args):
    generator = random.Random(args.seed + 1000)
    pair_ids = [
        'p{:03d}'.format(index + 1) for index in range(args.pairs)
    ]
    plan = build_plan(
        pair_ids,
        fixtures=_fixtures_from_args(args.fixtures),
        seed=args.seed,
    )
    errors = {}
    for pair_id in pair_ids:
        baseline_error = max(
            0.0,
            args.baseline_error_mm + generator.gauss(0.0, args.noise_mm),
        )
        intervention_error = max(
            0.0,
            baseline_error
            - args.effect_mm
            + generator.gauss(0.0, args.noise_mm / 3.0),
        )
        errors[pair_id] = {
            'baseline': baseline_error,
            'intervention': intervention_error,
        }
    records = []
    for entry in plan['entries']:
        error = errors[entry['pair_id']][entry['method']]
        probability = args.baseline_success
        if entry['method'] == 'intervention':
            probability += args.success_gain
        success = generator.random() < probability
        record = {
            'schema_version': SCHEMA_VERSION,
            'pair_id': entry['pair_id'],
            'trial_order': entry['trial_order'],
            'method': entry['method'],
            'object_pose_fixture': entry['object_pose_fixture'],
            'continuous_error_mm': round(error, 3),
            'grasp_success': success,
            'failure_code': None if success else 'MISS',
            'recorded_at': datetime.now(timezone.utc).isoformat(),
            'notes': 'synthetic simulation',
        }
        records.append(validate_record(record))
    return plan, records


def _command_plan(args):
    if args.pairs < 1:
        raise ValueError('--pairs must be at least 1')
    pair_ids = [
        'p{:03d}'.format(index + 1) for index in range(args.pairs)
    ]
    plan = build_plan(
        pair_ids,
        fixtures=_fixtures_from_args(args.fixtures),
        seed=args.seed,
    )
    if args.json_out:
        _write_json(args.json_out, plan)
    print(
        'PLAN pairs={} entries={} seed={}'.format(
            plan['n_pairs'], len(plan['entries']), plan['seed']
        )
    )
    return 0


def _command_record(args):
    plan = None
    if args.plan:
        with open(args.plan, 'r', encoding='utf-8') as handle:
            plan = json.load(handle)
    trial_order = args.trial_order
    fixture = args.fixture
    if plan is not None:
        entry = _find_plan_entry(plan, args.pair_id, args.method)
        if entry is None:
            raise ValueError(
                '{} / {} is not in the randomization plan'.format(
                    args.pair_id, args.method
                )
            )
        if trial_order is None:
            trial_order = entry['trial_order']
        if fixture is None:
            fixture = entry['object_pose_fixture']
    if trial_order is None:
        raise ValueError('--trial-order is required without --plan')
    if fixture is None:
        raise ValueError('--fixture is required without --plan')
    record = {
        'pair_id': args.pair_id,
        'trial_order': trial_order,
        'method': args.method,
        'object_pose_fixture': fixture,
        'continuous_error_mm': args.error_mm,
        'grasp_success': bool(args.success),
        'failure_code': args.failure_code,
        'recorded_at': datetime.now(timezone.utc).isoformat(),
    }
    if args.notes is not None:
        record['notes'] = args.notes
    append_record(args.jsonl_out, record, plan=plan)
    print(
        'RECORD {} {} trial_order={} success={}'.format(
            args.pair_id, args.method, trial_order, args.success
        )
    )
    return 0


def _command_validate(args):
    report = validate_dataset(load_records(args.input))
    if args.json_out:
        _write_json(args.json_out, report)
    print(
        'VALIDATE input={} valid={} invalid={} pairs={} '
        'incomplete={}'.format(
            report['n_input'],
            report['n_valid'],
            report['n_invalid'],
            len(report['complete_pairs']),
            len(report['incomplete_pairs']),
        )
    )
    if report['n_invalid']:
        print('RESULT FAIL')
        return 1
    if report['incomplete_pairs']:
        print('RESULT PARTIAL')
        return 0
    print('RESULT PASS')
    return 0


def _command_analyze(args):
    report = analyze(
        load_records(args.input),
        seed=args.seed,
        bootstrap_samples=args.bootstrap,
        confidence=args.confidence,
    )
    if args.json_out:
        _write_json(args.json_out, report)
    if args.markdown_out:
        _write_text(args.markdown_out, render_markdown(report))
    return _print_analysis(report)


def _command_simulate(args):
    if args.pairs < 1:
        raise ValueError('--pairs must be at least 1')
    plan, records = _simulate(args)
    report = analyze(
        records, seed=args.seed, bootstrap_samples=10000
    )
    if args.json_out:
        _write_json(
            args.json_out,
            {
                'plan': plan,
                'records': records,
                'analysis': report,
            },
        )
    if args.jsonl_out:
        directory = os.path.dirname(os.path.abspath(args.jsonl_out))
        if directory:
            os.makedirs(directory, exist_ok=True)
        with open(args.jsonl_out, 'w', encoding='utf-8') as handle:
            for record in records:
                handle.write(json.dumps(record) + '\n')
    if args.markdown_out:
        _write_text(args.markdown_out, render_markdown(report))
    return _print_analysis(report)


def main(argv=None):
    """Run the command-line entry point."""
    args = _parse_args(argv)
    try:
        if args.command == 'plan':
            return _command_plan(args)
        if args.command == 'record':
            return _command_record(args)
        if args.command == 'validate':
            return _command_validate(args)
        if args.command == 'analyze':
            return _command_analyze(args)
        if args.command == 'simulate':
            return _command_simulate(args)
    except (OSError, ValueError) as exc:
        print('error: {}'.format(exc), file=sys.stderr)
        return 2
    print('error: unknown command', file=sys.stderr)
    return 2


if __name__ == '__main__':
    sys.exit(main())
