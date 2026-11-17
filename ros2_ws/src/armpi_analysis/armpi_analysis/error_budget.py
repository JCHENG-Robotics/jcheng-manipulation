#!/usr/bin/env python3
"""
Error-budget aggregation and reporting for the ArmPi grasp pipeline.

The module is pure Python so it can run on the robot, in the Humble/Jazzy
containers, or on a laptop without ROS. It accepts a JSON budget with one
entry per error source, aggregates the measured entries per metric, and
renders a report. A source that has not been measured is kept in the budget
as an explicit gap; it is never silently replaced by a placeholder value.

The aggregation is deliberately simple and auditable:

* ``rss`` is the root-sum-square of all measured source magnitudes;
* ``worst_case`` is their linear sum;
* the 95% interval is a seeded Monte Carlo of the aggregate, and is only
  reported when every random/bounded source has a distribution.

An error budget is an engineering estimate, not a substitute for a physical
tolerance experiment. The framework also reports how complete the input is.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os
import random
import sys
SCHEMA_VERSION = 1
METRICS = ('translation_mm', 'rotation_deg')
KINDS = ('random', 'bounded', 'systematic')
DISTRIBUTIONS = ('normal', 'uniform')
CATEGORIES = (
    'calibration',
    'perception',
    'repeatability',
    'backlash',
    'execution',
    'gripper',
    'model',
    'other',
)
STATUSES = ('measured', 'unmeasured')
_Z_95 = 1.959963984540054
DEFAULT_LIMITATIONS = [
    'RSS assumes independent sources; correlated sources would be '
    'over-counted.',
    'Worst case is the linear sum of measured magnitudes and is '
    'conservative.',
    'Unmeasured sources are excluded from the totals; a partial budget is '
    'not a validated task error.',
    'A Monte Carlo interval is only reported when every random/bounded '
    'source has a distribution.',
    'The tolerance is a provisional gate unless a physical tolerance '
    'experiment has been recorded.',
]


def _is_number(value):
    return (
        isinstance(value, (int, float))
        and not isinstance(value, bool)
        and math.isfinite(float(value))
    )


def _normalize_source(source, index):
    """Validate one source and fill optional fields."""
    if not isinstance(source, dict):
        raise ValueError('source {} must be an object'.format(index))
    normalized = dict(source)
    source_id = source.get('id')
    if not isinstance(source_id, str) or not source_id:
        raise ValueError('source {} has no id'.format(index))
    metric = source.get('metric')
    if metric not in METRICS:
        raise ValueError(
            '{}: metric must be one of {}'.format(source_id, METRICS)
        )
    category = source.get('category', 'other')
    if category not in CATEGORIES:
        raise ValueError(
            '{}: category must be one of {}'.format(source_id, CATEGORIES)
        )
    kind = source.get('kind', 'bounded')
    if kind not in KINDS:
        raise ValueError(
            '{}: kind must be one of {}'.format(source_id, KINDS)
        )
    status = source.get('status')
    if status is None:
        status = (
            'measured' if source.get('value') is not None else 'unmeasured'
        )
    if status not in STATUSES:
        raise ValueError(
            '{}: status must be one of {}'.format(source_id, STATUSES)
        )
    basis = source.get('basis')
    distribution = source.get('distribution')
    if distribution in ('', None):
        distribution = None
    elif distribution not in DISTRIBUTIONS:
        raise ValueError(
            '{}: distribution must be one of {}'.format(
                source_id, DISTRIBUTIONS
            )
        )
    if kind == 'systematic' and distribution is not None:
        raise ValueError(
            '{}: a systematic source cannot have a distribution'.format(
                source_id
            )
        )
    normalized['id'] = source_id
    normalized['metric'] = metric
    normalized['category'] = category
    normalized['kind'] = kind
    normalized['status'] = status
    normalized['required'] = bool(source.get('required', True))
    normalized['distribution'] = distribution
    normalized['basis'] = basis
    normalized['evidence'] = source.get('evidence')
    normalized['notes'] = source.get('notes')
    if status == 'measured':
        value = source.get('value')
        if not _is_number(value) or float(value) < 0.0:
            raise ValueError(
                '{}: a measured source needs a finite non-negative '
                'value'.format(source_id)
            )
        if not isinstance(basis, str) or not basis:
            raise ValueError(
                '{}: a measured source needs a basis'.format(source_id)
            )
        normalized['value'] = float(value)
        sigma = source.get('sigma')
        if distribution == 'normal':
            if sigma is not None:
                if not _is_number(sigma) or float(sigma) <= 0.0:
                    raise ValueError(
                        '{}: sigma must be positive'.format(source_id)
                    )
                normalized['sigma'] = float(sigma)
            elif basis in ('std', 'ci95'):
                normalized['sigma'] = None
            else:
                raise ValueError(
                    '{}: normal distribution needs sigma or a std/ci95 '
                    'basis'.format(source_id)
                )
        else:
            normalized['sigma'] = None
    else:
        if source.get('value') is not None:
            raise ValueError(
                '{}: an unmeasured source must not have a value'.format(
                    source_id
                )
            )
        normalized['value'] = None
        normalized['sigma'] = None
    return normalized


def validate_budget(budget):
    """Return a normalized copy of a budget or raise ``ValueError``."""
    if not isinstance(budget, dict):
        raise ValueError('budget must be an object')
    if budget.get('schema_version') != SCHEMA_VERSION:
        raise ValueError(
            'schema_version must be {}'.format(SCHEMA_VERSION)
        )
    name = budget.get('name')
    if not isinstance(name, str) or not name:
        raise ValueError('budget needs a name')
    raw_sources = budget.get('sources')
    if not isinstance(raw_sources, list) or not raw_sources:
        raise ValueError('budget needs a non-empty sources list')
    sources = []
    seen = set()
    for index, raw_source in enumerate(raw_sources):
        source = _normalize_source(raw_source, index)
        if source['id'] in seen:
            raise ValueError('duplicate source id {}'.format(source['id']))
        seen.add(source['id'])
        sources.append(source)
    raw_tolerance = budget.get('tolerance') or {}
    if not isinstance(raw_tolerance, dict):
        raise ValueError('tolerance must be an object')
    tolerance = {}
    for metric, value in raw_tolerance.items():
        if metric not in METRICS:
            raise ValueError(
                'tolerance metric {} is not one of {}'.format(
                    metric, METRICS
                )
            )
        if not _is_number(value) or float(value) <= 0.0:
            raise ValueError(
                'tolerance {} must be positive'.format(metric)
            )
        tolerance[metric] = float(value)
    normalized = dict(budget)
    normalized['schema_version'] = SCHEMA_VERSION
    normalized['name'] = name
    normalized['sources'] = sources
    normalized['tolerance'] = tolerance
    return normalized


def load_budget(path):
    """Load and validate a JSON budget."""
    with open(path, 'r', encoding='utf-8') as handle:
        return validate_budget(json.load(handle))


def _source_sigma(source):
    """Return the standard deviation used for a normal draw."""
    if source.get('sigma') is not None:
        return float(source['sigma'])
    if source['basis'] == 'std':
        return float(source['value'])
    if source['basis'] == 'ci95':
        return float(source['value']) / _Z_95
    raise ValueError(
        '{}: cannot infer a normal sigma'.format(source['id'])
    )


def _percentile(values, fraction):
    """Linear-interpolated percentile of a non-empty sequence."""
    ordered = sorted(values)
    if len(ordered) == 1:
        return ordered[0]
    rank = (len(ordered) - 1) * fraction
    lower = int(math.floor(rank))
    upper = int(math.ceil(rank))
    if lower == upper:
        return ordered[lower]
    weight = rank - lower
    return ordered[lower] * (1.0 - weight) + ordered[upper] * weight


def _monte_carlo_rss(sources, samples, seed):
    """Draw aggregate RSS values from independent source distributions."""
    generator = random.Random(seed)
    values = []
    for _ in range(samples):
        total = 0.0
        for source in sources:
            if source['distribution'] == 'normal':
                draw = generator.gauss(0.0, _source_sigma(source))
            elif source['distribution'] == 'uniform':
                draw = generator.uniform(
                    -float(source['value']), float(source['value'])
                )
            else:
                draw = float(source['value'])
            total += draw * draw
        values.append(math.sqrt(total))
    return values


def _aggregate_metric(sources, metric, tolerance, samples, seed):
    """Aggregate the sources for one metric."""
    metric_sources = [
        source for source in sources if source['metric'] == metric
    ]
    measured = [
        source
        for source in metric_sources
        if source['status'] == 'measured'
    ]
    missing = [
        source
        for source in metric_sources
        if source['status'] == 'unmeasured' and source['required']
    ]
    optional_missing = [
        source
        for source in metric_sources
        if source['status'] == 'unmeasured' and not source['required']
    ]
    rss = math.sqrt(
        sum(float(source['value']) ** 2 for source in measured)
    )
    worst_case = sum(float(source['value']) for source in measured)
    contributions = []
    for source in measured:
        value = float(source['value'])
        contribution = (
            value * value / (rss * rss) * 100.0 if rss > 0.0 else 0.0
        )
        removed = math.sqrt(max(rss * rss - value * value, 0.0))
        contributions.append(
            {
                'id': source['id'],
                'value': value,
                'percent': contribution,
                'rss_without': removed,
                'reduction': rss - removed,
            }
        )
    dominant = None
    if contributions:
        dominant = max(contributions, key=lambda item: item['value'])['id']
    can_sample = bool(measured)
    for source in measured:
        if source['kind'] == 'systematic':
            continue
        if source['distribution'] not in DISTRIBUTIONS:
            can_sample = False
            break
    interval = None
    if can_sample:
        draws = _monte_carlo_rss(measured, samples, seed)
        interval = {
            'samples': samples,
            'seed': seed,
            'mean': sum(draws) / len(draws),
            'low': _percentile(draws, 0.025),
            'high': _percentile(draws, 0.975),
        }
    result = {
        'metric': metric,
        'n_sources': len(metric_sources),
        'n_measured': len(measured),
        'n_missing_required': len(missing),
        'complete': bool(measured) and not missing,
        'rss': rss,
        'worst_case': worst_case,
        'dominant': dominant,
        'contributions': contributions,
        'ci95': interval,
        'sources': metric_sources,
        'missing_required': [source['id'] for source in missing],
        'optional_missing': [source['id'] for source in optional_missing],
        'tolerance': tolerance,
        'pass_rss': None,
        'pass_worst_case': None,
        'margin': None,
        'budget_ratio': None,
    }
    if tolerance is not None:
        result['pass_rss'] = rss <= tolerance + 1e-12
        result['pass_worst_case'] = worst_case <= tolerance + 1e-12
        result['margin'] = tolerance - rss
        result['budget_ratio'] = rss / tolerance
    return result


def aggregate(budget, samples=10000, seed=7):
    """Aggregate a validated budget and return a JSON-serializable report."""
    if samples < 100:
        raise ValueError('samples must be at least 100')
    normalized = validate_budget(budget)
    metrics = {}
    for metric in METRICS:
        has_sources = any(
            source['metric'] == metric for source in normalized['sources']
        )
        if not has_sources and metric not in normalized['tolerance']:
            continue
        metrics[metric] = _aggregate_metric(
            normalized['sources'],
            metric,
            normalized['tolerance'].get(metric),
            samples,
            seed,
        )
    if not metrics:
        raise ValueError('budget has no metric sources or tolerances')
    complete = all(result['complete'] for result in metrics.values())
    limitations = list(normalized.get('limitations') or [])
    for item in DEFAULT_LIMITATIONS:
        if item not in limitations:
            limitations.append(item)
    return {
        'schema_version': SCHEMA_VERSION,
        'name': normalized['name'],
        'task': normalized.get('task'),
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'status': 'complete' if complete else 'partial',
        'complete': complete,
        'samples': samples,
        'seed': seed,
        'tolerance_basis': normalized.get('tolerance_basis'),
        'tolerance': normalized['tolerance'],
        'metrics': metrics,
        'limitations': limitations,
    }


def _format_number(value, digits=3):
    if value is None:
        return 'n/a'
    return '{:.{digits}f}'.format(value, digits=digits)


def _format_cell(value):
    if value is None or value == '':
        return '-'
    return str(value).replace('|', '\\|')


def render_markdown(report):
    """Render an aggregate report as Markdown."""
    lines = [
        '# Error Budget — {}'.format(report['name']),
        '',
        '- Generated: `{}`'.format(report['generated_at']),
        '- Task: {}'.format(report.get('task') or 'n/a'),
        '- Status: **{}**'.format(report['status'].upper()),
        '- Samples: {} (seed {})'.format(
            report['samples'], report['seed']
        ),
    ]
    if report.get('tolerance_basis'):
        lines.append(
            '- Tolerance basis: {}'.format(report['tolerance_basis'])
        )
    lines.append('')
    for metric, result in report['metrics'].items():
        unit = 'mm' if metric == 'translation_mm' else 'deg'
        lines.extend(
            [
                '## {}'.format(metric),
                '',
                '| Source | Category | Kind | Value ({}) | Basis | '
                'Contribution | Evidence |'.format(unit),
                '|---|---|---|---:|---|---:|---|',
            ]
        )
        for source in result['sources']:
            if source['status'] == 'measured':
                value = _format_number(source['value'])
                contribution = next(
                    (
                        _format_number(item['percent'], 1) + '%'
                        for item in result['contributions']
                        if item['id'] == source['id']
                    ),
                    '-',
                )
            else:
                value = 'unmeasured'
                contribution = '-'
            lines.append(
                '| `{}` | {} | {} | {} | {} | {} | {} |'.format(
                    source['id'],
                    source['category'],
                    source['kind'],
                    value,
                    _format_cell(source.get('basis')),
                    contribution,
                    _format_cell(source.get('evidence')),
                )
            )
        lines.extend(
            [
                '',
                '- RSS: **{} {}**'.format(
                    _format_number(result['rss']), unit
                ),
                '- Worst case: **{} {}**'.format(
                    _format_number(result['worst_case']), unit
                ),
            ]
        )
        if result['tolerance'] is not None:
            lines.append(
                '- Tolerance: {} {}; RSS pass **{}**; worst-case pass '
                '**{}**; margin {} {}'.format(
                    _format_number(result['tolerance']),
                    unit,
                    result['pass_rss'],
                    result['pass_worst_case'],
                    _format_number(result['margin']),
                    unit,
                )
            )
        if result['ci95'] is None:
            lines.append(
                '- 95% interval: not computed (a random/bounded source '
                'has no distribution).'
            )
        else:
            lines.append(
                '- 95% interval: [{}, {}] {} (Monte Carlo, {} draws, '
                'seed {}).'.format(
                    _format_number(result['ci95']['low']),
                    _format_number(result['ci95']['high']),
                    unit,
                    result['ci95']['samples'],
                    result['ci95']['seed'],
                )
            )
        if result['dominant']:
            lines.append('- Dominant source: `{}`'.format(
                result['dominant']
            ))
        if result['missing_required']:
            lines.append(
                '- **Missing required sources:** {}'.format(
                    ', '.join(
                        '`{}`'.format(item)
                        for item in result['missing_required']
                    )
                )
            )
        if result['optional_missing']:
            lines.append(
                '- Optional unmeasured sources: {}'.format(
                    ', '.join(
                        '`{}`'.format(item)
                        for item in result['optional_missing']
                    )
                )
            )
        lines.append('')
    lines.extend(['## Limitations', ''])
    for item in report['limitations']:
        lines.append('- {}'.format(item))
    lines.append('')
    if not report['complete']:
        lines.append(
            '> **Partial budget.** Required sources are unmeasured; the '
            'totals are lower bounds for those metrics, not a complete '
            'task error estimate.'
        )
        lines.append('')
    return '\n'.join(lines)


def template_budget():
    """Return a minimal budget template."""
    return {
        'schema_version': SCHEMA_VERSION,
        'name': 'replace_me',
        'task': 'fiducial_grasp',
        'tolerance': {
            'translation_mm': 5.0,
            'rotation_deg': 2.0,
        },
        'tolerance_basis': 'provisional gate; replace after a physical '
                           'tolerance experiment',
        'sources': [
            {
                'id': 'measured_example',
                'category': 'calibration',
                'metric': 'translation_mm',
                'value': 1.0,
                'kind': 'bounded',
                'basis': 'std',
                'status': 'measured',
                'distribution': 'normal',
                'sigma': 1.0,
                'evidence': 'evidence/path/to/report',
                'notes': 'replace with a measured source',
            },
            {
                'id': 'unmeasured_example',
                'category': 'repeatability',
                'metric': 'translation_mm',
                'value': None,
                'kind': 'bounded',
                'basis': None,
                'status': 'unmeasured',
                'required': True,
                'evidence': None,
                'notes': 'run the repeatability protocol first',
            },
        ],
        'limitations': ['template only'],
    }


def demo_budget():
    """Return a synthetic budget with known aggregate values."""
    return {
        'schema_version': SCHEMA_VERSION,
        'name': 'synthetic_error_budget_demo',
        'task': 'synthetic_validation',
        'tolerance': {
            'translation_mm': 4.0,
            'rotation_deg': 1.0,
        },
        'tolerance_basis': 'synthetic only',
        'sources': [
            {
                'id': 'calibration_translation',
                'category': 'calibration',
                'metric': 'translation_mm',
                'value': 2.0,
                'kind': 'random',
                'basis': 'std',
                'distribution': 'normal',
                'sigma': 2.0,
                'evidence': 'synthetic',
            },
            {
                'id': 'perception_translation',
                'category': 'perception',
                'metric': 'translation_mm',
                'value': 1.0,
                'kind': 'random',
                'basis': 'std',
                'distribution': 'normal',
                'sigma': 1.0,
                'evidence': 'synthetic',
            },
            {
                'id': 'repeatability_translation',
                'category': 'repeatability',
                'metric': 'translation_mm',
                'value': 0.5,
                'kind': 'random',
                'basis': 'std',
                'distribution': 'normal',
                'sigma': 0.5,
                'evidence': 'synthetic',
            },
            {
                'id': 'guard_translation',
                'category': 'other',
                'metric': 'translation_mm',
                'value': 0.3,
                'kind': 'bounded',
                'basis': 'max',
                'distribution': 'uniform',
                'evidence': 'synthetic',
            },
            {
                'id': 'calibration_rotation',
                'category': 'calibration',
                'metric': 'rotation_deg',
                'value': 0.4,
                'kind': 'random',
                'basis': 'std',
                'distribution': 'normal',
                'sigma': 0.4,
                'evidence': 'synthetic',
            },
            {
                'id': 'perception_rotation',
                'category': 'perception',
                'metric': 'rotation_deg',
                'value': 0.2,
                'kind': 'random',
                'basis': 'std',
                'distribution': 'normal',
                'sigma': 0.2,
                'evidence': 'synthetic',
            },
        ],
        'limitations': ['Synthetic input; not a hardware error budget.'],
    }


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


def _format_interval(interval):
    if interval is None:
        return 'n/a'
    return '[{:.3f}, {:.3f}]'.format(interval['low'], interval['high'])


def _print_summary(report):
    print(
        'ERROR_BUDGET name={} status={}'.format(
            report['name'], report['status']
        )
    )
    failed = False
    for metric, result in report['metrics'].items():
        print(
            'METRIC {} rss={:.3f} worst={:.3f} tolerance={} '
            'pass_rss={} pass_worst_case={} ci95={} dominant={}'.format(
                metric,
                result['rss'],
                result['worst_case'],
                _format_number(result['tolerance']),
                result['pass_rss'],
                result['pass_worst_case'],
                _format_interval(result['ci95']),
                result['dominant'],
            )
        )
        if result['pass_rss'] is False:
            failed = True
        if result['pass_worst_case'] is False:
            failed = True
        if result['missing_required']:
            print(
                'MISSING {} required={}'.format(
                    metric, ','.join(result['missing_required'])
                )
            )
    if report['status'] != 'complete':
        print('RESULT PARTIAL')
        return 0
    if failed:
        print('RESULT FAIL')
        return 1
    print('RESULT PASS')
    return 0


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            'Aggregate a JSON error budget and render a report. '
            'Unmeasured sources stay explicit; no value is invented.'
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    group = parser.add_mutually_exclusive_group(required=True)
    group.add_argument('--input', help='validated budget JSON')
    group.add_argument(
        '--template',
        action='store_true',
        help='print a budget template and exit',
    )
    group.add_argument(
        '--demo',
        action='store_true',
        help='aggregate the synthetic demo budget',
    )
    parser.add_argument('--json-out', default=None)
    parser.add_argument('--markdown-out', default=None)
    parser.add_argument('--samples', type=int, default=10000)
    parser.add_argument('--seed', type=int, default=7)
    return parser.parse_args(argv)


def main(argv=None):
    """Run the command-line entry point."""
    args = _parse_args(argv)
    if args.template:
        budget = template_budget()
        if args.json_out:
            _write_json(args.json_out, budget)
            print('TEMPLATE written={}'.format(args.json_out))
        else:
            json.dump(budget, sys.stdout, indent=2)
            sys.stdout.write('\n')
        return 0
    try:
        budget = demo_budget() if args.demo else load_budget(args.input)
        report = aggregate(budget, samples=args.samples, seed=args.seed)
    except (OSError, ValueError) as exc:
        print('error: {}'.format(exc), file=sys.stderr)
        return 2
    if args.json_out:
        _write_json(args.json_out, report)
    if args.markdown_out:
        _write_text(args.markdown_out, render_markdown(report))
    return _print_summary(report)


if __name__ == '__main__':
    sys.exit(main())
