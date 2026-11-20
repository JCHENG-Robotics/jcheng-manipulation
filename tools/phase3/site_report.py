#!/usr/bin/env python3
"""Render the 24-site capability and paired outcome table.

The site-diversity runner writes two append-only files:

- ``approaches.jsonl``: one locate/approach/return record per attempted pair,
  including an ``error`` when the pair aborted;
- ``trials.jsonl``: the paired-trial records from the validated recorder.

This tool joins them on ``pair_id``, takes the **last** approach attempt per
pair, and writes a markdown table plus a capability summary. It is a reporting
tool: it does not move the robot, does not change the records, and does not
recompute the paired statistics (those come from
``armpi_analysis.paired_trials analyze``).

Run after a session::

    python3 tools/phase3/site_report.py \
        --approaches evidence/phase3/onsite_20261120_sites/approaches.jsonl \
        --trials evidence/phase3/onsite_20261120_sites/trials.jsonl \
        --markdown-out evidence/phase3/onsite_20261120_sites/site_report.md
"""

import argparse
import json
import math
import os


def load_jsonl(path):
    """Return the non-empty JSON Lines records of a file (empty if absent)."""
    if not os.path.exists(path):
        return []
    records = []
    with open(path, 'r', encoding='utf-8') as handle:
        for line in handle:
            line = line.strip()
            if line:
                records.append(json.loads(line))
    return records


def latest_approaches(records):
    """Return one (pair_id, approach record) per pair, in first-seen order."""
    order = []
    latest = {}
    for record in records:
        pair_id = record['pair_id']
        if pair_id not in latest:
            order.append(pair_id)
        latest[pair_id] = record
    return [(pair_id, latest[pair_id]) for pair_id in order]


def trial_map(records):
    """Index the trial records by ``(pair_id, method)``."""
    return {(record['pair_id'], record['method']): record
            for record in records}


def radius_bearing(point):
    """Return ``(radius_m, bearing_deg)`` for a base-frame x/y point."""
    if not point:
        return None, None
    radius = math.hypot(float(point[0]), float(point[1]))
    bearing = math.degrees(math.atan2(float(point[1]), float(point[0])))
    return radius, bearing


def method_cell(record):
    """Return a short PASS/FAIL/— cell for one trial record."""
    if record is None:
        return '—'
    if record.get('grasp_success'):
        return 'PASS'
    return 'FAIL' if record.get('failure_code') is None else str(
        record['failure_code'])


def placement_cell(record):
    """Return the placement error cell for one trial record."""
    if record is None or record.get('continuous_error_mm') is None:
        return '—'
    return '{:.2f}'.format(float(record['continuous_error_mm']))


def approach_row(pair_id, record, trials):
    """Return one markdown row and a capability dict for a pair."""
    site = record.get('site') or {}
    approach = record.get('approach') or {}
    initial = approach.get('initial_point')
    initial_radius, initial_bearing = radius_bearing(initial)
    final = approach.get('final_point')
    envelope = approach.get('final_envelope') or {}
    pulses = approach.get('base_pulses') or []
    baseline = trials.get((pair_id, 'baseline'))
    intervention = trials.get((pair_id, 'intervention'))
    error = approach.get('error')
    if error:
        state = 'aborted: {}'.format(error)
    elif not envelope.get('x_ok') or not envelope.get('j1_ok'):
        state = 'outside envelope'
    elif final is None:
        state = 'not approached'
    else:
        state = 'ok'
    placement = intervention if intervention is not None else baseline
    row = {
        'pair_id': pair_id,
        'site_id': site.get('site_id', '?'),
        'site_radius_m': site.get('radius_m'),
        'site_bearing_deg': site.get('bearing_deg'),
        'initial_radius_m': initial_radius,
        'initial_bearing_deg': initial_bearing,
        'pulses': len(pulses),
        'final_x_m': None if final is None else float(final[0]),
        'final_j1_rad': approach.get('final_j1_rad'),
        'search_yaw_rad': approach.get('search_yaw_rad'),
        'baseline': method_cell(baseline),
        'intervention': method_cell(intervention),
        'placement_mm': placement_cell(placement),
        'state': state,
    }
    capability = {
        'pair_id': pair_id,
        'site_id': row['site_id'],
        'located': initial is not None,
        'approached': state == 'ok',
        'pulses': len(pulses),
        'search_yaw_rad': approach.get('search_yaw_rad'),
        'error': error,
    }
    return row, capability


def _fmt(value, pattern='{:.3f}'):
    return '—' if value is None else pattern.format(float(value))


def render(rows, capabilities, attempts=None):
    """Return the markdown report.

    ``rows`` has one last-attempt row per pair; ``attempts`` is the number of
    raw sidecar records, i.e. pairs plus retries.
    """
    pairs = len({row['pair_id'] for row in rows})
    records = len(rows) if attempts is None else attempts
    located = sum(1 for item in capabilities if item['located'])
    approached = sum(1 for item in capabilities if item['approached'])
    aborted = [item for item in capabilities if item['error']]
    search = sum(1 for item in capabilities
                 if item['search_yaw_rad'] not in (None, 0.0))
    lines = [
        '# 24-site session — capability and outcome table',
        '',
        '- pairs: {}; sidecar attempts: {}; retries: {}'.format(
            pairs, records, records - pairs),
        '- located (last attempt): {}/{}; approached (inside the empirical '
        'envelope): {}/{}'.format(located, pairs, approached, pairs),
        '- yaw-search rescues: {}; aborted last attempts: {}'.format(
            search, len(aborted)),
        '',
        '| Pair | Site | Site r/theta | Initial r/theta | Pulses | '
        'Final x (m) | Final j1 (rad) | Yaw (rad) | Baseline | '
        'Intervention | Placement (mm) | State |',
        '|---|---|---|---|---:|---:|---:|---:|---|---|---:|---|',
    ]
    for row in rows:
        site_rb = '{} / {:+.0f}'.format(
            _fmt(row['site_radius_m'], '{:.2f}'),
            row['site_bearing_deg']
            if row['site_bearing_deg'] is not None else 0.0)
        initial_rb = '{} / {:+.1f}'.format(
            _fmt(row['initial_radius_m'], '{:.3f}'),
            row['initial_bearing_deg']
            if row['initial_bearing_deg'] is not None else 0.0)
        lines.append(
            '| {pair_id} | {site_id} | {site_rb} | {initial_rb} | '
            '{pulses} | {final_x} | {final_j1} | {yaw} | {baseline} | '
            '{intervention} | {placement} | {state} |'.format(
                pair_id=row['pair_id'],
                site_id=row['site_id'],
                site_rb=site_rb,
                initial_rb=initial_rb,
                pulses=row['pulses'],
                final_x=_fmt(row['final_x_m']),
                final_j1=_fmt(row['final_j1_rad'], '{:+.3f}'),
                yaw=_fmt(row['search_yaw_rad'], '{:+.2f}'),
                baseline=row['baseline'],
                intervention=row['intervention'],
                placement=row['placement_mm'],
                state=row['state']))
    return '\n'.join(lines) + '\n'


def build_report(approaches_path, trials_path):
    """Return ``(markdown, rows, capabilities)`` for the two input files."""
    raw_approaches = load_jsonl(approaches_path)
    approaches = latest_approaches(raw_approaches)
    trials = trial_map(load_jsonl(trials_path))
    rows = []
    capabilities = []
    for pair_id, record in approaches:
        row, capability = approach_row(pair_id, record, trials)
        rows.append(row)
        capabilities.append(capability)
    return (render(rows, capabilities, attempts=len(raw_approaches)),
            rows, capabilities)


def build_parser():
    """Return the argument parser."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        '--approaches',
        default='evidence/phase3/onsite_20261120_sites/approaches.jsonl',
        help='locate/approach/return sidecar written by the session')
    parser.add_argument(
        '--trials',
        default='evidence/phase3/onsite_20261120_sites/trials.jsonl',
        help='paired-trial records')
    parser.add_argument('--markdown-out', default=None)
    return parser


def main(argv=None):
    """Render and optionally write the report."""
    args = build_parser().parse_args(argv)
    markdown, rows, capabilities = build_report(
        args.approaches, args.trials)
    print(markdown, end='')
    if args.markdown_out:
        os.makedirs(
            os.path.dirname(os.path.abspath(args.markdown_out)) or '.',
            exist_ok=True)
        with open(args.markdown_out, 'w', encoding='utf-8') as handle:
            handle.write(markdown)
        print('\nWROTE {}'.format(args.markdown_out))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
