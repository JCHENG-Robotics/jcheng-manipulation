"""Tests for the 24-site capability report (no robot)."""

import json
import os
import sys

import pytest

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(TOOLS))
for path in (TOOLS, os.path.join(REPO, 'tools', 'phase2'),
             os.path.join(REPO, 'ros2_ws', 'src', 'armpi_analysis')):
    if path not in sys.path:
        sys.path.insert(0, path)

import site_report  # noqa: E402


def _approach(pair_id, site_id, error=None, final=True, located=True):
    approach = {
        'locator': 'wrist_view',
        'search_yaw_rad': 0.0,
        'base_pulses': [
            {'vx': 1.0, 'vy': 0.0, 'duty': 20.0, 'duration': 1.0},
        ],
        'initial_point': [0.40, 0.10, -0.05] if located else None,
        'initial_valid': located,
        'final_point': [0.230, 0.001, -0.05] if final else None,
        'final_j1_rad': 0.01,
        'final_envelope': {
            'x_m': 0.230, 'j1_rad': 0.01, 'x_ok': True, 'j1_ok': True},
        'return_home': {'distance_m': 0.19, 'pulses': []},
    }
    if error:
        approach['error'] = error
    return {
        'pair_id': pair_id,
        'site': {'site_id': site_id, 'radius_m': 0.40,
                 'bearing_deg': 14.0, 'x_m': 0.388, 'y_m': 0.097},
        'approach': approach,
    }


def _trial(pair_id, method, success, error=None, code=None):
    return {
        'pair_id': pair_id, 'trial_order': 1, 'method': method,
        'object_pose_fixture': 's01', 'grasp_success': success,
        'continuous_error_mm': error, 'failure_code': code,
    }


def _write(path, records):
    with open(path, 'w', encoding='utf-8') as handle:
        for record in records:
            handle.write(json.dumps(record) + '\n')


def test_latest_attempt_wins_and_records_are_counted(tmp_path):
    approaches = tmp_path / 'approaches.jsonl'
    trials = tmp_path / 'trials.jsonl'
    _write(approaches, [
        _approach('p001', 's01', error='first attempt lost the cube'),
        _approach('p001', 's01'),
        _approach('p002', 's02', error='no cube found', final=False,
                  located=False),
    ])
    _write(trials, [
        _trial('p001', 'baseline', False, code='MISS'),
        _trial('p001', 'intervention', True, error=2.0),
    ])
    markdown, rows, capabilities = site_report.build_report(
        str(approaches), str(trials))
    assert len(rows) == 2
    assert rows[0]['pair_id'] == 'p001'
    assert rows[0]['state'] == 'ok'
    assert rows[0]['baseline'] == 'MISS'
    assert rows[0]['intervention'] == 'PASS'
    assert rows[0]['placement_mm'] == '2.00'
    assert rows[1]['state'].startswith('aborted:')
    assert rows[1]['baseline'] == '—'
    assert capabilities[0]['located'] is True
    assert capabilities[0]['approached'] is True
    assert capabilities[1]['located'] is False
    assert capabilities[1]['approached'] is False
    assert 'pairs: 2; approach records: 3' in markdown
    assert 'located (last attempt): 1/2' in markdown
    assert 'approached (inside the empirical envelope): 1/2' in markdown


def test_the_last_attempt_does_not_resurrect_a_failed_pair(tmp_path):
    approaches = tmp_path / 'approaches.jsonl'
    trials = tmp_path / 'trials.jsonl'
    _write(approaches, [
        _approach('p001', 's01'),
        _approach('p001', 's01', error='final measurement rejected'),
    ])
    _write(trials, [])
    markdown, rows, _ = site_report.build_report(str(approaches), str(trials))
    assert rows[0]['state'].startswith('aborted:')
    assert rows[0]['intervention'] == '—'
    assert 'approach records: 2' in markdown


def test_radius_bearing_and_blank_inputs(tmp_path):
    radius, bearing = site_report.radius_bearing([0.40, 0.10, -0.05])
    assert radius == pytest.approx(0.41231056256176607)
    assert bearing == pytest.approx(14.036243467926477)
    assert site_report.radius_bearing(None) == (None, None)
    markdown, rows, capabilities = site_report.build_report(
        str(tmp_path / 'missing.jsonl'), str(tmp_path / 'missing.jsonl'))
    assert rows == [] and capabilities == []
    assert 'pairs: 0' in markdown


def test_a_random_site_row_shows_random(tmp_path):
    approaches = tmp_path / 'approaches.jsonl'
    record = _approach('p001', 'r01')
    record['site'] = {'random': True, 'site_id': 'r01'}
    _write(approaches, [record])
    markdown, _, _ = site_report.build_report(
        str(approaches), str(tmp_path / 'missing.jsonl'))
    assert '| p001 | r01 | random |' in markdown
    assert '0.412 / +14.0' in markdown


def test_a_missing_final_point_is_not_approached(tmp_path):
    approaches = tmp_path / 'approaches.jsonl'
    _write(approaches, [_approach('p001', 's01', final=False)])
    markdown, rows, _ = site_report.build_report(
        str(approaches), str(tmp_path / 'missing.jsonl'))
    assert rows[0]['state'] == 'not approached'
    assert rows[0]['final_x_m'] is None
    assert 'approached (inside the empirical envelope): 0/1' in markdown
