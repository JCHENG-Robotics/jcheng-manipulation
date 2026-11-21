"""Tests for the random-placement session (no robot)."""

import json
import os
import sys

import numpy as np
import pytest

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(TOOLS))
for path in (TOOLS, os.path.join(REPO, 'tools', 'phase2'),
             os.path.join(REPO, 'ros2_ws', 'src', 'armpi_analysis')):
    if path not in sys.path:
        sys.path.insert(0, path)

import paired_trial_session as base  # noqa: E402
import random_region  # noqa: E402
import random_site_session as session  # noqa: E402


def _paths(tmp_path, n_pairs=24):
    payload = random_region.build_region(n_pairs=n_pairs)
    region_path = tmp_path / 'region.json'
    plan_path = tmp_path / 'plan.json'
    random_region.write_json(region_path, payload)
    random_region.write_plan(plan_path, payload)
    return region_path, plan_path, payload


def _args(tmp_path, n_pairs=24, extra=None):
    region_path, plan_path, _ = _paths(tmp_path, n_pairs)
    argv = [
        '--region', str(region_path),
        '--plan', str(plan_path),
        '--trials', str(tmp_path / 'trials.jsonl'),
        '--pairs', str(n_pairs),
    ] + list(extra or [])
    return session.build_parser().parse_args(argv)


def _fake_session():
    fake = object.__new__(session.RandomSiteSession)
    fake.approach = {}
    fake.region_payload = random_region.build_region()
    return fake


def test_the_front_capture_tool_is_installed():
    assert 'ext_cam_capture.py' in base.ROBOT_TOOLS


def test_plan_and_random_sites_load(tmp_path):
    site_session = session.RandomSiteSession(_args(tmp_path))
    assert len(site_session.sites) == 24
    assert list(site_session.sites)[0] == 'r01'
    assert site_session.plan['n_pairs'] == 24
    pending = site_session._pending_pairs()
    assert len(pending) == 24
    pair_id, entries, site = pending[0]
    assert site['random'] is True
    assert site['site_id'].startswith('r')
    assert entries[0]['object_pose_fixture'] == site['site_id']


def test_region_check_inside_and_outside():
    fake = _fake_session()
    radius, bearing, inside = fake._region_check([0.40, 0.10, 0.0])
    assert inside is True
    assert radius == pytest.approx(0.41231, abs=1e-4)
    assert bearing == pytest.approx(14.036, abs=1e-3)
    assert fake._region_check([0.60, 0.0, 0.0])[2] is False
    assert fake._region_check([0.40, 0.30, 0.0])[2] is False
    assert fake._region_check([0.20, 0.0, 0.0])[2] is False


def test_locator_chain_prefers_wrist_then_yaw_then_front():
    fake = _fake_session()
    fake._wide_measure = lambda tag: (
        np.array([0.40, 0.10, 0.0]), True, {'success': True})
    point, valid, report, locator = fake._locate_initial('t')
    assert locator == 'wrist_view'
    assert point[0] == pytest.approx(0.40)

    fake._wide_measure = lambda tag: (None, False, {})
    fake._search = lambda tag: (
        np.array([0.45, 0.20, 0.0]), True, {'success': True}, 0.14)
    point, valid, report, locator = fake._locate_initial('t')
    assert locator == 'wrist_yaw'
    assert fake.approach['search_yaw_rad'] == pytest.approx(0.14)

    fake._search = lambda tag: (None, False, {}, 0.0)
    fake._front_locate = lambda tag: (
        np.array([0.50, 0.10, 0.0]), False, {'success': False})
    point, valid, report, locator = fake._locate_initial('t')
    assert locator == 'front_camera'
    assert point[0] == pytest.approx(0.50)

    fake._front_locate = lambda tag: (None, False, None)
    point, valid, report, locator = fake._locate_initial('t')
    assert point is None
    assert locator == 'none'


def test_the_initial_hook_accepts_an_inside_point():
    fake = _fake_session()
    fake.args = type('Args', (), {'dry_run': True})()
    point = np.array([0.40, 0.10, 0.0])
    accepted = fake._validate_initial(
        point, {'success': True}, {'site_id': 'r01'}, 't', 'wrist_view')
    assert accepted is point
    assert fake.approach['locator'] == 'wrist_view'


def test_a_front_report_is_saved(tmp_path):
    fake = _fake_session()
    fake.front_evidence = str(tmp_path / 'front')
    fake._save_front_report('r01_front_0',
                            {'point_base': [0.5, 0.1, -0.05]})
    path = tmp_path / 'front' / 'r01_front_0.json'
    assert path.exists()
    with open(path, 'r', encoding='utf-8') as handle:
        assert json.load(handle)['point_base'] == [0.5, 0.1, -0.05]


def test_a_plan_from_another_region_is_refused(tmp_path):
    from armpi_analysis import paired_trials

    args = _args(tmp_path)
    _, _, payload = _paths(tmp_path)
    plan = paired_trials.build_plan(
        ['p{:03d}'.format(index + 1) for index in range(payload['n_pairs'])],
        fixtures=['x{:02d}'.format(index + 1)
                  for index in range(payload['n_pairs'])],
        seed=17)
    with open(args.plan, 'w', encoding='utf-8') as handle:
        json.dump(plan, handle)
    with pytest.raises(SystemExit):
        session.RandomSiteSession(args)


def test_dry_run_writes_48_records(tmp_path):
    pytest.importorskip('armpi_grasp_planner.reachability')
    from armpi_analysis import paired_trials

    region_path, plan_path, _ = _paths(tmp_path)
    trials = tmp_path / 'trials.jsonl'
    assert session.main([
        '--dry-run',
        '--region', str(region_path),
        '--plan', str(plan_path),
        '--trials', str(trials),
    ]) == 0
    records = paired_trials.load_records(str(trials))
    assert len(records) == 48
    report = paired_trials.validate_dataset(records)
    assert report['n_valid'] == 48
    assert report['n_invalid'] == 0
    assert len(report['complete_pairs']) == 24
    assert {record['object_pose_fixture'] for record in records} == {
        'r{:02d}'.format(index + 1) for index in range(24)}
    analysis = paired_trials.analyze(records)
    assert analysis['methods']['intervention']['successes'] == 24
    assert analysis['methods']['baseline']['successes'] == 0
    notes = json.loads(records[0]['notes'])
    assert notes['site']['random'] is True
    assert len(notes['approach']['initial_point']) == 3
    approaches = [
        json.loads(line) for line in
        open(tmp_path / 'approaches.jsonl', encoding='utf-8')]
    assert len(approaches) == 24
    assert 'return_home' in approaches[0]['approach']
