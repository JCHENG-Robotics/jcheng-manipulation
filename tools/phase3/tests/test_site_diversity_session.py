"""Tests for the site-diversity paired session (no robot)."""

import json
import math
import os
import sys

import pytest

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(TOOLS))
for path in (TOOLS, os.path.join(REPO, 'tools', 'phase2'),
             os.path.join(REPO, 'ros2_ws', 'src', 'armpi_analysis')):
    if path not in sys.path:
        sys.path.insert(0, path)

import site_diversity_session as session  # noqa: E402
import site_layout  # noqa: E402


def _layout_path(tmp_path, sites=None):
    sites = sites or site_layout.build_sites()
    path = tmp_path / 'layout.json'
    site_layout.write_json(path, site_layout.layout_payload(sites))
    return str(path), sites


def _args(tmp_path, sites=None, extra=None):
    path, sites = _layout_path(tmp_path, sites)
    argv = [
        '--sites', path,
        '--plan', str(tmp_path / 'plan.json'),
        '--trials', str(tmp_path / 'trials.jsonl'),
        '--pairs', str(len(sites)),
    ] + list(extra or [])
    return session.build_parser().parse_args(argv)


def test_nominal_travel_prefers_the_pulse_report():
    assert session.nominal_travel_mm({'nominal_travel_mm': 123.4}) == 123.4
    assert session.nominal_travel_mm(
        {'duty': 20.0, 'duration': 1.0}) == pytest.approx(109.89, rel=1e-3)


def test_approach_plan_stops_inside_the_band():
    assert session.approach_plan([0.2305, 0.0, -0.05], 20.0, 0.015) is None
    plan = session.approach_plan([0.45, 0.15, -0.05], 20.0, 0.015)
    assert plan is not None
    vx, vy, duration = plan
    assert vx > 0.0 and vy > 0.0
    assert 0.0 < duration <= 3.0


def test_net_base_travel_uses_the_pulse_plan_model():
    fake = object.__new__(session.SiteSession)
    fake.approach = {'base_pulses': [
        {'vx': 0.8, 'vy': 0.6, 'duty': 20.0, 'duration': 1.0,
         'nominal_travel_mm': 109.89, 'subscribers_matched': 1},
        {'vx': 1.0, 'vy': 0.0, 'duty': 20.0, 'duration': 0.5,
         'nominal_travel_mm': 54.95},
    ]}
    net = fake._net_base_travel()
    # 20 duty * 4.9 mm/s = 98 mm/s, not the report's 109.9 mm/s nominal:
    # 0.098 m * (0.8 * 1.0 + 1.0 * 0.5) = 0.1274 m.
    assert net[0] == pytest.approx(0.098 * 1.3)
    assert net[1] == pytest.approx(0.098 * 0.6)


def test_net_base_travel_skips_a_dropped_pulse():
    fake = object.__new__(session.SiteSession)
    fake.approach = {'base_pulses': [
        {'vx': 1.0, 'vy': 0.0, 'duty': 20.0, 'duration': 1.0,
         'nominal_travel_mm': 109.89, 'subscribers_matched': 0},
        {'vx': 1.0, 'vy': 0.0, 'duty': 20.0, 'duration': 0.5,
         'nominal_travel_mm': 54.95, 'subscribers_matched': 1},
    ]}
    net = fake._net_base_travel()
    assert net[0] == pytest.approx(0.049)
    assert net[1] == pytest.approx(0.0)


def test_the_plan_uses_one_site_per_pair(tmp_path):
    args = _args(tmp_path)
    first = session.SiteSession(args)
    plan = first.plan
    assert plan['n_pairs'] == 24
    assert plan['seed'] == 17
    assert {entry['object_pose_fixture'] for entry in plan['entries']} == {
        's{:02d}'.format(index + 1) for index in range(24)}
    fixture_of = {pair['pair_id']: pair['object_pose_fixture']
                  for pair in plan['pairs']}
    assert fixture_of['p001'] == 's01'
    assert fixture_of['p024'] == 's24'
    # Both methods of a pair are adjacent in plan order.
    entries = plan['entries']
    for index in range(0, len(entries), 2):
        assert entries[index]['pair_id'] == entries[index + 1]['pair_id']
        assert entries[index]['method'] != entries[index + 1]['method']
    # Reloading the same file does not regenerate it.
    with open(tmp_path / 'plan.json', 'r', encoding='utf-8') as handle:
        reloaded = json.load(handle)
    assert reloaded == plan
    second = session.SiteSession(args)
    assert second.plan == plan


def test_a_plan_from_another_layout_is_refused(tmp_path):
    from armpi_analysis import paired_trials

    _, sites = _layout_path(tmp_path)
    other = paired_trials.build_plan(
        ['p{:03d}'.format(index + 1) for index in range(len(sites))],
        fixtures=['x{:02d}'.format(index + 1) for index in range(len(sites))],
        seed=17)
    with open(tmp_path / 'plan.json', 'w', encoding='utf-8') as handle:
        json.dump(other, handle)
    with pytest.raises(SystemExit):
        session.SiteSession(_args(tmp_path))


def test_a_partial_pair_is_re_located_with_both_entries_or_one(tmp_path):
    args = _args(tmp_path)
    site_session = session.SiteSession(args)
    plan = site_session.plan
    first_pair = plan['pairs'][0]
    first_entries = [
        entry for entry in plan['entries']
        if entry['pair_id'] == first_pair['pair_id']]
    site_session.done = {
        (first_pair['pair_id'], first_entries[0]['method'])}
    pending = site_session._pending_pairs()
    pending_pair = [item for item in pending
                    if item[0] == first_pair['pair_id']][0]
    assert len(pending_pair[1]) == 1
    assert pending_pair[1][0]['method'] == first_entries[1]['method']
    assert pending_pair[2]['site_id'] == first_entries[0][
        'object_pose_fixture']


def test_a_plan_with_a_swapped_site_is_refused(tmp_path):
    from armpi_analysis import paired_trials

    _, sites = _layout_path(tmp_path)
    pair_ids = ['p{:03d}'.format(index + 1) for index in range(len(sites))]
    plan = paired_trials.build_plan(
        pair_ids, fixtures=[site['site_id'] for site in sites], seed=17)
    plan['pairs'][0]['object_pose_fixture'], \
        plan['pairs'][1]['object_pose_fixture'] = (
            plan['pairs'][1]['object_pose_fixture'],
            plan['pairs'][0]['object_pose_fixture'])
    with open(tmp_path / 'plan.json', 'w', encoding='utf-8') as handle:
        json.dump(plan, handle)
    with pytest.raises(SystemExit):
        session.SiteSession(_args(tmp_path))


class _FakeRobot:
    def __init__(self):
        self.calls = []

    def base_pulse(self, vx, vy, duty, duration, allow_long=False):
        self.calls.append({
            'vx': vx, 'vy': vy, 'duty': duty, 'duration': duration})
        return {'vx': vx, 'vy': vy, 'duty': duty, 'duration': duration,
                'nominal_travel_mm': duty * duration / 0.182,
                'subscribers_matched': 1}


def test_return_home_drives_the_reverse_command(tmp_path, monkeypatch):
    args = _args(tmp_path)
    site_session = session.SiteSession(args)
    site_session.robot = _FakeRobot()
    site_session.approach = {'base_pulses': [
        {'vx': 0.6, 'vy': 0.8, 'duty': 20.0, 'duration': 1.0,
         'nominal_travel_mm': 109.89, 'subscribers_matched': 1},
    ]}
    monkeypatch.setattr(session.time, 'sleep', lambda seconds: None)
    site_session.return_home('p001')
    assert len(site_session.robot.calls) == 1
    call = site_session.robot.calls[0]
    assert call['vx'] == pytest.approx(-0.6)
    assert call['vy'] == pytest.approx(-0.8)
    assert call['duty'] == pytest.approx(args.base_duty)
    expected = 0.098 * 1000.0 / (20.0 * 4.9)
    assert call['duration'] == pytest.approx(expected, rel=1e-6)
    assert site_session.approach['return_home']['pulses']
    assert site_session.approach['return_home']['done'] is True
    # The run's finally calls return_home again; the base is already home.
    site_session.return_home('p001')
    assert len(site_session.robot.calls) == 1


def test_before_release_returns_home_while_holding(tmp_path):
    args = _args(tmp_path)
    site_session = session.SiteSession(args)
    site_session.approach = {'base_pulses': [
        {'vx': 1.0, 'vy': 0.0, 'duty': 20.0, 'duration': 1.0,
         'nominal_travel_mm': 109.89, 'subscribers_matched': 1},
    ]}
    calls = []

    def fake_return_home(pair_id):
        calls.append(pair_id)
        site_session.approach['return_home'] = {
            'done': True, 'pulses': [{'vx': -1.0}]}

    site_session.return_home = fake_return_home
    fired = site_session._before_release({'pair_id': 'p001'}, {}, {})
    assert fired is True
    assert calls == ['p001']
    assert site_session.approach['return_home']['while_holding'] is True


def test_an_aborted_pair_still_returns_the_base_home(tmp_path):
    args = _args(tmp_path)
    site_session = session.SiteSession(args)
    site_session.args.dry_run = True
    site_session.prepare = lambda: None
    site = {'site_id': 's01', 'radius_m': 0.30, 'bearing_deg': -10.0,
            'x_m': 0.295, 'y_m': -0.052}
    entry = {'pair_id': 'p001', 'sequence': 1, 'trial_order': 1,
             'method': 'baseline', 'object_pose_fixture': 's01'}
    site_session._pending_pairs = lambda: [('p001', [entry], site)]
    site_session.locate_and_approach = lambda site, pair_id: setattr(
        site_session, 'approach', {'base_pulses': [
            {'vx': 1.0, 'vy': 0.0, 'duty': 20.0, 'duration': 1.0,
             'subscribers_matched': 1}]})
    calls = []

    def fail(*unused, **unused_kwargs):
        raise RuntimeError('trajectory failed')

    site_session.trial = fail
    site_session.return_home = lambda pair_id: calls.append(pair_id)
    with pytest.raises(RuntimeError):
        site_session.run()
    assert calls == ['p001']


def test_dry_run_writes_48_records(tmp_path):
    pytest.importorskip('armpi_grasp_planner.reachability')
    from armpi_analysis import paired_trials

    path, _ = _layout_path(tmp_path)
    plan = tmp_path / 'plan.json'
    trials = tmp_path / 'trials.jsonl'
    assert session.main([
        '--dry-run', '--sites', path,
        '--plan', str(plan), '--trials', str(trials),
    ]) == 0
    records = paired_trials.load_records(str(trials))
    assert len(records) == 48
    report = paired_trials.validate_dataset(records)
    assert report['n_valid'] == 48
    assert report['n_invalid'] == 0
    assert len(report['complete_pairs']) == 24
    assert {record['object_pose_fixture'] for record in records} == {
        's{:02d}'.format(index + 1) for index in range(24)}
    analysis = paired_trials.analyze(records)
    assert analysis['methods']['intervention']['successes'] == 24
    assert analysis['methods']['baseline']['successes'] == 0
    notes = json.loads(records[0]['notes'])
    assert notes['site']['site_id'] == records[0]['object_pose_fixture']
    assert math.isfinite(notes['approach']['final_j1_rad'])
