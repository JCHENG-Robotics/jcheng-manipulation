"""Tests for the random-placement region and plan (no robot)."""

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

import random_region as region  # noqa: E402


def test_default_region_and_fixtures():
    payload = region.build_region()
    assert payload['n_pairs'] == 24
    assert payload['seed'] == 17
    assert payload['fixtures'][0] == 'r01'
    assert payload['fixtures'][-1] == 'r24'
    assert payload['region'] == {
        'min_radius_m': 0.25, 'max_radius_m': 0.55,
        'min_bearing_deg': -20.0, 'max_bearing_deg': 25.0}
    assert payload['placement_rule']['operator_random'] is True
    assert payload['placement_rule']['min_separation_m'] == 0.03
    assert payload['locators'] == [
        'wrist_view', 'front_camera', 'wrist_yaw']


def test_invalid_regions_are_refused():
    with pytest.raises(ValueError):
        region.build_region(n_pairs=0)
    with pytest.raises(ValueError):
        region.build_region(min_radius=0.60, max_radius=0.55)
    with pytest.raises(ValueError):
        region.build_region(min_bearing=-50.0)
    with pytest.raises(ValueError):
        region.build_region(min_bearing=30.0, max_bearing=20.0)
    with pytest.raises(ValueError):
        region.build_region(min_separation=0.0)


def test_round_trip_and_tamper_checks(tmp_path):
    payload = region.build_region(n_pairs=4)
    path = tmp_path / 'region.json'
    region.write_json(path, payload)
    loaded = region.load_region(str(path))
    assert loaded['fixtures'] == ['r01', 'r02', 'r03', 'r04']
    loaded['fixtures'] = ['r01', 'r02', 'r03', 'r99']
    path.write_text(json.dumps(loaded), encoding='utf-8')
    with pytest.raises(ValueError):
        region.load_region(str(path))


def test_plan_has_one_random_fixture_per_pair(tmp_path):
    payload = region.build_region(n_pairs=4)
    path = tmp_path / 'plan.json'
    plan = region.write_plan(path, payload)
    assert plan['n_pairs'] == 4
    assert plan['seed'] == 17
    assert len(plan['entries']) == 8
    fixtures = {entry['object_pose_fixture'] for entry in plan['entries']}
    assert fixtures == {'r01', 'r02', 'r03', 'r04'}
    pairs = {pair['pair_id']: pair['object_pose_fixture']
             for pair in plan['pairs']}
    assert pairs['p001'] == 'r01'
    assert pairs['p004'] == 'r04'
    entries = plan['entries']
    for index in range(0, len(entries), 2):
        assert entries[index]['pair_id'] == entries[index + 1]['pair_id']
        assert entries[index]['method'] != entries[index + 1]['method']
