"""Tests for the rotation-tolerance runner (no robot)."""

import json
import math
import os
import sys

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(TOOLS))
for path in (TOOLS, os.path.join(REPO, 'tools', 'phase2')):
    if path not in sys.path:
        sys.path.insert(0, path)

import rotation_tolerance_run as rot  # noqa: E402


def test_build_schedule_covers_each_yaw_once_per_block():
    schedule = rot.build_schedule([0, 5, -5, 10], 2, 7)
    assert len(schedule) == 8
    counts = {}
    for block, yaw in schedule:
        counts[(block, yaw)] = counts.get((block, yaw), 0) + 1
    for block in (1, 2):
        for yaw in (0, 5, -5, 10):
            assert counts[(block, yaw)] == 1


def test_build_schedule_is_seeded_and_reproducible():
    first = rot.build_schedule([0, 5, -5, 10, -10], 2, 11)
    second = rot.build_schedule([0, 5, -5, 10, -10], 2, 11)
    assert first == second
    assert first != rot.build_schedule([0, 5, -5, 10, -10], 2, 12)


def test_measured_yaw_uses_the_joint5_register_mapping():
    radians_per_tick = 240.0 / 360.0 * (2.0 * math.pi) / 1000.0
    assert rot.measured_yaw_deg(500) == 0.0
    assert abs(rot.measured_yaw_deg(870)
               - math.degrees(370.0 * radians_per_tick)) < 1e-9
    assert abs(rot.measured_yaw_deg(120)
               - math.degrees(-380.0 * radians_per_tick)) < 1e-9


def test_achieved_relative_yaw_subtracts_the_wrist_turn():
    """A +10 deg wrist turn removes 10 deg from a +4 deg cube face."""
    assert rot.achieved_relative_deg(4.0, 10.0) == -6.0
    assert rot.achieved_relative_deg(-4.0, -10.0) == 6.0


def test_group_records_counts_successes():
    rows = [
        {'yaw_deg': 0, 'success': True},
        {'yaw_deg': 0, 'success': True},
        {'yaw_deg': 15, 'success': False},
        {'yaw_deg': 15, 'success': True},
    ]
    records = rot.group_records(rows)
    assert records[0]['yaw_deg'] == 0.0
    assert records[0]['trials'] == 2
    assert records[0]['successes'] == 2
    assert records[1]['yaw_deg'] == 15.0
    assert records[1]['successes'] == 1


def test_dry_run_writes_measurement_and_analysis(tmp_path):
    trials = tmp_path / 'trials.jsonl'
    measurement = tmp_path / 'measurement.json'
    rc = rot.main([
        '--dry-run', '--repeats', '1', '--yaw-offsets', '0,5,-5',
        '--trials', str(trials), '--measurement-out', str(measurement),
    ])
    assert rc == 0
    rows = [json.loads(line) for line in trials.read_text().splitlines()]
    assert len(rows) == 3
    data = json.loads(measurement.read_text())
    assert data['protocol'] == 'gripper_envelope'
    assert sum(sample['trials'] for sample in data['samples']) == 3
    analysis = json.loads(
        (tmp_path / 'measurement_analysis.json').read_text())
    assert analysis['envelope_deg'] == 5.0
