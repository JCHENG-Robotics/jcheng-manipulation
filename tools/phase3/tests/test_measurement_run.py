"""Tests for the tag-based measurement runner (no robot)."""

import json
import math
import os
import sys

import numpy as np
import pytest

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(TOOLS))
for path in (TOOLS, os.path.join(REPO, 'tools', 'phase2')):
    if path not in sys.path:
        sys.path.insert(0, path)

import measurement_run as run  # noqa: E402


def _rotation(deg):
    angle = math.radians(deg)
    return [[math.cos(angle), -math.sin(angle), 0.0],
            [math.sin(angle), math.cos(angle), 0.0],
            [0.0, 0.0, 1.0]]


def test_write_poses_repeats_the_target(tmp_path):
    path = tmp_path / 'poses.json'
    run.write_poses(str(path), [0.0, -0.66, -1.8, -0.39, 0.0], 4, 'rep')
    data = json.loads(path.read_text())
    assert len(data['poses']) == 4
    assert all(pose['joints'] == [0.0, -0.66, -1.8, -0.39, 0.0]
               for pose in data['poses'])
    assert data['poses'][0]['name'] == 'rep_01'


def test_rotation_spread_deg_of_a_perfect_repeat_is_zero():
    rows = [{'rotation': _rotation(0.0)} for _ in range(5)]
    maximum, mean = run.rotation_spread_deg(rows)
    assert maximum < 1e-6
    assert mean < 1e-6


def test_rotation_spread_deg_reports_the_geodesic_deviation():
    """Two samples 2 deg apart sit 1 deg either side of the mean."""
    rows = [{'rotation': _rotation(0.0)}, {'rotation': _rotation(2.0)}]
    maximum, mean = run.rotation_spread_deg(rows)
    assert 0.9 < maximum < 1.1
    assert math.isclose(mean, maximum, rel_tol=1e-6)


def test_build_measurement_repeatability(tmp_path):
    """One dataset becomes one sample list with a mean nominal point."""
    pytest.importorskip('armpi_grasp_planner.reachability')
    dataset = {
        'samples': [{
            'name': 'a',
            'measured_joints': [0.0, -0.66, -1.8, -0.39, 0.0],
            'tag': {'rvec': [0.0, 0.0, 0.0], 'tvec': [0.0, 0.0, 0.30],
                    'rms_px': 0.4},
        }, {
            'name': 'b',
            'measured_joints': [0.0, -0.66, -1.8, -0.39, 0.0],
            'tag': {'rvec': [0.0, 0.0, 0.0], 'tvec': [0.0, 0.0, 0.30],
                    'rms_px': 0.4},
        }],
    }
    path = tmp_path / 'samples.json'
    path.write_text(json.dumps(dataset))
    measurement = run.build_measurement(
        'repeatability', [str(path)], np.eye(4))
    assert len(measurement['samples']) == 2
    assert len(measurement['nominal']) == 3
    assert measurement['samples'][0]['position_m'] == \
        measurement['samples'][1]['position_m']


def test_build_measurement_backlash_needs_two_groups(tmp_path):
    pytest.importorskip('armpi_grasp_planner.reachability')
    sample = {
        'name': 'a',
        'measured_joints': [0.0, -0.66, -1.8, -0.39, 0.0],
        'tag': {'rvec': [0.0, 0.0, 0.0], 'tvec': [0.0, 0.0, 0.30],
                'rms_px': 0.4},
    }
    path = tmp_path / 'samples.json'
    path.write_text(json.dumps({'samples': [sample]}))
    measurement = run.build_measurement(
        'backlash', [str(path), str(path)], np.eye(4))
    assert set(measurement['samples']) == {'forward', 'reverse'}
    with pytest.raises(SystemExit):
        run.build_measurement('backlash', [str(path)], np.eye(4))
