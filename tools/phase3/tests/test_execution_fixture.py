"""Tests for the execution/fixture measurement runner (no robot)."""

import math
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

import execution_fixture_run as efr  # noqa: E402


def test_commanded_pulses_match_the_deep_family():
    """joint2 -0.6618 rad is the measured 342 pulse of the deep family."""
    pulses = efr.commanded_pulses([0.0, -0.6618, -1.8012, -0.3917, 0.0])
    assert pulses[0] == pytest.approx(500.0)
    assert pulses[1] == pytest.approx(342.0, abs=0.1)
    assert pulses[2] == pytest.approx(930.0, abs=0.1)


def test_no_pulse_difference_means_no_joint_delta():
    joints = [0.1, -0.6, -1.7, -0.3, 0.0]
    pulses = [round(value) for value in efr.commanded_pulses(joints)]
    achieved = efr.joint_delta_from_pulses(joints, pulses)
    assert np.allclose(achieved, joints, atol=2e-3)


def test_one_pulse_on_joint2_is_about_4_milliradians():
    joints = [0.0, -0.6618, -1.8012, -0.3917, 0.0]
    pulses = [round(value) for value in efr.commanded_pulses(joints)]
    pulses[1] += 1
    achieved = efr.joint_delta_from_pulses(joints, pulses)
    # One tick, minus the sub-tick rounding of the commanded pulse.
    assert achieved[1] - joints[1] == pytest.approx(
        efr.RAD_PER_TICK, rel=0.01)


def _planar_frame(joints):
    """Fake frame: x = j1, y = -j2, z = j3, rotation about z by j4."""
    position = np.array([joints[0], -joints[1], joints[2]])
    angle = joints[3]
    rotation = np.array([
        [math.cos(angle), -math.sin(angle), 0.0],
        [math.sin(angle), math.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ])
    return position, rotation


def test_execution_error_is_zero_for_the_commanded_pulses():
    joints = [0.2, -0.5, -1.4, -0.3, 0.0]
    pulses = efr.commanded_pulses(joints)
    result = efr.execution_error(joints, pulses, _planar_frame)
    assert result['translation_mm'] == pytest.approx(0.0, abs=1e-9)
    assert result['rotation_deg'] == pytest.approx(0.0, abs=1e-9)


def test_execution_error_reports_a_large_rotation_on_joint4():
    joints = [0.0, -0.6618, -1.8012, 0.0, 0.0]
    pulses = [round(value) for value in efr.commanded_pulses(joints)]
    pulses[3] += 10  # joint4 moves by 10 ticks
    result = efr.execution_error(joints, pulses, _planar_frame)
    assert result['rotation_deg'] == pytest.approx(
        math.degrees(10 * efr.RAD_PER_TICK), rel=1e-6)


def _family_args():
    """Return the fixture-envelope arguments used by the validator."""
    import argparse
    return argparse.Namespace(
        fixture_x_min=efr.DEFAULT_FIXTURE_X[0],
        fixture_x_max=efr.DEFAULT_FIXTURE_X[1],
        fixture_y_max=efr.DEFAULT_FIXTURE_Y,
        max_z_residual_mm=efr.DEFAULT_MAX_Z_RESIDUAL_MM)


def test_validate_family_accepts_the_measured_fixture():
    efr.validate_family([0.230, 0.0, -0.049],
                        {'residual_mm': [0.0, 0.0, 2.0]}, _family_args())


def test_validate_family_refuses_the_far_placement():
    """The 2026-11-20 first placement at x = 0.262 m had a 23 mm residual."""
    with pytest.raises(SystemExit):
        efr.validate_family([0.262, 0.0, -0.049],
                            {'residual_mm': [0.0, 0.0, 23.4]},
                            _family_args())


def test_validate_family_refuses_a_high_residual_at_the_fixture():
    with pytest.raises(SystemExit):
        efr.validate_family([0.230, 0.0, -0.049],
                            {'residual_mm': [0.0, 0.0, 12.0]},
                            _family_args())


def test_summarize_reports_sample_statistics():
    stats = efr.summarize([1.0, 2.0, 3.0])
    assert stats['n'] == 3
    assert stats['mean'] == pytest.approx(2.0)
    assert stats['std'] == pytest.approx(1.0)
    assert stats['max'] == 3.0
    assert stats['range'] == 2.0


def test_summarize_of_empty_is_just_n():
    assert efr.summarize([]) == {'n': 0}


def test_summarize_vectors_reports_radial_spread():
    stats = efr.summarize_vectors([[0.0, 0.0, 0.0], [0.003, 0.0, 0.0],
                                   [-0.003, 0.0, 0.0]])
    assert stats['n'] == 3
    assert stats['std'][0] == pytest.approx(0.003)
    assert stats['radial_mm']['max'] == pytest.approx(3.0)


def test_fixture_report_builds_a_markdown_table():
    records = [{'point': [0.23, 0.0, -0.048]},
               {'point': [0.231, 0.001, -0.048]}]
    report = efr.fixture_report(records)
    assert report['n'] == 2
    assert 'Radial max' in report['markdown']
    assert 'Placements | 2' in report['markdown']


def test_trial_report_separates_execution_from_placement():
    execution = {'translation_mm': 0.8, 'rotation_deg': 0.2,
                 'achieved_joints': [], 'delta_joints': []}
    trials = [
        {'grasp_success': True, 'execution': execution,
         'placement_xy_mm': 2.0, 'placement_vector_m': [0.002, 0.0]},
        {'grasp_success': False, 'execution': execution,
         'placement_xy_mm': None, 'placement_vector_m': None},
    ]
    report = efr.trial_report(records=trials, attempts=trials)
    assert report['attempts'] == 2
    assert report['successes'] == 1
    assert report['execution_translation_mm']['n'] == 2
    assert report['placement_xy_mm']['n'] == 1
    assert 'Grasp execution' in report['markdown']
