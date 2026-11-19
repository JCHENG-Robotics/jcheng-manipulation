"""Tests for the paired-trial session runner (no robot)."""

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

import paired_trial_session as session  # noqa: E402


def test_cube_point_accepts_a_valid_top_face():
    report = {'selected': {'grasp_point_base': [0.23, 0.0, -0.05]}}
    point, valid = session.cube_point(report)
    assert valid is True
    assert np.allclose(point, [0.23, 0.0, -0.05])


def test_cube_point_marks_a_rejected_view():
    report = {'selected': {'rejected_point_base': [0.21, 0.03, -0.048]},
              'rejection': {'reason': 'partial_top_view'}}
    point, valid = session.cube_point(report)
    assert valid is False
    assert np.allclose(point, [0.21, 0.03, -0.048])


def test_cube_point_without_a_cluster_is_none():
    point, valid = session.cube_point({'success': False})
    assert point is None
    assert valid is False


def _selected(height, size):
    return {'selected': {'height_above_floor_m': height,
                         'bounds': {'size': size}}}


def test_placement_quality_accepts_a_flat_cube():
    ok, reason = session.placement_quality(
        _selected(0.0394, [0.0384, 0.0403, 0.0041]))
    assert ok, reason


def test_placement_quality_rejects_an_elevated_cube():
    """p019 baseline: the cube sat 20 mm above the floor."""
    ok, reason = session.placement_quality(
        _selected(0.0593, [0.038, 0.0386, 0.0043]))
    assert not ok
    assert 'height' in reason


def test_placement_quality_rejects_a_merged_cluster():
    """p023: the cube merged with a neighbour into 53.7 x 72.4 mm."""
    ok, reason = session.placement_quality(
        _selected(0.0198, [0.0537, 0.0724, 0.0056]))
    assert not ok


def test_placement_quality_rejects_a_missing_measurement():
    ok, _ = session.placement_quality({})
    assert not ok


def test_cube_box_ignores_a_background_cluster():
    """A far cluster is not the cube; an in-box cluster is."""
    point = [0.232, 0.022, -0.049]
    report = {'segmentation': {'cluster_details': [
        {'bounds': {'centroid': [0.499, -0.249, -0.083]}, 'height_m': 0.016},
    ]}}
    assert session.cube_box_present(report, point) is False
    report['segmentation']['cluster_details'].append(
        {'bounds': {'centroid': [0.235, 0.020, -0.049]}, 'height_m': 0.039})
    assert session.cube_box_present(report, point) is True


def test_correction_is_converted_to_metres(tmp_path):
    """The CLI takes millimetres; the target arithmetic is in metres."""
    args = session.build_parser().parse_args([
        '--dry-run', '--plan', str(tmp_path / 'p.json'),
        '--trials', str(tmp_path / 't.jsonl'),
    ])
    sess = session.Session(args)
    assert np.allclose(sess.correction, [0.0137, 0.0039, 0.0089])


def test_validate_target_rejects_a_unit_error():
    """A 13.9 m target is refused before any motion."""
    fit = {'xy_error_mm': 0.0, 'reach_limited': False}
    with pytest.raises(ValueError):
        session.validate_target([13.93, 0.0039, 0.0089], fit, 3.0)


def test_validate_target_rejects_a_failed_fit():
    fit = {'xy_error_mm': 14.0, 'reach_limited': False}
    with pytest.raises(ValueError):
        session.validate_target([0.24, 0.0, -0.04], fit, 3.0)


def test_validate_target_accepts_the_validated_target():
    fit = {'xy_error_mm': 0.01, 'reach_limited': False}
    session.validate_target([0.23992, -0.00205, -0.04166], fit, 3.0)


def test_dry_run_writes_an_analyzable_dataset(tmp_path):
    """The dry session produces records the framework validates."""
    pytest.importorskip('armpi_grasp_planner.reachability')
    plan = tmp_path / 'plan.json'
    trials = tmp_path / 'trials.jsonl'
    assert session.main([
        '--pairs', '2', '--seed', '7', '--dry-run',
        '--plan', str(plan), '--trials', str(trials),
    ]) == 0
    from armpi_analysis import paired_trials

    records = paired_trials.load_records(str(trials))
    assert len(records) == 4
    report = paired_trials.validate_dataset(records)
    assert report['n_valid'] == 4
    assert report['n_invalid'] == 0
    assert len(report['complete_pairs']) == 2
    analysis = paired_trials.analyze(records)
    assert analysis['methods']['intervention']['successes'] == 2
    assert analysis['methods']['baseline']['successes'] == 0
