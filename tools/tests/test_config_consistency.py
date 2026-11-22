"""Tests for the read-only limit / correction consistency checker."""

from pathlib import Path

import pytest

import verify_config_consistency as v

ROOT = Path(__file__).resolve().parents[2]


def test_current_repo_is_consistent():
    report = v.check_repo(ROOT)
    assert report.issues == []
    assert set(report.limits) == set(v.JOINTS)
    expected = set(v.CORRECTION_FILES) | {v.TAKEOVER}
    assert set(report.corrections) == expected
    assert report.corrections[v.TAKEOVER] == (13.7, 3.9, 8.9)
    assert report.fast_motion == {
        "vmax_rad_s": 4.0,
        "amax_rad_s2": 20.0,
    }
    assert set(report.view_poses) == set(v.VIEW_POSE_FILES)
    reference = next(iter(report.view_poses.values()))
    assert all(pose == reference for pose in report.view_poses.values())


def test_compare_limits_reports_a_mismatch():
    issues = v.compare_limits(
        "joint1",
        {"xacro": (-1.0, 1.0), "safety": (-1.0, 1.0001)},
        tolerance=1e-6,
    )
    assert len(issues) == 1
    assert "upper" in issues[0]


def test_correction_parser_accepts_a_string(tmp_path):
    path = tmp_path / "runner.py"
    path.write_text("DEFAULT_CORRECTION_MM = '13.7,3.9,8.9'\n")
    assert v.correction_values(path) == (13.7, 3.9, 8.9)


def test_correction_parser_accepts_a_tuple(tmp_path):
    path = tmp_path / "runner.py"
    path.write_text("EMPIRICAL_CORRECTION_MM = (13.7, 3.9, 8.9)\n")
    assert v.correction_values(path) == (13.7, 3.9, 8.9)


def test_fast_motion_defaults_parser(tmp_path):
    path = tmp_path / "fast_motion.py"
    path.write_text(
        "parser.add_argument('--vmax', type=float, default=4.0)\n"
        "parser.add_argument('--amax', type=float, default=20.0)\n"
    )
    assert v.fast_motion_defaults(path) == {
        "vmax_rad_s": 4.0,
        "amax_rad_s2": 20.0,
    }


def test_view_pose_parser_accepts_list_and_string(tmp_path):
    list_path = tmp_path / "list.py"
    list_path.write_text(
        "DEFAULT_VIEW_POSE = "
        "[0.0, 0.2574, -1.18773, -1.70, -0.01676]\n"
    )
    string_path = tmp_path / "string.py"
    string_path.write_text(
        "DEFAULT_VIEW_POSE = '0,0.2574,-1.18773,-1.70,-0.01676'\n"
    )
    expected = (0.0, 0.2574, -1.18773, -1.70, -0.01676)
    assert v.view_pose(list_path) == expected
    assert v.view_pose(string_path) == expected


def test_view_pose_parser_rejects_wrong_length(tmp_path):
    path = tmp_path / "short.py"
    path.write_text("DEFAULT_VIEW_POSE = [0.0, 1.0]\n")
    with pytest.raises(ValueError):
        v.view_pose(path)
