"""Tests for the read-only limit / correction consistency checker."""

from pathlib import Path

import verify_config_consistency as v

ROOT = Path(__file__).resolve().parents[2]


def test_current_repo_is_consistent():
    report = v.check_repo(ROOT)
    assert report.issues == []
    assert set(report.limits) == set(v.JOINTS)
    expected = set(v.CORRECTION_FILES) | {v.TAKEOVER}
    assert set(report.corrections) == expected
    assert report.corrections[v.TAKEOVER] == (13.7, 3.9, 8.9)


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
