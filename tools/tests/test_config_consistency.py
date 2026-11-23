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
    assert set(report.arm_joints) == set(v.ARM_JOINT_FILES)
    assert all(
        joints == v.CANONICAL_ARM_JOINTS
        for joints in report.arm_joints.values()
    )
    assert set(report.servo_ids) == set(v.SERVO_ID_FILES)
    for ids in report.servo_ids.values():
        for joint, expected in v.CANONICAL_SERVO_IDS.items():
            assert ids[joint] == expected
    assert set(report.gripper_servo_ids) == set(v.GRIPPER_ID_FILES)
    assert all(value == 1 for value in report.gripper_servo_ids.values())
    assert set(report.profiles) >= {"moveit", "step", "pose", "fast"}
    assert report.profiles["moveit"]["max_velocity_rad_s"] == 1.2
    assert report.profiles["moveit"]["max_acceleration_rad_s2"] == 0.5
    assert report.profiles["pose"]["max_velocity_rad_s"] == 0.5
    assert report.profiles["fast"]["max_velocity_rad_s"] == 4.0
    assert report.profiles["fast"]["max_acceleration_rad_s2"] == 20.0
    assert report.profiles["fast"]["requires_bus_verification"] is True


def test_safety_profiles_parser(tmp_path):
    path = tmp_path / "safety_limits.yaml"
    path.write_text(
        "profiles:\n"
        "  moveit:\n"
        "    max_velocity_rad_s: 1.2\n"
        "  fast:\n"
        "    max_velocity_rad_s: 4.0\n"
    )
    profiles = v.safety_profiles(path)
    assert profiles["moveit"]["max_velocity_rad_s"] == 1.2
    assert profiles["fast"]["max_velocity_rad_s"] == 4.0


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


def test_arm_joint_and_servo_id_parsers(tmp_path):
    path = tmp_path / "constants.py"
    path.write_text(
        "ARM_JOINTS = ('joint1', 'joint2', 'joint3', 'joint4', 'joint5')\n"
        "SERVO_IDS = {'joint1': 6, 'joint2': 5, 'joint3': 4, "
        "'joint4': 3, 'joint5': 2, 'gripper': 1}\n"
        "GRIPPER_SERVO_ID = 1\n"
    )
    assert v.arm_joints(path) == v.CANONICAL_ARM_JOINTS
    ids = v.servo_ids(path)
    for joint, expected in v.CANONICAL_SERVO_IDS.items():
        assert ids[joint] == expected
    assert ids["gripper"] == 1
    assert v.gripper_servo_id(path) == 1
