"""Tests for the joint-space reachability sampler."""

import math

from armpi_grasp_planner.reachability import ArmKinematics
import pytest


def _distance(a, b):
    return math.sqrt(sum((a[index] - b[index]) ** 2 for index in range(3)))


def test_local_fk_matches_recorded_moveit_pose():
    """Local KDL FK reproduces the MoveIt pose recorded on 2026-11-16."""
    kinematics = ArmKinematics()
    frame = kinematics.forward((0.3, 0.9, -1.8, -1.7, 0.4))
    assert frame.p.x() == pytest.approx(0.1382, abs=1e-4)
    assert frame.p.y() == pytest.approx(0.0225, abs=1e-4)
    assert frame.p.z() == pytest.approx(0.0896, abs=1e-4)
    assert frame.M[0, 2] == pytest.approx(0.4876, abs=1e-4)
    assert frame.M[1, 2] == pytest.approx(0.1579, abs=1e-4)
    assert frame.M[2, 2] == pytest.approx(-0.8587, abs=1e-4)


def test_sampler_returns_fk_consistent_candidates():
    """Sampled candidates lie near the target and match their seed FK."""
    kinematics = ArmKinematics()
    target = (0.1382, 0.0225, 0.0896)
    candidates = kinematics.sample_candidates(
        target,
        tcp_offset_m=0.0745612,
        radius_m=0.02,
        position_step_rad=0.3,
        roll_step_rad=0.5,
        max_candidates=20,
    )
    assert candidates
    for candidate in candidates:
        assert candidate.target_offset_m <= 0.02 + 1e-9
        frame = kinematics.forward(candidate.seed_joints)
        tcp = (frame.p.x(), frame.p.y(), frame.p.z())
        if candidate.source == 'joint_sample':
            assert _distance(tcp, candidate.tcp_position) < 1e-6
        else:
            assert _distance(tcp, candidate.tcp_position) <= 0.02 + 1e-9
