"""Tests for the joint-space reachability sampler."""

import math

from armpi_grasp_planner.joint_limits import (
    DEFAULT_JOINT_LIMIT_MARGIN_RAD,
)
from armpi_grasp_planner.reachability import _axis_values, ArmKinematics
import pytest


def test_axis_values_keeps_margin_boundary_and_grid_phase():
    """The margin boundary is added and interior grid points are kept."""
    values = _axis_values(-0.6618, 1.2985, 0.2, 0.05)
    assert values[0] == pytest.approx(-0.6118)
    assert values[-1] == pytest.approx(1.2485)
    assert -0.4618 in [round(value, 4) for value in values]
    assert all(value > -0.6618 and value < 1.2985 for value in values)


def test_axis_values_without_margin_is_the_historical_grid():
    """A zero margin reproduces the pre-margin endpoints exactly."""
    values = _axis_values(-0.6618, 1.2985, 0.2, 0.0)
    assert values[0] == pytest.approx(-0.6618)
    assert values[-1] == pytest.approx(1.1382)


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
    """
    Sampled candidates lie near the target and match their seed FK.

    The target comes from ``(0.3, 0.9, -1.6, -1.3, 0.4)``, which is inside the
    adopted EEPROM-bounded limits. The historical Phase 0 pose
    ``(0.3, 0.9, -1.8, -1.7, 0.4)`` is no longer usable: its joint4 = -1.7 is
    outside the adopted low bound -1.5917.
    """
    kinematics = ArmKinematics()
    target = (0.17746831500810384, 0.0347880210621325, 0.1629644615294479)
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


def test_sampler_keeps_margin_from_joint_limits():
    """Every sampled seed stays a full margin inside the adopted limits."""
    kinematics = ArmKinematics()
    target = (0.17746831500810384, 0.0347880210621325, 0.1629644615294479)
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
        for name, value in zip(
            kinematics.joint_names, candidate.seed_joints
        ):
            lower, upper = kinematics.limits[name]
            assert (
                value - lower
                >= DEFAULT_JOINT_LIMIT_MARGIN_RAD - 1e-9
            )
            assert (
                upper - value
                >= DEFAULT_JOINT_LIMIT_MARGIN_RAD - 1e-9
            )


def test_sampler_rejects_negative_joint_limit_margin():
    """A negative margin is reported instead of widening the grid."""
    kinematics = ArmKinematics()
    with pytest.raises(ValueError):
        kinematics.sample_candidates(
            (0.1775, 0.0348, 0.1630),
            tcp_offset_m=0.0745612,
            joint_limit_margin_rad=-0.01,
        )
