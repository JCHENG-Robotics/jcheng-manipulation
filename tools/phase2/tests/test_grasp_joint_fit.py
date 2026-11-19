"""Tests for the deep-family grasp joint fit."""

import numpy as np

import grasp_joint_fit as fit


class FakeKinematics:
    """A planar arm: j1 is a yaw, j4 sets the reach."""

    def tcp(self, joints):
        yaw, reach = joints[0], joints[3]
        radius = 0.20 + 0.10 * reach
        return np.array([radius * np.cos(yaw), radius * np.sin(yaw), -0.04])


def test_fit_recovers_a_reachable_target():
    """The solver lands on a reachable xy target."""
    target = [0.15, -0.01, -0.05]
    result = fit.fit(target, kinematics=FakeKinematics())
    assert result['xy_error_mm'] < 0.01
    assert result['joints'][1] == fit.DEEP_J2
    assert result['joints'][2] == fit.DEEP_J3
    assert result['joints'][4] == fit.DEFAULT_J5


def test_z_is_not_fitted():
    """A target far above the family leaves the xy fit exact."""
    target = [0.15, 0.0, 0.5]
    result = fit.fit(target, kinematics=FakeKinematics())
    assert result['xy_error_mm'] < 0.01
    assert abs(result['residual_mm'][2]) > 500.0


def test_joint4_limit_is_respected():
    """An unreachable radius clamps joint4 and reports the residual."""
    target = [0.02, 0.0, -0.05]
    result = fit.fit(target, kinematics=FakeKinematics())
    assert result['joints'][3] >= fit.J4_LIMITS[0] - 1e-9
    assert result['xy_error_mm'] > 10.0


def test_format_joints_uses_four_values():
    """The command line takes j1..j4 only (j5 is fixed at zero)."""
    text = fit.format_joints([0.1, -0.2, -1.8, -0.34, 0.0])
    assert text == '0.1000,-0.2000,-1.8000,-0.3400'


def test_real_model_reproduces_the_recorded_validation_fit():
    """The KDL model reproduces the 2026-11-19 validation joints."""
    import pytest

    pytest.importorskip('armpi_grasp_planner.reachability')
    result = fit.fit([0.23992, -0.00205, -0.04166])
    joints = result['joints']
    assert abs(joints[0] - (-0.015)) < 0.005
    assert abs(joints[3] - (-0.340)) < 0.005
    assert result['xy_error_mm'] < 0.01
    assert abs(result['residual_mm'][2] - 4.4) < 0.5
