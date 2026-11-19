"""Tests for the vector base-pulse wheel mixing (no ROS needed)."""

import math

import pytest

import base_pulse_vector as pulse


def test_forward_signs():
    wheels = pulse.wheel_speeds(1.0, 0.0, 10.0)
    assert wheels == {1: 10.0, 2: -10.0, 3: -10.0, 4: 10.0}


def test_left_signs():
    wheels = pulse.wheel_speeds(0.0, 1.0, 10.0)
    assert wheels == {1: 10.0, 2: 10.0, 3: 10.0, 4: 10.0}


def test_backward_is_the_forward_negation():
    forward = pulse.wheel_speeds(1.0, 0.0, 10.0)
    backward = pulse.wheel_speeds(-1.0, 0.0, 10.0)
    for motor_id in forward:
        assert backward[motor_id] == pytest.approx(-forward[motor_id])


def test_diagonal_is_normalised():
    # a 45 deg forward-left direction at duty 10: the rear-right and
    # front-left wheels cancel and the other two take 10 * sqrt(2).
    wheels = pulse.wheel_speeds(1.0, 0.0, 10.0)
    diagonal = pulse.wheel_speeds(1.0, 1.0, 10.0)
    assert diagonal[1] == pytest.approx(10.0 * math.sqrt(2.0))
    assert diagonal[2] == pytest.approx(0.0)
    assert diagonal[3] == pytest.approx(0.0)
    assert diagonal[4] == pytest.approx(10.0 * math.sqrt(2.0))
    assert max(abs(v) for v in diagonal.values()) < pulse.MAX_WHEEL_DUTY
    assert wheels[1] < diagonal[1]


def test_zero_direction_is_rejected():
    with pytest.raises(ValueError):
        pulse.wheel_speeds(0.0, 0.0, 10.0)


def test_diagonal_duty_margin():
    # duty 10 stays under the 15 % wheel cap; duty 12 on a diagonal does not,
    # so the CLI rejects it.
    ok = pulse.wheel_speeds(1.0, 1.0, 10.0)
    assert max(abs(v) for v in ok.values()) < pulse.MAX_WHEEL_DUTY
    too_much = pulse.wheel_speeds(1.0, 1.0, 12.0)
    assert max(abs(v) for v in too_much.values()) > pulse.MAX_WHEEL_DUTY
