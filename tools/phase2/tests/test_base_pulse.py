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
    # the cap pair is consistent: duty 20 on a 45-degree diagonal is 28.3 on
    # the two active wheels, inside the 30 % wheel cap.
    ok = pulse.wheel_speeds(1.0, 1.0, 20.0)
    peak = max(abs(v) for v in ok.values())
    assert peak == pytest.approx(20.0 * math.sqrt(2.0))
    assert peak < pulse.MAX_WHEEL_DUTY
    pulse.check_duty(20.0, ok)


def test_duty_above_the_cap_is_rejected():
    with pytest.raises(ValueError):
        pulse.check_duty(21.0, pulse.wheel_speeds(1.0, 0.0, 21.0))
    with pytest.raises(ValueError):
        pulse.check_duty(0.0, pulse.wheel_speeds(1.0, 0.0, 0.0))


def test_wheel_cap_is_enforced_independently():
    with pytest.raises(ValueError):
        pulse.check_duty(20.0, {1: 31.0, 2: 0.0, 3: 0.0, 4: 0.0})


def test_speed_to_duty_uses_the_measured_scale():
    assert pulse.speed_to_duty(55.0) == pytest.approx(10.01)
    assert pulse.speed_to_duty(110.0) == pytest.approx(pulse.MAX_DUTY)
    with pytest.raises(ValueError):
        pulse.speed_to_duty(111.0)


def test_travel_matches_the_measured_speed():
    # 10 % duty for 2.0 s is ~110 mm at the measured 55 mm/s.
    assert pulse.travel_mm(10.0, 2.0) == pytest.approx(109.9, abs=0.1)
