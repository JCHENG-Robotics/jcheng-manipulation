"""Tests for the fast-motion e2e base planner (no robot)."""

import math
import os
import sys

import pytest

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(TOOLS))
for path in (TOOLS, os.path.join(REPO, 'tools', 'phase2'),
             os.path.join(REPO, 'ros2_ws', 'src', 'armpi_analysis')):
    if path not in sys.path:
        sys.path.insert(0, path)

import fast_motion_e2e as e2e  # noqa: E402


def test_inside_the_band_needs_no_pulse():
    assert e2e.base_pulse_plan([0.2305, 0.0, -0.048]) is None
    assert e2e.base_pulse_plan([0.226, 0.0, 0.0]) is None
    assert e2e.base_pulse_plan([0.234, 0.0, 0.0]) is None


def test_far_cube_pulses_toward_the_forward_axis():
    # the 2026-11-20 cube at (0.2824, -0.0552): out of the band and off-axis.
    vx, vy, duration = e2e.base_pulse_plan([0.2824, -0.0552, -0.048])
    dx, dy = 0.2824 - 0.230, -0.0552
    travel = math.hypot(dx, dy)
    assert vx == pytest.approx(dx / travel)
    assert vy == pytest.approx(dy / travel)
    assert duration == pytest.approx(
        travel * 1000.0 / (e2e.BASE_DUTY * e2e.BASE_SPEED_PER_DUTY), rel=0.02)


def test_a_lateral_offset_is_strafed_out():
    # the 2026-11-20 v2 run left the cube at (0.2200, -0.0530): the radius was
    # inside the band but joint1 had to yaw -0.288 rad, so the plan is a pure
    # strafe back onto the forward axis.
    vx, vy, duration = e2e.base_pulse_plan([0.2200, -0.0530, -0.048])
    dx, dy = 0.2200 - 0.230, -0.0530
    travel = math.hypot(dx, dy)
    assert vx == pytest.approx(dx / travel)
    assert vy == pytest.approx(dy / travel)
    assert vy < -0.9          # mostly a right strafe
    assert duration == pytest.approx(
        travel * 1000.0 / (e2e.BASE_DUTY * e2e.BASE_SPEED_PER_DUTY), rel=0.02)


def test_near_cube_pulses_away():
    vx, vy, _ = e2e.base_pulse_plan([0.200, 0.0, -0.048])
    assert vx < 0.0
    assert vy == pytest.approx(0.0)


def test_a_dead_band_correction_is_skipped():
    # 0.5 mm outside the band is inside the ~0.15 s dead band: do nothing.
    assert e2e.base_pulse_plan([0.2345, 0.0, 0.0]) is None


def test_duration_is_clamped():
    # a cube 0.6 m away would need 3.7 s at duty 20: cap at 3 s.
    _, _, duration = e2e.base_pulse_plan([0.600, 0.0, 0.0])
    assert duration == pytest.approx(3.0)


def test_duty_20_is_twice_duty_10():
    _, _, fast = e2e.base_pulse_plan([0.30, 0.0, 0.0], duty=20.0)
    _, _, slow = e2e.base_pulse_plan([0.30, 0.0, 0.0], duty=10.0)
    assert slow == pytest.approx(2.0 * fast)


def test_origin_is_rejected():
    with pytest.raises(ValueError):
        e2e.base_pulse_plan([0.0, 0.0, 0.0])
