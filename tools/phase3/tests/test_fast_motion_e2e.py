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


class FakeBase:
    """Record base pulses instead of driving the chassis."""

    def __init__(self):
        self.pulses = []

    def base_pulse(self, vx, vy, duty, duration):
        self.pulses.append({
            'vx': vx, 'vy': vy, 'duty': duty, 'duration': duration,
            'nominal_travel_mm': duty * duration * 4.9,
            'subscribers_matched': 1,
        })
        return self.pulses[-1]


def test_net_travel_sums_the_commanded_pulses():
    steps = [
        {'vx': 1.0, 'vy': 0.0, 'duty': 20.0, 'duration': 1.0,
         'subscribers_matched': 1},
        {'vx': 0.0, 'vy': 1.0, 'duty': 20.0, 'duration': 0.5,
         'subscribers_matched': 1},
    ]
    net = e2e.net_base_travel(steps)
    assert net[0] == pytest.approx(0.098)   # 20 * 1.0 * 4.9 mm
    assert net[1] == pytest.approx(0.049)


def test_a_dropped_pulse_is_not_undone():
    # a pulse with no matched subscriber never moved the robot
    steps = [{'vx': 1.0, 'vy': 0.0, 'duty': 20.0, 'duration': 1.0,
              'subscribers_matched': 0}]
    assert list(e2e.net_base_travel(steps)) == [0.0, 0.0]
    robot = FakeBase()
    report = e2e.return_home(robot, steps)
    assert report['pulses'] == [] and robot.pulses == []


def test_return_home_reverses_the_approach():
    steps = [{'vx': 1.0, 'vy': 0.0, 'duty': 20.0, 'duration': 1.0,
              'subscribers_matched': 1},
             {'vx': 0.0, 'vy': -1.0, 'duty': 20.0, 'duration': 1.0,
              'subscribers_matched': 1}]
    net = e2e.net_base_travel(steps)
    distance = float(math.hypot(net[0], net[1]))
    robot = FakeBase()
    report = e2e.return_home(robot, steps)
    assert report['distance_m'] == pytest.approx(distance)
    assert len(robot.pulses) == 1
    pulse = robot.pulses[0]
    # opposite direction, and the commanded duration undoes the travel
    assert pulse['vx'] == pytest.approx(-net[0] / distance)
    assert pulse['vy'] == pytest.approx(-net[1] / distance)
    assert pulse['vy'] > 0.0        # the outward -y pulse is strafed back
    assert pulse['duration'] == pytest.approx(
        distance * 1000.0 / (20.0 * 4.9), rel=0.01)


def test_return_home_chunks_at_three_seconds():
    steps = [{'vx': 1.0, 'vy': 0.0, 'duty': 20.0, 'duration': 3.0,
              'subscribers_matched': 1},
             {'vx': 1.0, 'vy': 0.0, 'duty': 20.0, 'duration': 3.0,
              'subscribers_matched': 1}]
    robot = FakeBase()
    report = e2e.return_home(robot, steps)
    assert all(pulse['duration'] <= 3.0 + 1e-9 for pulse in robot.pulses)
    assert sum(pulse['duration'] for pulse in robot.pulses) == pytest.approx(
        report['distance_m'] * 1000.0 / (20.0 * 4.9), rel=0.01)
