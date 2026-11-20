"""Tests for the S-curve motion profile and the vendor pulse mapping."""

import math

import motion_profile as profile
import vendor_servo_map as servos

VIEW_POSE = [0.0, 0.2574, -1.18773, -1.70, -0.01676]
APPROACH = [0.0, -0.6618, -1.8012, 0.0, 0.0]
GRASP = [0.0, -0.6618, -1.8012, -0.32, 0.0]


def test_endpoints_are_exact():
    """The first waypoint is the measured pose, the last is the target."""
    times, points, info = profile.plan_move(VIEW_POSE, APPROACH, 1.2, 6.0)
    assert points[0] == VIEW_POSE
    assert points[-1] == APPROACH
    assert times[0] == 0.0
    assert times[-1] == info['total']


def test_times_are_monotonic_and_capped_by_dt():
    """Segments never exceed dt and never go backwards."""
    times, _, _ = profile.plan_move(VIEW_POSE, APPROACH, 1.2, 6.0, dt=0.05)
    steps = [b - a for a, b in zip(times, times[1:])]
    assert all(step > 0.0 for step in steps)
    assert max(steps) <= 0.05 + 1e-12


def test_velocity_and_acceleration_bounds():
    """Finite differences stay inside vmax and amax."""
    vmax, amax = 1.2, 6.0
    times, points, _ = profile.plan_move(VIEW_POSE, APPROACH, vmax, amax)
    for joint in range(5):
        velocities = [
            (points[index + 1][joint] - points[index][joint])
            / (times[index + 1] - times[index])
            for index in range(len(points) - 1)
        ]
        assert max(abs(value) for value in velocities) <= vmax * 1.001
        accelerations = [
            (velocities[index + 1] - velocities[index])
            / (times[index + 2] - times[index])
            for index in range(len(velocities) - 1)
        ]
        assert max(abs(value) for value in accelerations) <= amax * 1.05


def test_every_joint_follows_the_same_parameter():
    """The path is a straight line in joint space (all joints together)."""
    times, points, info = profile.plan_move(VIEW_POSE, APPROACH, 1.2, 6.0)
    for point in points:
        fractions = [
            (value - start) / (target - start)
            for value, start, target in zip(point, VIEW_POSE, APPROACH)
            if abs(target - start) > 0.1
        ]
        assert max(fractions) - min(fractions) < 1e-12


def test_short_move_is_triangular():
    """A short move never reaches vmax and takes two ramps."""
    v_peak, t_accel, t_cruise, total = profile.profile(0.05, 3.0, 6.0)
    assert v_peak < 3.0
    assert t_cruise == 0.0
    assert math.isclose(total, 2.0 * t_accel)


def test_long_move_cruises_at_vmax():
    """A long move has a constant-velocity middle at exactly vmax."""
    v_peak, t_accel, t_cruise, total = profile.profile(1.0, 1.0, 6.0)
    assert math.isclose(v_peak, 1.0)
    assert t_cruise > 0.0
    assert math.isclose(total, 2.0 * t_accel + t_cruise)


def test_zero_move_has_a_single_point():
    """No displacement produces no motion points."""
    times, points, info = profile.plan_move(GRASP, GRASP, 1.2, 6.0)
    assert times == [0.0]
    assert points == [GRASP]
    assert info['total'] == 0.0


def test_segment_shorter_than_the_vendor_floor_is_rejected():
    """A dt under the vendor's 20 ms clamp would silently stretch the move."""
    try:
        profile.plan_move(GRASP, APPROACH, 1.2, 6.0, dt=0.01)
    except ValueError as error:
        assert 'dt' in str(error)
    else:  # pragma: no cover - the raise is the behaviour under test
        raise AssertionError('dt under the vendor floor was accepted')


def test_longer_amax_reduces_the_duration():
    """The acceleration limit, not vmax, binds these short arm moves."""
    _, _, slow = profile.plan_move(VIEW_POSE, APPROACH, 1.2, 6.0)
    _, _, fast = profile.plan_move(VIEW_POSE, APPROACH, 1.2, 12.0)
    assert fast['total'] < slow['total']
    assert slow['total'] < 2.0


def test_validated_poses_pass_the_bounds_check():
    """The view, approach and grasp poses are inside the servo limits."""
    warnings = servos.check_waypoints([VIEW_POSE, APPROACH, GRASP])
    assert warnings, 'the view pose should warn on the adopted joint4 bound'


def test_a_pose_past_the_servo_limit_is_refused():
    """A target far below the joint4 EEPROM limit is refused."""
    bad = list(GRASP)
    bad[3] = -2.0   # 22 pulse, below the 100-pulse EEPROM limit
    try:
        servos.check_waypoints([bad])
    except ValueError as error:
        assert 'joint4' in str(error)
    else:  # pragma: no cover
        raise AssertionError('an out-of-range pose was accepted')


def test_gripper_pulses_match_the_measured_stops():
    """The measured grip angles map to the measured raw pulses."""
    assert servos.gripper_pulse(-2.22) == 170
    assert servos.gripper_pulse(-1.6) == 318
    assert servos.gripper_pulse(-0.63) == 550


def test_waypoint_pulses_use_the_flipped_joint3():
    """joint3 is flipped: a negative angle is a register above 500."""
    pulses = servos.waypoint_pulses([GRASP])['joint3']
    assert pulses == [930]
