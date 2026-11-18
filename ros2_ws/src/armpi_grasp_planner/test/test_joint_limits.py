"""Unit tests for the joint-limit margin policy."""

from armpi_grasp_planner.joint_limits import (
    check_joint_margin,
    clamp_joints,
    shrink_limits,
)
import pytest

LIMITS = {
    'joint1': (-1.8, 1.8),
    'joint2': (-0.66, 1.3),
}
NAMES = ('joint1', 'joint2')


def test_shrink_limits_shrinks_both_sides():
    """The margin is removed from both bounds of every joint."""
    shrunk = shrink_limits(LIMITS, 0.05)
    assert shrunk['joint1'] == pytest.approx((-1.75, 1.75))
    assert shrunk['joint2'] == pytest.approx((-0.61, 1.25))


def test_shrink_limits_rejects_negative_margin():
    """A negative margin is a programming error, not a wider range."""
    with pytest.raises(ValueError):
        shrink_limits(LIMITS, -0.01)


def test_shrink_limits_rejects_empty_range():
    """A margin that consumes the whole range is refused."""
    with pytest.raises(ValueError):
        shrink_limits({'joint': (0.0, 0.05)}, 0.03)


def test_check_joint_margin_flags_exact_bound():
    """Regression: the table-alignment plan sat exactly on joint2's bound."""
    limits = {
        'joint1': (-1.8012, 1.8012),
        'joint2': (-0.6618, 1.2985),
    }
    report = check_joint_margin(
        (0.0, -0.6618), limits, 0.05, joint_names=NAMES
    )
    assert not report['ok']
    assert report['min_margin_rad'] == pytest.approx(0.0, abs=1e-9)
    assert len(report['violations']) == 1
    violation = report['violations'][0]
    assert violation['joint'] == 'joint2'
    assert violation['side'] == 'lower'
    assert violation['distance_rad'] == pytest.approx(0.0, abs=1e-9)


def test_check_joint_margin_accepts_interior_pose():
    """A pose inside the shrunk range passes with the expected minimum."""
    report = check_joint_margin((0.0, 0.0), LIMITS, 0.05, joint_names=NAMES)
    assert report['ok']
    assert report['violations'] == []
    assert report['min_margin_rad'] == pytest.approx(0.66)


def test_check_joint_margin_flags_outside_value():
    """A value beyond the limit has a negative distance and fails."""
    report = check_joint_margin(
        (0.0, -0.70), LIMITS, 0.05, joint_names=NAMES
    )
    assert not report['ok']
    assert report['violations'][0]['distance_rad'] < 0.0


def test_check_joint_margin_rejects_wrong_length():
    """The report is only defined for one value per named joint."""
    with pytest.raises(ValueError):
        check_joint_margin((0.0,), LIMITS, 0.05, joint_names=NAMES)


def test_clamp_joints_moves_values_into_margin():
    """Seeds outside the shrunk range are clamped and reported."""
    values, changes = clamp_joints(
        (-1.9, -0.6618), LIMITS, 0.05, joint_names=NAMES
    )
    assert values == pytest.approx((-1.75, -0.61))
    assert [change['joint'] for change in changes] == [
        'joint1',
        'joint2',
    ]
    assert changes[0]['from'] == pytest.approx(-1.9)
    assert changes[0]['to'] == pytest.approx(-1.75)


def test_clamp_joints_leaves_interior_values_alone():
    """A valid seed is returned unchanged with no change records."""
    values, changes = clamp_joints((0.1, 0.2), LIMITS, 0.05, joint_names=NAMES)
    assert values == pytest.approx((0.1, 0.2))
    assert changes == []
