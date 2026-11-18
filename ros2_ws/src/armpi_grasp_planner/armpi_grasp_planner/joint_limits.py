"""
Joint-limit margin policy shared by the grasp sampler and the evaluator.

The ArmPi limits in the URDF and ``joint_limits.yaml`` are conservative
command limits (servo EEPROM limits minus 20 pulses), not a metrology claim;
see ``docs/phase2/JOINT_LIMITS.md``. A pose that sits exactly on one of those
bounds leaves no room for IK clamping, servo tracking error or a later
re-plan, so the planner keeps an additional margin:

* the joint-space sampler grids the limits shrunk by the margin;
* the evaluator rejects an IK solution that is closer than the margin to any
  bound (status ``JOINT_LIMIT_MARGIN``) instead of planning it;
* IK seeds are clamped into the shrunk range before they are sent, because
  some historical seeds (for example SRDF ``init_pose``) predate the adopted
  limits.

The functions are pure Python so they can be unit-tested without ROS.
"""

# 0.03 rad = 1.7 deg, about 7 servo pulses. This matches the calibration
# pose selector's joint margin. The G5 trajectory recorder measured a
# worst-case joint tracking error of 0.008 rad, so this is about 3.6 times the
# observed tracking error. A 0.05 rad margin was also tried, but joint2 has a
# long lever arm: it shifts the TCP by about 30 mm and removed every candidate
# for the 9 cm table-alignment target, so 0.03 is the default.
DEFAULT_JOINT_LIMIT_MARGIN_RAD = 0.03


def _names(limits, joint_names):
    if joint_names is None:
        return list(limits)
    return list(joint_names)


def shrink_limits(limits, margin_rad):
    """Return ``{joint: (lower, upper)}`` shrunk by *margin_rad* on both sides."""
    if margin_rad < 0.0:
        raise ValueError('joint-limit margin must be non-negative')
    shrunk = {}
    for name, (lower, upper) in limits.items():
        if upper - lower <= 2.0 * margin_rad:
            raise ValueError(
                'joint-limit margin {} rad leaves no range for {}'.format(
                    margin_rad, name
                )
            )
        shrunk[name] = (lower + margin_rad, upper - margin_rad)
    return shrunk


def check_joint_margin(joints, limits, margin_rad, joint_names=None):
    """
    Report how close *joints* come to the limits.

    ``joints`` is in ``joint_names`` order (the order of ``limits`` when
    omitted). A joint fails when its distance to the nearer bound is smaller
    than *margin_rad*; a value outside the limits has a negative distance and
    also fails.
    """
    if margin_rad < 0.0:
        raise ValueError('joint-limit margin must be non-negative')
    names = _names(limits, joint_names)
    if len(joints) != len(names):
        raise ValueError(
            'expected {} joint values, got {}'.format(len(names), len(joints))
        )
    violations = []
    distances = []
    for name, value in zip(names, joints):
        lower, upper = limits[name]
        value = float(value)
        to_lower = value - lower
        to_upper = upper - value
        distance = min(to_lower, to_upper)
        distances.append(distance)
        if distance < margin_rad - 1e-9:
            violations.append(
                {
                    'joint': name,
                    'value': value,
                    'lower': lower,
                    'upper': upper,
                    'distance_rad': distance,
                    'side': 'lower' if to_lower <= to_upper else 'upper',
                }
            )
    return {
        'checked': True,
        'ok': not violations,
        'margin_rad': float(margin_rad),
        'min_margin_rad': min(distances) if distances else None,
        'violations': violations,
    }


def clamp_joints(joints, limits, margin_rad, joint_names=None):
    """
    Clamp joint values into the margin-shrunk limits.

    Returns ``(clamped_joints, changes)``. *changes* lists one dict per value
    that moved, with ``joint``, ``from`` and ``to``. A negative margin is a
    programming error and raises ``ValueError``.
    """
    if margin_rad < 0.0:
        raise ValueError('joint-limit margin must be non-negative')
    names = _names(limits, joint_names)
    if len(joints) != len(names):
        raise ValueError(
            'expected {} joint values, got {}'.format(len(names), len(joints))
        )
    values = []
    changes = []
    for name, value in zip(names, joints):
        lower, upper = limits[name]
        low = lower + margin_rad
        high = upper - margin_rad
        if high < low:
            raise ValueError(
                'joint-limit margin {} rad leaves no range for {}'.format(
                    margin_rad, name
                )
            )
        clamped = min(max(float(value), low), high)
        values.append(clamped)
        if clamped != float(value):
            changes.append(
                {'joint': name, 'from': float(value), 'to': clamped}
            )
    return tuple(values), changes
