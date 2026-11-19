"""Tests for the gripper tolerance-envelope runner (no robot)."""

import math
import os
import sys

TOOLS = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(os.path.dirname(TOOLS))
for path in (TOOLS, os.path.join(REPO, 'tools', 'phase2')):
    if path not in sys.path:
        sys.path.insert(0, path)

import gripper_envelope_run as env  # noqa: E402


def test_build_schedule_covers_each_point_once_per_block():
    schedule = env.build_schedule([0, 3, -3], [0, 3, -3], [0, 3, -3], 2, 7)
    # x keeps 3 points; y and z drop zero, so 2 each: 7 per block, 14 total.
    assert len(schedule) == 14
    counts = {}
    for block, axis, offset in schedule:
        counts[(axis, offset)] = counts.get((axis, offset), 0) + 1
    assert counts[('x', 0)] == 2
    assert counts.get(('y', 0)) is None
    assert counts.get(('z', 0)) is None
    assert counts[('x', 3)] == 2
    assert counts[('y', -3)] == 2
    assert counts[('z', 3)] == 2
    for block in (1, 2):
        assert sum(1 for entry in schedule if entry[0] == block) == 7


def test_build_schedule_is_seeded_and_reproducible():
    first = env.build_schedule([0, 3, -3, 6], [0, 3], [0], 2, 11)
    second = env.build_schedule([0, 3, -3, 6], [0, 3], [0], 2, 11)
    assert first == second
    assert first != env.build_schedule([0, 3, -3, 6], [0, 3], [0], 2, 12)


def _report(yaw_rad):
    return {'selected': {'top': {'yaw_rad': yaw_rad}}}


def test_relative_yaw_folds_a_square_face():
    """A face 5 deg off the arm is reported as 5 deg, not 85."""
    point = [0.23, 0.0, -0.05]  # azimuth 0
    assert abs(env.relative_yaw_deg(_report(math.radians(5.0)), point)
               - 5.0) < 1e-6
    assert abs(env.relative_yaw_deg(_report(math.radians(85.0)), point)
               + 5.0) < 1e-6


def test_relative_yaw_detects_the_diagonal_case():
    """The 2026-11-19 failed placement: face 10 deg, arm at 19 deg."""
    point = [0.216, 0.075, -0.048]
    relative = env.relative_yaw_deg(_report(math.radians(80.0)), point)
    assert abs(abs(relative) - 29.0) < 1.0
