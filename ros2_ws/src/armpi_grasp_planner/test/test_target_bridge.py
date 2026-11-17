"""Unit tests for the fiducial target bridge helpers."""

import math
from types import SimpleNamespace

from armpi_grasp_planner.target_bridge import (
    grasp_point_from_pose,
    marker_tilt_deg,
    pose_age_s,
)
import pytest


def pose(x=0.0, y=0.0, z=0.0, quaternion=(0.0, 0.0, 0.0, 1.0)):
    """Build a minimal Pose-like object for the pure helpers."""
    return SimpleNamespace(
        position=SimpleNamespace(x=x, y=y, z=z),
        orientation=SimpleNamespace(
            x=quaternion[0],
            y=quaternion[1],
            z=quaternion[2],
            w=quaternion[3],
        ),
    )


def test_pose_age_is_non_negative_and_zero_for_missing_stamp():
    """A missing stamp is treated as fresh; a past stamp has its age."""
    assert pose_age_s(10.0, 0.0) == 0.0
    assert pose_age_s(10.0, 9.5) == pytest.approx(0.5)
    assert pose_age_s(10.0, 11.0) == 0.0


def test_marker_tilt_uses_the_marker_z_axis():
    """An upward-facing marker has zero tilt; a flipped marker has 180."""
    assert marker_tilt_deg((0.0, 0.0, 0.0, 1.0)) == pytest.approx(0.0)
    flipped = (1.0, 0.0, 0.0, 0.0)
    assert marker_tilt_deg(flipped) == pytest.approx(180.0)


def test_grasp_point_applies_marker_frame_offset():
    """An offset along marker +z moves the grasp point along base +z."""
    point = grasp_point_from_pose(
        pose(0.1, 0.2, 0.3), (0.0, 0.0, 0.01)
    )
    assert point == pytest.approx((0.1, 0.2, 0.31))
    point = grasp_point_from_pose(pose(0.0, 0.0, 0.0), (0.01, 0.0, 0.0))
    assert point == pytest.approx((0.01, 0.0, 0.0))


def test_grasp_point_offset_rotates_with_the_marker():
    """A 90 degree marker yaw rotates a marker-frame x offset into base y."""
    yaw = math.pi / 2.0
    quaternion = (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))
    point = grasp_point_from_pose(
        pose(0.0, 0.0, 0.0, quaternion), (0.01, 0.0, 0.0)
    )
    assert point == pytest.approx((0.0, 0.01, 0.0), abs=1e-9)
