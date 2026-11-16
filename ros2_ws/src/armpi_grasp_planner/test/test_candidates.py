"""Unit tests for grasp-candidate geometry and policy checks."""

import math

from armpi_grasp_planner.candidates import (
    assess_sanity,
    DEFAULT_TCP_OFFSET_M,
    generate_candidates,
    mark_exact_fk,
    sort_candidates,
)
from armpi_grasp_planner.geometry import (
    decompose_tool_orientation,
    matrix_from_axes,
    matrix_from_quaternion,
    quaternion_from_matrix,
    rotation_angle_deg,
    rotation_from_tool_angles,
    tool_axes,
    wrap_degrees,
)
import pytest


def _dot(a, b):
    return sum(x * y for x, y in zip(a, b))


def _cross(a, b):
    return (
        a[1] * b[2] - a[2] * b[1],
        a[2] * b[0] - a[0] * b[2],
        a[0] * b[1] - a[1] * b[0],
    )


def _assert_vector_close(actual, expected, tolerance=1e-9):
    assert actual == pytest.approx(expected, abs=tolerance)


def test_top_down_frame():
    """A zero-pitch, zero-yaw, zero-roll frame points straight down."""
    x_axis, y_axis, z_axis = tool_axes(0.0, 0.0, 0.0)
    _assert_vector_close(z_axis, (0.0, 0.0, -1.0))
    _assert_vector_close(y_axis, (0.0, 1.0, 0.0))
    _assert_vector_close(x_axis, (-1.0, 0.0, 0.0))


def test_tool_frame_is_right_handed_and_orthonormal():
    """The candidate frame is a proper rotation for arbitrary parameters."""
    for pitch, yaw, roll in ((15, 30, -20), (45, 0, 0), (60, 170, 40)):
        x_axis, y_axis, z_axis = tool_axes(pitch, yaw, roll)
        assert _dot(x_axis, x_axis) == pytest.approx(1.0)
        assert _dot(y_axis, y_axis) == pytest.approx(1.0)
        assert _dot(z_axis, z_axis) == pytest.approx(1.0)
        assert _dot(x_axis, y_axis) == pytest.approx(0.0, abs=1e-9)
        determinant = _dot(x_axis, _cross(y_axis, z_axis))
        assert determinant == pytest.approx(1.0)


def test_decomposition_round_trip():
    """Decomposing and rebuilding a candidate frame is lossless."""
    for pitch in (8.0, 15.0, 33.3, 60.0, 89.0):
        for yaw in (-170.0, -30.0, 0.0, 45.0, 179.0):
            for roll in (-120.0, -20.0, 0.0, 35.0, 110.0):
                original = rotation_from_tool_angles(pitch, yaw, roll)
                parameters = decompose_tool_orientation(original)
                rebuilt = rotation_from_tool_angles(**parameters)
                assert rotation_angle_deg(original, rebuilt) < 1e-6


def test_decomposition_vertical_has_zero_yaw():
    """At zero pitch the approach yaw is unobservable and reported as zero."""
    original = rotation_from_tool_angles(0.0, 0.0, 35.0)
    parameters = decompose_tool_orientation(original)
    assert parameters['pitch_deg'] == pytest.approx(0.0, abs=1e-9)
    assert parameters['approach_yaw_deg'] == pytest.approx(0.0)
    assert parameters['roll_deg'] == pytest.approx(35.0)
    rebuilt = rotation_from_tool_angles(**parameters)
    assert rotation_angle_deg(original, rebuilt) < 1e-9


def test_quaternion_matrix_round_trip():
    """Quaternion conversion preserves the rotation."""
    for quaternion in (
        (0.0, 0.0, 0.0, 1.0),
        (0.0, 0.0, 1.0, 0.0),
        (0.5, -0.5, 0.5, 0.5),
        (0.1, 0.2, 0.3, math.sqrt(1.0 - 0.14)),
    ):
        matrix = matrix_from_quaternion(quaternion)
        rebuilt = quaternion_from_matrix(matrix)
        assert abs(_dot(quaternion, rebuilt)) == pytest.approx(1.0)
        rebuilt_matrix = matrix_from_quaternion(rebuilt)
        assert rotation_angle_deg(matrix, rebuilt_matrix) < 1e-9


def test_rotation_angle_reports_ninety_degrees():
    """A quarter turn about z is reported as 90 degrees."""
    identity = matrix_from_axes((1, 0, 0), (0, 1, 0), (0, 0, 1))
    rotated = matrix_from_axes((0, 1, 0), (-1, 0, 0), (0, 0, 1))
    assert rotation_angle_deg(identity, rotated) == pytest.approx(90.0)


def test_tcp_and_gripper_offsets():
    """The TCP and gripper targets are offset along the approach axis."""
    point = (0.15, 0.02, 0.08)
    candidates = generate_candidates(
        point,
        pitches_deg=[30.0],
        approach_yaws_deg=[20.0],
        rolls_deg=[10.0],
        standoffs_m=[0.02],
        tcp_offset_m=DEFAULT_TCP_OFFSET_M,
    )
    candidate = candidates[0]
    tool_z = candidate.tool_z
    expected_tcp = tuple(
        point[index] - 0.02 * tool_z[index] for index in range(3)
    )
    _assert_vector_close(candidate.tcp_position, expected_tcp)
    expected_gripper = tuple(
        expected_tcp[index] - DEFAULT_TCP_OFFSET_M * tool_z[index]
        for index in range(3)
    )
    _assert_vector_close(candidate.gripper_position, expected_gripper)
    distance = math.dist(candidate.tcp_position, candidate.gripper_position)
    assert distance == pytest.approx(DEFAULT_TCP_OFFSET_M)


def test_generated_grid_is_unique():
    """The Cartesian product has the expected size and unique IDs."""
    candidates = generate_candidates(
        (0.1, 0.0, 0.1),
        pitches_deg=[15.0, 30.0, 45.0],
        approach_yaws_deg=[0.0, 90.0],
        rolls_deg=[-20.0, 0.0, 20.0],
        standoffs_m=[0.0, 0.03],
    )
    assert len(candidates) == 36
    assert len({candidate.candidate_id for candidate in candidates}) == 36


def test_mark_exact_fk_and_sort():
    """The exact FK candidate is flagged and sorted first."""
    candidates = generate_candidates(
        (0.1, 0.0, 0.1),
        pitches_deg=[15.0, 30.0],
        approach_yaws_deg=[0.0],
        rolls_deg=[0.0],
        standoffs_m=[0.0, 0.03],
    )
    marked, found = mark_exact_fk(candidates, 30.0, 0.0, 0.0)
    assert found
    exact = [candidate for candidate in marked if candidate.is_exact_fk]
    assert len(exact) == 1
    assert exact[0].standoff_m == 0.0
    assert sort_candidates(marked)[0].is_exact_fk


def test_assess_sanity_rejects_steep_and_wide():
    """Policy checks reject steep approaches and oversized objects."""
    candidate = generate_candidates(
        (0.1, 0.0, 0.1),
        pitches_deg=[80.0],
        approach_yaws_deg=[0.0],
        rolls_deg=[0.0],
        standoffs_m=[0.0],
    )[0]
    assert not assess_sanity(candidate, max_pitch_deg=75.0)['ok']
    assert assess_sanity(candidate, max_pitch_deg=85.0)['ok']
    narrow = assess_sanity(
        candidate,
        max_pitch_deg=85.0,
        object_width_m=0.05,
        gripper_max_opening_m=0.04,
    )
    assert not narrow['ok']


def test_wrap_degrees():
    """Degree wrapping keeps angles in (-180, 180]."""
    assert wrap_degrees(0.0) == pytest.approx(0.0)
    assert wrap_degrees(180.0) == pytest.approx(180.0)
    assert wrap_degrees(-180.0) == pytest.approx(180.0)
    assert wrap_degrees(370.0) == pytest.approx(10.0)
    assert wrap_degrees(-370.0) == pytest.approx(-10.0)
