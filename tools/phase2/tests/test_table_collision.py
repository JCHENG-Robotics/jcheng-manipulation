"""Tests for the table collision-object geometry (pure functions)."""

import pytest

from table_collision import (
    load_plane,
    normalize,
    quaternion_from_z_axis,
    rotate_vector,
    table_box,
)

MEASURED_NORMAL = (
    0.010305373291790987,
    -0.06301369005824492,
    0.9979594551616625,
)


def test_identity_quaternion_for_flat_plane():
    """A +z normal needs no rotation."""
    quaternion = quaternion_from_z_axis((0.0, 0.0, 1.0))
    assert quaternion == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_quaternion_rotates_z_onto_the_measured_normal():
    """The box +z axis follows the tilted table normal."""
    normal = normalize(MEASURED_NORMAL)
    quaternion = quaternion_from_z_axis(normal)
    rotated = rotate_vector(quaternion, (0.0, 0.0, 1.0))
    assert rotated == pytest.approx(normal, abs=1e-9)


def test_top_face_lies_on_the_plane():
    """The box top is exactly the measured surface, not above it."""
    normal = (0.0, -0.063, 0.998)
    offset = 0.078
    thickness = 0.05
    center, _, dimensions = table_box(
        normal, offset, 1.0, 1.0, thickness
    )
    unit = normalize(normal)
    top = tuple(
        center[axis] + unit[axis] * dimensions[2] / 2.0
        for axis in range(3)
    )
    height = sum(
        unit[axis] * top[axis] for axis in range(3)
    ) + offset
    assert height == pytest.approx(0.0, abs=1e-9)


def test_box_center_is_half_a_thickness_below_the_plane():
    """A flat plane at z = -0.078 gives a box centred at -0.103."""
    center, _, dimensions = table_box(
        (0.0, 0.0, 1.0), 0.078, 1.0, 1.0, 0.05
    )
    assert center == pytest.approx((0.0, 0.0, -0.103))
    assert dimensions == pytest.approx((1.0, 1.0, 0.05))


def test_downward_normal_is_rejected():
    """A downward normal would put the box above the table."""
    with pytest.raises(ValueError):
        quaternion_from_z_axis((0.0, 0.0, -1.0))
    with pytest.raises(ValueError):
        table_box((0.0, 0.0, -1.0), 0.0, 1.0, 1.0, 0.05)


def test_zero_normal_is_rejected():
    """A zero vector cannot define a plane."""
    with pytest.raises(ValueError):
        normalize((0.0, 0.0, 0.0))


def test_load_plane_from_a_report(tmp_path):
    """The tool reads the same plane record the safety config uses."""
    path = tmp_path / 'plane.json'
    path.write_text(
        '{"plane_base": {"normal": [0.0, 0.0, 1.0], "offset": 0.5}}',
        encoding='utf-8',
    )
    normal, offset = load_plane(str(path))
    assert normal == (0.0, 0.0, 1.0)
    assert offset == pytest.approx(0.5)


def test_load_plane_rejects_a_missing_section(tmp_path):
    """A report without a plane section is an error, not a default."""
    path = tmp_path / 'plane.json'
    path.write_text('{}', encoding='utf-8')
    with pytest.raises(ValueError):
        load_plane(str(path))
