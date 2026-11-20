"""Synthetic tests for the front-camera collision object placement."""

import math

import numpy as np
import pytest

import ext_cam_collision as collision


def quaternion_to_matrix(quaternion):
    """Rebuild a rotation matrix from an (x, y, z, w) quaternion."""
    x, y, z, w = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def test_box_on_the_identity_transform_keeps_the_offset():
    """With no rotation the box centre is the offset plus the translation."""
    transform = [[1.0, 0.0, 0.0, 0.139],
                 [0.0, 1.0, 0.0, 0.027],
                 [0.0, 0.0, 1.0, 0.037],
                 [0.0, 0.0, 0.0, 1.0]]
    center, quaternion, size = collision.compose_box(transform)
    assert center == pytest.approx([0.139, 0.027, 0.047])
    assert quaternion == pytest.approx((0.0, 0.0, 0.0, 1.0))
    assert size == collision.BODY_SIZE


def test_box_rotates_with_the_optical_frame():
    """The offset follows the calibrated camera rotation."""
    angle = math.radians(32.6)
    rotation = [[math.cos(angle), 0.0, math.sin(angle)],
                [0.0, 1.0, 0.0],
                [-math.sin(angle), 0.0, math.cos(angle)]]
    transform = [[rotation[i][j] for j in range(3)] + [[0.139, 0.027, 0.037][i]]
                 for i in range(3)] + [[0.0, 0.0, 0.0, 1.0]]
    center, quaternion, _ = collision.compose_box(transform)
    expected = np.array([0.139, 0.027, 0.037]) + np.asarray(rotation) @ \
        np.asarray(collision.BODY_OFFSET)
    assert center == pytest.approx(expected.tolist())
    assert np.allclose(quaternion_to_matrix(quaternion),
                       np.asarray(rotation), atol=1e-9)


def test_padding_grows_every_axis():
    """Padding is applied to all three box dimensions."""
    transform = [[1.0, 0.0, 0.0, 0.0],
                 [0.0, 1.0, 0.0, 0.0],
                 [0.0, 0.0, 1.0, 0.0],
                 [0.0, 0.0, 0.0, 1.0]]
    size = tuple(d + 0.01 for d in collision.BODY_SIZE)
    _, _, padded = collision.compose_box(transform, size=size)
    assert padded == pytest.approx(size)


def test_quaternion_is_unit_for_a_general_rotation():
    """The returned quaternion is normalised."""
    rotation = np.linalg.qr(np.array([[0.36, -0.48, 0.80],
                                      [0.80, 0.60, 0.00],
                                      [-0.48, 0.64, 0.60]]))[0]
    if np.linalg.det(rotation) < 0:
        rotation[:, 0] *= -1.0
    quaternion = collision.rotation_matrix_to_quaternion(rotation)
    assert math.sqrt(sum(value * value for value in quaternion)) == \
        pytest.approx(1.0)
    assert np.allclose(quaternion_to_matrix(quaternion), rotation, atol=1e-9)
