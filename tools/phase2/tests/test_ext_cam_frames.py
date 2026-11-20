"""Synthetic tests for the off-board camera frame publisher helpers."""

import json
import math

import numpy as np
import pytest

import ext_cam_frames as frames


def quaternion_to_matrix(quaternion):
    """Rebuild a rotation matrix from an (x, y, z, w) quaternion."""
    x, y, z, w = quaternion
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def test_identity_rotation_gives_the_identity_quaternion():
    """The identity matrix maps to (0, 0, 0, 1)."""
    quaternion = frames.rotation_matrix_to_quaternion(np.eye(3))
    assert quaternion == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_quarter_turn_roundtrip():
    """A 90 deg turn about z survives the matrix/quaternion round trip."""
    angle = math.pi / 2.0
    matrix = np.array([[math.cos(angle), -math.sin(angle), 0.0],
                       [math.sin(angle), math.cos(angle), 0.0],
                       [0.0, 0.0, 1.0]])
    quaternion = frames.rotation_matrix_to_quaternion(matrix)
    assert quaternion == pytest.approx((0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)))
    assert np.allclose(quaternion_to_matrix(quaternion), matrix, atol=1e-9)


def test_tilted_rotation_roundtrip():
    """A general rotation round-trips within float tolerance."""
    rotation = np.array([
        [0.36, -0.48, 0.80],
        [0.80, 0.60, 0.00],
        [-0.48, 0.64, 0.60],
    ])
    rotation = np.linalg.qr(rotation)[0]
    if np.linalg.det(rotation) < 0:
        rotation[:, 0] *= -1.0
    quaternion = frames.rotation_matrix_to_quaternion(rotation)
    assert np.allclose(quaternion_to_matrix(quaternion), rotation, atol=1e-9)


def test_camera_info_fields_copy_the_model():
    """K/D/width/height/P come from the calibration report."""
    calibration = {
        'width': 1280,
        'height': 720,
        'model': 'plumb_bob',
        'camera_matrix': [[892.1, 0.0, 648.0],
                          [0.0, 892.1, 380.2],
                          [0.0, 0.0, 1.0]],
        'dist_coeffs': [0.01, -0.02, 0.001, 0.0, 0.0],
    }
    fields = frames.camera_info_fields(calibration)
    assert fields['width'] == 1280
    assert fields['height'] == 720
    assert fields['distortion_model'] == 'plumb_bob'
    assert fields['k'] == pytest.approx(
        [892.1, 0.0, 648.0, 0.0, 892.1, 380.2, 0.0, 0.0, 1.0])
    assert fields['p'] == pytest.approx(
        [892.1, 0.0, 648.0, 0.0, 0.0, 892.1, 380.2, 0.0,
         0.0, 0.0, 1.0, 0.0])
    assert fields['d'] == pytest.approx([0.01, -0.02, 0.001, 0.0, 0.0])


def test_load_calibration_reads_both_reports(tmp_path):
    """The loader joins the extrinsic transform with the intrinsics."""
    extrinsic = {
        'T_base_front': [[1.0, 0.0, 0.0, 0.139],
                         [0.0, 1.0, 0.0, 0.027],
                         [0.0, 0.0, 1.0, 0.037],
                         [0.0, 0.0, 0.0, 1.0]],
    }
    intrinsics = {
        'camera_matrix': [[892.1, 0.0, 648.0],
                          [0.0, 892.1, 380.2],
                          [0.0, 0.0, 1.0]],
        'dist_coeffs': [0.01, -0.02, 0.001, 0.0, 0.0],
        'image_size': [1280, 720],
        'model': 'plumb_bob',
    }
    extrinsic_path = tmp_path / 'extrinsic.json'
    intrinsics_path = tmp_path / 'intrinsics.json'
    extrinsic_path.write_text(json.dumps(extrinsic))
    intrinsics_path.write_text(json.dumps(intrinsics))
    calibration = frames.load_calibration(
        str(extrinsic_path), str(intrinsics_path))
    assert calibration['translation'] == pytest.approx([0.139, 0.027, 0.037])
    assert calibration['rotation'] == [[1.0, 0.0, 0.0],
                                       [0.0, 1.0, 0.0],
                                       [0.0, 0.0, 1.0]]
    assert calibration['width'] == 1280
    assert calibration['height'] == 720
