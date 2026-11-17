"""Synthetic regression tests for the chessboard hand-eye solver."""

import math

import cv2
import numpy as np
import pytest

import chessboard_hand_eye as hand_eye


def rotation_from_axis_angle(axis, angle):
    """Return a rotation matrix from an axis and an angle in radians."""
    axis = np.asarray(axis, dtype=np.float64)
    axis = axis / np.linalg.norm(axis)
    return cv2.Rodrigues(axis * angle)[0]


def test_make_transform_and_angle_roundtrip():
    """A known rotation has the expected geodesic angle."""
    rotation = rotation_from_axis_angle([0.0, 0.0, 1.0], math.pi / 2.0)
    transform = hand_eye.make_transform(rotation, [1.0, 2.0, 3.0])
    assert transform[0, 3] == pytest.approx(1.0)
    assert hand_eye.rotation_angle_deg(rotation) == pytest.approx(90.0)


def test_board_pose_stats_is_zero_for_identical_poses():
    """Identical board poses have zero scatter."""
    transform = hand_eye.make_transform(np.eye(3), [0.4, 0.1, 0.2])
    stats = hand_eye.board_pose_stats([transform, transform, transform])
    assert stats['translation_std_m'] == pytest.approx(0.0, abs=1e-12)
    assert stats['rotation_std_deg'] == pytest.approx(0.0, abs=1e-12)


def test_sample_joints_prefers_measured_servo_positions():
    """Measured servo positions win over cached commanded joints."""
    sample = {
        'joints': [0.0, 0.1, -0.2, 0.3, 0.0],
        'measured_joints': [0.01, 0.2, -0.3, 0.4, 0.0],
    }
    assert hand_eye.sample_joints(sample) == sample['measured_joints']


def test_synthetic_hand_eye_recovers_known_transform():
    """Noise-free synthetic samples recover the injected hand-eye transform."""
    known = hand_eye.make_transform(
        rotation_from_axis_angle([0.0, 0.0, 1.0], math.radians(90.0)),
        [0.03, -0.01, 0.06])
    T_base_board = hand_eye.make_transform(
        np.eye(3), [0.40, 0.10, 0.20])
    rng = np.random.default_rng(7)
    t_base_link4 = []
    t_cam_board = []
    for _ in range(24):
        axis = rng.normal(size=3)
        rotation = rotation_from_axis_angle(
            axis, rng.uniform(0.15, 0.9))
        translation = np.array([
            0.10 + rng.uniform(-0.04, 0.04),
            rng.uniform(-0.04, 0.04),
            0.30 + rng.uniform(-0.04, 0.04),
        ])
        T_base_link4 = hand_eye.make_transform(rotation, translation)
        T_cam_board = (
            np.linalg.inv(known)
            @ np.linalg.inv(T_base_link4)
            @ T_base_board)
        t_base_link4.append(T_base_link4)
        t_cam_board.append(T_cam_board)

    results = hand_eye.solve_hand_eye_methods(t_base_link4, t_cam_board)
    selected = hand_eye.select_best_method(results)
    recovered = np.asarray(results[selected]['T_link4_camera'])
    assert np.max(np.abs(recovered[:3, 3] - known[:3, 3])) < 1e-6
    assert hand_eye.rotation_angle_deg(
        known[:3, :3].T @ recovered[:3, :3]) < 1e-4
    assert results[selected]['board_translation_std_m'] < 1e-9
    assert results[selected]['board_rotation_std_deg'] < 1e-9
    leave_out = hand_eye.leave_one_out(
        t_base_link4, t_cam_board, selected)
    assert leave_out['max_translation_error_m'] < 1e-5
    assert leave_out['max_rotation_error_deg'] < 1e-3


def test_select_best_method_uses_board_scatter():
    """The selector ignores failed methods and minimizes the scatter score."""
    results = {
        'A': {
            'board_translation_std_m': 0.005,
            'board_rotation_std_deg': 0.5,
        },
        'B': {
            'board_translation_std_m': 0.001,
            'board_rotation_std_deg': 1.0,
        },
        'C': {'error': 'not enough motion'},
    }
    assert hand_eye.select_best_method(results) == 'B'
    with pytest.raises(RuntimeError):
        hand_eye.select_best_method({'C': {'error': 'x'}})
