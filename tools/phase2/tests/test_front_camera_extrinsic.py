"""Synthetic regression tests for the front-camera extrinsic chain."""

import math

import cv2
import numpy as np
import pytest

import chessboard_hand_eye as hand_eye
import front_camera_extrinsic as extrinsic
import test_uvc_intrinsics as intrinsics_tests

IM_SIZE = (1280, 720)


def random_transform(rotation_vector, translation):
    """Build a transform from a rotation vector and a translation."""
    return hand_eye.make_transform(
        cv2.Rodrigues(np.asarray(rotation_vector, np.float64))[0],
        translation)


def test_compose_extrinsic_recovers_the_camera_pose():
    """Two observations of one board recover the camera pose exactly."""
    t_base_board = random_transform([0.2, -0.1, 0.4], [0.25, 0.01, -0.07])
    t_base_cam = random_transform([-1.9, 0.05, 0.1], [0.10, 0.0, 0.13])
    t_front_board = np.linalg.inv(t_base_cam) @ t_base_board
    solved = extrinsic.compose_extrinsic(t_base_board, t_front_board)
    assert np.allclose(solved, t_base_cam, atol=1e-9)


def test_board_pose_chain_matches_the_hand_eye_result():
    """FK x T_link4_camera x T_cam_board reproduces the known board pose."""
    t_base_board = random_transform([0.05, 0.02, 1.2], [0.24, 0.0, -0.078])
    t_base_link4 = random_transform([-0.6, 0.05, 0.0], [0.18, 0.0, 0.10])
    t_link4_camera = random_transform([1.5, 0.0, -1.56], [-0.035, -0.009, 0.059])
    t_cam_board = (np.linalg.inv(t_base_link4 @ t_link4_camera)
                   @ t_base_board)
    recovered = (t_base_link4 @ t_link4_camera @ t_cam_board)
    assert np.allclose(recovered, t_base_board, atol=1e-9)


def test_axis_report_reads_pitch_and_yaw():
    """A camera looking forward and 30 deg down reports those angles."""
    # A 120 deg rotation about +y turns the camera axis (0, 0, 1) into
    # (sin120, 0, cos120) = (0.866, 0, -0.5): forward and 30 deg down.
    rotation = cv2.Rodrigues(np.array([0.0, math.radians(120.0), 0.0]))[0]
    t_base_cam = hand_eye.make_transform(rotation, [0.1, 0.0, 0.13])
    report = extrinsic.axis_report(t_base_cam, board_height_m=-0.078)
    assert report['pitch_below_horizontal_deg'] == pytest.approx(30.0, abs=1e-6)
    assert report['yaw_deg'] == pytest.approx(0.0, abs=1e-6)
    assert report['height_above_board_m'] == pytest.approx(0.208)


def test_board_plane_tilt_is_zero_for_a_flat_board():
    """A board whose normal is +z reports no tilt."""
    flat = hand_eye.make_transform(np.eye(3), [0.25, 0.0, -0.078])
    assert extrinsic.board_plane_tilt_deg(flat) == pytest.approx(0.0)
    tilted = flat.copy()
    tilted[:3, :3] = cv2.Rodrigues(np.array([0.2, 0.0, 0.0]))[0]
    assert extrinsic.board_plane_tilt_deg(tilted) == pytest.approx(11.46, abs=0.1)


def test_front_board_pose_recovers_the_rendered_pose():
    """PnP on a rendered board returns the pose the render used."""
    rvec = np.array([0.8, 0.05, 0.3])
    centre_camera = np.array([0.01, -0.02, 0.38])
    image = intrinsics_tests.render_board(rvec, centre_camera)
    camera_matrix = intrinsics_tests.TRUE_K
    detection = extrinsic.front_board_pose(image, camera_matrix, np.zeros(5))
    assert detection is not None
    rotation = cv2.Rodrigues(rvec)[0]
    half_cols = 0.5 * (extrinsic.BOARD_COLS - 1) * extrinsic.SQUARE_M
    half_rows = 0.5 * (extrinsic.BOARD_ROWS - 1) * extrinsic.SQUARE_M
    expected_t = centre_camera - rotation @ np.array([half_cols, half_rows, 0.0])
    # The symmetric grid is ambiguous up to a 180 deg relabelling, so the
    # detection must match the render either directly or through the flip.
    direct = np.allclose(detection['transform'][:3, 3], expected_t,
                         atol=0.001)
    flipped = np.allclose(
        (detection['transform'] @ extrinsic._board_flip())[:3, 3],
        expected_t, atol=0.001)
    assert direct or flipped


def test_build_report_averages_observations():
    """Independent observations collapse to the mean board poses."""
    t_base_board = random_transform([0.0, 0.0, 0.9], [0.25, 0.0, -0.078])
    t_base_cam = random_transform([-1.9, 0.05, 0.1], [0.10, 0.0, 0.13])
    t_front_board = np.linalg.inv(t_base_cam) @ t_base_board
    report = extrinsic.build_report([t_base_board] * 3, [t_front_board] * 4)
    assert report['wrist_observations'] == 3
    assert report['front_observations'] == 4
    assert report['wrist_board_spread']['translation_m'] == pytest.approx(0.0)
    camera = np.asarray(report['T_base_front'])
    assert np.allclose(camera, t_base_cam, atol=1e-9)
    assert report['board_plane_tilt_deg'] == pytest.approx(0.0, abs=1e-6)


def test_build_report_rejects_empty_input():
    """A missing observation set is an error, not a silent solve."""
    with pytest.raises(RuntimeError):
        extrinsic.build_report([], [np.eye(4)])
    with pytest.raises(RuntimeError):
        extrinsic.build_report([np.eye(4)], [])
