"""Synthetic regression tests for the UVC chessboard intrinsics tool."""

import math

import cv2
import numpy as np
import pytest

import uvc_chessboard_intrinsics as uvc

IMAGE_SIZE = (1280, 720)
TRUE_K = np.array([[900.0, 0.0, 640.0],
                   [0.0, 900.0, 360.0],
                   [0.0, 0.0, 1.0]])
SQUARE_PX = 120
MARGIN_SQ = 1


def render_board(rvec, tvec, image_size=IMAGE_SIZE, camera_matrix=TRUE_K):
    """Render the 12 x 9 square board under a known pinhole camera."""
    n_sq_x = uvc.BOARD_COLS + 1
    n_sq_y = uvc.BOARD_ROWS + 1
    width = (n_sq_x + 2 * MARGIN_SQ) * SQUARE_PX
    height = (n_sq_y + 2 * MARGIN_SQ) * SQUARE_PX
    texture = np.full((height, width), 255, np.uint8)
    for i in range(n_sq_x):
        for j in range(n_sq_y):
            if (i + j) % 2 == 0:
                x0 = (i + MARGIN_SQ) * SQUARE_PX
                y0 = (j + MARGIN_SQ) * SQUARE_PX
                texture[y0:y0 + SQUARE_PX, x0:x0 + SQUARE_PX] = 0
    # Texture pixels -> board metres.  The first inner corner sits one square
    # inside the pattern's outer boundary (the object points start at 0, 0).
    scale = uvc.SQUARE_M / SQUARE_PX
    offset = (MARGIN_SQ + 1) * SQUARE_PX
    texture_to_board = np.array([
        [scale, 0.0, -offset * scale],
        [0.0, scale, -offset * scale],
        [0.0, 0.0, 1.0]])
    rotation = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))[0]
    # The object points start at the first inner corner, so recentre the board
    # on the image before applying the request translation.
    centre = np.array([0.5 * (uvc.BOARD_COLS - 1) * uvc.SQUARE_M,
                       0.5 * (uvc.BOARD_ROWS - 1) * uvc.SQUARE_M,
                       0.0])
    translation = np.asarray(tvec, dtype=np.float64) - rotation @ centre
    board_to_camera = np.column_stack(
        [rotation[:, 0], rotation[:, 1], translation])
    homography = camera_matrix @ board_to_camera @ texture_to_board
    warped = cv2.warpPerspective(
        texture, homography, image_size, flags=cv2.INTER_LINEAR,
        borderValue=255)
    return cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)


def board_poses(count=12):
    """Return deterministic board poses that span the image."""
    poses = []
    for index in range(count):
        angle = 2.0 * math.pi * index / count
        rvec = np.array([0.40 * math.cos(angle),
                         0.35 * math.sin(angle),
                         0.45 * math.sin(2.0 * angle)])
        tvec = np.array([0.050 * math.cos(angle),
                         0.050 * math.sin(angle),
                         0.45 + 0.04 * (index % 4)])
        poses.append((rvec, tvec))
    return poses


def test_detect_board_finds_a_rendered_grid():
    """A rendered board is detected with the full corner grid."""
    image = render_board([0.1, -0.05, 0.2], [0.0, 0.0, 0.32])
    detection = uvc.detect_board(image)
    assert detection is not None
    assert detection['corners'].shape == (uvc.BOARD_COLS * uvc.BOARD_ROWS, 2)
    x, y, w, h = detection['bbox']
    assert w > 400 and h > 300


def test_intrinsics_recover_the_synthetic_camera():
    """A 12-view sweep recovers fx, cx and the reprojection error."""
    frames = [
        render_board(rvec, tvec) for rvec, tvec in board_poses()]
    views = []
    object_points = uvc.board_object_points()
    for index, image in enumerate(frames):
        detection = uvc.detect_board(image)
        assert detection is not None, 'view {} not detected'.format(index)
        features = uvc.view_features(detection['corners'], IMAGE_SIZE)
        views.append({
            'frame': 'synthetic_{:02d}.png'.format(index),
            'corners': detection['corners'],
            'features': features,
            'area': features['area'],
        })
    selected = uvc.select_diverse(views, max_views=30)
    assert len(selected) >= 8
    result = uvc.calibrate(selected, IMAGE_SIZE, object_points)
    matrix = result['camera_matrix']
    assert result['rms_px'] < 0.5
    assert matrix[0, 0] == pytest.approx(TRUE_K[0, 0], rel=0.01)
    assert matrix[1, 1] == pytest.approx(TRUE_K[1, 1], rel=0.01)
    assert matrix[0, 2] == pytest.approx(TRUE_K[0, 2], abs=8.0)
    assert matrix[1, 2] == pytest.approx(TRUE_K[1, 2], abs=8.0)


def test_select_diverse_keeps_distinct_buckets():
    """Near-identical views collapse to one entry per bucket."""
    corners = np.zeros((uvc.BOARD_ROWS, uvc.BOARD_COLS, 2), np.float64)
    ys, xs = np.mgrid[0:uvc.BOARD_ROWS, 0:uvc.BOARD_COLS]
    corners[:, :, 0] = 200.0 + xs * 40.0
    corners[:, :, 1] = 100.0 + ys * 40.0
    flat = corners.reshape(-1, 2)
    views = [{
        'frame': 'frame_{}.png'.format(i),
        'corners': flat.copy(),
        'features': uvc.view_features(flat, IMAGE_SIZE),
        'area': uvc.view_features(flat, IMAGE_SIZE)['area'],
    } for i in range(10)]
    selected = uvc.select_diverse(views, max_views=5)
    assert len(selected) == 1


def test_view_features_rejects_a_folded_quad():
    """A degenerate (folded) corner grid is not offered as a view."""
    grid = np.zeros((uvc.BOARD_ROWS, uvc.BOARD_COLS, 2), np.float64)
    grid[:, :, 0] = 100.0
    grid[:, :, 1] = 100.0
    features = uvc.view_features(grid.reshape(-1, 2), IMAGE_SIZE)
    assert features['area'] < 1e-4


def test_build_report_needs_enough_views():
    """Too few board views is an error, not a silent weak calibration."""
    blank = np.full((720, 1280, 3), 255, np.uint8)
    with pytest.raises(RuntimeError):
        uvc.build_report(['missing.png'] * 6, IMAGE_SIZE)
    del blank
