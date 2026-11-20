"""Tests for the pose-independent front-camera top-face measurement.

OpenCV is required by the rectification and segmentation paths; the pure-numpy
plane helpers are tested first. The full synthetic test projects a known
39.4 mm square through the real calibration and checks that the rectified
segmentation recovers its centre.
"""

import json
import os

import numpy as np
import pytest

cv2 = pytest.importorskip('cv2')

import ext_cam_measure as measure  # noqa: E402

REPO = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
EXTRINSIC = os.path.join(
    REPO, 'evidence', 'calibration', '20261120',
    'front_camera_extrinsic.json')
INTRINSICS = os.path.join(
    REPO, 'evidence', 'calibration', '20261120',
    'uvc_color_intrinsics_refined.json')


def load(path):
    """Return a JSON file."""
    with open(path, 'r', encoding='utf-8') as handle:
        return json.load(handle)


def test_fit_plane_recovers_a_known_plane():
    """RANSAC + least squares recover a tilted plane with 1 mm noise."""
    rng = np.random.default_rng(1)
    normal = np.array([0.02, -0.01, 0.99975])
    normal = normal / np.linalg.norm(normal)
    offset = -0.09
    u = np.array([1.0, 0.0, 0.0])
    u = u - (u @ normal) * normal
    u = u / np.linalg.norm(u)
    v = np.cross(normal, u)
    points = (0.2 + rng.uniform(-0.1, 0.1, 4000)[:, None] * u
              + rng.uniform(-0.1, 0.1, 4000)[:, None] * v
              - offset * normal)
    points = points + rng.normal(0.0, 0.001, points.shape)
    fitted, fitted_offset, mask, rms = measure.fit_plane(points)
    assert np.degrees(np.arccos(abs(fitted @ normal))) < 0.5
    assert mask.mean() > 0.95
    assert rms < 0.0015


def test_depth_points_uses_metres_and_the_valid_range():
    """Depth millimetres convert to metres and 0/out-of-range pixels drop."""
    depth = np.full((4, 4), 500, dtype=np.uint16)
    depth[0, 0] = 0
    depth[0, 1] = 2000
    info = {
        'scale_m_per_unit': 0.001,
        'camera_info': {
            'k': [400.0, 0.0, 2.0, 0.0, 400.0, 2.0, 0.0, 0.0, 1.0]},
    }
    points = measure.depth_points(depth, info)
    assert len(points) == 14
    assert np.allclose(points[:, 2], 0.5)


def test_plane_to_base_with_identity_transforms_keeps_the_plane():
    """Identity depth->colour and base->colour keep the plane unchanged."""
    normal = np.array([0.0, 0.0, 1.0])
    offset = -0.09
    identity = np.eye(4)
    identity[2, 3] = 0.0
    depth_to_color = {'R': np.eye(3).tolist(),
                      't_m': [0.0, 0.0, 0.0]}
    fitted, fitted_offset, point = measure.plane_to_base(
        normal, offset, depth_to_color, identity)
    assert np.allclose(fitted, normal)
    assert fitted_offset == pytest.approx(offset)
    assert point[2] == pytest.approx(0.09)


def test_plane_ray_hit_returns_the_expected_point():
    """A ray straight down hits a horizontal plane at the right height."""
    hit = measure.plane_ray_hit(
        np.array([0.2, 0.0, 0.1]), np.array([0.0, 0.0, -1.0]),
        np.array([0.0, 0.0, 1.0]), -0.05)
    assert np.allclose(hit, [0.2, 0.0, -0.05])


def test_base_point_inverts_the_rectified_pixel_mapping():
    """The rectified centre maps to the anchor and a shifted pixel to u/v."""
    anchor = np.array([0.23, -0.01, -0.05])
    axis_u = np.array([1.0, 0.0, 0.0])
    axis_v = np.array([0.0, 1.0, 0.0])
    scale = 0.0005
    width, height = 240, 180
    centre = measure.base_point(anchor, axis_u, axis_v, scale,
                                (width / 2.0, height / 2.0), width, height)
    assert np.allclose(centre, anchor)
    shifted = measure.base_point(anchor, axis_u, axis_v, scale,
                                 (width / 2.0 + 20.0, height / 2.0 - 10.0),
                                 width, height)
    assert np.allclose(shifted, anchor + np.array(
        [20 * scale, -10 * scale, 0.0]))


def test_synthetic_square_is_recovered_through_the_real_calibration():
    """Project a known top-face square, rectify it and recover its centre."""
    extrinsic = load(EXTRINSIC)
    intrinsics = load(INTRINSICS)
    matrix = np.asarray(intrinsics['camera_matrix'], dtype=np.float64)
    distortion = np.asarray(
        intrinsics['dist_coeffs'], dtype=np.float64).ravel()
    base_from_camera = np.asarray(extrinsic['T_base_front'], dtype=np.float64)
    # A flat top face at z = -0.05, square centred at (0.2315, -0.008).
    normal = np.array([0.0, 0.0, 1.0])
    constant = -0.05
    centre = np.array([0.2315, -0.008, -0.05])
    edge = 0.0394
    image = np.full((720, 1280, 3), 180, dtype=np.uint8)
    rotation = base_from_camera[:3, :3]
    translation = base_from_camera[:3, 3]
    corners = []
    for du, dv in ((0.5, 0.5), (-0.5, 0.5), (-0.5, -0.5), (0.5, -0.5)):
        point = centre + np.array([du * edge, dv * edge, 0.0])
        camera = rotation.T @ (point - translation)
        projected, _ = cv2.projectPoints(
            camera.reshape(1, 1, 3), np.zeros(3), np.zeros(3),
            matrix, distortion)
        corners.append(projected.reshape(2))
    polygon = np.round(np.array(corners)).astype(np.int32)
    cv2.fillPoly(image, [polygon.reshape(-1, 1, 2)], (30, 60, 90))
    # The camera's plane is known, so the anchor starts at the true centre.
    rectified, _, axis_u, axis_v = measure.rectify(
        image, matrix, distortion, base_from_camera, normal, constant,
        centre, 0.0005, (120.0, 90.0))
    mask, centroid, rectangle = measure.segment(rectified)
    assert mask is not None
    assert rectangle is not None
    height, width = mask.shape
    recovered = measure.base_point(centre, axis_u, axis_v, 0.0005,
                                   centroid, width, height)
    assert np.linalg.norm(recovered[:2] - centre[:2]) < 0.001
    extent = sorted(value * 0.0005 * 1000.0 for value in rectangle[1])
    assert 30.0 <= extent[0] <= 55.0
