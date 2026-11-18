"""Tests for the same-frame RGB-D analysis geometry (pure functions)."""

from types import SimpleNamespace

import numpy as np
import pytest

import rgbd_board_check as board
import rgbd_tag_check as tag


def test_board_object_points():
    """The object grid matches 11x8 inner corners at 20 mm."""
    points = board.board_object_points(11, 8, 0.02)
    assert points.shape == (88, 3)
    assert points[0].tolist() == [0.0, 0.0, 0.0]
    assert points[-1].tolist() == pytest.approx([0.20, 0.14, 0.0])


def test_plane_from_points_recovers_a_plane():
    """SVD recovers the normal, offset and noise level."""
    rng = np.random.default_rng(0)
    xy = rng.uniform(-0.2, 0.2, size=(2000, 2))
    points = np.column_stack(
        [xy, np.full(len(xy), 0.5) + rng.normal(0.0, 0.0005, len(xy))])
    normal, offset, rms = board.plane_from_points(points)
    assert abs(normal[2]) == pytest.approx(1.0, abs=1e-3)
    assert offset == pytest.approx(-0.5, abs=1e-3)
    assert rms < 0.001


def test_plane_to_base_translates_the_plane():
    """A camera plane at z=0.5 becomes z=0.6 in base."""
    transform = np.eye(4)
    transform[2, 3] = 0.1
    normal, offset = board.plane_to_base(
        np.array([0.0, 0.0, 1.0]), -0.5, transform)
    assert normal == pytest.approx([0.0, 0.0, 1.0])
    assert offset == pytest.approx(-0.6)
    assert board.z_at_plane(normal, offset, 0.0, 0.0) == pytest.approx(0.6)


def test_z_at_a_tilted_plane():
    """The height follows the plane slope in x and y."""
    normal = np.array([0.0, 0.1, 0.995])
    offset = -0.5
    assert board.z_at_plane(normal, offset, 0.0, 0.1) == pytest.approx(
        0.4925, abs=1e-3)


def test_polygon_mask_uses_the_convex_hull():
    """Row-major inner corners still produce the board outline."""
    corners = [
        [10.0, 10.0], [20.0, 10.0],
        [10.0, 20.0], [20.0, 20.0],
    ]
    mask = board.polygon_mask((30, 30, 3), corners)
    assert mask[15, 15] == 255
    assert mask[5, 5] == 0


def test_outside_radius_mask_excludes_the_centre():
    """The table mask is empty at the tag centre."""
    mask = tag.outside_radius_mask((100, 100, 3), (50.0, 50.0), 20.0)
    assert mask[50, 50] == 0
    assert mask[10, 10] == 255


def test_ring_mask_excludes_the_tag_and_keeps_the_margin():
    """The paper ring starts outside the tag polygon."""
    corners = [[40.0, 40.0], [60.0, 40.0],
               [60.0, 60.0], [40.0, 60.0]]
    ring = tag.ring_mask((100, 100, 3), corners, dilate_px=10)
    assert ring[50, 50] == 0
    assert ring[50, 66] == 255


def test_project_cloud_uses_the_pinhole_model():
    """The optical axis maps to the principal point."""
    camera = np.array([
        [400.0, 0.0, 320.0],
        [0.0, 400.0, 200.0],
        [0.0, 0.0, 1.0],
    ])
    uv, valid = board.project_cloud(
        np.array([[0.0, 0.0, 1.0], [0.0, 0.0, -1.0]]), camera)
    assert valid.tolist() == [True, False]
    assert uv[0].tolist() == pytest.approx([320.0, 200.0])


def test_select_points_applies_the_mask_and_range():
    """Only in-mask, in-range points are returned."""
    points = np.array([
        [0.0, 0.0, 0.30],
        [0.0, 0.0, 0.05],
        [0.0, 0.0, 2.0],
    ])
    uv = np.array([[10.0, 10.0], [10.0, 10.0], [10.0, 10.0]])
    valid = np.array([True, True, True])
    mask = np.zeros((20, 20), np.uint8)
    mask[10, 10] = 255
    selected = board.select_points(points, uv, valid, mask)
    assert selected.shape == (1, 3)
    assert selected[0, 2] == pytest.approx(0.30)


def test_load_hand_eye_supports_both_layouts(tmp_path):
    """The table-down and original calibration layouts both load."""
    table_down = tmp_path / 'table_down.json'
    table_down.write_text(
        '{"hand_eye": {"methods": {"TSAI": '
        '{"T_link4_camera": [[1,0,0,0],[0,1,0,0],'
        '[0,0,1,0],[0,0,0,1]]}}}}',
        encoding='utf-8',
    )
    original = tmp_path / 'original.json'
    original.write_text(
        '{"hand_eye": {"T_link4_camera": '
        '[[1,0,0,0],[0,1,0,0],[0,0,1,0],[0,0,0,1]]}}',
        encoding='utf-8',
    )
    transform, _ = tag.load_hand_eye(str(table_down))
    assert transform.shape == (4, 4)
    transform, _ = tag.load_hand_eye(str(original))
    assert transform.shape == (4, 4)


def test_tag_transform_uses_the_detection_pose():
    """The tag transform is the detection rotation and translation."""
    rotation = ((1.0, 0.0, 0.0),
                (0.0, 1.0, 0.0),
                (0.0, 0.0, 1.0))
    detection = SimpleNamespace(
        rotation_matrix=rotation,
        tvec=(0.1, 0.2, 0.3),
    )
    transform = tag.tag_transform(detection)
    assert transform[:3, 3].tolist() == pytest.approx([0.1, 0.2, 0.3])
    assert transform[3, 3] == pytest.approx(1.0)


def test_angle_between_normals():
    """Parallel and perpendicular normals give 0 and 90 degrees."""
    assert board.angle_between(
        np.array([0.0, 0.0, 1.0]),
        np.array([0.0, 0.0, 1.0]),
    ) == pytest.approx(0.0)
    assert board.angle_between(
        np.array([0.0, 0.0, 1.0]),
        np.array([1.0, 0.0, 0.0]),
    ) == pytest.approx(90.0)
