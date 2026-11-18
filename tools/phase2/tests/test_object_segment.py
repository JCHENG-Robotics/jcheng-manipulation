"""Tests for the markerless RGB-D segmentation helpers (no ROS needed)."""

import math

import numpy as np
import pytest

import object_segment_rgbd as segment


def _cube_top(center, size=0.0394, count=21, noise=0.0002, seed=0):
    """Return the top-face points of a cube at *center* (x, y, z_top)."""
    rng = np.random.default_rng(seed)
    axis = np.linspace(-size / 2.0, size / 2.0, count)
    xx, yy = np.meshgrid(axis, axis)
    zz = np.full_like(xx, center[2])
    points = np.stack([xx + center[0], yy + center[1], zz], axis=-1)
    points = points.reshape(-1, 3)
    points[:, 2] += rng.normal(0.0, noise, len(points))
    return points


def test_connected_components_two_blobs():
    """Two 2 cm blobs 5 cm apart are two clusters."""
    first = np.array([[x, y, 0.0] for x in np.arange(0, 0.02, 0.004)
                      for y in np.arange(0, 0.02, 0.004)])
    second = first + np.array([0.05, 0.0, 0.0])
    labels, clusters = segment.connected_components(
        np.vstack([first, second]), pitch=0.005, min_points=4)
    assert len(clusters) == 2
    assert labels[0] != labels[-1]


def test_connected_components_ignores_small_clusters():
    """A 2-point speck is below min_points."""
    points = np.array([[0, 0, 0], [0.001, 0, 0], [0.05, 0.05, 0]])
    _, clusters = segment.connected_components(points, 0.005, min_points=2)
    assert len(clusters) == 1


def test_min_area_rectangle_rotated():
    """A 40 x 20 mm rectangle at 30 deg returns its side lengths."""
    yaw = math.radians(30.0)
    local = np.array([[x, y] for x in (-0.02, 0.02) for y in (-0.01, 0.01)])
    rotation = np.array([[math.cos(yaw), -math.sin(yaw)],
                         [math.sin(yaw), math.cos(yaw)]])
    xy = local @ rotation.T
    angle, width, depth, area = segment.min_area_rectangle(xy)
    assert math.isclose(angle, angle)  # float, not a numpy scalar
    # The sweep reports the frame yaw; the two side lengths are what matter.
    assert sorted((width, depth)) == pytest.approx([0.02, 0.04], abs=0.002)
    assert math.isclose(area, 0.0008, rel_tol=0.05)


def test_fit_slab_centroid_and_tilt():
    """A flat cube top gives the centre and a near-zero tilt."""
    points = _cube_top((0.24, 0.0, -0.039), noise=0.0)
    top = segment.fit_slab(points, slab_m=0.002)
    assert np.allclose(top['centroid'], [0.24, 0.0, -0.039], atol=1e-6)
    assert top['tilt_deg'] < 1e-3
    assert math.isclose(top['extent_m'][0], 0.0394, abs_tol=0.004)


def test_fit_slab_rejects_empty():
    """An empty cluster is an error, not a silent target."""
    try:
        segment.fit_slab(np.empty((0, 3)))
    except ValueError as error:
        assert 'empty' in str(error)
    else:
        raise AssertionError('expected ValueError')


def _cluster(x, y, z_top, size=0.0394):
    points = _cube_top((x, y, z_top), size=size, noise=0.0)
    return {'label': 0, 'indices': np.arange(len(points)), 'points': points}


def test_select_cluster_accepts_cube():
    """A 39 mm cube inside the region is selected."""
    gates = {
        'min_points': 4, 'max_xy_m': 0.09,
        'min_height_m': 0.02, 'max_height_m': 0.07,
        'x_range': (0.15, 0.32), 'y_range': 0.09,
    }
    cluster = _cluster(0.24, 0.0, -0.039)
    selected, reasons = segment.select_cluster(
        [cluster], floor_z=-0.0787, gates=gates)
    assert selected is cluster
    assert reasons[0]['ok'] is True


def test_select_cluster_rejects_wide_object():
    """A 120 mm object is rejected by the width gate."""
    gates = {
        'min_points': 4, 'max_xy_m': 0.09,
        'min_height_m': 0.02, 'max_height_m': 0.07,
        'x_range': (0.15, 0.32), 'y_range': 0.09,
    }
    cluster = _cluster(0.24, 0.0, -0.039, size=0.12)
    selected, reasons = segment.select_cluster(
        [cluster], floor_z=-0.0787, gates=gates)
    assert selected is None
    assert 'too_wide' in reasons[0]['why']


def test_select_cluster_rejects_outside_region():
    """A cube 30 cm to the side is rejected."""
    gates = {
        'min_points': 4, 'max_xy_m': 0.09,
        'min_height_m': 0.02, 'max_height_m': 0.07,
        'x_range': (0.15, 0.32), 'y_range': 0.09,
    }
    cluster = _cluster(0.24, 0.30, -0.039)
    selected, reasons = segment.select_cluster(
        [cluster], floor_z=-0.0787, gates=gates)
    assert selected is None
    assert 'y_out_of_region' in reasons[0]['why']


def test_top_view_gate_accepts_full_face():
    """A 39.4 mm full top face passes the 35 mm gate."""
    top = {'extent_m': [0.0390, 0.0410]}
    ok, reason, extent = segment.top_view_gate(top, 0.035)
    assert ok is True
    assert reason is None
    assert extent == 0.0390


def test_top_view_gate_rejects_clipped_face():
    """The measured 37.8 x 31.9 mm partial view is rejected."""
    top = {'extent_m': [0.0378, 0.0319]}
    ok, reason, extent = segment.top_view_gate(top, 0.035)
    assert ok is False
    assert reason == 'partial_top_view'
    assert extent == 0.0319
