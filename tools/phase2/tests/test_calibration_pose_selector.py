"""Tests for the offline calibration pose selector."""

import json
from pathlib import Path

import numpy as np
import pytest

import calibration_pose_selector as selector

REPO_ROOT = Path(__file__).resolve().parents[3]
CALIBRATION_JSON = (
    REPO_ROOT / 'evidence' / 'calibration' / '20261117'
    / 'calibration_servo_20261117.json')


def test_axis_values_include_endpoints():
    """The grid helper is deterministic and includes both endpoints."""
    values = selector._axis_values(-0.1, 0.1, 0.05)
    assert values == pytest.approx([-0.1, -0.05, 0.0, 0.05, 0.1])


def test_polygon_area():
    """A 2x3 rectangle has area six."""
    points = np.array([[0.0, 0.0], [2.0, 0.0], [2.0, 3.0], [0.0, 3.0]])
    assert selector.polygon_area(points) == pytest.approx(6.0)


def test_project_camera_points_fronto_parallel_board():
    """A centred board at 0.5 m projects around the principal point."""
    camera = selector.CameraSpec(
        width=640,
        height=400,
        k=np.array([
            [418.0, 0.0, 320.0],
            [0.0, 418.0, 200.0],
            [0.0, 0.0, 1.0],
        ]),
        d=np.zeros(5),
        T_link4_camera=np.eye(4))
    board = selector.BoardSpec(
        cols=11, rows=8, square_m=0.02,
        T_base_board=selector.make_transform(
            np.eye(3), [0.0, 0.0, 0.5]))
    config = selector.SelectorConfig(min_image_margin_px=5.0)
    T_cam_board = selector.make_transform(
        np.eye(3), [0.0, 0.0, 0.5])
    points_cam = selector.transform_points(
        T_cam_board, selector.board_corners(board))
    metrics = selector._projected_metrics(points_cam, camera, config)
    assert metrics is not None
    center_x, center_y = metrics['image_center']
    assert center_x == pytest.approx(320.0 + 418.0 * 0.10 / 0.5, abs=2.0)
    assert center_y == pytest.approx(200.0 + 418.0 * 0.07 / 0.5, abs=2.0)
    assert metrics['margin_px'] > 5.0


def test_projected_metrics_rejects_out_of_view_board():
    """A board far outside the image is rejected by the margin check."""
    camera = selector.CameraSpec(
        width=640, height=400,
        k=np.array([
            [418.0, 0.0, 320.0],
            [0.0, 418.0, 200.0],
            [0.0, 0.0, 1.0],
        ]),
        d=np.zeros(5),
        T_link4_camera=np.eye(4))
    board = selector.BoardSpec(
        cols=11, rows=8, square_m=0.02,
        T_base_board=selector.make_transform(
            np.eye(3), [0.0, 0.0, 0.5]))
    config = selector.SelectorConfig(min_image_margin_px=25.0)
    points_cam = selector.transform_points(
        selector.make_transform(np.eye(3), [2.0, 0.0, 0.5]),
        selector.board_corners(board))
    assert selector._projected_metrics(points_cam, camera, config) is None


def test_select_diverse_returns_unique_poses_and_improves_spread():
    """Greedy selection avoids duplicates and beats the first-N baseline."""
    rng = np.random.default_rng(3)
    candidates = []
    for index in range(60):
        feature = rng.normal(size=4)
        candidates.append(selector.Candidate(
            joints=(
                0.0, 0.0, 0.0, 0.0, 0.0),
            T_base_camera=np.eye(4),
            image_center=(0.0, 0.0),
            image_roll_rad=0.0,
            distance_m=0.3,
            incidence_deg=1.0,
            area_fraction=0.2,
            image_margin_px=30.0,
            feature=feature,
            quality=0.5))
    selected = selector.select_diverse(candidates, (0.0,) * 5, 10)
    assert len(selected) == 10
    assert len({id(candidate) for candidate in selected}) == 10
    selected_features = np.asarray(
        [candidate.feature for candidate in selected])
    min_selected = min(
        np.linalg.norm(selected_features[i] - selected_features[j])
        for i in range(len(selected_features))
        for j in range(i + 1, len(selected_features)))
    first_features = np.asarray(
        [candidate.feature for candidate in candidates[:10]])
    min_first = min(
        np.linalg.norm(first_features[i] - first_features[j])
        for i in range(len(first_features))
        for j in range(i + 1, len(first_features)))
    assert min_selected > min_first
    repeated = selector.select_diverse(candidates, (0.0,) * 5, 10)
    assert [candidate.joints for candidate in repeated] == [
        candidate.joints for candidate in selected]


def test_pose_entries_are_five_joint_and_mark_base():
    """Serialized poses always carry five joints and flag the base pose."""
    candidate = selector.Candidate(
        joints=(0.0, 0.1, -0.2, 0.3, 0.0),
        T_base_camera=np.eye(4),
        image_center=(320.0, 200.0),
        image_roll_rad=0.1,
        distance_m=0.3,
        incidence_deg=2.0,
        area_fraction=0.2,
        image_margin_px=30.0,
        feature=np.zeros(4),
        quality=0.5)
    entries = selector._pose_entries([candidate], (0.0, 0.1, -0.2, 0.3, 0.0))
    assert entries[0]['joints'] == [0.0, 0.1, -0.2, 0.3, 0.0]
    assert entries[0]['is_base'] is True


@pytest.mark.skipif(
    not CALIBRATION_JSON.exists(), reason='calibration evidence missing')
def test_real_board_pose_generates_diverse_feasible_poses():
    """The validated board/camera pair yields a passing 20-pose set."""
    pytest.importorskip('armpi_grasp_planner.reachability')
    from armpi_grasp_planner.reachability import ArmKinematics

    board, camera, _ = selector.load_calibration_inputs(
        str(CALIBRATION_JSON))
    base_pose = selector.DEFAULT_BASE_POSE
    config = selector.SelectorConfig(count=20)
    kinematics = ArmKinematics(tip_link='link4')
    candidates, grid_count = selector.generate_candidates(
        kinematics, board, camera, base_pose, config)
    assert grid_count > 0
    assert candidates
    assert all(candidate.joints[4] == base_pose[4]
               for candidate in candidates)
    selected = selector.select_diverse(candidates, base_pose, 20)
    assert len(selected) == 20
    report = selector.diversity_report(selected, base_pose, config)
    criteria = selector.selection_criteria(selected, report, config)
    assert criteria['success'], json.dumps(criteria, indent=2)
    assert min(report['joint_std_rad'][:4]) > 0.02
    assert report['view_angle_spread_deg']['max'] > 10.0
    assert report['coplanarity']['planarity_ratio'] > 0.01


def test_parse_base_pose_requires_five_values():
    """The CLI base pose parser rejects incomplete vectors."""
    assert selector._parse_base_pose('0,0.1,-0.2,0.3,0') == (
        0.0, 0.1, -0.2, 0.3, 0.0)
    with pytest.raises(ValueError):
        selector._parse_base_pose('0,0.1')
