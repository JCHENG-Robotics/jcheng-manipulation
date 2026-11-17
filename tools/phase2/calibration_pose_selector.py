#!/usr/bin/env python3
"""Select diverse, executable chessboard poses for hand-eye calibration.

The camera is rigidly mounted on ``link4``, so ``joint5`` does not change the
view. Candidate joint vectors are generated over ``joint1..joint4`` and
filtered by:

- the project URDF joint limits, with a safety margin;
- forward kinematics through the measured ``T_link4_camera``;
- the whole chessboard inside the image with a pixel margin;
- board distance, incidence angle and projected area bounds.

Selection is a greedy farthest-point search in a weighted feature space
(camera position in the board frame, viewing direction, image roll and image
position). The report also quantifies camera-pose diversity, rotational
excitation and coplanarity, because a set of nearly coplanar views cannot
constrain a hand-eye calibration.

This is an offline planning tool. It never moves the robot. The output
``poses.json`` is consumed by ``chessboard_capture.py --poses-file`` on the
robot.

Example (laptop, Jazzy workspace sourced)::

    source /opt/ros/jazzy/setup.bash
    source ros2_ws/install/setup.bash
    python3 tools/phase2/calibration_pose_selector.py \
        --calibration-json \
            evidence/calibration/20261117/calibration_servo_20261117.json \
        --output evidence/calibration/20261117/pose_selection/ \
        --count 24 --plot
"""

import argparse
import json
import math
import os
from dataclasses import dataclass
from datetime import datetime, timezone

import cv2
import numpy as np

DEFAULT_BASE_POSE = (0.0, 0.59888, -1.25664, -0.71209, 0.0)
DEFAULT_CALIBRATION_JSON = (
    'evidence/calibration/20261117/calibration_servo_20261117.json')
DEFAULT_IMAGE_SIZE = (640, 400)
ARM_JOINT_NAMES = ['joint1', 'joint2', 'joint3', 'joint4']


def make_transform(rotation, translation):
    """Build a 4x4 homogeneous transform."""
    out = np.eye(4)
    out[:3, :3] = np.asarray(rotation, dtype=np.float64)
    out[:3, 3] = np.asarray(translation, dtype=np.float64).reshape(3)
    return out


def invert_transform(transform):
    """Return the inverse of a rigid 4x4 transform."""
    rotation = transform[:3, :3]
    translation = transform[:3, 3]
    out = np.eye(4)
    out[:3, :3] = rotation.T
    out[:3, 3] = -rotation.T @ translation
    return out


def transform_points(transform, points):
    """Apply a 4x4 transform to an Nx3 point array."""
    points = np.asarray(points, dtype=np.float64).reshape(-1, 3)
    return (transform[:3, :3] @ points.T).T + transform[:3, 3]


def frame_to_transform(frame):
    """Convert a PyKDL Frame to a 4x4 numpy transform."""
    rotation = np.array(
        [[frame.M[row, column] for column in range(3)] for row in range(3)],
        dtype=np.float64)
    translation = np.array(
        [frame.p.x(), frame.p.y(), frame.p.z()], dtype=np.float64)
    return make_transform(rotation, translation)


def rotation_angle_deg(rotation):
    """Return the geodesic angle of a rotation matrix in degrees."""
    cosine = (np.trace(rotation) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


def board_corners(board):
    """Return the four inner-corner rectangle corners in the board frame."""
    width = (board.cols - 1) * board.square_m
    height = (board.rows - 1) * board.square_m
    return np.array([
        [0.0, 0.0, 0.0],
        [width, 0.0, 0.0],
        [width, height, 0.0],
        [0.0, height, 0.0],
    ], dtype=np.float64)


def polygon_area(points):
    """Return the absolute area of a 2D polygon."""
    x = points[:, 0]
    y = points[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, -1))
                           - np.dot(y, np.roll(x, -1))))


def project_camera_points(points_cam, camera):
    """Project Nx3 camera-frame points into pixels with the camera model."""
    points = np.asarray(points_cam, dtype=np.float64).reshape(-1, 3)
    projected, _ = cv2.projectPoints(
        points, np.zeros(3), np.zeros(3), camera.k, camera.d)
    return projected.reshape(-1, 2)


@dataclass(frozen=True)
class BoardSpec:
    """Chessboard geometry and its fixed pose in ``base_link``."""

    cols: int
    rows: int
    square_m: float
    T_base_board: np.ndarray


@dataclass(frozen=True)
class CameraSpec:
    """Camera intrinsics and the measured camera pose in ``link4``."""

    width: int
    height: int
    k: np.ndarray
    d: np.ndarray
    T_link4_camera: np.ndarray


@dataclass(frozen=True)
class SelectorConfig:
    """Search and feasibility parameters for pose selection."""

    count: int = 24
    grid_step_rad: float = 0.05
    # Conservative envelope around the validated 2026-11-17 capture pose.
    # Mechanical limits are not measured yet; widen only after the joint-limit
    # survey. These ranges are deliberately larger than the 15-pose run
    # (joint1/joint4 +/-0.12, joint2/joint3 +/-0.06) but stay near the tested
    # region.
    joint_ranges_rad: tuple = (
        (-0.35, 0.35), (-0.18, 0.18), (-0.15, 0.15), (-0.35, 0.35))
    joint_margin_rad: float = 0.03
    min_distance_m: float = 0.18
    max_distance_m: float = 0.60
    max_incidence_deg: float = 60.0
    min_area_fraction: float = 0.02
    min_image_margin_px: float = 25.0
    feature_position_scale_m: float = 0.20
    feature_view_weight: float = 1.5
    feature_roll_weight: float = 0.5
    feature_image_weight: float = 0.5
    duplicate_position_epsilon_m: float = 0.001
    duplicate_view_epsilon_deg: float = 0.5
    min_camera_spread_m: float = 0.05
    min_view_spread_deg: float = 10.0
    min_image_center_std_px: float = 20.0
    min_joint_std_rad: float = 0.02
    min_rotation_axis_rank_ratio: float = 0.05
    min_planarity_ratio: float = 0.01


@dataclass
class Candidate:
    """A feasible candidate pose and its predicted image metrics."""

    joints: tuple
    T_base_camera: np.ndarray
    image_center: tuple
    image_roll_rad: float
    distance_m: float
    incidence_deg: float
    area_fraction: float
    image_margin_px: float
    feature: np.ndarray
    quality: float


def _axis_values(lower, upper, step):
    """Return a deterministic inclusive grid from lower to upper."""
    if upper < lower:
        raise ValueError(
            'invalid grid range [{}, {}]'.format(lower, upper))
    if step <= 0.0:
        raise ValueError('grid step must be positive')
    count = int(math.floor((upper - lower) / step))
    values = [lower + index * step for index in range(count + 1)]
    if abs(values[-1] - upper) > 1e-9:
        values.append(upper)
    return values


def _joint_limits(kinematics, config):
    """Return per-joint URDF limits including the safety margin."""
    lower = []
    upper = []
    if len(kinematics.joint_names) != 4:
        raise ValueError(
            'expected a four-joint link4 chain, got {}'.format(
                kinematics.joint_names))
    for name in ARM_JOINT_NAMES:
        limit_lower, limit_upper = kinematics.limits[name]
        bound_lower = limit_lower + config.joint_margin_rad
        bound_upper = limit_upper - config.joint_margin_rad
        if bound_lower > bound_upper:
            raise ValueError(
                'no usable range for {} inside [{}, {}]'.format(
                    name, limit_lower, limit_upper))
        lower.append(bound_lower)
        upper.append(bound_upper)
    return lower, upper


def _projected_metrics(points_cam, camera, config):
    """Return image metrics for board corners in the camera frame."""
    projected = project_camera_points(points_cam, camera)
    if not np.all(np.isfinite(projected)):
        return None
    width = camera.width
    height = camera.height
    margin = min(
        float(projected[:, 0].min()), float(projected[:, 1].min()),
        float(width - projected[:, 0].max()),
        float(height - projected[:, 1].max()))
    if margin < config.min_image_margin_px:
        return None
    area = polygon_area(projected) / float(width * height)
    if area < config.min_area_fraction or area > 0.95:
        return None
    center = projected.mean(axis=0)
    x_axis = projected[1] - projected[0]
    roll = math.atan2(float(x_axis[1]), float(x_axis[0]))
    return {
        'projected': projected,
        'margin_px': margin,
        'area_fraction': area,
        'image_center': (float(center[0]), float(center[1])),
        'image_roll_rad': roll,
    }


def _quality(metrics, config):
    """Return a scalar quality score in [0, 1] for tie-breaking."""
    margin_score = min(
        1.0, metrics['margin_px'] / (2.0 * config.min_image_margin_px))
    area_score = min(1.0, metrics['area_fraction'] / 0.10)
    distance_mid = 0.5 * (config.min_distance_m + config.max_distance_m)
    half_range = max(
        1e-6, 0.5 * (config.max_distance_m - config.min_distance_m))
    distance_score = max(
        0.0, 1.0 - abs(metrics['distance_m'] - distance_mid) / half_range)
    incidence_score = max(
        0.0, 1.0 - metrics['incidence_deg'] / config.max_incidence_deg)
    return float(np.mean([
        margin_score, area_score, distance_score, incidence_score]))


def candidate_from_joints(kinematics, joints, board, camera, config):
    """Build a candidate, or return None when a feasibility test fails."""
    joints = tuple(float(value) for value in joints)
    frame = kinematics.forward(joints[:4])
    T_base_link4 = frame_to_transform(frame)
    T_base_camera = T_base_link4 @ camera.T_link4_camera
    T_cam_board = invert_transform(T_base_camera) @ board.T_base_board
    corners_cam = transform_points(
        T_cam_board, board_corners(board))
    center_cam = corners_cam.mean(axis=0)
    distance = float(np.linalg.norm(center_cam))
    if not config.min_distance_m <= distance <= config.max_distance_m:
        return None
    unit_to_camera = -center_cam / max(distance, 1e-9)
    board_normal_cam = T_cam_board[:3, :3][:, 2]
    cosine = abs(float(np.dot(board_normal_cam, unit_to_camera)))
    incidence = math.degrees(
        math.acos(float(np.clip(cosine, -1.0, 1.0))))
    if incidence > config.max_incidence_deg:
        return None
    metrics = _projected_metrics(corners_cam, camera, config)
    if metrics is None:
        return None
    metrics['distance_m'] = distance
    metrics['incidence_deg'] = incidence
    T_board_camera = invert_transform(T_cam_board)
    position_board = T_board_camera[:3, 3]
    view_board = T_board_camera[:3, :3][:, 2]
    image_center = metrics['image_center']
    image_norm = np.array([
        (image_center[0] - 0.5 * camera.width) / (0.5 * camera.width),
        (image_center[1] - 0.5 * camera.height) / (0.5 * camera.height),
    ])
    feature = np.concatenate([
        position_board / config.feature_position_scale_m,
        config.feature_view_weight * view_board,
        config.feature_roll_weight * np.array([
            math.cos(metrics['image_roll_rad']),
            math.sin(metrics['image_roll_rad']),
        ]),
        config.feature_image_weight * image_norm,
    ])
    return Candidate(
        joints=joints,
        T_base_camera=T_base_camera,
        image_center=metrics['image_center'],
        image_roll_rad=metrics['image_roll_rad'],
        distance_m=distance,
        incidence_deg=incidence,
        area_fraction=metrics['area_fraction'],
        image_margin_px=metrics['margin_px'],
        feature=feature,
        quality=_quality(metrics, config))


def generate_candidates(kinematics, board, camera, base_pose, config):
    """Enumerate the joint grid and keep the feasible candidates."""
    lower, upper = _joint_limits(kinematics, config)
    grids = [
        _axis_values(
            max(lower[index], base_pose[index] + config.joint_ranges_rad[
                index][0]),
            min(upper[index], base_pose[index] + config.joint_ranges_rad[
                index][1]),
            config.grid_step_rad)
        for index in range(4)
    ]
    candidates = []
    grid_count = 0
    for j1 in grids[0]:
        for j2 in grids[1]:
            for j3 in grids[2]:
                for j4 in grids[3]:
                    grid_count += 1
                    candidate = candidate_from_joints(
                        kinematics,
                        (j1, j2, j3, j4, base_pose[4]),
                        board, camera, config)
                    if candidate is not None:
                        candidates.append(candidate)
    # Keep the validated base pose even when no grid point lands on it. It is
    # the safest start point and can be repeated as the held-out final base.
    base_candidate = candidate_from_joints(
        kinematics, base_pose, board, camera, config)
    if base_candidate is not None and not any(
            _joint_distance(candidate.joints, base_pose) <= 1e-9
            for candidate in candidates):
        candidates.append(base_candidate)
        grid_count += 1
    if not candidates:
        raise RuntimeError(
            'no feasible poses in the search grid; check board pose, '
            'camera intrinsics and distance/incidence bounds')
    return candidates, grid_count


def _joint_distance(first, second):
    """Return the max-abs joint difference between two poses."""
    return max(abs(a - b) for a, b in zip(first, second))


def select_diverse(candidates, base_pose, count):
    """Greedily select poses that maximize the minimum pairwise distance."""
    if count < 2:
        raise ValueError('count must be at least two')
    if count > len(candidates):
        raise ValueError(
            'requested {} poses but only {} are feasible'.format(
                count, len(candidates)))
    features = np.asarray([candidate.feature for candidate in candidates])
    qualities = np.asarray([candidate.quality for candidate in candidates])
    base_index = min(
        range(len(candidates)),
        key=lambda index: _joint_distance(
            candidates[index].joints, base_pose))
    selected = [base_index]
    min_distance = np.linalg.norm(features - features[base_index], axis=1)
    while len(selected) < count:
        scores = min_distance + 1e-6 * qualities
        scores[selected] = -np.inf
        best = int(np.argmax(scores))
        selected.append(best)
        distance = np.linalg.norm(features - features[best], axis=1)
        min_distance = np.minimum(min_distance, distance)
    return [candidates[index] for index in selected]


def _pairwise_rotation_axis_rank(candidates):
    """Return SVD singular values of angle-weighted relative rotations."""
    vectors = []
    for first in range(len(candidates)):
        rotation_first = candidates[first].T_base_camera[:3, :3]
        for second in range(first + 1, len(candidates)):
            rotation_second = candidates[second].T_base_camera[:3, :3]
            relative = rotation_first.T @ rotation_second
            vector = cv2.Rodrigues(relative)[0].reshape(3)
            vectors.append(vector)
    if not vectors:
        return [0.0, 0.0, 0.0]
    singular = np.linalg.svd(
        np.asarray(vectors), compute_uv=False)
    return [float(value) for value in singular[:3]]


def _coplanarity(candidates):
    """Return SVD and plane-residual metrics for the camera centers."""
    positions = np.asarray([
        candidate.T_base_camera[:3, 3] for candidate in candidates])
    centered = positions - positions.mean(axis=0)
    _, _, vt = np.linalg.svd(centered, full_matrices=False)
    scalar = centered @ vt[2]
    singular = np.linalg.svd(centered, compute_uv=False)
    return {
        'camera_position_singular_values': [
            float(value) for value in singular],
        'planarity_ratio': float(
            singular[2] / singular[0]) if singular[0] > 0.0 else 0.0,
        'rms_plane_residual_m': float(np.sqrt(np.mean(scalar ** 2))),
        'max_plane_residual_m': float(np.max(np.abs(scalar))),
    }


def diversity_report(candidates, base_pose, config):
    """Quantify camera-pose diversity, excitation and coplanarity."""
    positions = np.asarray([
        candidate.T_base_camera[:3, 3] for candidate in candidates])
    views = np.asarray([
        candidate.T_base_camera[:3, 2] for candidate in candidates])
    position_distances = []
    view_angles = []
    rotation_angles = []
    duplicate_scores = []
    for first in range(len(candidates)):
        for second in range(first + 1, len(candidates)):
            position_distance = float(np.linalg.norm(
                positions[first] - positions[second]))
            position_distances.append(position_distance)
            cosine = float(np.clip(
                np.dot(views[first], views[second]), -1.0, 1.0))
            view_angle = math.degrees(math.acos(cosine))
            view_angles.append(view_angle)
            duplicate_scores.append(max(
                position_distance / config.duplicate_position_epsilon_m,
                view_angle / config.duplicate_view_epsilon_deg))
            relative = (
                candidates[first].T_base_camera[:3, :3].T
                @ candidates[second].T_base_camera[:3, :3])
            rotation_angles.append(rotation_angle_deg(relative))
    joint_values = np.asarray([candidate.joints for candidate in candidates])
    distances = [candidate.distance_m for candidate in candidates]
    incidences = [candidate.incidence_deg for candidate in candidates]
    areas = [candidate.area_fraction for candidate in candidates]
    margins = [candidate.image_margin_px for candidate in candidates]
    image_centers = np.asarray(
        [candidate.image_center for candidate in candidates])
    rolls = np.asarray(
        [candidate.image_roll_rad for candidate in candidates])
    axis_singular = _pairwise_rotation_axis_rank(candidates)
    coplanarity = _coplanarity(candidates)

    def values(items):
        return {
            'min': float(np.min(items)),
            'mean': float(np.mean(items)),
            'max': float(np.max(items)),
        }

    report = {
        'camera_position_spread_m': values(position_distances),
        'view_angle_spread_deg': values(view_angles),
        'relative_rotation_spread_deg': values(rotation_angles),
        'min_pair_duplicate_score': float(min(duplicate_scores)),
        'board_distance_m': values(distances),
        'incidence_deg': values(incidences),
        'board_area_fraction': values(areas),
        'image_margin_px': values(margins),
        'image_center_px_std': [
            float(image_centers[:, 0].std()),
            float(image_centers[:, 1].std()),
        ],
        'image_roll_deg_std': float(np.degrees(rolls).std()),
        'joint_std_rad': [
            float(value) for value in joint_values.std(axis=0)],
        'joint5_range_rad': [
            float(joint_values[:, 4].min()),
            float(joint_values[:, 4].max()),
        ],
        'rotation_axis_singular_values': axis_singular,
        'rotation_axis_rank_ratio': float(
            axis_singular[2] / axis_singular[0])
        if axis_singular[0] > 0.0 else 0.0,
        'coplanarity': coplanarity,
        'joint_offsets_from_base_rad': {
            name: [
                float(value) for value in (
                    joint_values[:, index] - base_pose[index])
            ]
            for index, name in enumerate(ARM_JOINT_NAMES)
        },
    }
    return report


def selection_criteria(selected, report, config):
    """Return pass/fail criteria for the selected pose set."""
    checks = {
        'requested_count': len(selected) == config.count,
        'joint5_fixed': (
            report['joint5_range_rad'][0]
            == report['joint5_range_rad'][1]),
        'no_duplicate_views': (
            report['min_pair_duplicate_score'] > 1.0),
        'camera_spread': (
            report['camera_position_spread_m']['max']
            >= config.min_camera_spread_m),
        'view_spread': (
            report['view_angle_spread_deg']['max']
            >= config.min_view_spread_deg),
        'image_coverage': (
            min(report['image_center_px_std'])
            >= config.min_image_center_std_px),
        'joint_spread': (
            min(report['joint_std_rad'][:4])
            >= config.min_joint_std_rad),
        'rotation_excitation': (
            report['rotation_axis_rank_ratio']
            >= config.min_rotation_axis_rank_ratio),
        'not_coplanar': (
            report['coplanarity']['planarity_ratio']
            >= config.min_planarity_ratio),
    }
    thresholds = {
        'requested_count': config.count,
        'duplicate_position_epsilon_m':
            config.duplicate_position_epsilon_m,
        'duplicate_view_epsilon_deg':
            config.duplicate_view_epsilon_deg,
        'min_camera_spread_m': config.min_camera_spread_m,
        'min_view_spread_deg': config.min_view_spread_deg,
        'min_image_center_std_px': config.min_image_center_std_px,
        'min_joint_std_rad': config.min_joint_std_rad,
        'min_rotation_axis_rank_ratio':
            config.min_rotation_axis_rank_ratio,
        'min_planarity_ratio': config.min_planarity_ratio,
        'joint5_fixed': True,
    }
    return {
        'checks': checks,
        'thresholds': thresholds,
        'diagnostics': {
            'min_pairwise_camera_distance_m':
                report['camera_position_spread_m']['min'],
            'min_pairwise_view_angle_deg':
                report['view_angle_spread_deg']['min'],
        },
        'success': all(checks.values()),
    }


def load_calibration_inputs(path, selected_method=None):
    """Load board, camera and hand-eye data from a calibration JSON."""
    with open(path, 'r', encoding='utf-8') as handle:
        data = json.load(handle)
    if 'hand_eye' not in data or 'methods' not in data['hand_eye']:
        raise ValueError(
            '{} is not a hand-eye calibration report'.format(path))
    method_name = selected_method or data['hand_eye'].get('selected')
    if method_name not in data['hand_eye']['methods']:
        raise ValueError(
            'method {} is not in {}'.format(method_name, path))
    method = data['hand_eye']['methods'][method_name]
    T_link4_camera = np.asarray(
        method['T_link4_camera'], dtype=np.float64)
    T_base_board = np.asarray(
        method['board_mean_transform'], dtype=np.float64)
    intrinsics = data.get('intrinsics', {})
    k = np.asarray(
        intrinsics.get('driver_k', data.get('camera_matrix')),
        dtype=np.float64)
    d = np.asarray(
        intrinsics.get('driver_d', data.get('dist_coeffs')),
        dtype=np.float64)
    if k.shape != (3, 3):
        raise ValueError('calibration JSON has no 3x3 camera matrix')
    board_data = data.get('board', {})
    board = BoardSpec(
        cols=int(board_data.get('cols', 11)),
        rows=int(board_data.get('rows', 8)),
        square_m=float(board_data.get('square_m', 0.020)),
        T_base_board=T_base_board)
    camera = CameraSpec(
        width=DEFAULT_IMAGE_SIZE[0],
        height=DEFAULT_IMAGE_SIZE[1],
        k=k,
        d=d,
        T_link4_camera=T_link4_camera)
    source = {
        'calibration_json': path,
        'selected_method': method_name,
        'board_pose_source': (
            'hand_eye.methods.{}.board_mean_transform'.format(
                method_name)),
        'link4_camera_source': (
            'hand_eye.methods.{}.T_link4_camera'.format(method_name)),
        'intrinsics_source': 'intrinsics.driver_k / intrinsics.driver_d',
    }
    return board, camera, source


def load_board_pose(path):
    """Load a standalone 4x4 board pose JSON."""
    with open(path, 'r', encoding='utf-8') as handle:
        data = json.load(handle)
    if 'T_base_board' not in data:
        raise ValueError(
            '{} needs a T_base_board 4x4 matrix'.format(path))
    return np.asarray(data['T_base_board'], dtype=np.float64)


def _pose_entries(selected, base_pose):
    """Return serializable selected pose entries."""
    out = []
    for index, candidate in enumerate(selected):
        out.append({
            'index': index,
            'joints': [float(value) for value in candidate.joints],
            'is_base': _joint_distance(
                candidate.joints, base_pose) <= 1e-9,
            'predicted': {
                'board_distance_m': candidate.distance_m,
                'incidence_deg': candidate.incidence_deg,
                'image_center_px': [
                    candidate.image_center[0],
                    candidate.image_center[1]],
                'image_roll_deg': math.degrees(
                    candidate.image_roll_rad),
                'board_area_fraction': candidate.area_fraction,
                'image_margin_px': candidate.image_margin_px,
                'quality': candidate.quality,
            },
        })
    return out


def _capture_command(poses_path):
    """Return a copy-paste capture command template."""
    return (
        'sudo docker cp tools/phase2/chessboard_capture.py '
        'ArmPiUltra:/tmp/ && '
        'sudo docker cp {poses} ArmPiUltra:/tmp/poses.json && '
        "sudo docker exec -it -u ubuntu -w /home/ubuntu ArmPiUltra "
        "/bin/zsh -c 'source ~/.zshrc >/dev/null 2>&1; "
        "python3 /tmp/chessboard_capture.py "
        "--output-dir /tmp/chessboard_calib_selected "
        "--poses-file /tmp/poses.json --pre-offset 0.08 "
        "--pre-joint1-only --final-base --servo-reads 3'"
    ).format(poses=poses_path)


def save_plot(report, selected, board, camera, path):
    """Save a three-panel summary plot for the selected pose set."""
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt

    positions = np.asarray([
        candidate.T_base_camera[:3, 3] for candidate in selected])
    views = np.asarray([
        candidate.T_base_camera[:3, 2] for candidate in selected])
    centers = np.asarray(
        [candidate.image_center for candidate in selected])
    fig = plt.figure(figsize=(15, 4.5))
    axis = fig.add_subplot(1, 3, 1, projection='3d')
    axis.scatter(
        positions[:, 0], positions[:, 1], positions[:, 2],
        c='tab:blue', label='camera')
    board_center = board.T_base_board[:3, 3]
    axis.scatter(
        [board_center[0]], [board_center[1]], [board_center[2]],
        c='tab:red', marker='s', label='board')
    for position, view in zip(positions, views):
        axis.plot(
            [position[0], position[0] + 0.05 * view[0]],
            [position[1], position[1] + 0.05 * view[1]],
            [position[2], position[2] + 0.05 * view[2]],
            color='tab:blue', alpha=0.4, linewidth=0.8)
    axis.set_xlabel('x [m]')
    axis.set_ylabel('y [m]')
    axis.set_zlabel('z [m]')
    axis.set_title('Camera positions in base_link')
    axis.legend(loc='best')

    image_axis = fig.add_subplot(1, 3, 2)
    image_axis.add_patch(plt.Rectangle(
        (0, 0), camera.width, camera.height, fill=False, color='black'))
    image_axis.scatter(
        centers[:, 0], centers[:, 1], c='tab:green')
    for index, center in enumerate(centers):
        image_axis.annotate(
            str(index), (center[0], center[1]),
            textcoords='offset points', xytext=(3, 3), fontsize=6)
    image_axis.set_xlim(0, camera.width)
    image_axis.set_ylim(camera.height, 0)
    image_axis.set_aspect('equal')
    image_axis.set_title('Predicted board centre in image')
    image_axis.set_xlabel('u [px]')
    image_axis.set_ylabel('v [px]')

    joint_axis = fig.add_subplot(1, 3, 3)
    joint_axis.boxplot([
        report['joint_offsets_from_base_rad'][name]
        for name in ARM_JOINT_NAMES])
    joint_axis.set_xticklabels(ARM_JOINT_NAMES)
    joint_axis.axhline(0.0, color='black', linewidth=0.8)
    joint_axis.set_ylabel('offset from base [rad]')
    joint_axis.set_title('Selected joint offsets')
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    plt.close(fig)


def _parse_base_pose(text):
    """Parse a comma-separated five-joint base pose."""
    values = [float(value) for value in text.split(',')]
    if len(values) != 5:
        raise ValueError('base pose needs five joint values')
    return tuple(values)


def main():
    """Run the pose selector and write the report."""
    parser = argparse.ArgumentParser(
        description='Select diverse chessboard poses for hand-eye '
                    'calibration.')
    parser.add_argument(
        '--calibration-json', default=DEFAULT_CALIBRATION_JSON,
        help='hand-eye calibration JSON with board pose and intrinsics')
    parser.add_argument(
        '--selected-method', default=None,
        help='hand-eye method to use (default: the report selected method)')
    parser.add_argument(
        '--board-pose', default=None,
        help='optional standalone JSON with T_base_board, overriding the '
             'calibration board pose')
    parser.add_argument(
        '--output', default='evidence/calibration/20261117/pose_selection',
        help='output directory')
    parser.add_argument(
        '--poses-output', default=None,
        help='slim pose JSON for chessboard_capture.py '
             '(default: <output>/poses.json)')
    parser.add_argument('--count', type=int, default=24)
    parser.add_argument(
        '--base-pose', default=','.join(str(value) for value in
                                        DEFAULT_BASE_POSE))
    parser.add_argument('--grid-step', type=float, default=0.05)
    parser.add_argument('--plot', action='store_true')
    parser.add_argument(
        '--joint1-range', default='-0.35,0.35',
        help='search range around the base pose, in radians')
    parser.add_argument(
        '--joint2-range', default='-0.18,0.18',
        help='search range around the base pose, in radians')
    parser.add_argument(
        '--joint3-range', default='-0.15,0.15',
        help='search range around the base pose, in radians')
    parser.add_argument(
        '--joint4-range', default='-0.35,0.35',
        help='search range around the base pose, in radians')
    args = parser.parse_args()

    def parse_range(text):
        values = [float(value) for value in text.split(',')]
        if len(values) != 2:
            raise ValueError('joint range needs lower,upper')
        return (values[0], values[1])

    base_pose = _parse_base_pose(args.base_pose)
    config = SelectorConfig(
        count=args.count,
        grid_step_rad=args.grid_step,
        joint_ranges_rad=(
            parse_range(args.joint1_range),
            parse_range(args.joint2_range),
            parse_range(args.joint3_range),
            parse_range(args.joint4_range),
        ))
    board, camera, source = load_calibration_inputs(
        args.calibration_json, args.selected_method)
    if args.board_pose:
        board = BoardSpec(
            cols=board.cols, rows=board.rows,
            square_m=board.square_m,
            T_base_board=load_board_pose(args.board_pose))
        source['board_pose_source'] = args.board_pose

    from armpi_grasp_planner.reachability import ArmKinematics
    kinematics = ArmKinematics(tip_link='link4')
    candidates, grid_count = generate_candidates(
        kinematics, board, camera, base_pose, config)
    selected = select_diverse(candidates, base_pose, config.count)
    report_diversity = diversity_report(selected, base_pose, config)
    criteria = selection_criteria(selected, report_diversity, config)
    poses = _pose_entries(selected, base_pose)

    os.makedirs(args.output, exist_ok=True)
    run_date = datetime.now(timezone.utc).strftime('%Y%m%d')
    report_name = 'pose_selection_{}.json'.format(run_date)
    plot_name = 'pose_selection_{}.png'.format(run_date)
    poses_path = args.poses_output or os.path.join(
        args.output, 'poses.json')
    report = {
        'schema_version': 1,
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'mode': 'offline_calibration_pose_selection',
        'source': source,
        'board': {
            'cols': board.cols,
            'rows': board.rows,
            'square_m': board.square_m,
            'T_base_board': board.T_base_board.tolist(),
        },
        'camera': {
            'width': camera.width,
            'height': camera.height,
            'k': camera.k.tolist(),
            'd': camera.d.tolist(),
            'T_link4_camera': camera.T_link4_camera.tolist(),
        },
        'base_pose': [float(value) for value in base_pose],
        'search': {
            'grid_step_rad': config.grid_step_rad,
            'joint_ranges_rad': config.joint_ranges_rad,
            'joint_margin_rad': config.joint_margin_rad,
            'min_distance_m': config.min_distance_m,
            'max_distance_m': config.max_distance_m,
            'max_incidence_deg': config.max_incidence_deg,
            'min_area_fraction': config.min_area_fraction,
            'min_image_margin_px': config.min_image_margin_px,
        },
        'counts': {
            'grid': grid_count,
            'feasible': len(candidates),
            'selected': len(selected),
        },
        'poses': poses,
        'diversity': report_diversity,
        'criteria': criteria,
        'capture_command': _capture_command(poses_path),
    }
    report_path = os.path.join(args.output, report_name)
    with open(report_path, 'w', encoding='utf-8') as handle:
        json.dump(report, handle, indent=2)
    with open(poses_path, 'w', encoding='utf-8') as handle:
        json.dump({
            'schema_version': 1,
            'kind': 'armpi_calibration_poses',
            'generated_at': report['generated_at'],
            'board': report['board'],
            'poses': [
                {'index': entry['index'], 'joints': entry['joints']}
                for entry in poses
            ],
        }, handle, indent=2)
    if args.plot:
        save_plot(
            report_diversity, selected, board, camera,
            os.path.join(args.output, plot_name))

    print(
        'grid={grid} feasible={feasible} selected={selected}'.format(
            grid=grid_count, feasible=len(candidates),
            selected=len(selected)))
    print(
        'camera distance min/mean/max = {:.3f}/{:.3f}/{:.3f} m'.format(
            report_diversity['camera_position_spread_m']['min'],
            report_diversity['camera_position_spread_m']['mean'],
            report_diversity['camera_position_spread_m']['max']))
    print(
        'view angle min/mean/max = {:.1f}/{:.1f}/{:.1f} deg'.format(
            report_diversity['view_angle_spread_deg']['min'],
            report_diversity['view_angle_spread_deg']['mean'],
            report_diversity['view_angle_spread_deg']['max']))
    print(
        'rotation axis rank ratio = {:.3f}, '
        'planarity ratio = {:.3f}'.format(
            report_diversity['rotation_axis_rank_ratio'],
            report_diversity['coplanarity']['planarity_ratio']))
    print('criteria:', 'PASS' if criteria['success'] else 'FAIL')
    for name, passed in criteria['checks'].items():
        print('  {}: {}'.format(name, 'ok' if passed else 'FAILED'))
    print('wrote', report_path)
    print('wrote', poses_path)
    if args.plot:
        print('wrote', os.path.join(args.output, plot_name))
    return 0 if criteria['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
