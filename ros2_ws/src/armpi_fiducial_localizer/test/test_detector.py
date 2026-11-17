"""Tests for the fiducial detector and synthetic renderer."""

import math

from armpi_fiducial_localizer.detector import (
    available_dictionaries,
    DEFAULT_DICTIONARY,
    FiducialDetector,
    rotation_to_quaternion,
)
from armpi_fiducial_localizer.synthetic import (
    default_camera_matrix,
    render_scene,
)

import numpy as np
import pytest


def _rotation_error_deg(rvec_a, rvec_b):
    import cv2

    rotation_a = cv2.Rodrigues(np.asarray(rvec_a, dtype=np.float64))[0]
    rotation_b = cv2.Rodrigues(np.asarray(rvec_b, dtype=np.float64))[0]
    relative = rotation_a.T @ rotation_b
    skew = 0.5 * np.array(
        [
            relative[2, 1] - relative[1, 2],
            relative[0, 2] - relative[2, 0],
            relative[1, 0] - relative[0, 1],
        ]
    )
    return math.degrees(
        math.atan2(
            float(np.linalg.norm(skew)),
            float(0.5 * (np.trace(relative) - 1.0)),
        )
    )


def test_default_dictionary_is_available():
    """The default AprilTag dictionary is present in this OpenCV build."""
    assert DEFAULT_DICTIONARY in available_dictionaries()


def test_unknown_dictionary_raises():
    """An unknown dictionary name is rejected."""
    with pytest.raises(ValueError):
        FiducialDetector(dictionary='DICT_NOT_A_DICTIONARY')


def test_quaternion_from_identity_and_quarter_turn():
    """Known rotations produce known quaternions."""
    identity = np.eye(3)
    assert rotation_to_quaternion(identity) == pytest.approx(
        (0.0, 0.0, 0.0, 1.0)
    )
    quarter = np.array(
        [[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]]
    )
    assert rotation_to_quaternion(quarter) == pytest.approx(
        (0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5))
    )


def test_blank_image_has_no_detections():
    """A blank image yields no detections."""
    detector = FiducialDetector()
    camera = default_camera_matrix()
    image = np.full((480, 640), 255, dtype=np.uint8)
    assert detector.detect(image, camera) == []


def test_marker_id_is_reported():
    """The rendered marker ID is the detected ID."""
    detector = FiducialDetector()
    camera = default_camera_matrix()
    image, _ = render_scene(
        marker_id=7,
        marker_size_m=0.06,
        camera_matrix=camera,
        rvec=(math.pi, 0.0, 0.0),
        tvec=(0.0, 0.0, 0.35),
    )
    detections = detector.detect(image, camera)
    assert detections
    assert detections[0].marker_id == 7


@pytest.mark.parametrize(
    ('rvec', 'tvec'),
    (
        ((math.pi, 0.0, 0.0), (0.0, 0.0, 0.30)),
        (
            (math.pi + 0.15, -0.05, 0.10),
            (0.04, -0.03, 0.35),
        ),
        (
            (math.pi - 0.15, 0.10, -0.10),
            (-0.05, 0.03, 0.40),
        ),
        (
            (math.pi + 0.25, 0.15, 0.20),
            (0.06, 0.04, 0.45),
        ),
    ),
)
def test_synthetic_pose_round_trip(rvec, tvec):
    """The detector recovers a synthetic marker pose within tolerance."""
    detector = FiducialDetector()
    camera = default_camera_matrix()
    image, _ = render_scene(
        marker_id=0,
        marker_size_m=0.06,
        camera_matrix=camera,
        rvec=rvec,
        tvec=tvec,
    )
    detections = detector.detect(image, camera)
    assert detections
    detection = detections[0]
    translation_error = (
        np.linalg.norm(np.asarray(detection.tvec) - np.asarray(tvec))
        * 1000.0
    )
    assert translation_error < 4.0
    assert _rotation_error_deg(rvec, detection.rvec) < 3.0
    assert detection.reprojection_error_px < 1.0


def test_fronto_parallel_pose_refinement():
    """Pose refinement removes the fronto-parallel IPPE ambiguity."""
    detector = FiducialDetector()
    camera = default_camera_matrix()
    rvec = (math.pi, 0.0, 0.0)
    image, _ = render_scene(
        marker_id=0,
        marker_size_m=0.06,
        camera_matrix=camera,
        rvec=rvec,
        tvec=(0.0, 0.0, 0.30),
    )
    detections = detector.detect(image, camera)
    assert detections
    assert _rotation_error_deg(rvec, detections[0].rvec) < 0.5
    assert detections[0].reprojection_error_px < 1.0


def test_pose_refinement_can_be_disabled():
    """A detector with refinement disabled still returns a pose."""
    detector = FiducialDetector(refine_pose=False)
    camera = default_camera_matrix()
    image, _ = render_scene(
        marker_id=0,
        marker_size_m=0.06,
        camera_matrix=camera,
        rvec=(math.pi, 0.0, 0.0),
        tvec=(0.0, 0.0, 0.30),
    )
    assert detector.detect(image, camera)
