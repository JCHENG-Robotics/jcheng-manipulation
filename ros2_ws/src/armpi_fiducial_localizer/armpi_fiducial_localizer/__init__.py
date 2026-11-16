"""AprilTag/ArUco target localization for ArmPi Ultra."""

from .detector import (
    available_dictionaries,
    DEFAULT_DICTIONARY,
    DEFAULT_MARKER_SIZE_M,
    FiducialDetection,
    FiducialDetector,
    rotation_to_quaternion,
)
from .synthetic import (
    default_camera_matrix,
    marker_object_points,
    render_scene,
)

__all__ = [
    'DEFAULT_DICTIONARY',
    'DEFAULT_MARKER_SIZE_M',
    'FiducialDetection',
    'FiducialDetector',
    'available_dictionaries',
    'default_camera_matrix',
    'marker_object_points',
    'render_scene',
    'rotation_to_quaternion',
]
