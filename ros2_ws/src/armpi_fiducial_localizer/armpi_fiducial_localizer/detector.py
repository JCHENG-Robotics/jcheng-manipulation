"""
AprilTag/ArUco detection and 6D pose estimation.

The detector uses a square marker of known edge length. Object points follow
OpenCV's IPPE square convention: x right, y up, z out of the marker plane
toward the viewer. The returned pose is the marker frame in the camera optical
frame:

    p_camera = R_camera_marker * p_marker + t_camera_marker

The pose is computed with ``solvePnP(SOLVEPNP_IPPE_SQUARE)`` and filtered by a
mean corner reprojection error. This module has no ROS dependency.
"""

import dataclasses
import math

import cv2
import numpy as np

DEFAULT_DICTIONARY = 'DICT_APRILTAG_36h11'
DEFAULT_MARKER_SIZE_M = 0.06
DEFAULT_MAX_REPROJECTION_ERROR_PX = 3.0


def available_dictionaries():
    """Return the ArUco predefined dictionary names in the installed OpenCV."""
    return sorted(
        name for name in dir(cv2.aruco) if name.startswith('DICT_')
    )


def make_dictionary(name):
    """Create an ArUco dictionary, supporting both OpenCV APIs."""
    if name not in available_dictionaries():
        raise ValueError('unknown ArUco dictionary {}'.format(name))
    constant = getattr(cv2.aruco, name)
    if hasattr(cv2.aruco, 'Dictionary_get'):
        return cv2.aruco.Dictionary_get(constant)
    return cv2.aruco.getPredefinedDictionary(constant)


def make_detector_parameters(refine_corners=True):
    """
    Create detector parameters with optional sub-pixel refinement.

    OpenCV 4.6 exposes the newer ``DetectorParameters`` type but constructing
    it directly and setting properties can crash. Prefer the legacy factory
    when it exists.
    """
    if hasattr(cv2.aruco, 'DetectorParameters_create'):
        parameters = cv2.aruco.DetectorParameters_create()
    else:
        parameters = cv2.aruco.DetectorParameters()
    if refine_corners:
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    else:
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    return parameters


def draw_marker_image(
    dictionary_name,
    marker_id,
    side_pixels,
    border_bits=1,
):
    """Render a marker image, supporting both OpenCV APIs."""
    dictionary = make_dictionary(dictionary_name)
    if hasattr(cv2.aruco, 'drawMarker'):
        return cv2.aruco.drawMarker(
            dictionary, marker_id, side_pixels, border_bits
        )
    return cv2.aruco.generateImageMarker(
        dictionary, marker_id, side_pixels, border_bits
    )


def _detect_markers(image, dictionary, parameters):
    if hasattr(cv2.aruco, 'detectMarkers'):
        return cv2.aruco.detectMarkers(
            image, dictionary, parameters=parameters
        )
    detector = cv2.aruco.ArucoDetector(dictionary, parameters)
    return detector.detectMarkers(image)


def _to_gray(image):
    array = np.asarray(image)
    if array.ndim == 2:
        return array
    if array.shape[2] == 4:
        return cv2.cvtColor(array, cv2.COLOR_BGRA2GRAY)
    return cv2.cvtColor(array, cv2.COLOR_BGR2GRAY)


def rotation_to_quaternion(rotation):
    """Convert a 3x3 rotation matrix to a unit quaternion ``(x, y, z, w)``."""
    matrix = np.asarray(rotation, dtype=np.float64)
    trace = matrix[0, 0] + matrix[1, 1] + matrix[2, 2]
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(
            1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]
        ) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(
            1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]
        ) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(
            1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]
        ) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-15:
        raise ValueError('cannot normalize a zero quaternion')
    return (x / norm, y / norm, z / norm, w / norm)


@dataclasses.dataclass(frozen=True)
class FiducialDetection:
    """One detected marker with its pose in the camera optical frame."""

    marker_id: int
    corners_px: tuple
    rvec: tuple
    tvec: tuple
    quaternion: tuple
    rotation_matrix: tuple
    reprojection_error_px: float
    marker_side_px: float

    def as_dict(self):
        """Return a JSON-friendly representation of the detection."""
        return {
            'marker_id': self.marker_id,
            'corners_px': [list(point) for point in self.corners_px],
            'rvec': list(self.rvec),
            'tvec': list(self.tvec),
            'quaternion_xyzw': list(self.quaternion),
            'rotation_matrix': [
                list(row) for row in self.rotation_matrix
            ],
            'reprojection_error_px': self.reprojection_error_px,
            'marker_side_px': self.marker_side_px,
        }


class FiducialDetector:
    """Detect square fiducials and estimate their 6D pose."""

    def __init__(
        self,
        dictionary=DEFAULT_DICTIONARY,
        marker_size_m=DEFAULT_MARKER_SIZE_M,
        max_reprojection_error_px=DEFAULT_MAX_REPROJECTION_ERROR_PX,
        refine_corners=True,
    ):
        self.dictionary_name = dictionary
        self.marker_size_m = float(marker_size_m)
        self.max_reprojection_error_px = float(
            max_reprojection_error_px
        )
        self.dictionary = make_dictionary(dictionary)
        self.parameters = make_detector_parameters(refine_corners)
        half = self.marker_size_m / 2.0
        self.object_points = np.array(
            [
                [-half, half, 0.0],
                [half, half, 0.0],
                [half, -half, 0.0],
                [-half, -half, 0.0],
            ],
            dtype=np.float64,
        )

    def detect(self, image, camera_matrix, dist_coeffs=None):
        """Detect markers and sort them by reprojection error."""
        gray = _to_gray(image)
        corners, ids, _ = _detect_markers(
            gray, self.dictionary, self.parameters
        )
        if ids is None:
            return []
        camera = np.asarray(camera_matrix, dtype=np.float64).reshape(3, 3)
        distortion = self._distortion(dist_coeffs)
        detections = []
        for marker_corners, marker_id in zip(corners, ids.reshape(-1)):
            points = marker_corners.reshape(4, 2).astype(np.float64)
            solved, rvec, tvec = cv2.solvePnP(
                self.object_points,
                points.reshape(-1, 1, 2),
                camera,
                distortion,
                flags=cv2.SOLVEPNP_IPPE_SQUARE,
            )
            if not solved:
                continue
            reprojection = self._reprojection_error(
                rvec, tvec, points, camera, distortion
            )
            if reprojection > self.max_reprojection_error_px:
                continue
            rotation = cv2.Rodrigues(rvec)[0]
            detections.append(
                FiducialDetection(
                    marker_id=int(marker_id),
                    corners_px=tuple(
                        tuple(float(value) for value in point)
                        for point in points
                    ),
                    rvec=tuple(
                        float(value) for value in rvec.reshape(3)
                    ),
                    tvec=tuple(
                        float(value) for value in tvec.reshape(3)
                    ),
                    quaternion=rotation_to_quaternion(rotation),
                    rotation_matrix=tuple(
                        tuple(float(value) for value in row)
                        for row in rotation
                    ),
                    reprojection_error_px=reprojection,
                    marker_side_px=self._mean_side_px(points),
                )
            )
        detections.sort(
            key=lambda detection: detection.reprojection_error_px
        )
        return detections

    def _distortion(self, dist_coeffs):
        if dist_coeffs is None:
            return np.zeros(5, dtype=np.float64)
        distortion = np.asarray(dist_coeffs, dtype=np.float64).reshape(-1)
        if distortion.size == 0:
            return np.zeros(5, dtype=np.float64)
        return distortion

    def _reprojection_error(
        self, rvec, tvec, points, camera, distortion
    ):
        projected, _ = cv2.projectPoints(
            self.object_points, rvec, tvec, camera, distortion
        )
        return float(
            np.mean(
                np.linalg.norm(
                    projected.reshape(4, 2) - points, axis=1
                )
            )
        )

    @staticmethod
    def _mean_side_px(points):
        sides = [
            np.linalg.norm(points[(index + 1) % 4] - points[index])
            for index in range(4)
        ]
        return float(np.mean(sides))
