"""Detect a physical AprilTag in captured frames and report base_link poses.

Reads the PPM/JSON pairs written by ``capture_real_frames.py``, runs the
project's ``FiducialDetector`` with the captured intrinsics, and composes:

    T_base_marker = T_base_rgb_body @ T_rgb_optical @ T_optical_marker

``T_base_rgb_body`` comes from the captured TF; ``T_rgb_optical`` comes from
``armpi_fiducial_localizer/config/camera_optical_frame.yaml``. The script also
reports frame-to-frame translation/rotation spread so a static tag can be
checked for stability.

This is an offline analysis of real images. It does not move the robot.
"""

import argparse
import glob
import json
import math
import os
import sys

import cv2
import numpy as np
import yaml

from armpi_fiducial_localizer.detector import FiducialDetector


def quaternion_to_matrix(quaternion):
    """Return a 3x3 rotation matrix from xyzw quaternion."""
    x, y, z, w = [float(value) for value in quaternion]
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm == 0.0:
        raise ValueError('zero quaternion')
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def transform_from(translation, quaternion):
    """Return a 4x4 matrix from translation and xyzw quaternion."""
    matrix = np.eye(4)
    matrix[:3, :3] = quaternion_to_matrix(quaternion)
    matrix[:3, 3] = [float(value) for value in translation]
    return matrix


def rotation_angle_deg(rotation_a, rotation_b):
    """Return the geodesic angle between two rotation matrices."""
    relative = rotation_a.T @ rotation_b
    cosine = max(-1.0, min(1.0, 0.5 * (np.trace(relative) - 1.0)))
    return math.degrees(math.acos(cosine))


def matrix_to_quaternion(matrix):
    """Return an xyzw quaternion from a 3x3 rotation matrix."""
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        w = 0.25 * scale
        x = (matrix[2, 1] - matrix[1, 2]) / scale
        y = (matrix[0, 2] - matrix[2, 0]) / scale
        z = (matrix[1, 0] - matrix[0, 1]) / scale
    elif matrix[0, 0] > matrix[1, 1] and matrix[0, 0] > matrix[2, 2]:
        scale = math.sqrt(
            1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
        w = (matrix[2, 1] - matrix[1, 2]) / scale
        x = 0.25 * scale
        y = (matrix[0, 1] + matrix[1, 0]) / scale
        z = (matrix[0, 2] + matrix[2, 0]) / scale
    elif matrix[1, 1] > matrix[2, 2]:
        scale = math.sqrt(
            1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
        w = (matrix[0, 2] - matrix[2, 0]) / scale
        x = (matrix[0, 1] + matrix[1, 0]) / scale
        y = 0.25 * scale
        z = (matrix[1, 2] + matrix[2, 1]) / scale
    else:
        scale = math.sqrt(
            1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
        w = (matrix[1, 0] - matrix[0, 1]) / scale
        x = (matrix[0, 2] + matrix[2, 0]) / scale
        y = (matrix[1, 2] + matrix[2, 1]) / scale
        z = 0.25 * scale
    return [x, y, z, w]


def load_optical_transform(path):
    """Load the measured body -> optical transform from YAML."""
    with open(path, encoding='utf-8') as handle:
        config = yaml.safe_load(handle)
    return transform_from(
        config['translation_m'], config['quaternion_xyzw']), config


def marker_normal_tilt_deg(rotation):
    """Return the angle between the marker z axis and the camera -z axis."""
    normal = rotation[:, 2]
    cosine = float(np.dot(normal, np.array([0.0, 0.0, -1.0])))
    cosine = max(-1.0, min(1.0, cosine))
    return math.degrees(math.acos(cosine))


def analyze_frame(image_path, metadata, detector, optical, marker_id):
    """Detect the selected marker and return a result dict, or None."""
    image = cv2.imread(image_path)
    if image is None:
        raise RuntimeError(f'cannot read {image_path}')
    camera = np.asarray(
        metadata['camera_info']['k'], dtype=np.float64).reshape(3, 3)
    distortion = np.asarray(
        metadata['camera_info']['d'], dtype=np.float64)
    detections = detector.detect(image, camera, distortion)
    detections = [
        detection for detection in detections
        if detection.marker_id == marker_id]
    if not detections:
        return None, image, camera, distortion
    detection = detections[0]
    rotation_optical = np.asarray(
        detection.rotation_matrix, dtype=np.float64)
    translation_optical = np.asarray(detection.tvec, dtype=np.float64)
    transform_optical_marker = np.eye(4)
    transform_optical_marker[:3, :3] = rotation_optical
    transform_optical_marker[:3, 3] = translation_optical

    base_body = np.asarray(
        metadata['tf_base_camera']['matrix'], dtype=np.float64)
    transform_base_marker = base_body @ optical @ transform_optical_marker
    rotation_base = transform_base_marker[:3, :3]
    translation_base = transform_base_marker[:3, 3]
    return {
        'marker_id': detection.marker_id,
        'reprojection_error_px': detection.reprojection_error_px,
        'rvec': list(detection.rvec),
        'tvec': list(detection.tvec),
        'camera_translation_m': translation_optical.tolist(),
        'camera_distance_m': float(np.linalg.norm(translation_optical)),
        'camera_tilt_deg': marker_normal_tilt_deg(rotation_optical),
        'base_translation_m': translation_base.tolist(),
        'base_quaternion_xyzw': matrix_to_quaternion(rotation_base),
        'marker_side_px': detection.marker_side_px,
    }, image, camera, distortion


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--frames-dir', required=True)
    parser.add_argument(
        '--optical-config',
        default=os.path.join(
            os.path.dirname(__file__), '..', '..', 'ros2_ws', 'src',
            'armpi_fiducial_localizer', 'config',
            'camera_optical_frame.yaml'))
    parser.add_argument('--marker-dictionary', default='DICT_APRILTAG_36h11')
    parser.add_argument('--marker-size', type=float, default=0.06)
    parser.add_argument('--marker-id', type=int, default=0)
    parser.add_argument('--output', help='summary JSON path')
    args = parser.parse_args()

    optical, _ = load_optical_transform(args.optical_config)
    detector = FiducialDetector(
        dictionary=args.marker_dictionary,
        marker_size_m=args.marker_size,
        max_reprojection_error_px=3.0)

    results = []
    for image_path in sorted(glob.glob(
            os.path.join(args.frames_dir, 'frame_*.ppm'))):
        metadata_path = image_path[:-4] + '.json'
        with open(metadata_path, encoding='utf-8') as handle:
            metadata = json.load(handle)
        result, image, camera, distortion = analyze_frame(
            image_path, metadata, detector, optical, args.marker_id)
        name = os.path.basename(image_path)
        if result is None:
            print(f'{name}: no detection')
            continue
        if result['marker_id'] != args.marker_id:
            print(f"{name}: wrong marker id {result['marker_id']}")
            continue
        result['frame'] = name
        results.append(result)
        print(
            f"{name}: id={result['marker_id']} "
            f"dist={result['camera_distance_m']:.3f}m "
            f"reproj={result['reprojection_error_px']:.2f}px "
            f"tilt={result['camera_tilt_deg']:.1f}deg "
            f"base=({result['base_translation_m'][0]:.3f}, "
            f"{result['base_translation_m'][1]:.3f}, "
            f"{result['base_translation_m'][2]:.3f})")
        overlay = image.copy()
        cv2.drawFrameAxes(
            overlay, camera, distortion,
            np.asarray(result['rvec']), np.asarray(result['tvec']),
            0.03)
        cv2.imwrite(image_path[:-4] + '_overlay.png', overlay)

    if len(results) < 2:
        print('not enough detections for a stability check')
        return 1

    translations = np.asarray(
        [result['base_translation_m'] for result in results])
    rotations = [
        quaternion_to_matrix(result['base_quaternion_xyzw'])
        for result in results]
    reference = rotations[0]
    rotation_errors = [
        rotation_angle_deg(reference, rotation) for rotation in rotations]
    summary = {
        'frames': len(results),
        'marker_id': args.marker_id,
        'marker_size_m': args.marker_size,
        'base_translation_mean_m': translations.mean(axis=0).tolist(),
        'base_translation_std_mm': (
            translations.std(axis=0) * 1000.0).tolist(),
        'base_translation_range_mm': (
            (translations.max(axis=0) - translations.min(axis=0))
            * 1000.0).tolist(),
        'base_rotation_spread_deg': max(rotation_errors),
        'camera_distance_mean_m': float(np.mean(
            [result['camera_distance_m'] for result in results])),
        'camera_tilt_mean_deg': float(np.mean(
            [result['camera_tilt_deg'] for result in results])),
        'max_reprojection_error_px': max(
            result['reprojection_error_px'] for result in results),
        'detections': results,
    }
    print(
        f"base translation mean={summary['base_translation_mean_m']} m")
    print(
        f"translation std={summary['base_translation_std_mm']} mm "
        f"range={summary['base_translation_range_mm']} mm")
    print(
        f"rotation spread={summary['base_rotation_spread_deg']:.3f} deg")
    print(
        f"camera distance mean={summary['camera_distance_mean_m']:.3f} m "
        f"tilt mean={summary['camera_tilt_mean_deg']:.2f} deg")

    if args.output:
        with open(args.output, 'w', encoding='utf-8') as handle:
            json.dump(summary, handle, indent=2)
        print(f'wrote {args.output}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
