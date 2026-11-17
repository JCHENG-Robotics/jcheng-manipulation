#!/usr/bin/env python3
"""Detect fiducials on real backgrounds and on semi-synthetic overlays.

The real frames come from ``tools/phase2/capture_still.py`` (no robot motion).
Two checks are produced:

1. A negative control: the unmodified real frames must produce zero
   detections of the configured dictionary.
2. A semi-synthetic accuracy check: an AprilTag with a known camera-frame pose
   is composited onto a real frame, then detected with the same detector and
   intrinsics. The corner projection is exact; the marker interior is warped
   with a planar homography, so lens distortion is not modelled inside the
   marker. This is a detector/pose-accuracy test, not a physical-target test.

Run on the laptop with the Jazzy workspace (OpenCV + numpy only):

    python3 tools/phase2/real_background_fiducial_check.py \
        --capture ~/jcheng/phase2/real_capture_20261117/capture.json \
        --output evidence/phase2/real_background_20261117/check.json \
        --contact-sheet evidence/phase2/real_background_20261117/contact_sheet.png
"""

import argparse
import json
import math
import os
import sys

import cv2
import numpy as np

REPO_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__))))
SRC_ROOT = os.path.join(REPO_ROOT, 'ros2_ws', 'src')
PACKAGE_ROOT = os.path.join(SRC_ROOT, 'armpi_fiducial_localizer')
for path in (SRC_ROOT, PACKAGE_ROOT):
    if path not in sys.path:
        sys.path.insert(0, path)

from armpi_fiducial_localizer.detector import (  # noqa: E402
    DEFAULT_DICTIONARY,
    draw_marker_image,
    FiducialDetector,
)


# The detector module owns the object-point convention; reproduce the four
# corners here so the renderer and detector cannot drift silently.
def marker_object_points(marker_size_m):
    """Return the four marker corners in the marker frame (x right, y up)."""
    half = float(marker_size_m) / 2.0
    return np.array(
        [
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ],
        dtype=np.float64,
    )


def rotation_angle_deg(rotation_a, rotation_b):
    """Return the geodesic angle between two rotation matrices."""
    relative = rotation_a.T @ rotation_b
    cosine = (np.trace(relative) - 1.0) / 2.0
    return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


def pose_from_rvec_tvec(rvec, tvec):
    """Return a 4x4 transform from an OpenCV rvec/tvec pair."""
    rotation, _ = cv2.Rodrigues(np.asarray(rvec, dtype=np.float64))
    transform = np.eye(4)
    transform[:3, :3] = rotation
    transform[:3, 3] = np.asarray(tvec, dtype=np.float64).reshape(3)
    return transform


def compose_tilt(roll_deg, pitch_deg, yaw_deg):
    """Compose a front-facing marker rotation with in-plane and tilt angles."""
    # The marker face points toward the camera: rotate 180 deg about x, then
    # apply in-plane yaw about the marker normal, then small tilts.
    rx_pi = cv2.Rodrigues(np.array([math.pi, 0.0, 0.0]))[0]
    rz = cv2.Rodrigues(np.array([0.0, 0.0, math.radians(yaw_deg)]))[0]
    rx = cv2.Rodrigues(np.array([math.radians(roll_deg), 0.0, 0.0]))[0]
    ry = cv2.Rodrigues(np.array([0.0, math.radians(pitch_deg), 0.0]))[0]
    return rx_pi @ rz @ rx @ ry


def tvec_from_pixel(center_px, depth_m, camera_matrix):
    """Return the camera-frame point that projects to a pixel at a depth."""
    cx, cy = center_px
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    ox = camera_matrix[0, 2]
    oy = camera_matrix[1, 2]
    return np.array(
        [(cx - ox) * depth_m / fx, (cy - oy) * depth_m / fy, depth_m],
        dtype=np.float64,
    )


def overlay_marker(background, rotation, tvec, camera_matrix, dist_coeffs,
                   marker_id, dictionary, marker_size_m, side_pixels=500,
                   quiet_zone_ratio=0.2):
    """Composite a marker at a known camera pose and return image + corners.

    A white quiet zone is added around the marker, as on a printed sticker.
    The ``marker_size_m`` applies to the black-border marker square, not to
    the quiet zone.
    """
    height, width = background.shape[:2]
    marker = draw_marker_image(dictionary, marker_id, side_pixels)
    pad = int(round(side_pixels * quiet_zone_ratio))
    marker = cv2.copyMakeBorder(
        marker, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    object_points = marker_object_points(marker_size_m)
    projected, _ = cv2.projectPoints(
        object_points,
        cv2.Rodrigues(rotation)[0],
        tvec,
        camera_matrix,
        dist_coeffs,
    )
    destination = projected.reshape(4, 2).astype(np.float32)
    source = np.float32(
        [
            [pad, pad],
            [pad + side_pixels, pad],
            [pad + side_pixels, pad + side_pixels],
            [pad, pad + side_pixels],
        ]
    )
    transform = cv2.getPerspectiveTransform(source, destination)
    warped = cv2.warpPerspective(
        marker, transform, (width, height),
        flags=cv2.INTER_LINEAR, borderValue=0)
    if warped.ndim == 2:
        warped = cv2.cvtColor(warped, cv2.COLOR_GRAY2BGR)
    mask = cv2.warpPerspective(
        np.full(marker.shape, 255, dtype=np.uint8),
        transform, (width, height),
        flags=cv2.INTER_NEAREST, borderValue=0)
    alpha = (mask.astype(np.float32) / 255.0)[..., None]
    if background.ndim == 2:
        background = cv2.cvtColor(background, cv2.COLOR_GRAY2BGR)
    composited = (
        background.astype(np.float32) * (1.0 - alpha)
        + warped.astype(np.float32) * alpha
    )
    return np.clip(composited, 0, 255).astype(np.uint8), projected.reshape(4, 2)


def load_capture(path):
    """Load a capture.json and the referenced RGB frames."""
    with open(path, 'r', encoding='utf-8') as handle:
        capture = json.load(handle)
    base = os.path.dirname(os.path.abspath(path))
    frames = []
    for frame in capture['frames']:
        image_path = os.path.join(base, frame['path'])
        image = cv2.imread(image_path)
        if image is None:
            raise RuntimeError('cannot read {}'.format(image_path))
        frames.append((frame, image))
    camera_info = capture['frames'][0]['camera_info']
    camera_matrix = np.asarray(
        camera_info['k'], dtype=np.float64).reshape(3, 3)
    dist_coeffs = np.asarray(camera_info['d'], dtype=np.float64)
    return capture, frames, camera_matrix, dist_coeffs


def negative_control(frames, camera_matrix, dist_coeffs, detector):
    """Run the detector on unmodified real frames."""
    results = []
    for frame, image in frames:
        detections = detector.detect(image, camera_matrix, dist_coeffs)
        results.append(
            {
                'index': frame['index'],
                'path': frame['path'],
                'detections': len(detections),
                'marker_ids': [int(d.marker_id) for d in detections],
                'reprojection_errors_px': [
                    float(d.reprojection_error_px) for d in detections
                ],
            }
        )
    return results


def make_cases(camera_matrix):
    """Return the semi-synthetic pose cases."""
    return [
        {
            'name': 'centre_z025',
            'center_px': (320, 200),
            'depth_m': 0.25,
            'roll_deg': 0.0,
            'pitch_deg': 0.0,
            'yaw_deg': 0.0,
        },
        {
            'name': 'left_z028_yaw30',
            'center_px': (200, 180),
            'depth_m': 0.28,
            'roll_deg': 0.0,
            'pitch_deg': 0.0,
            'yaw_deg': 30.0,
        },
        {
            'name': 'right_z023_yaw-20',
            'center_px': (450, 220),
            'depth_m': 0.23,
            'roll_deg': 0.0,
            'pitch_deg': 0.0,
            'yaw_deg': -20.0,
        },
        {
            'name': 'low_z030_tiltx10',
            'center_px': (320, 260),
            'depth_m': 0.30,
            'roll_deg': 10.0,
            'pitch_deg': 0.0,
            'yaw_deg': 0.0,
        },
        {
            'name': 'upperleft_z026_tilty-12_yaw15',
            'center_px': (260, 160),
            'depth_m': 0.26,
            'roll_deg': 0.0,
            'pitch_deg': -12.0,
            'yaw_deg': 15.0,
        },
    ]


def run_accuracy_cases(frames, camera_matrix, dist_coeffs, detector,
                       marker_size_m, dictionary, marker_id):
    """Composite markers at known poses, detect them and compare."""
    cases = make_cases(camera_matrix)
    results = []
    overlays = []
    backgrounds = [frames[0], frames[3], frames[6]]
    for background_index, (frame, image) in enumerate(backgrounds):
        for case in cases:
            rotation = compose_tilt(
                case['roll_deg'], case['pitch_deg'], case['yaw_deg'])
            tvec = tvec_from_pixel(
                case['center_px'], case['depth_m'], camera_matrix)
            overlay, projected = overlay_marker(
                image, rotation, tvec, camera_matrix, dist_coeffs,
                marker_id, dictionary, marker_size_m)
            detections = detector.detect(
                overlay, camera_matrix, dist_coeffs)
            candidates = [
                detection for detection in detections
                if detection.marker_id == marker_id
            ]
            entry = {
                'case': 'bg{}_{}'.format(background_index, case['name']),
                'background_index': int(frame['index']),
                'ground_truth': {
                    'rotation': rotation.tolist(),
                    'translation_m': tvec.tolist(),
                    'projected_corners_px': projected.tolist(),
                    'center_px': list(case['center_px']),
                    'depth_m': case['depth_m'],
                    'roll_deg': case['roll_deg'],
                    'pitch_deg': case['pitch_deg'],
                    'yaw_deg': case['yaw_deg'],
                },
                'detected': None,
                'error': None,
            }
            if candidates:
                detection = min(
                    candidates,
                    key=lambda item: item.reprojection_error_px)
                detected_transform = pose_from_rvec_tvec(
                    detection.rvec, detection.tvec)
                translation_error_m = float(np.linalg.norm(
                    detected_transform[:3, 3] - tvec))
                rotation_error_deg = float(rotation_angle_deg(
                    rotation, detected_transform[:3, :3]))
                entry['detected'] = detection.as_dict()
                entry['error'] = {
                    'translation_m': translation_error_m,
                    'translation_mm': translation_error_m * 1000.0,
                    'rotation_deg': rotation_error_deg,
                }
            else:
                entry['error'] = {
                    'translation_m': None,
                    'translation_mm': None,
                    'rotation_deg': None,
                }
            results.append(entry)
            overlays.append((entry, overlay))
    return results, overlays


def draw_overlay(entry, image):
    """Draw the ground-truth and detected marker outlines on one image."""
    output = image.copy()
    corners = np.asarray(
        entry['ground_truth']['projected_corners_px'], dtype=np.int32)
    cv2.polylines(output, [corners.reshape(-1, 1, 2)], True, (0, 255, 0), 2)
    if entry['detected'] is not None:
        detected = np.asarray(
            entry['detected']['corners_px'], dtype=np.float32).reshape(-1, 2)
        cv2.polylines(
            output, [detected.astype(np.int32).reshape(-1, 1, 2)],
            True, (0, 0, 255), 2)
        error = entry['error']
        text = '{} dt={:.2f}mm dr={:.2f}deg'.format(
            entry['case'], error['translation_mm'], error['rotation_deg'])
    else:
        text = '{} NOT DETECTED'.format(entry['case'])
    cv2.putText(
        output, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
        (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(
        output, text, (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55,
        (255, 255, 255), 1, cv2.LINE_AA)
    return output


def make_contact_sheet(overlays, path, columns=3, scale=0.5):
    """Write a labelled contact sheet of the semi-synthetic cases."""
    if not overlays:
        return
    tiles = []
    for entry, image in overlays:
        tile = draw_overlay(entry, image)
        tile = cv2.resize(
            tile, None, fx=scale, fy=scale, interpolation=cv2.INTER_AREA)
        tiles.append(tile)
    tile_height, tile_width = tiles[0].shape[:2]
    rows = int(math.ceil(len(tiles) / float(columns)))
    sheet = np.full(
        (rows * tile_height, columns * tile_width, 3), 24, dtype=np.uint8)
    for index, tile in enumerate(tiles):
        row = index // columns
        column = index % columns
        sheet[
            row * tile_height:(row + 1) * tile_height,
            column * tile_width:(column + 1) * tile_width,
        ] = tile
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    cv2.imwrite(path, sheet)


def main():
    """Run the negative control and the semi-synthetic accuracy check."""
    parser = argparse.ArgumentParser()
    parser.add_argument('--capture', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--contact-sheet', default=None)
    parser.add_argument('--marker-dictionary', default=DEFAULT_DICTIONARY)
    parser.add_argument('--marker-id', type=int, default=0)
    parser.add_argument('--marker-size-m', type=float, default=0.06)
    parser.add_argument('--max-translation-mm', type=float, default=5.0)
    parser.add_argument('--max-rotation-deg', type=float, default=2.0)
    args = parser.parse_args()

    capture, frames, camera_matrix, dist_coeffs = load_capture(
        args.capture)
    detector = FiducialDetector(
        dictionary=args.marker_dictionary,
        marker_size_m=args.marker_size_m,
    )
    negative = negative_control(
        frames, camera_matrix, dist_coeffs, detector)
    accuracy, overlays = run_accuracy_cases(
        frames, camera_matrix, dist_coeffs, detector,
        args.marker_size_m, args.marker_dictionary, args.marker_id)

    detected = [
        entry for entry in accuracy if entry['detected'] is not None
    ]
    translation_errors = [
        entry['error']['translation_mm'] for entry in detected
    ]
    rotation_errors = [
        entry['error']['rotation_deg'] for entry in detected
    ]
    summary = {
        'cases': len(accuracy),
        'detected': len(detected),
        'translation_error_mm': {
            'mean': float(np.mean(translation_errors)) if detected else None,
            'max': float(np.max(translation_errors)) if detected else None,
        },
        'rotation_error_deg': {
            'mean': float(np.mean(rotation_errors)) if detected else None,
            'max': float(np.max(rotation_errors)) if detected else None,
        },
        'negative_control_frames': len(negative),
        'negative_control_detections': int(sum(
            entry['detections'] for entry in negative)),
        'success': bool(
            len(detected) == len(accuracy)
            and translation_errors
            and max(translation_errors) <= args.max_translation_mm
            and rotation_errors
            and max(rotation_errors) <= args.max_rotation_deg
            and sum(entry['detections'] for entry in negative) == 0
        ),
        'criteria': {
            'all_cases_detected': True,
            'max_translation_mm': args.max_translation_mm,
            'max_rotation_deg': args.max_rotation_deg,
            'negative_control_detections': 0,
        },
    }
    report = {
        'schema_version': 1,
        'mode': 'real_background_fiducial_check',
        'capture': {
            'path': os.path.abspath(args.capture),
            'frames': len(frames),
            'camera_info': capture['frames'][0]['camera_info'],
        },
        'marker': {
            'dictionary': args.marker_dictionary,
            'marker_id': args.marker_id,
            'marker_size_m': args.marker_size_m,
        },
        'negative_control': negative,
        'semi_synthetic': accuracy,
        'summary': summary,
        'limitations': [
            'The marker interior is warped with a planar homography; lens '
            'distortion is applied to the corners but not inside the marker.',
            'The background is a real frame captured at the current robot '
            'pose; the marker is not physically present.',
            'This validates the detector and pose math, not the camera '
            'extrinsic or a real grasp.',
        ],
    }
    os.makedirs(os.path.dirname(os.path.abspath(args.output)), exist_ok=True)
    with open(args.output, 'w', encoding='utf-8') as handle:
        json.dump(report, handle, indent=2)
    if args.contact_sheet:
        make_contact_sheet(overlays, args.contact_sheet)
    print(
        'NEGATIVE control: {} frames, {} detections'.format(
            summary['negative_control_frames'],
            summary['negative_control_detections']))
    print(
        'SEMI-SYNTHETIC: {}/{} detected, dt mean={} max={} mm, '
        'dr mean={} max={} deg'.format(
            summary['detected'], summary['cases'],
            None if not detected else round(
                summary['translation_error_mm']['mean'], 4),
            None if not detected else round(
                summary['translation_error_mm']['max'], 4),
            None if not detected else round(
                summary['rotation_error_deg']['mean'], 4),
            None if not detected else round(
                summary['rotation_error_deg']['max'], 4)))
    print('RESULT {}'.format('PASS' if summary['success'] else 'FAIL'))
    print('WROTE', args.output)
    if args.contact_sheet:
        print('WROTE', args.contact_sheet)
    return 0 if summary['success'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
