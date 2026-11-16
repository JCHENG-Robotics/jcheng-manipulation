"""
Synthetic fiducial accuracy check.

Renders a marker at known camera-frame poses, runs the detector, and reports
translation, rotation and reprojection errors. The output JSON and overlay
images are intended as hardware-free evidence for the detection pipeline.

This is a synthetic check. It does not measure a real camera, lens, lighting
or mounting transform.
"""

import argparse
from datetime import datetime, timezone
import json
import math
import os
import sys

import cv2
import numpy as np

from .detector import DEFAULT_DICTIONARY, FiducialDetector
from .synthetic import default_camera_matrix, render_scene

DEFAULT_POSES = (
    ((math.pi, 0.0, 0.0), (0.00, 0.00, 0.30)),
    ((math.pi + 0.15, -0.05, 0.10), (0.04, -0.03, 0.35)),
    ((math.pi - 0.15, 0.10, -0.10), (-0.05, 0.03, 0.40)),
    ((math.pi + 0.25, 0.15, 0.20), (0.06, 0.04, 0.45)),
    ((math.pi - 0.25, -0.15, -0.20), (-0.04, -0.05, 0.45)),
)


def rotation_error_deg(rvec_a, rvec_b):
    """Return the geodesic angle between two Rodrigues vectors."""
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
    sine = float(np.linalg.norm(skew))
    cosine = float(0.5 * (np.trace(relative) - 1.0))
    return math.degrees(math.atan2(sine, cosine))


def _parse_args(argv):
    parser = argparse.ArgumentParser(
        description='Check AprilTag/ArUco accuracy on synthetic images.',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument('--output-dir', default='.')
    parser.add_argument(
        '--marker-dictionary', default=DEFAULT_DICTIONARY
    )
    parser.add_argument('--marker-id', type=int, default=0)
    parser.add_argument('--marker-size', type=float, default=0.06)
    parser.add_argument('--side-pixels', type=int, default=300)
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--fx', type=float, default=700.0)
    parser.add_argument('--fy', type=float, default=700.0)
    parser.add_argument('--noise-sigma', type=float, default=0.0)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument(
        '--max-reprojection-error',
        type=float,
        default=3.0,
    )
    parser.add_argument(
        '--translation-threshold-mm',
        type=float,
        default=3.0,
    )
    parser.add_argument(
        '--rotation-threshold-deg',
        type=float,
        default=2.5,
    )
    parser.add_argument('--json-out', default=None)
    parser.add_argument('--no-images', action='store_true')
    return parser.parse_args(argv)


def _overlay(image, detection, index, translation_error_mm,
             angle_error_deg):
    overlay = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    points = np.asarray(
        detection.corners_px, dtype=np.int32
    ).reshape(-1, 1, 2)
    cv2.polylines(overlay, [points], True, (0, 0, 255), 2)
    centre = tuple(
        int(value) for value in points.reshape(-1, 2).mean(axis=0)
    )
    cv2.putText(
        overlay,
        str(detection.marker_id),
        centre,
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 255),
        2,
        cv2.LINE_AA,
    )
    text = (
        'pose {}  id {}  dt {:.2f} mm  dr {:.2f} deg  '
        'reproj {:.2f} px'.format(
            index,
            detection.marker_id,
            translation_error_mm,
            angle_error_deg,
            detection.reprojection_error_px,
        )
    )
    cv2.putText(
        overlay,
        text,
        (12, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (0, 0, 255),
        1,
        cv2.LINE_AA,
    )
    return overlay


def _contact_sheet(overlays):
    tiles = [
        cv2.resize(overlay, None, fx=0.5, fy=0.5)
        for overlay in overlays
    ]
    blank = np.full_like(tiles[0], 255)
    while len(tiles) % 2:
        tiles.append(blank)
    rows = [
        np.hstack((tiles[index], tiles[index + 1]))
        for index in range(0, len(tiles), 2)
    ]
    return np.vstack(rows)


def main(argv=None):
    """Run the synthetic accuracy check. Returns a process exit code."""
    args = _parse_args(argv)
    output_dir = os.path.abspath(args.output_dir)
    os.makedirs(output_dir, exist_ok=True)
    camera_matrix = default_camera_matrix(
        args.width, args.height, args.fx, args.fy
    )
    detector = FiducialDetector(
        dictionary=args.marker_dictionary,
        marker_size_m=args.marker_size,
        max_reprojection_error_px=args.max_reprojection_error,
    )
    records = []
    overlays = []
    for index, (rvec, tvec) in enumerate(DEFAULT_POSES):
        image, projected = render_scene(
            marker_id=args.marker_id,
            dictionary=args.marker_dictionary,
            marker_size_m=args.marker_size,
            camera_matrix=camera_matrix,
            rvec=rvec,
            tvec=tvec,
            image_size=(args.width, args.height),
            side_pixels=args.side_pixels,
            noise_sigma=args.noise_sigma,
            seed=args.seed + index,
        )
        detections = detector.detect(image, camera_matrix)
        matching = [
            detection
            for detection in detections
            if detection.marker_id == args.marker_id
        ]
        record = {
            'index': index,
            'ground_truth_rvec': list(rvec),
            'ground_truth_tvec': list(tvec),
            'projected_corners_px': [
                list(point) for point in projected
            ],
            'detected': bool(matching),
        }
        if matching:
            detection = matching[0]
            translation_error = (
                np.linalg.norm(
                    np.asarray(detection.tvec) - np.asarray(tvec)
                )
                * 1000.0
            )
            angle_error = rotation_error_deg(rvec, detection.rvec)
            record.update(
                {
                    'detection': detection.as_dict(),
                    'translation_error_mm': float(translation_error),
                    'rotation_error_deg': float(angle_error),
                }
            )
            overlays.append(
                _overlay(
                    image,
                    detection,
                    index,
                    translation_error,
                    angle_error,
                )
            )
            print(
                'POSE {} id={} dt={:.2f}mm dr={:.2f}deg '
                'reproj={:.2f}px'.format(
                    index,
                    detection.marker_id,
                    translation_error,
                    angle_error,
                    detection.reprojection_error_px,
                )
            )
        else:
            record.update(
                {
                    'detection': None,
                    'translation_error_mm': None,
                    'rotation_error_deg': None,
                }
            )
            overlays.append(cv2.cvtColor(image, cv2.COLOR_GRAY2BGR))
            print(
                'POSE {} id={} NOT DETECTED'.format(
                    index, args.marker_id
                )
            )
        records.append(record)

    detected = [record for record in records if record['detected']]
    translation_errors = [
        record['translation_error_mm'] for record in detected
    ]
    rotation_errors = [record['rotation_error_deg'] for record in detected]
    max_translation = max(translation_errors, default=None)
    max_rotation = max(rotation_errors, default=None)
    success = (
        len(detected) == len(records)
        and max_translation is not None
        and max_translation <= args.translation_threshold_mm
        and max_rotation is not None
        and max_rotation <= args.rotation_threshold_deg
    )
    summary = {
        'schema_version': 1,
        'generated_at': datetime.now(timezone.utc).isoformat(),
        'mode': 'synthetic',
        'dictionary': args.marker_dictionary,
        'marker_id': args.marker_id,
        'marker_size_m': args.marker_size,
        'camera_matrix': camera_matrix.tolist(),
        'image_size': [args.width, args.height],
        'noise_sigma': args.noise_sigma,
        'thresholds': {
            'translation_mm': args.translation_threshold_mm,
            'rotation_deg': args.rotation_threshold_deg,
            'max_reprojection_error_px': args.max_reprojection_error,
        },
        'poses_total': len(records),
        'poses_detected': len(detected),
        'max_translation_error_mm': max_translation,
        'max_rotation_error_deg': max_rotation,
        'mean_translation_error_mm': (
            float(np.mean(translation_errors))
            if translation_errors
            else None
        ),
        'mean_rotation_error_deg': (
            float(np.mean(rotation_errors)) if rotation_errors else None
        ),
        'poses': records,
        'success': success,
    }
    if not args.no_images:
        for index, overlay in enumerate(overlays):
            path = os.path.join(
                output_dir,
                'synthetic_pose_{:02d}_overlay.png'.format(index),
            )
            cv2.imwrite(path, overlay)
        if overlays:
            cv2.imwrite(
                os.path.join(
                    output_dir, 'synthetic_contact_sheet.png'
                ),
                _contact_sheet(overlays),
            )
    json_path = args.json_out or os.path.join(
        output_dir, 'synthetic_pose_accuracy.json'
    )
    with open(json_path, 'w', encoding='utf-8') as handle:
        json.dump(summary, handle, indent=2)
    print(
        'SYNTHETIC_FIDUCIAL poses={} detected={} max_dt={} '
        'max_dr={}'.format(
            summary['poses_total'],
            summary['poses_detected'],
            'n/a'
            if max_translation is None
            else '{:.2f}mm'.format(max_translation),
            'n/a' if max_rotation is None else '{:.2f}deg'.format(
                max_rotation
            ),
        )
    )
    print('RESULT {}'.format('PASS' if success else 'FAIL'))
    return 0 if success else 1


if __name__ == '__main__':
    sys.exit(main())
