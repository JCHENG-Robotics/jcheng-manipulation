"""
Synthetic marker rendering for hardware-free detector tests.

The renderer projects a generated marker image onto the image plane at a known
camera-frame pose. The default ``rvec`` is 180 degrees about x so the printed
marker face points toward the camera; a marker rendered with ``rvec`` near zero
would show its mirrored back face and would not be detected reliably.

The renderer is for accuracy checks only. It does not model lighting, lens
distortion, motion blur or sensor noise; ``noise_sigma`` adds simple additive
Gaussian noise.
"""

import cv2
import numpy as np

from .detector import DEFAULT_DICTIONARY, draw_marker_image


def default_camera_matrix(
    width=640,
    height=480,
    fx=700.0,
    fy=700.0,
):
    """Return a pinhole camera matrix centred on the image."""
    return np.array(
        [
            [float(fx), 0.0, width / 2.0],
            [0.0, float(fy), height / 2.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )


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


def render_scene(
    marker_id=0,
    dictionary=DEFAULT_DICTIONARY,
    marker_size_m=0.06,
    camera_matrix=None,
    rvec=(3.141592653589793, 0.0, 0.0),
    tvec=(0.0, 0.0, 0.4),
    image_size=(640, 480),
    side_pixels=300,
    border_bits=1,
    background=255,
    noise_sigma=0.0,
    seed=0,
):
    """
    Render one marker at a known pose and return ``(image, corners)``.

    ``corners`` are the projected marker corners in pixels. They are useful
    for a detection overlay and for a rendering sanity check.
    """
    width, height = image_size
    camera = (
        default_camera_matrix(width, height)
        if camera_matrix is None
        else np.asarray(camera_matrix, dtype=np.float64)
    )
    marker = draw_marker_image(
        dictionary, marker_id, side_pixels, border_bits
    )
    object_points = marker_object_points(marker_size_m)
    projected, _ = cv2.projectPoints(
        object_points,
        np.asarray(rvec, dtype=np.float64),
        np.asarray(tvec, dtype=np.float64),
        camera,
        np.zeros(5, dtype=np.float64),
    )
    source = np.float32(
        [
            [0.0, 0.0],
            [side_pixels, 0.0],
            [side_pixels, side_pixels],
            [0.0, side_pixels],
        ]
    )
    destination = projected.reshape(4, 2).astype(np.float32)
    transform = cv2.getPerspectiveTransform(source, destination)
    warped = cv2.warpPerspective(
        marker,
        transform,
        (width, height),
        flags=cv2.INTER_LINEAR,
        borderValue=background,
    )
    image = np.full((height, width), background, dtype=np.uint8)
    mask = warped < background
    image[mask] = warped[mask]
    if noise_sigma > 0.0:
        generator = np.random.default_rng(seed)
        noise = generator.normal(0.0, noise_sigma, image.shape)
        image = np.clip(
            image.astype(np.float64) + noise, 0, 255
        ).astype(np.uint8)
    return image, projected.reshape(4, 2)
