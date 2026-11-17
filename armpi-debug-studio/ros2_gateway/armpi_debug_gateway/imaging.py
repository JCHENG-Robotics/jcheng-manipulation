"""Image decoding helpers for the gateway (no ROS imports).

The vendor container ships OpenCV and numpy but not `cv_bridge`, so the gateway
must be able to decode a `sensor_msgs/Image` directly. `cv_bridge` is still used
when installed; this module is only the fallback.
"""

from __future__ import annotations

from typing import Any

try:
    import cv2
    import numpy as np
except ImportError:  # pragma: no cover - depends on the robot image
    cv2 = None
    np = None


def image_to_bgr(message: Any) -> "np.ndarray":
    """Decode a sensor_msgs/Image to BGR without cv_bridge.

    Supports the encodings the Aurora 930 RGB stream and common mono cameras
    use. Raises ValueError for anything else so the caller can log and skip the
    frame instead of guessing.
    """
    if cv2 is None or np is None:
        raise RuntimeError("opencv/numpy are not available")

    height = int(message.height)
    width = int(message.width)
    encoding = str(message.encoding).lower()
    if encoding in {"bgr8", "rgb8"}:
        channels = 3
    elif encoding == "mono8":
        channels = 1
    else:
        raise ValueError(f"unsupported image encoding without cv_bridge: {message.encoding}")

    row_bytes = width * channels
    step = int(message.step) or row_bytes
    raw = np.frombuffer(message.data, dtype=np.uint8)
    if raw.size < step * height:
        raise ValueError(
            f"image buffer too small: {raw.size} < step*height={step * height}"
        )
    rows = raw[: step * height].reshape(height, step)[:, :row_bytes]
    pixels = rows.reshape(height, width, channels)

    if encoding == "mono8":
        return cv2.cvtColor(pixels, cv2.COLOR_GRAY2BGR)
    if encoding == "rgb8":
        return cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    return pixels.copy()
