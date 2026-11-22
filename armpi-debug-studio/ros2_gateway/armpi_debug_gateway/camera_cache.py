"""Thread-safe camera frame cache for the gateway node.

Owns the JPEG snapshot, frame metadata and the FPS window. The node keeps the
ROS subscription and passes the optional ``cv_bridge`` adapter; this module has
no ROS imports and can be unit-tested with a plain message-like object.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from typing import Any, Callable

from .imaging import cv2, image_to_bgr


class CameraCache:
    """One decoded RGB stream and its most recent JPEG frame."""

    def __init__(
        self,
        *,
        topic: str,
        jpeg_quality: int,
        bridge: Any | None,
        now_iso: Callable[[], str],
        logger: Any | None = None,
    ) -> None:
        self._topic = topic
        self._jpeg_quality = max(30, min(95, int(jpeg_quality)))
        self._bridge = bridge
        self._now_iso = now_iso
        self._logger = logger
        self._lock = threading.Lock()
        self._jpeg: bytes | None = None
        self._at: float | None = None
        self._wall: str | None = None
        self._width: int | None = None
        self._height: int | None = None
        self._encoding: str | None = None
        self._times: deque[float] = deque(maxlen=30)
        self._count = 0

    def on_image(self, message: Any) -> None:
        """Decode one ROS Image message and replace the cached JPEG."""
        if cv2 is None:
            return
        try:
            if self._bridge is not None:
                frame = self._bridge.imgmsg_to_cv2(
                    message, desired_encoding="bgr8"
                )
            else:
                frame = image_to_bgr(message)
            ok, encoded = cv2.imencode(
                ".jpg",
                frame,
                [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_quality],
            )
        except Exception as exc:  # noqa: BLE001 - a bad frame must not kill the node
            if self._logger is not None:
                self._logger.warning(
                    f"camera frame conversion failed: {exc}"
                )
            return
        if not ok:
            return
        now = time.monotonic()
        with self._lock:
            self._jpeg = encoded.tobytes()
            self._at = now
            self._wall = self._now_iso()
            self._width = int(message.width)
            self._height = int(message.height)
            self._encoding = str(message.encoding)
            self._times.append(now)
            self._count += 1

    def snapshot(self) -> bytes | None:
        """Return the latest JPEG, if any."""
        with self._lock:
            return self._jpeg

    def status_payload(self) -> dict[str, Any]:
        """Return the same shape as the read-only gateway camera status."""
        with self._lock:
            has_frame = self._jpeg is not None
            at = self._at
            wall = self._wall
            width = self._width
            height = self._height
            encoding = self._encoding
            fps = self._fps_locked()
            count = self._count
        age_ms = self._age_ms(at)
        available = has_frame and age_ms is not None and age_ms < 3000
        if available:
            validity = "valid"
            truth = "measured"
        elif has_frame:
            validity = "stale"
            truth = "measured"
        else:
            validity = "unavailable"
            truth = "unknown"
        notes = []
        if cv2 is None:
            notes.append("opencv is not available in this Python environment.")
        elif self._bridge is None:
            notes.append(
                "cv_bridge is not installed; frames were decoded with the "
                "numpy fallback."
            )
        return {
            "available": available,
            "truth": truth,
            "validity": validity,
            "source": "armpi_debug_gateway",
            "topic": self._topic,
            "width": width,
            "height": height,
            "encoding": encoding,
            "fps": round(fps, 1),
            "last_frame_at": wall,
            "sample_age_ms": age_ms,
            "frame_count": count,
            "notes": notes,
        }

    def _fps_locked(self) -> float:
        if len(self._times) < 2:
            return 0.0
        span = self._times[-1] - self._times[0]
        if span <= 0:
            return 0.0
        return (len(self._times) - 1) / span

    @staticmethod
    def _age_ms(at: float | None) -> int | None:
        if at is None:
            return None
        return max(0, int((time.monotonic() - at) * 1000.0))
