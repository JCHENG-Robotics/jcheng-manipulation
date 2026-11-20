"""Aux-camera JPEG for the chassis-mounted Intel RealSense D435i.

Two sources are supported:

- :class:`UvcCamera` opens ``/dev/video4`` directly (legacy single-process
  mode). It works only when no other process owns the device.
- :class:`RosImageCamera` subscribes to ``/ext_cam/color/image_raw`` published
  by the robot-side ``ext_cam_node``. That node is the single UVC owner, so the
  studio preview and ROS consumers share one stream; this is the deployed mode
  once ``ext_cam_node`` runs as ``armpi-ext-cam.service``.

Depth and IR need librealsense and are not touched here. The camera is
optional: a missing device or topic only makes the aux endpoints report
``available: false``; it never stops the ROS gateway.
"""

from __future__ import annotations

import threading
import time
from collections import deque
from datetime import datetime, timezone
from typing import Any, Callable

FRAME_AVAILABLE_AGE_MS = 3000
METADATA_NOTE = (
    "UVC color stream only (RealSense D435i); depth and IR need librealsense."
)
ROS_NOTE = (
    "ROS colour topic from ext_cam_node (single UVC owner); depth and IR need "
    "librealsense."
)


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class UvcCamera:
    """Capture the latest BGR frame of a V4L2 device as JPEG.

    ``capture_factory`` and ``encoder`` are injectable so unit tests can run
    without OpenCV or a camera device. ``poll_once()`` performs one blocking
    read and is what the background thread calls in a loop.
    """

    def __init__(
        self,
        device: str = "/dev/video4",
        *,
        width: int = 640,
        height: int = 480,
        fps: float = 10.0,
        quality: int = 80,
        capture_factory: Callable[[], Any] | None = None,
        encoder: Callable[[Any], bytes | None] | None = None,
        log: Callable[[str], None] | None = None,
    ) -> None:
        self.device = device
        self.width = max(1, int(width))
        self.height = max(1, int(height))
        self.fps = max(1.0, float(fps))
        self.quality = max(30, min(95, int(quality)))
        self._capture_factory = capture_factory
        self._encoder = encoder
        self._log = log or (lambda _message: None)

        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._capture: Any | None = None

        self._jpeg: bytes | None = None
        self._frame_at: float | None = None
        self._frame_wall: str | None = None
        self._frame_width: int | None = None
        self._frame_height: int | None = None
        self._frame_times: deque[float] = deque(maxlen=30)
        self._frame_count = 0
        self._last_error: str | None = None

    # ------------------------------------------------------------------ life
    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(
            target=self._run, name="aux-camera", daemon=True
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        thread = self._thread
        if thread is not None:
            thread.join(timeout=2.0)
        self._thread = None
        self._release_capture()

    # ------------------------------------------------------------------ poll
    def poll_once(self) -> bool:
        """Read and encode one frame. Returns False on any failure."""
        capture = self._capture or self._open_capture()
        if capture is None:
            return False
        try:
            ok, frame = capture.read()
        except Exception as exc:  # noqa: BLE001 - a bad frame must not kill the loop
            self._fail(f"UVC read failed on {self.device}: {exc}")
            return False
        if not ok or frame is None:
            self._fail(f"UVC device {self.device} returned no frame")
            return False
        jpeg = self._encode_frame(frame)
        if jpeg is None:
            self._fail(f"JPEG encoding failed for {self.device}")
            return False

        now = time.monotonic()
        shape = getattr(frame, "shape", None)
        with self._lock:
            self._jpeg = jpeg
            self._frame_at = now
            self._frame_wall = utc_now_iso()
            if shape is not None and len(shape) >= 2:
                self._frame_height = int(shape[0])
                self._frame_width = int(shape[1])
            else:
                self._frame_width = self.width
                self._frame_height = self.height
            self._frame_times.append(now)
            self._frame_count += 1
            self._last_error = None
        return True

    # --------------------------------------------------------------- payload
    def status_payload(self) -> dict[str, Any]:
        with self._lock:
            jpeg = self._jpeg
            at = self._frame_at
            wall = self._frame_wall
            width = self._frame_width
            height = self._frame_height
            fps = self._fps_locked()
            count = self._frame_count
            error = self._last_error
        age_ms = self._age_ms(at)
        available = jpeg is not None and age_ms is not None and age_ms < FRAME_AVAILABLE_AGE_MS
        if available:
            validity = "valid"
        elif jpeg is not None:
            validity = "stale"
        else:
            validity = "unavailable"
        notes = [METADATA_NOTE]
        if not available:
            notes.append(error or f"no frame from {self.device} yet")
        return {
            "available": available,
            "truth": "measured" if jpeg is not None else "unknown",
            "validity": validity,
            "source": "armpi_debug_gateway_uvc",
            "topic": f"uvc:{self.device}",
            "device": self.device,
            "width": width,
            "height": height,
            "encoding": "yuyv",
            "fps": round(fps, 1),
            "last_frame_at": wall,
            "sample_age_ms": age_ms,
            "frame_count": count,
            "notes": notes,
        }

    def snapshot(self) -> bytes | None:
        with self._lock:
            return self._jpeg


    # --------------------------------------------------------------- internals
    def _run(self) -> None:
        interval = 1.0 / self.fps
        while not self._stop.is_set():
            ok = self.poll_once()
            self._stop.wait(interval if ok else 1.5)

    def _open_capture(self) -> Any | None:
        factory = self._capture_factory or self._default_capture_factory
        try:
            capture = factory()
        except Exception as exc:  # noqa: BLE001 - retried by the next poll
            self._fail(f"cannot open {self.device}: {exc}")
            return None
        if capture is None or not capture.isOpened():
            self._release(capture)
            self._fail(f"cannot open {self.device}")
            return None
        with self._lock:
            self._capture = capture
            self._last_error = None
        self._log(f"aux camera {self.device} opened ({self.width}x{self.height} @ {self.fps:g} fps)")
        return capture

    def _default_capture_factory(self) -> Any:
        import cv2  # local import: the gateway tests run without OpenCV

        capture = cv2.VideoCapture(self.device, cv2.CAP_V4L2)
        if not capture.isOpened():
            return capture
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return capture

    def _encode_frame(self, frame: Any) -> bytes | None:
        if self._encoder is not None:
            return self._encoder(frame)
        import cv2  # local import: see _default_capture_factory

        ok, encoded = cv2.imencode(
            ".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), self.quality]
        )
        if not ok:
            return None
        return encoded.tobytes()

    def _fail(self, message: str) -> None:
        with self._lock:
            self._last_error = message
        self._release_capture()

    def _release_capture(self) -> None:
        with self._lock:
            capture = self._capture
            self._capture = None
        self._release(capture)

    @staticmethod
    def _release(capture: Any | None) -> None:
        if capture is None:
            return
        try:
            capture.release()
        except Exception:  # noqa: BLE001 - releasing must never raise
            pass

    @staticmethod
    def _age_ms(at: float | None) -> int | None:
        if at is None:
            return None
        return max(0, int((time.monotonic() - at) * 1000.0))

    def _fps_locked(self) -> float:
        if len(self._frame_times) < 2:
            return 0.0
        span = self._frame_times[-1] - self._frame_times[0]
        if span <= 0:
            return 0.0
        return (len(self._frame_times) - 1) / span


class RosImageCamera(UvcCamera):
    """Serve the latest JPEG from a ROS ``sensor_msgs/Image`` topic.

    The off-board D435i has exactly one owner: ``ext_cam_node`` on the robot
    opens the UVC device and publishes the colour stream. The gateway
    subscribes instead of opening the device, so the studio preview and any ROS
    consumer share the same frames. Frames arrive through :meth:`offer`; there
    is no capture thread and ``poll_once`` stays false.
    """

    def __init__(
        self,
        topic: str,
        *,
        quality: int = 80,
        encoder: Callable[[Any], bytes | None] | None = None,
        log: Callable[[str], None] | None = None,
    ) -> None:
        super().__init__(
            device="-",
            width=1280,
            height=720,
            fps=30.0,
            quality=quality,
            encoder=encoder,
            log=log,
        )
        self.ros_topic = str(topic)

    def start(self) -> None:
        """Nothing to start: the gateway node owns the subscription."""

    def stop(self) -> None:
        """Nothing to stop."""

    def poll_once(self) -> bool:
        """Frames arrive by callback; there is nothing to poll."""
        return False

    def offer(self, message: Any) -> bool:
        """Encode one ``sensor_msgs/Image`` and make it the latest JPEG."""
        import numpy as np

        encoding = str(getattr(message, "encoding", ""))
        channels = {"bgr8": 3, "rgb8": 3, "mono8": 1}.get(encoding)
        height = int(getattr(message, "height", 0))
        width = int(getattr(message, "width", 0))
        if channels is None or height <= 0 or width <= 0:
            self._fail(f"unsupported image {width}x{height} {encoding or '?'}")
            return False
        try:
            frame = np.frombuffer(message.data, dtype=np.uint8)
        except TypeError:
            frame = np.asarray(message.data, dtype=np.uint8)
        if frame.size != width * height * channels:
            self._fail(
                f"image size {frame.size} does not match {width}x{height}x"
                f"{channels} ({encoding})"
            )
            return False
        frame = frame.reshape(height, width, channels)
        if encoding == "rgb8":
            frame = frame[:, :, ::-1]
        jpeg = self._encode_frame(np.ascontiguousarray(frame))
        if jpeg is None:
            self._fail("JPEG encoding failed for the ROS image")
            return False

        now = time.monotonic()
        with self._lock:
            self._jpeg = jpeg
            self._frame_at = now
            self._frame_wall = utc_now_iso()
            self._frame_width = width
            self._frame_height = height
            self._frame_times.append(now)
            self._frame_count += 1
            self._last_error = None
        return True

    def status_payload(self) -> dict[str, Any]:
        payload = super().status_payload()
        notes = [ROS_NOTE]
        if not payload["available"]:
            notes.extend(payload["notes"])
        payload.update(
            {
                "source": "armpi_debug_gateway_ros",
                "topic": f"ros:{self.ros_topic}",
                "device": None,
                "encoding": "bgr8",
                "notes": notes,
            }
        )
        return payload
