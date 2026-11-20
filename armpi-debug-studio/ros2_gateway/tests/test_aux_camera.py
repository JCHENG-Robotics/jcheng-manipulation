from __future__ import annotations

import time

from armpi_debug_gateway.aux_camera import UvcCamera


class FakeFrame:
    shape = (480, 640, 3)


class FakeCapture:
    def __init__(self, *, opened: bool = True, frame: object | None = None) -> None:
        self._opened = opened
        self._frame = frame
        self.released = False

    def isOpened(self) -> bool:  # noqa: N802 - mirrors cv2.VideoCapture
        return self._opened

    def read(self) -> tuple[bool, object | None]:
        if self._frame is None:
            return False, None
        return True, self._frame

    def release(self) -> None:
        self.released = True


def _camera(capture: FakeCapture, **kwargs: object) -> UvcCamera:
    return UvcCamera(
        "/dev/video4",
        capture_factory=lambda: capture,
        encoder=lambda _frame: b"\xff\xd8fake-jpeg",
        **kwargs,
    )


def test_uvc_camera_reports_frames_and_snapshot() -> None:
    camera = _camera(FakeCapture(frame=FakeFrame()))
    assert camera.snapshot() is None
    assert camera.status_payload()["available"] is False

    assert camera.poll_once() is True

    status = camera.status_payload()
    assert status["available"] is True
    assert status["validity"] == "valid"
    assert status["truth"] == "measured"
    assert status["width"] == 640
    assert status["height"] == 480
    assert status["encoding"] == "yuyv"
    assert status["topic"] == "uvc:/dev/video4"
    assert status["frame_count"] == 1
    assert camera.snapshot() == b"\xff\xd8fake-jpeg"


def test_uvc_camera_open_failure_is_reported_not_raised() -> None:
    capture = FakeCapture(opened=False)
    camera = _camera(capture)

    assert camera.poll_once() is False

    status = camera.status_payload()
    assert status["available"] is False
    assert status["validity"] == "unavailable"
    assert status["truth"] == "unknown"
    assert any("cannot open /dev/video4" in note for note in status["notes"])
    assert capture.released is True


def test_uvc_camera_read_failure_releases_capture_for_retry() -> None:
    capture = FakeCapture(frame=None)
    camera = _camera(capture)

    assert camera.poll_once() is False
    assert capture.released is True
    assert camera.snapshot() is None
    notes = " ".join(camera.status_payload()["notes"])
    assert "returned no frame" in notes


def test_uvc_camera_thread_captures_then_stops() -> None:
    camera = _camera(FakeCapture(frame=FakeFrame()), fps=50)
    camera.start()
    try:
        deadline = time.monotonic() + 2.0
        while camera.snapshot() is None and time.monotonic() < deadline:
            time.sleep(0.02)
        assert camera.snapshot() == b"\xff\xd8fake-jpeg"
    finally:
        camera.stop()
    assert camera._thread is None


class FakeImage:
    """Minimal sensor_msgs/Image stand-in."""

    def __init__(self, encoding: str = "bgr8", width: int = 4, height: int = 3,
                 data: object | None = None) -> None:
        self.encoding = encoding
        self.width = width
        self.height = height
        self.data = data if data is not None else bytearray(width * height * 3)


def _ros_camera(**kwargs: object):
    from armpi_debug_gateway.aux_camera import RosImageCamera

    return RosImageCamera(
        "/ext_cam/color/image_raw",
        encoder=lambda _frame: b"\xff\xd8fake-jpeg",
        **kwargs,
    )


def test_ros_camera_offers_an_image() -> None:
    camera = _ros_camera()
    assert camera.poll_once() is False
    assert camera.snapshot() is None
    before = camera.snapshot()

    assert camera.offer(FakeImage()) is True
    payload = camera.status_payload()
    assert camera.snapshot() is not None
    assert camera.snapshot() != before
    assert payload["available"] is True
    assert payload["source"] == "armpi_debug_gateway_ros"
    assert payload["topic"] == "ros:/ext_cam/color/image_raw"
    assert payload["width"] == 4 and payload["height"] == 3
    assert payload["frame_count"] == 1
    assert "ext_cam_node" in payload["notes"][0]


def test_ros_camera_rejects_unsupported_encodings() -> None:
    camera = _ros_camera()
    assert camera.offer(FakeImage(encoding="yuv422")) is False
    payload = camera.status_payload()
    assert payload["available"] is False
    assert payload["validity"] == "unavailable"
    assert any("unsupported image" in note for note in payload["notes"])


def test_ros_camera_rejects_mismatched_payload_size() -> None:
    camera = _ros_camera()
    bad = FakeImage(width=4, height=3, data=bytearray(3))
    assert camera.offer(bad) is False
    assert camera.snapshot() is None
    assert any("does not match" in note
               for note in camera.status_payload()["notes"])


def test_ros_camera_encodes_rgb8_and_mono8() -> None:
    camera = _ros_camera()
    assert camera.offer(FakeImage(encoding="rgb8", width=4, height=3,
                                  data=bytearray(4 * 3 * 3))) is True
    assert camera.offer(FakeImage(encoding="mono8", width=4, height=3,
                                  data=bytearray(4 * 3))) is True
    assert camera.status_payload()["frame_count"] == 2
