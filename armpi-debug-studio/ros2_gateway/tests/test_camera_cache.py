"""Tests for the ROS-free camera frame cache."""

from __future__ import annotations

from types import SimpleNamespace

import pytest

from armpi_debug_gateway.camera_cache import CameraCache
from armpi_debug_gateway.imaging import cv2

pytestmark = pytest.mark.skipif(cv2 is None, reason="opencv is not installed")


def _image(
    width: int = 2,
    height: int = 2,
    encoding: str = "bgr8",
) -> SimpleNamespace:
    channels = 1 if encoding == "mono8" else 3
    return SimpleNamespace(
        width=width,
        height=height,
        encoding=encoding,
        step=width * channels,
        data=bytes(width * height * channels),
    )


class _Logger:
    def __init__(self) -> None:
        self.warnings: list[str] = []

    def warning(self, message: str) -> None:
        self.warnings.append(message)


def test_snapshot_none_before_first_frame():
    cache = CameraCache(
        topic="/camera",
        jpeg_quality=80,
        bridge=None,
        now_iso=lambda: "2026-11-22T00:00:00Z",
    )
    assert cache.snapshot() is None
    payload = cache.status_payload()
    assert payload["available"] is False
    assert payload["validity"] == "unavailable"
    assert payload["frame_count"] == 0


def test_frame_is_stored_as_jpeg_and_reported():
    cache = CameraCache(
        topic="/camera",
        jpeg_quality=80,
        bridge=None,
        now_iso=lambda: "2026-11-22T00:00:00Z",
    )
    cache.on_image(_image())
    payload = cache.status_payload()
    assert payload["available"] is True
    assert payload["truth"] == "measured"
    assert payload["validity"] == "valid"
    assert payload["width"] == 2
    assert payload["height"] == 2
    assert payload["encoding"] == "bgr8"
    assert payload["frame_count"] == 1
    assert payload["fps"] == 0.0
    assert payload["last_frame_at"] == "2026-11-22T00:00:00Z"
    snapshot = cache.snapshot()
    assert snapshot is not None
    assert snapshot.startswith(b"\xff\xd8")  # JPEG SOI marker


def test_bad_frame_is_ignored_and_logged():
    logger = _Logger()
    cache = CameraCache(
        topic="/camera",
        jpeg_quality=80,
        bridge=None,
        now_iso=lambda: "2026-11-22T00:00:00Z",
        logger=logger,
    )
    cache.on_image(_image(encoding="yuv422"))
    assert cache.snapshot() is None
    assert logger.warnings
    assert cache.status_payload()["frame_count"] == 0
