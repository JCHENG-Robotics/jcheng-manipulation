"""On-demand point-cloud capture for the G8 locate endpoint.

A 2.5 MB registered cloud takes noticeable Python time to deserialize. Running
the subscription inside the gateway process competed with the 50 Hz servo
callbacks and the HTTP handlers (the cache advanced only every ~2.4 s). The
capture therefore runs in a short-lived **subprocess** with its own ROS
context: the gateway's motion path never shares a GIL with cloud decoding.

The gateway calls :meth:`PointCloudRunner.capture` from the `/pointcloud/meta`
handler, keeps the captured bytes for the following `/pointcloud` request and
returns plain JSON metadata. No ROS import here.
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import tempfile
import time
from datetime import datetime, timezone
from typing import Any


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class PointCloudRunner:
    def __init__(self, topic: str, *, timeout_s: float = 8.0) -> None:
        self.topic = topic
        self.timeout_s = float(timeout_s)
        self._meta: dict[str, Any] | None = None
        self._body: bytes | None = None
        self._at: float | None = None
        self._error: str | None = None

    def capture(self) -> dict[str, Any]:
        """Run one capture subprocess and cache its cloud for the body call."""
        with tempfile.TemporaryDirectory(prefix="armpi_cloud_") as tmp:
            out = os.path.join(tmp, "cloud_xyz.bin")
            meta_path = os.path.join(tmp, "cloud_meta.json")
            command = [
                sys.executable,
                "-m",
                "armpi_debug_gateway.cloud_capture",
                "--topic",
                self.topic,
                "--out",
                out,
                "--meta-out",
                meta_path,
                "--timeout",
                f"{self.timeout_s:.1f}",
            ]
            try:
                proc = subprocess.run(
                    command,
                    capture_output=True,
                    text=True,
                    timeout=self.timeout_s + 5.0,
                    check=False,
                )
            except subprocess.TimeoutExpired as exc:
                self._error = f"point cloud capture timed out: {exc}"
                raise RuntimeError(self._error) from exc
            if proc.returncode != 0 or not os.path.isfile(out):
                message = (proc.stderr or proc.stdout or "capture failed").strip()
                self._error = message.splitlines()[-1] if message else "capture failed"
                raise RuntimeError(self._error)
            with open(out, "rb") as handle:
                body = handle.read()
            with open(meta_path, "r", encoding="utf-8") as handle:
                meta = json.load(handle)
        self._body = body
        self._meta = meta
        self._at = time.monotonic()
        self._error = None
        return meta

    def meta(self) -> dict[str, Any]:
        """Capture and return the fresh cloud metadata."""
        meta = dict(self.capture())
        meta["available"] = True
        meta["topic"] = self.topic
        meta["wall_time"] = utc_now()
        meta["sample_age_ms"] = 0
        return meta

    def payload(self) -> tuple[bytes, str]:
        """Return the cloud captured by the preceding ``meta`` call."""
        if self._body is None:
            self.meta()
        assert self._body is not None
        return self._body, "application/octet-stream"

    def status(self) -> dict[str, Any]:
        """Report the last capture without triggering a new one."""
        base = {
            "available": self._meta is not None,
            "topic": self.topic,
            "count": int(self._meta.get("count", 0)) if self._meta else 0,
            "sample_age_ms": (
                int(round((time.monotonic() - self._at) * 1000.0))
                if self._at is not None
                else None
            ),
            "last_capture_at": (
                utc_now() if self._at is not None else None
            ),
            "error": self._error,
        }
        if self._meta:
            for key in ("frame_id", "width", "height", "point_step", "layout"):
                base[key] = self._meta.get(key)
        return base
