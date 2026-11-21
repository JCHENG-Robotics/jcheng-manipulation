"""Minimal HTTP server around GatewayNode.

Endpoints:
    GET /health
    GET /state
    GET /battery
    GET /camera/status
    GET /camera/snapshot
    GET /aux-camera/status
    GET /aux-camera/snapshot
    GET /tf
    GET /motion/status
    POST /motion/dispatch   (supervised instance only)
    POST /motion/stop       (supervised instance only)
    POST /chassis/pulse     (supervised instance with an enabled chassis block)

All endpoints require `X-Armpi-Token` when a token is configured. The motion
endpoints answer ``MOTION_DISABLED`` unless the gateway was started with
``--enable-motion`` and a validated limits file; they never fall through to the
read-only handlers.
"""

from __future__ import annotations

import json
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import TYPE_CHECKING, Any
from urllib.parse import urlparse

from .chassis import ChassisRejected
from .motion import MotionRejected

if TYPE_CHECKING:
    from .node import GatewayNode

MAX_MOTION_BODY_BYTES = 256 * 1024


class GatewayHTTPServer(ThreadingHTTPServer):
    daemon_threads = True
    allow_reuse_address = True

    def __init__(
        self,
        server_address: tuple[str, int],
        node: GatewayNode,
        *,
        token: str | None = None,
    ) -> None:
        self.node = node
        self.token = token
        super().__init__(server_address, GatewayRequestHandler)

    def serve_until(self, stop_event: threading.Event, *, join_timeout_s: float = 5.0) -> None:
        """Serve in a worker thread until *stop_event* is set, then shut down.

        ``BaseServer.shutdown()`` must not be called from the thread running
        ``serve_forever()`` or it deadlocks. Keeping the serving loop in a
        worker thread lets a signal handler (main thread) only set an event.
        """
        serve_thread = threading.Thread(
            target=self.serve_forever,
            name="gateway-http",
            daemon=True,
        )
        serve_thread.start()
        try:
            stop_event.wait()
        finally:
            self.shutdown()
            serve_thread.join(timeout=join_timeout_s)
            self.server_close()


class GatewayRequestHandler(BaseHTTPRequestHandler):
    server: GatewayHTTPServer

    def log_message(self, format: str, *args: Any) -> None:  # noqa: A003
        # Keep the robot console quiet; errors still reach the client as JSON.
        return None

    def _authorized(self) -> bool:
        if not self.server.token:
            return True
        return self.headers.get("X-Armpi-Token") == self.server.token

    def _json(self, status: int, payload: dict[str, Any]) -> None:
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(status)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _bytes(self, status: int, body: bytes, content_type: str) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _error(self, status: int, code: str, message: str) -> None:
        self._json(status, {"error": {"code": code, "message": message}})

    def do_GET(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if not self._authorized():
            self._error(401, "UNAUTHORIZED", "missing or invalid X-Armpi-Token")
            return

        if path == "/health":
            self._json(200, self.server.node.health_payload())
            return
        if path == "/state":
            self._json(200, self.server.node.state_payload())
            return
        if path == "/battery":
            self._json(200, self.server.node.battery_payload())
            return
        if path == "/camera/status":
            self._json(200, self.server.node.camera_status_payload())
            return
        if path == "/camera/snapshot":
            frame = self.server.node.camera_snapshot()
            if frame is None:
                self._error(503, "CAMERA_UNAVAILABLE", "no camera frame received yet")
                return
            self._bytes(200, frame, "image/jpeg")
            return
        if path == "/aux-camera/status":
            self._json(200, self.server.node.aux_camera_status_payload())
            return
        if path == "/aux-camera/snapshot":
            frame = self.server.node.aux_camera_snapshot()
            if frame is None:
                self._error(503, "AUX_CAMERA_UNAVAILABLE", "no aux camera frame available")
                return
            self._bytes(200, frame, "image/jpeg")
            return
        if path == "/tf":
            self._json(200, self.server.node.tf_payload())
            return
        if path == "/pointcloud/meta":
            meta_fn = getattr(self.server.node, "pointcloud_meta_payload", None)
            if callable(meta_fn):
                self._json(200, meta_fn())
            else:
                self._json(
                    200,
                    {"available": False, "topic": None, "count": 0},
                )
            return
        if path == "/pointcloud":
            cloud_fn = getattr(self.server.node, "pointcloud_payload", None)
            if not callable(cloud_fn):
                self._error(
                    503,
                    "POINTCLOUD_UNAVAILABLE",
                    "this gateway build has no point-cloud endpoint",
                )
                return
            try:
                body, content_type = cloud_fn()
            except RuntimeError as exc:
                self._error(503, "POINTCLOUD_UNAVAILABLE", str(exc))
                return
            self._bytes(200, body, content_type)
            return
        if path == "/motion/status":
            status_fn = getattr(self.server.node, "motion_status_payload", None)
            if callable(status_fn):
                self._json(200, status_fn())
            else:
                self._json(
                    200,
                    {
                        "motion_enabled": False,
                        "state": "disabled",
                        "measured": {},
                        "measured_ready": False,
                    },
                )
            return
        if path in ("/motion/dispatch", "/motion/stop", "/chassis/pulse"):
            self._error(405, "METHOD_NOT_ALLOWED", "use POST for motion endpoints")
            return

        self._error(404, "NOT_FOUND", f"unknown path: {path}")

    def _read_json_body(self) -> dict[str, Any] | None:
        length_raw = self.headers.get("Content-Length")
        try:
            length = int(length_raw or "0")
        except ValueError:
            self._error(400, "MOTION_BAD_REQUEST", "invalid Content-Length")
            return None
        if length <= 0 or length > MAX_MOTION_BODY_BYTES:
            self._error(400, "MOTION_BAD_REQUEST", "request body is missing or too large")
            return None
        payload = self.rfile.read(length)
        try:
            body = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError):
            self._error(400, "MOTION_BAD_REQUEST", "request body must be JSON")
            return None
        if not isinstance(body, dict):
            self._error(400, "MOTION_BAD_REQUEST", "request body must be an object")
            return None
        return body

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if not self._authorized():
            self._error(401, "UNAUTHORIZED", "missing or invalid X-Armpi-Token")
            return
        if path == "/motion/dispatch":
            if not getattr(self.server.node, "motion_enabled", False):
                self._error(
                    403,
                    "MOTION_DISABLED",
                    "hardware motion is disabled on this gateway instance",
                )
                return
            body = self._read_json_body()
            if body is None:
                return
            try:
                response = self.server.node.dispatch_motion(body)
            except MotionRejected as exc:
                self._json(
                    exc.http_status,
                    {
                        "error": {
                            "code": exc.code,
                            "message": str(exc),
                            "details": exc.details,
                        }
                    },
                )
                return
            self._json(200, response)
            return
        if path == "/motion/stop":
            if not getattr(self.server.node, "motion_enabled", False):
                self._error(
                    403,
                    "MOTION_DISABLED",
                    "hardware motion is disabled on this gateway instance",
                )
                return
            body = self._read_json_body()
            if body is None:
                return
            try:
                response = self.server.node.stop_motion(body)
            except MotionRejected as exc:
                self._json(
                    exc.http_status,
                    {
                        "error": {
                            "code": exc.code,
                            "message": str(exc),
                            "details": exc.details,
                        }
                    },
                )
                return
            self._json(200, response)
            return
        if path == "/chassis/pulse":
            if not getattr(self.server.node, "chassis_enabled", False):
                self._error(
                    403,
                    "CHASSIS_DISABLED",
                    "the chassis write path is disabled on this gateway instance",
                )
                return
            body = self._read_json_body()
            if body is None:
                return
            try:
                response = self.server.node.dispatch_chassis(body)
            except ChassisRejected as exc:
                self._json(
                    exc.http_status,
                    {
                        "error": {
                            "code": exc.code,
                            "message": str(exc),
                            "details": exc.details,
                        }
                    },
                )
                return
            self._json(200, response)
            return
        self._error(405, "METHOD_NOT_ALLOWED", "read-only gateway")
