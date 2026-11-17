"""Minimal read-only HTTP server around GatewayNode.

Endpoints:
    GET /health
    GET /state
    GET /battery
    GET /camera/status
    GET /camera/snapshot
    *   /motion/dispatch, /motion/stop -> 501 NOT_IMPLEMENTED

All endpoints require `X-Armpi-Token` when a token is configured.
"""

from __future__ import annotations

import json
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from typing import Any
from urllib.parse import urlparse

from .node import GatewayNode


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
        if path in ("/motion/dispatch", "/motion/stop"):
            self._error(501, "NOT_IMPLEMENTED", "hardware motion is disabled in this phase")
            return

        self._error(404, "NOT_FOUND", f"unknown path: {path}")

    def do_POST(self) -> None:  # noqa: N802
        path = urlparse(self.path).path
        if not self._authorized():
            self._error(401, "UNAUTHORIZED", "missing or invalid X-Armpi-Token")
            return
        if path in ("/motion/dispatch", "/motion/stop"):
            self._error(501, "NOT_IMPLEMENTED", "hardware motion is disabled in this phase")
            return
        self._error(405, "METHOD_NOT_ALLOWED", "read-only gateway")
