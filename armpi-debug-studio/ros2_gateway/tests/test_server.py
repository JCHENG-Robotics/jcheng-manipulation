from __future__ import annotations

import json
import threading
import urllib.error
import urllib.request
from collections.abc import Iterator

import pytest

from armpi_debug_gateway.server import GatewayHTTPServer


class FakeNode:
    def health_payload(self) -> dict[str, object]:
        return {"status": "ok", "motion_enabled": False}

    def state_payload(self) -> dict[str, object]:
        return {"joints": [], "truth": "commanded"}

    def battery_payload(self) -> dict[str, object]:
        return {"voltage_mv": 7424, "truth": "measured"}

    def camera_status_payload(self) -> dict[str, object]:
        return {"width": 640, "height": 400, "encoding": "bgr8"}

    def camera_snapshot(self) -> bytes | None:
        return b"\xff\xd8fake-jpeg"

    def tf_payload(self) -> dict[str, object]:
        return {
            "available": True,
            "status": "ok",
            "truth": "measured_tf",
            "source": "tf2_topics",
            "dynamic_edges": 1,
            "static_edges": 1,
            "transforms": [
                {
                    "parent": "base_link",
                    "child": "link1",
                    "translation_m": [0.0, 0.0, 0.1],
                    "rotation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    "stamp_ns": 1,
                    "age_ms": 5,
                    "static": False,
                }
            ],
            "notes": [],
        }


def _start(token: str | None) -> tuple[GatewayHTTPServer, threading.Event, threading.Thread]:
    server = GatewayHTTPServer(("127.0.0.1", 0), FakeNode(), token=token)
    stop_event = threading.Event()
    thread = threading.Thread(
        target=server.serve_until,
        args=(stop_event,),
        name="test-gateway",
        daemon=True,
    )
    thread.start()
    return server, stop_event, thread


@pytest.fixture()
def gateway() -> Iterator[tuple[GatewayHTTPServer, threading.Event, threading.Thread]]:
    server, stop_event, thread = _start("secret-token")
    try:
        yield server, stop_event, thread
    finally:
        stop_event.set()
        thread.join(timeout=5)
    assert not thread.is_alive()


def _request(
    server: GatewayHTTPServer,
    path: str,
    *,
    token: str | None = None,
    method: str = "GET",
) -> tuple[int, bytes, str]:
    request = urllib.request.Request(f"http://127.0.0.1:{server.server_port}{path}", method=method)
    if token is not None:
        request.add_header("X-Armpi-Token", token)
    try:
        with urllib.request.urlopen(request, timeout=5) as response:
            return response.status, response.read(), response.headers.get("Content-Type", "")
    except urllib.error.HTTPError as exc:
        return exc.code, exc.read(), exc.headers.get("Content-Type", "")


def test_token_is_required(gateway: tuple[GatewayHTTPServer, threading.Event, threading.Thread]) -> None:
    server, _, _ = gateway
    status, body, _ = _request(server, "/health")
    assert status == 401
    assert json.loads(body)["error"]["code"] == "UNAUTHORIZED"

    status, _, _ = _request(server, "/health", token="wrong")
    assert status == 401

    status, body, _ = _request(server, "/health", token="secret-token")
    assert status == 200
    assert json.loads(body)["status"] == "ok"


def test_read_only_endpoints(gateway: tuple[GatewayHTTPServer, threading.Event, threading.Thread]) -> None:
    server, _, _ = gateway
    for path in ("/health", "/state", "/battery", "/camera/status", "/tf"):
        status, body, content_type = _request(server, path, token="secret-token")
        assert status == 200, path
        assert content_type.startswith("application/json")
        assert isinstance(json.loads(body), dict)

    status, body, _ = _request(server, "/tf", token="secret-token")
    assert json.loads(body)["transforms"][0]["child"] == "link1"

    status, body, content_type = _request(server, "/camera/snapshot", token="secret-token")
    assert status == 200
    assert content_type == "image/jpeg"
    assert body == b"\xff\xd8fake-jpeg"


def test_motion_paths_stay_disabled(gateway: tuple[GatewayHTTPServer, threading.Event, threading.Thread]) -> None:
    server, _, _ = gateway
    for method in ("GET", "POST"):
        status, body, _ = _request(server, "/motion/dispatch", token="secret-token", method=method)
        assert status == 501, method
        assert json.loads(body)["error"]["code"] == "NOT_IMPLEMENTED"


def test_unknown_path_is_404(gateway: tuple[GatewayHTTPServer, threading.Event, threading.Thread]) -> None:
    server, _, _ = gateway
    status, body, _ = _request(server, "/nope", token="secret-token")
    assert status == 404
    assert json.loads(body)["error"]["code"] == "NOT_FOUND"


def test_localhost_without_token_allows_reads() -> None:
    server, stop_event, thread = _start(None)
    try:
        status, _, _ = _request(server, "/health")
        assert status == 200
    finally:
        stop_event.set()
        thread.join(timeout=5)


def test_serve_until_stops_and_releases_the_socket() -> None:
    server, stop_event, thread = _start("secret-token")
    stop_event.set()
    thread.join(timeout=5)
    assert not thread.is_alive()

    with pytest.raises((urllib.error.URLError, ConnectionError)):
        urllib.request.urlopen(f"http://127.0.0.1:{server.server_port}/health", timeout=1)
