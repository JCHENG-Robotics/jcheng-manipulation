"""HTTP client for the on-robot `armpi_debug_gateway`.

The gateway is intentionally a separate process on the machine that owns the
vendor ROS 2 stack. This client is read-only: it only calls `/health`,
`/state`, `/battery`, `/camera/status`, and `/camera/snapshot`.
"""

from __future__ import annotations

import json
from typing import Any
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


class GatewayError(RuntimeError):
    def __init__(self, message: str, *, status: int | None = None) -> None:
        super().__init__(message)
        self.status = status


class GatewayClient:
    def __init__(
        self,
        base_url: str,
        *,
        token: str | None = None,
        timeout_s: float = 2.0,
    ) -> None:
        self.base_url = base_url.rstrip("/")
        self.token = token
        self.timeout_s = timeout_s

    def _headers(self, accept: str) -> dict[str, str]:
        headers = {"Accept": accept}
        if self.token:
            headers["X-Armpi-Token"] = self.token
        return headers

    def _open(self, path: str, accept: str):
        url = f"{self.base_url}{path}"
        request = Request(url, headers=self._headers(accept), method="GET")
        try:
            return urlopen(request, timeout=self.timeout_s)
        except HTTPError as exc:
            raise GatewayError(f"gateway returned HTTP {exc.code} for {path}", status=exc.code) from exc
        except (URLError, TimeoutError, OSError) as exc:
            raise GatewayError(f"gateway request failed for {path}: {exc}") from exc

    def get_json(self, path: str) -> dict[str, Any]:
        with self._open(path, "application/json") as response:
            payload = response.read()
        try:
            data = json.loads(payload.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            raise GatewayError(f"gateway returned invalid JSON for {path}") from exc
        if not isinstance(data, dict):
            raise GatewayError(f"gateway returned a non-object JSON payload for {path}")
        return data

    def get_bytes(self, path: str) -> tuple[bytes, str]:
        with self._open(path, "*/*") as response:
            payload = response.read()
            content_type = response.headers.get_content_type() or "application/octet-stream"
        return payload, content_type
