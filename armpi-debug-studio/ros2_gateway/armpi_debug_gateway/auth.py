"""Token resolution for the read-only gateway.

Kept free of ROS imports so the precedence rules can be unit-tested on a
laptop. Precedence: explicit token, token file, environment token.
"""

from __future__ import annotations

import os
from collections.abc import Mapping
from pathlib import Path


class TokenError(ValueError):
    """Raised when a configured token source is missing or empty."""


def resolve_token(
    *,
    token: str | None = None,
    token_file: str | None = None,
    environ: Mapping[str, str] | None = None,
) -> str | None:
    if token:
        return token
    if token_file:
        path = Path(token_file)
        if not path.is_file():
            raise TokenError(f"token file not found: {path}")
        value = path.read_text(encoding="utf-8").strip()
        if not value:
            raise TokenError(f"token file is empty: {path}")
        return value
    env = os.environ if environ is None else environ
    return env.get("ARMPI_GATEWAY_TOKEN") or None
