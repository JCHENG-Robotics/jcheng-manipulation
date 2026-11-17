from __future__ import annotations

from pathlib import Path

import pytest

from armpi_debug_gateway.auth import TokenError, resolve_token


def test_explicit_token_wins(tmp_path: Path) -> None:
    token_file = tmp_path / "token"
    token_file.write_text("from-file\n", encoding="utf-8")
    assert resolve_token(
        token="explicit",
        token_file=str(token_file),
        environ={"ARMPI_GATEWAY_TOKEN": "from-env"},
    ) == "explicit"


def test_token_file_wins_over_env(tmp_path: Path) -> None:
    token_file = tmp_path / "token"
    token_file.write_text("  from-file\n", encoding="utf-8")
    assert resolve_token(
        token_file=str(token_file),
        environ={"ARMPI_GATEWAY_TOKEN": "from-env"},
    ) == "from-file"


def test_env_token_is_used_as_a_value_not_a_path() -> None:
    assert resolve_token(environ={"ARMPI_GATEWAY_TOKEN": "from-env"}) == "from-env"


def test_no_source_returns_none() -> None:
    assert resolve_token(environ={}) is None


def test_missing_token_file_is_an_error(tmp_path: Path) -> None:
    with pytest.raises(TokenError, match="not found"):
        resolve_token(token_file=str(tmp_path / "missing"), environ={})


def test_empty_token_file_is_an_error(tmp_path: Path) -> None:
    token_file = tmp_path / "token"
    token_file.write_text(" \n", encoding="utf-8")
    with pytest.raises(TokenError, match="empty"):
        resolve_token(token_file=str(token_file), environ={})
