"""Tests for the phase-3 path bootstrap."""

from __future__ import annotations

import sys
from pathlib import Path

import _bootstrap


def test_bootstrap_adds_existing_paths_once():
    _bootstrap.bootstrap()
    for path in (
        _bootstrap.TOOLS,
        _bootstrap.PHASE2,
        _bootstrap.ANALYSIS,
        _bootstrap.SHARED,
    ):
        assert path in sys.path
        assert Path(path).is_dir()

    before = list(sys.path)
    _bootstrap.bootstrap()
    assert sys.path == before


def test_repo_points_at_the_checkout_root():
    assert Path(_bootstrap.REPO, "README.md").is_file()
    assert Path(_bootstrap.REPO, "tools", "phase3").is_dir()
