"""Resolve repository tool/package paths for phase-3 scripts.

Phase-3 runners are executed both as ``python tools/phase3/foo.py`` and as
imported modules from the tests.  Keeping the path setup here means a new
runner only needs ``from _bootstrap import bootstrap; bootstrap()`` instead of
repeating two or three ``sys.path.insert`` calls.
"""

from __future__ import annotations

import sys
from pathlib import Path

PHASE3_DIR = Path(__file__).resolve().parent
TOOLS_DIR = PHASE3_DIR.parent
REPO = str(TOOLS_DIR.parent)
TOOLS = str(TOOLS_DIR)
PHASE2 = str(TOOLS_DIR / "phase2")
ANALYSIS = str(TOOLS_DIR.parent / "ros2_ws" / "src" / "armpi_analysis")
SHARED = str(TOOLS_DIR.parent / "packages" / "jcheng_core")

_PATHS = (SHARED, TOOLS, PHASE2, str(PHASE3_DIR), ANALYSIS)


def bootstrap() -> None:
    """Put the repo tool and analysis packages on ``sys.path`` once."""
    for path in _PATHS:
        if path not in sys.path:
            sys.path.insert(0, path)
