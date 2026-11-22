"""Add the repository modules needed by the offline parity tests."""

import sys
from pathlib import Path

TOOLS = Path(__file__).resolve().parents[1]
ROOT = TOOLS.parent
for relative in (
    "tools",
    "tools/phase2",
    "ros2_ws/src/armpi_analysis",
    "armpi-debug-studio/backend",
):
    path = ROOT / relative
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
