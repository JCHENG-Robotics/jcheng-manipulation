"""G5 recorder output must pass the G2 import contract unchanged."""

from __future__ import annotations

import sys
from pathlib import Path

from app.domain.trajectory import analyze, normalize_dataset

GATEWAY_ROOT = Path(__file__).resolve().parents[2] / "ros2_gateway"
if str(GATEWAY_ROOT) not in sys.path:
    sys.path.insert(0, str(GATEWAY_ROOT))

from armpi_debug_gateway.trajectory_capture import (  # noqa: E402
    ARM_JOINTS,
    build_dataset,
    build_static_planned,
)


def _samples(start: float, count: int, value: float) -> list[dict[str, object]]:
    return [
        {"t_s": start + index * 0.02, "positions": [value] * len(ARM_JOINTS)}
        for index in range(count)
    ]


def test_recorder_dataset_is_accepted_by_g2_and_tracking_is_measured() -> None:
    planned = build_static_planned(ARM_JOINTS, [0.0] * 5, duration_s=1.0)
    dataset = build_dataset(
        name="g5 contract bridge",
        joint_names=list(ARM_JOINTS),
        planned=planned,
        commanded=_samples(0.02, 20, 0.0),
        feedback=_samples(0.03, 20, 0.001),
        commanded_source="vendor command echo",
        feedback_source="servo_states",
    )
    normalized = normalize_dataset(dataset)
    assert normalized["feedback"]["truth"] == "measured"
    assert len(normalized["feedback"]["points"]) == 20

    result = analyze(normalized)
    assert result["tracking"]["available"] is True
    assert result["tracking"]["aggregate"]["max_abs_error_rad"] > 0
    assert result["command_fidelity"]["available"] is True
