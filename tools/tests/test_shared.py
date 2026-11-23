"""Direct tests for the shared jcheng_stats / jcheng_vision packages."""

from __future__ import annotations

import numpy as np
import pytest

from jcheng_stats import (
    bootstrap_mean_ci,
    exact_binomial_two_sided,
    percentile,
    summary,
)
from jcheng_vision import (
    connected_components,
    describe_cluster,
    fit_slab,
    select_cluster,
)


def test_stats_summary_and_empty():
    assert summary([]) == {
        "n": 0,
        "mean": None,
        "std": None,
        "min": None,
        "max": None,
        "ci95_mean": None,
    }
    result = summary([0.5, 1.0, 2.5, 3.25, 6.0])
    assert result["n"] == 5
    assert result["mean"] == pytest.approx(2.65)
    assert result["min"] == 0.5
    assert result["max"] == 6.0


def test_stats_percentile_and_bootstrap_are_deterministic():
    assert percentile([1.0, 2.0, 3.0, 4.0], 0.5) == pytest.approx(2.5)
    first = bootstrap_mean_ci([1.0, 2.0, 3.0, 4.0], 200, 7, 0.95)
    second = bootstrap_mean_ci([1.0, 2.0, 3.0, 4.0], 200, 7, 0.95)
    assert first == second
    assert first["low"] <= first["mean"] <= first["high"]


def test_stats_exact_binomial():
    assert exact_binomial_two_sided(24, 0) == pytest.approx(
        2.0 / 2**24, abs=1e-15
    )
    assert exact_binomial_two_sided(0, 0) == 1.0


def _cloud() -> np.ndarray:
    rng = np.random.default_rng(7)
    floor = rng.normal((0.20, 0.0, -0.080), (0.025, 0.025, 0.002), (300, 3))
    cube = rng.normal((0.25, 0.02, -0.040), (0.012, 0.012, 0.002), (150, 3))
    return np.vstack([floor, cube])


def test_vision_components_and_slab():
    points = _cloud()
    labels, clusters = connected_components(points, 0.01)
    assert len(clusters) == 2
    assert len(labels) == len(points)
    top = min(clusters, key=lambda cluster: len(cluster["points"]))
    fitted = fit_slab(top["points"])
    assert fitted["slab_points"] > 0
    assert fitted["extent_m"][0] > 0.0
    bounds = describe_cluster(top["points"])
    assert bounds["points"] == len(top["points"])


def test_vision_select_cluster_gate():
    points = _cloud()
    _, clusters = connected_components(points, 0.01)
    gates = {
        "min_points": 20,
        "max_xy_m": 0.10,
        "min_height_m": 0.02,
        "max_height_m": 0.10,
        "x_range": (0.15, 0.35),
        "y_range": 0.10,
    }
    best, reasons = select_cluster(clusters, -0.08, gates)
    assert best is not None
    assert len(reasons) == len(clusters)
    assert any(reason["ok"] for reason in reasons)
