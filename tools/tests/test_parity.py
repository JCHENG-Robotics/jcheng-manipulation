"""Parity and characterization tests for duplicated offline implementations.

These tests lock the current behaviour before any deduplication:

* ``domain/experiments.py`` vs ``armpi_analysis/paired_trials.py`` — the
  statistics helpers must agree for the same inputs;
* ``domain/profile.py`` vs ``tools/phase2/motion_profile.py`` — the S-curve
  waypoints must agree for non-zero moves; the zero-move shape is intentionally
  different;
* ``domain/locate.py`` vs ``tools/phase2/object_segment_rgbd.py`` — the
  segmentation helpers must agree; the default minimum top extent intentionally
  differs (0.030 vs 0.035).
"""

from __future__ import annotations

import random

import numpy as np
import pytest

from app.domain import experiments
from app.domain import locate as backend_locate
from app.domain import profile as backend_profile
from armpi_analysis import paired_trials
import motion_profile as tool_profile
import object_segment_rgbd as tool_locate

JOINTS = ("joint1", "joint2", "joint3", "joint4", "joint5")
VALUES = [0.5, 1.0, 2.5, 3.25, 6.0]


# --------------------------------------------------------------- statistics
def test_summary_matches():
    expected = experiments._summary(VALUES)
    actual = paired_trials._summary(VALUES)
    assert set(expected) == set(actual)
    for key in expected:
        if expected[key] is None:
            assert actual[key] is None
        else:
            assert actual[key] == pytest.approx(expected[key], abs=1e-12)


def test_empty_summary_matches():
    assert experiments._summary([]) == paired_trials._summary([])


def test_stats_helpers_are_shared_objects():
    """Both experiment modules now import the same jcheng_stats callables."""
    assert experiments._summary is paired_trials._summary
    assert (
        experiments._bootstrap_mean_ci is paired_trials._bootstrap_mean_ci
    )
    assert (
        experiments._exact_binomial_two_sided
        is paired_trials._exact_binomial_two_sided
    )


def test_bootstrap_mean_ci_matches():
    expected = experiments._bootstrap_mean_ci(VALUES, 200, 7, 0.95)
    actual = paired_trials._bootstrap_mean_ci(VALUES, 200, 7, 0.95)
    for key in ("low", "high", "mean"):
        assert actual[key] == pytest.approx(expected[key], abs=1e-12)


@pytest.mark.parametrize(
    "successes,failures",
    [(24, 0), (12, 12), (9, 15), (0, 24)],
)
def test_exact_binomial_matches(successes, failures):
    assert paired_trials._exact_binomial_two_sided(
        successes, failures
    ) == pytest.approx(
        experiments._exact_binomial_two_sided(successes, failures),
        abs=1e-15,
    )


# ------------------------------------------------------------------ profile
def _random_pose(rng: random.Random) -> dict[str, float]:
    return {name: rng.uniform(-0.5, 0.5) for name in JOINTS}


def test_profile_waypoints_match_for_nonzero_moves():
    rng = random.Random(7)
    for _ in range(10):
        start = _random_pose(rng)
        target = _random_pose(rng)
        params = backend_profile.ProfileParams(1.2, 6.0, 0.05)
        backend = backend_profile.plan_move(start, target, params)
        times, waypoints, info = tool_profile.plan_move(
            [start[name] for name in JOINTS],
            [target[name] for name in JOINTS],
            1.2,
            6.0,
            0.05,
        )
        assert len(backend.durations_s) == len(times) - 1
        for expected, actual in zip(
            backend.durations_s,
            [b - a for a, b in zip(times, times[1:])],
        ):
            assert actual == pytest.approx(expected, abs=1e-12)
        assert len(backend.points) == len(waypoints) - 1
        for point, waypoint in zip(backend.points, waypoints[1:]):
            for index, name in enumerate(JOINTS):
                assert waypoint[index] == pytest.approx(
                    point[name], abs=1e-12
                )
        assert backend.total_s == pytest.approx(info["total"], abs=1e-12)


def test_zero_move_is_one_hold_segment_in_both():
    start = _random_pose(random.Random(11))
    backend = backend_profile.plan_move(
        start, start, backend_profile.ProfileParams(1.2, 6.0, 0.05)
    )
    times, waypoints, info = tool_profile.plan_move(
        [start[name] for name in JOINTS],
        [start[name] for name in JOINTS],
        1.2,
        6.0,
        0.05,
    )
    expected = [start[name] for name in JOINTS]
    assert backend.total_s == pytest.approx(0.05)
    assert info["total"] == pytest.approx(0.05)
    assert backend.segments == info["segments"] == 1
    assert backend.durations_s == [pytest.approx(0.05)]
    assert times == [0.0, 0.05]
    assert waypoints == [expected, expected]
    assert backend.points == [pytest.approx(start)]


# ------------------------------------------------------------------- vision
def _blob(
    rng: np.random.Generator,
    center: tuple[float, float, float],
    count: int,
    spread: float,
) -> np.ndarray:
    return np.asarray(center) + rng.normal(0.0, spread, size=(count, 3))


def _synthetic_cloud() -> np.ndarray:
    rng = np.random.default_rng(7)
    floor = _blob(rng, (0.20, 0.00, -0.080), 600, 0.025)
    cube = _blob(rng, (0.25, 0.02, -0.040), 300, 0.012)
    return np.vstack([floor, cube])


def test_connected_components_match():
    points = _synthetic_cloud()
    labels_backend, clusters_backend = backend_locate.connected_components(
        points, 0.01
    )
    labels_tool, clusters_tool = tool_locate.connected_components(points, 0.01)
    assert np.array_equal(labels_backend, labels_tool)
    assert [len(cluster["points"]) for cluster in clusters_backend] == [
        len(cluster["points"]) for cluster in clusters_tool
    ]


def test_fit_slab_and_describe_cluster_match():
    points = _synthetic_cloud()
    _, clusters = backend_locate.connected_components(points, 0.01)
    top = min(clusters, key=lambda cluster: len(cluster["points"]))

    expected_fit = backend_locate.fit_slab(top["points"])
    actual_fit = tool_locate.fit_slab(top["points"])
    for key in (
        "slab_points",
        "slab_thickness_m",
        "tilt_deg",
        "yaw_rad",
        "area_m2",
        "z_max",
    ):
        assert actual_fit[key] == pytest.approx(expected_fit[key], abs=1e-12)
    for key in ("centroid", "normal", "extent_m"):
        assert actual_fit[key] == pytest.approx(expected_fit[key], abs=1e-12)

    expected_bounds = backend_locate.describe_cluster(top["points"])
    actual_bounds = tool_locate.describe_cluster(top["points"])
    assert actual_bounds["points"] == expected_bounds["points"]
    for key in ("min", "max", "size", "centroid"):
        assert actual_bounds[key] == pytest.approx(
            expected_bounds[key], abs=1e-12
        )


def test_select_cluster_matches_for_the_same_gates():
    points = _synthetic_cloud()
    _, clusters_backend = backend_locate.connected_components(points, 0.01)
    _, clusters_tool = tool_locate.connected_components(points, 0.01)
    gates = {
        "min_points": 20,
        "max_xy_m": 0.10,
        "min_height_m": 0.02,
        "max_height_m": 0.10,
        "x_range": (0.15, 0.35),
        "y_range": 0.10,
    }
    best_backend, reasons_backend = backend_locate.select_cluster(
        clusters_backend, -0.08, gates
    )
    best_tool, reasons_tool = tool_locate.select_cluster(
        clusters_tool, -0.08, gates
    )
    assert (best_backend is None) == (best_tool is None)
    if best_backend is not None:
        assert best_backend["label"] == best_tool["label"]
    assert [reason["ok"] for reason in reasons_backend] == [
        reason["ok"] for reason in reasons_tool
    ]
    assert [reason["why"] for reason in reasons_backend] == [
        reason["why"] for reason in reasons_tool
    ]


def test_min_top_extent_defaults_are_intentionally_different():
    # Studio accepts the measured 34.6 mm top (0.030); the phase-2 tool default
    # is still 0.035. Change both and this test when unifying the pipeline.
    assert backend_locate.DEFAULT_MIN_TOP_EXTENT_M == 0.030
    assert tool_locate.DEFAULT_MIN_TOP_EXTENT_M == 0.035
