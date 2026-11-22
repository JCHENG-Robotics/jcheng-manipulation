"""Tests for the pure telemetry formatting helpers."""

from __future__ import annotations

from armpi_debug_gateway.telemetry_format import battery_state, tf_health


def test_battery_state_thresholds_and_age():
    full = battery_state(8400, 0)
    assert full["percent_estimate"] == 100.0
    assert full["status"] == "ok"
    assert full["validity"] == "valid"
    assert full["voltage_v"] == 8.4

    critical = battery_state(6500, 1000)
    assert critical["percent_estimate"] == 5.0
    assert critical["status"] == "critical"
    assert critical["validity"] == "valid"

    low = battery_state(6800, 6000)
    assert low["percent_estimate"] == 20.0
    assert low["status"] == "low"
    assert low["validity"] == "stale"

    unknown_age = battery_state(6000, None)
    assert unknown_age["percent_estimate"] == 0.0
    assert unknown_age["validity"] == "stale"


def test_tf_health_empty():
    summary = tf_health([], 100.0)
    assert summary["status"] == "unavailable"
    assert summary["dynamic_edges"] == 0
    assert summary["static_edges"] == 0
    assert summary["freshest_age_ms"] is None
    assert any("No /tf" in note for note in summary["notes"])


def test_tf_health_static_only():
    edges = [
        {
            "parent": "base_link",
            "child": "camera",
            "static": True,
            "received_monotonic": 90.0,
        }
    ]
    summary = tf_health(edges, 100.0)
    assert summary["status"] == "static_only"
    assert summary["dynamic_edges"] == 0
    assert summary["static_edges"] == 1
    assert summary["max_static_age_ms"] == 10000


def test_tf_health_ok_and_stale():
    fresh = [
        {
            "parent": "base_link",
            "child": "link4",
            "static": False,
            "received_monotonic": 99.9,
        }
    ]
    summary = tf_health(fresh, 100.0)
    assert summary["status"] == "ok"
    assert summary["freshest_age_ms"] in (99, 100)
    assert summary["transforms"][0]["age_ms"] in (99, 100)
    assert "received_monotonic" not in summary["transforms"][0]

    stale = [
        {
            "parent": "base_link",
            "child": "link4",
            "static": False,
            "received_monotonic": 99.0,
        }
    ]
    summary = tf_health(stale, 100.0)
    assert summary["status"] == "stale"
    assert any("1000 ms old" in note for note in summary["notes"])


def test_tf_health_reports_dynamic_and_static_maxima():
    edges = [
        {
            "parent": "base_link",
            "child": "link4",
            "static": False,
            "received_monotonic": 99.5,
        },
        {
            "parent": "base_link",
            "child": "camera",
            "static": True,
            "received_monotonic": 90.0,
        },
    ]
    summary = tf_health(edges, 100.0)
    assert summary["dynamic_edges"] == 1
    assert summary["static_edges"] == 1
    assert summary["max_dynamic_age_ms"] == 500
    assert summary["max_static_age_ms"] == 10000
    assert summary["max_age_ms"] == 10000
