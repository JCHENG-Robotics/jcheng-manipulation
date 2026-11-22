"""Pure formatting helpers for the gateway telemetry payloads."""

from __future__ import annotations

from typing import Any

BATTERY_EMPTY_MV = 6400.0
BATTERY_FULL_MV = 8400.0
TF_FRESH_MS = 500


def battery_state(voltage_mv: int, age_ms: int | None) -> dict[str, Any]:
    """Return percent/status/validity for a 2-cell pack."""
    percent = max(
        0.0,
        min(
            100.0,
            (voltage_mv - BATTERY_EMPTY_MV)
            / (BATTERY_FULL_MV - BATTERY_EMPTY_MV)
            * 100.0,
        ),
    )
    if percent < 10.0:
        status = "critical"
    elif percent < 25.0:
        status = "low"
    else:
        status = "ok"
    validity = "valid" if age_ms is not None and age_ms < 5000 else "stale"
    return {
        "voltage_v": round(voltage_mv / 1000.0, 3),
        "percent_estimate": round(percent, 1),
        "status": status,
        "validity": validity,
    }


def tf_health(
    edges: list[dict[str, Any]],
    now: float,
    *,
    fresh_ms: int = TF_FRESH_MS,
) -> dict[str, Any]:
    """Summarize TF edges without the node's lock or ROS types."""
    enriched: list[dict[str, Any]] = []
    for edge in edges:
        item = dict(edge)
        item["age_ms"] = max(
            0,
            int((now - float(item["received_monotonic"])) * 1000.0),
        )
        item.pop("received_monotonic", None)
        enriched.append(item)

    dynamic = [edge for edge in enriched if not edge["static"]]
    static = [edge for edge in enriched if edge["static"]]
    dynamic_ages = [int(edge["age_ms"]) for edge in dynamic]
    static_ages = [int(edge["age_ms"]) for edge in static]
    all_ages = [int(edge["age_ms"]) for edge in enriched]
    freshest = min(dynamic_ages) if dynamic_ages else None
    if not enriched:
        status = "unavailable"
    elif not dynamic:
        status = "static_only"
    elif freshest is not None and freshest <= fresh_ms:
        status = "ok"
    else:
        status = "stale"

    notes = [
        "TF is published by robot nodes. Dynamic transforms may be derived "
        "from commanded joint states, not encoder feedback.",
    ]
    if status == "static_only":
        notes.append(
            "Only /tf_static has been received; dynamic links are not live."
        )
    elif status == "stale":
        notes.append(f"Newest dynamic TF edge is {freshest} ms old.")
    elif status == "unavailable":
        notes.append("No /tf or /tf_static message has been received.")

    return {
        "status": status,
        "dynamic_edges": len(dynamic),
        "static_edges": len(static),
        "freshest_age_ms": freshest,
        "max_age_ms": max(all_ages) if all_ages else None,
        "max_dynamic_age_ms": max(dynamic_ages) if dynamic_ages else None,
        "max_static_age_ms": max(static_ages) if static_ages else None,
        "transforms": enriched,
        "notes": notes,
    }
