"""Tests for the ROS-free telemetry cache."""

from __future__ import annotations

from types import SimpleNamespace

from armpi_debug_gateway.telemetry_cache import TelemetryCache

TICK = 0.004188790204786391


def _cache(on_measured=None, feedback_max_age_ms: int = 500) -> TelemetryCache:
    return TelemetryCache(
        joint_state_topic="/joints",
        servo_state_topic="/servos",
        servo_joints={
            1: "r_joint",
            2: "joint5",
            3: "joint4",
            4: "joint3",
            5: "joint2",
            6: "joint1",
        },
        pulse_to_radians=lambda joint, pulse: (pulse - 500) * TICK,
        feedback_max_age_ms=feedback_max_age_ms,
        required_joints=("joint1", "joint2", "joint3", "joint4", "joint5"),
        now_iso=lambda: "2026-11-23T00:00:00Z",
        on_measured=on_measured,
    )


def test_battery_and_joint_state():
    cache = _cache()
    cache.on_battery(SimpleNamespace(data=7400))
    voltage, at, wall = cache.battery()
    assert voltage == 7400
    assert at is not None
    assert wall == "2026-11-23T00:00:00Z"

    cache.on_joint_state(
        SimpleNamespace(
            name=["joint1", "joint2"],
            position=[0.1, 0.2],
            velocity=[0.0, 0.0],
        )
    )
    joints, _at, _wall = cache.joints()
    assert joints["joint1"]["position_rad"] == 0.1
    assert joints["joint2"]["position_rad"] == 0.2
    assert joints["joint1"]["truth"] == "commanded"
    assert joints["joint1"]["source"] == "/joints"


def test_servo_states_ready_and_guard_callback():
    seen = []

    def on_measured(measured, now):
        seen.append((measured, now))

    cache = _cache(on_measured=on_measured)
    cache.on_servo_states(
        SimpleNamespace(
            servo_state=[
                SimpleNamespace(id=1, position=224),
                SimpleNamespace(id=2, position=501),
                SimpleNamespace(id=3, position=113),
                SimpleNamespace(id=4, position=834),
                SimpleNamespace(id=5, position=597),
                SimpleNamespace(id=6, position=497),
            ]
        )
    )
    measured = cache.measured_with_age()
    assert measured["joint1"]["raw_position"] == 497
    assert measured["r_joint"]["raw_position"] == 224
    assert measured["joint1"]["truth"] == "commanded"
    assert cache.measured_ready() is True
    assert len(seen) == 1
    assert seen[0][1] is not None


def test_out_of_range_servo_sample_is_ignored():
    seen = []
    cache = _cache(on_measured=lambda measured, now: seen.append(measured))
    cache.on_servo_states(
        SimpleNamespace(servo_state=[SimpleNamespace(id=6, position=1001)])
    )
    assert cache.measured_with_age() == {}
    assert seen == []


def test_actual_and_tf_state():
    cache = _cache()
    cache.record_actual(
        {
            "joint1": {
                "raw_position": 497,
                "position_rad": -0.0125,
                "truth": "measured_on_demand",
            }
        }
    )
    actual = cache.actual_with_age()
    assert actual["joint1"]["raw_position"] == 497

    transform = SimpleNamespace(
        header=SimpleNamespace(
            frame_id="base_link",
            stamp=SimpleNamespace(sec=1, nanosec=2),
        ),
        child_frame_id="link4",
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=1.0, y=2.0, z=3.0),
            rotation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
        ),
    )
    message = SimpleNamespace(transforms=[transform])
    cache.on_tf(message, static=True)
    edges, last_at = cache.tf_edges()
    assert last_at is not None
    assert len(edges) == 1
    assert edges[0]["parent"] == "base_link"
    assert edges[0]["child"] == "link4"
    assert edges[0]["stamp_ns"] == 1_000_000_002
    assert edges[0]["static"] is True

    cache.on_tf(message, static=False)
    edges, _last_at = cache.tf_edges()
    assert edges[0]["static"] is False
