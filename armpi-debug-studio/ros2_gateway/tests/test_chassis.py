"""Chassis pulse guard tests (pure; no ROS)."""

from __future__ import annotations

import pytest

from armpi_debug_gateway.chassis import (
    ChassisConfig,
    ChassisConfigError,
    ChassisRejected,
    chassis_config_from_dict,
    plan_chassis_pulse,
)


def test_absent_chassis_block_is_disabled() -> None:
    config = chassis_config_from_dict(None)
    assert config.enabled is False
    assert config.max_duty == 20.0


def test_chassis_config_caps() -> None:
    with pytest.raises(ChassisConfigError):
        chassis_config_from_dict({"enabled": True, "max_duty": 25})
    with pytest.raises(ChassisConfigError):
        chassis_config_from_dict(
            {"enabled": True, "max_duty": 20, "wheel_max_duty": 15}
        )
    with pytest.raises(ChassisConfigError):
        chassis_config_from_dict({"enabled": True, "min_duration_s": 1.0,
                                  "max_duration_s": 0.5})
    config = chassis_config_from_dict(
        {"enabled": True, "max_duty": 10, "wheel_max_duty": 20}
    )
    assert config.enabled is True
    assert config.max_duty == 10


def test_plan_rejects_the_bad_pulses() -> None:
    enabled = ChassisConfig(enabled=True)
    with pytest.raises(ChassisRejected) as disabled:
        plan_chassis_pulse(
            {"vx": 1, "vy": 0, "duty": 10, "duration_s": 0.5},
            ChassisConfig(enabled=False),
        )
    assert disabled.value.code == "CHASSIS_DISABLED"
    assert disabled.value.http_status == 403

    for body, code in (
        ({"vx": 0, "vy": 0, "duty": 10, "duration_s": 0.5}, "CHASSIS_BAD_REQUEST"),
        ({"vx": 1, "vy": 0, "duty": 21, "duration_s": 0.5}, "CHASSIS_DUTY"),
        ({"vx": 1, "vy": 0, "duty": 10, "duration_s": 0.1}, "CHASSIS_DURATION"),
        ({"vx": 1, "vy": 0, "duty": 10, "duration_s": 3.5}, "CHASSIS_DURATION"),
        ({"vx": 1, "vy": 0, "duty": -1, "duration_s": 0.5}, "CHASSIS_DUTY"),
        ({"vx": 1, "vy": 0, "duty": 10}, "CHASSIS_BAD_REQUEST"),
    ):
        with pytest.raises(ChassisRejected) as error:
            plan_chassis_pulse(body, enabled)
        assert error.value.code == code


def test_plan_normalises_and_builds_the_wheels() -> None:
    config = ChassisConfig(enabled=True)
    forward = plan_chassis_pulse(
        {"vx": 0.013, "vy": 0.0, "duty": 10, "duration_s": 0.3}, config
    )
    assert forward.vx == pytest.approx(1.0)
    assert forward.vy == pytest.approx(0.0)
    assert forward.wheels == {1: 10.0, 2: -10.0, 3: -10.0, 4: 10.0}
    assert forward.nominal_travel_mm == pytest.approx(14.7, abs=0.1)

    diagonal = plan_chassis_pulse(
        {"vx": 1.0, "vy": 1.0, "duty": 20, "duration_s": 0.4}, config
    )
    assert diagonal.wheels[1] == pytest.approx(28.28, abs=0.01)
    assert diagonal.wheels[2] == pytest.approx(0.0, abs=0.01)
    assert max(abs(value) for value in diagonal.wheels.values()) <= 30.0


def test_public_payload_shape() -> None:
    config = chassis_config_from_dict({"enabled": True})
    assert config.to_public_dict()["enabled"] is True
    pulse = plan_chassis_pulse(
        {"vx": 0, "vy": 1, "duty": 10, "duration_s": 0.5}, config
    ).to_public_dict()
    assert pulse["nominal_travel_mm"] > 0
    assert set(pulse["wheels"]) == {"1", "2", "3", "4"}
