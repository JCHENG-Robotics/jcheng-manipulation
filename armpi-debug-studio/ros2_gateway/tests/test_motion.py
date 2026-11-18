from __future__ import annotations

import pytest

from armpi_debug_gateway.motion import (
    ARM_JOINTS,
    MotionConfigError,
    MotionGuard,
    MotionRejected,
    motion_config_from_dict,
)

TICK = 0.004188790204786391
BOUNDS = {
    "joint1": (6, 70, 930, 1),
    "joint2": (5, 342, 810, 1),
    "joint3": (4, 500, 930, -1),
    "joint4": (3, 120, 880, 1),
    "joint5": (2, 120, 870, 1),
}


def _raw_config() -> dict:
    return {
        "schema_version": 1,
        "mapping_version": "test-mapping",
        "radians_per_tick": TICK,
        "joints": {
            joint: {
                "servo_id": servo_id,
                "min_pulse": low,
                "max_pulse": high,
                "init_pulse": 500,
                "direction": direction,
                "radians_per_tick": TICK,
            }
            for joint, (servo_id, low, high, direction) in BOUNDS.items()
        },
        "safety": {
            "max_step_rad": 0.10,
            "min_duration_s": 1.0,
            "max_duration_s": 10.0,
            "feedback_max_age_ms": 500,
            "tolerance_pulses": 3,
            "settle_samples": 3,
            "timeout_margin_s": 2.0,
            "single_joint_only": True,
        },
    }


def _config():
    return motion_config_from_dict(_raw_config())


def _measured(pulses: dict[str, int] | None = None, age_ms: int = 5) -> dict:
    pulses = pulses or {}
    return {
        joint: {
            "raw_position": pulses.get(joint, 500),
            "sample_age_ms": age_ms,
            "received_at": "2026-11-18T00:00:00Z",
        }
        for joint in ARM_JOINTS
    }


def _target(config, joint: str, pulse: int) -> dict:
    return {
        "joint": joint,
        "servo_id": config.joints[joint].servo_id,
        "target_pulse": pulse,
        "target_rad": config.joints[joint].pulse_to_rad(pulse),
    }


def _payload(command_id: str, joint: str = "joint1", pulse: int = 520) -> dict:
    config = _config()
    return {
        "command_id": command_id,
        "duration_s": 1.0,
        "joints": [_target(config, joint, pulse)],
    }


def test_valid_command_is_quantized_and_tracked() -> None:
    config = _config()
    guard = MotionGuard(config)
    plan = guard.validate(
        _payload("cmd_ok"),
        _measured(),
        10.0,
    )
    assert plan.command_id == "cmd_ok"
    assert plan.targets[0].target_pulse == 520
    assert plan.deadline == pytest.approx(13.0)
    guard.start(plan, _measured())
    status = guard.status()
    assert status["state"] == "executing"
    assert status["command_id"] == "cmd_ok"

    for second in (10.1, 10.2, 10.3):
        guard.observe(_measured({"joint1": 520}), second)
    status = guard.status()
    assert status["state"] == "completed"
    assert status["command_id"] == "cmd_ok"
    assert status["measured"]["joint1"]["raw_position"] == 520


def test_out_of_limit_pulse_is_rejected() -> None:
    guard = MotionGuard(_config())
    with pytest.raises(MotionRejected) as caught:
        guard.validate(_payload("cmd_oob", pulse=940), _measured(), 0.0)
    assert caught.value.code == "MOTION_OUT_OF_LIMIT"
    assert caught.value.http_status == 400


def test_step_beyond_supervised_cap_is_rejected() -> None:
    guard = MotionGuard(_config())
    # 24 pulses > floor(0.10 / 0.00418879) = 23 pulses.
    with pytest.raises(MotionRejected) as caught:
        guard.validate(_payload("cmd_step", pulse=524), _measured(), 0.0)
    assert caught.value.code == "MOTION_STEP_EXCEEDED"
    assert caught.value.http_status == 409


def test_stale_measured_feedback_is_rejected() -> None:
    guard = MotionGuard(_config())
    with pytest.raises(MotionRejected) as caught:
        guard.validate(
            _payload("cmd_stale"),
            _measured(age_ms=600),
            0.0,
        )
    assert caught.value.code == "MOTION_STALE_FEEDBACK"
    assert caught.value.http_status == 409


def test_missing_measured_feedback_is_rejected() -> None:
    guard = MotionGuard(_config())
    with pytest.raises(MotionRejected) as caught:
        guard.validate(_payload("cmd_missing"), {}, 0.0)
    assert caught.value.code == "MOTION_STALE_FEEDBACK"


def test_multi_joint_is_rejected_in_supervised_mode() -> None:
    config = _config()
    guard = MotionGuard(config)
    payload = {
        "command_id": "cmd_multi",
        "duration_s": 1.0,
        "joints": [
            _target(config, "joint1", 510),
            _target(config, "joint2", 510),
        ],
    }
    with pytest.raises(MotionRejected) as caught:
        guard.validate(payload, _measured(), 0.0)
    assert caught.value.code == "MOTION_BAD_REQUEST"


def test_joint3_uses_inverted_pulse_mapping() -> None:
    config = _config()
    limit = config.joints["joint3"]
    assert limit.pulse_to_rad(500) == pytest.approx(0.0)
    assert limit.pulse_to_rad(600) == pytest.approx(-600 * TICK + 500 * TICK)
    assert limit.rad_to_pulse(-0.2) > 500


def test_busy_until_the_active_command_finishes() -> None:
    guard = MotionGuard(_config())
    plan = guard.validate(_payload("cmd_first"), _measured(), 0.0)
    guard.start(plan, _measured())
    with pytest.raises(MotionRejected) as caught:
        guard.validate(_payload("cmd_second", pulse=510), _measured(), 0.1)
    assert caught.value.code == "MOTION_BUSY"
    assert caught.value.http_status == 409


def test_unknown_after_deadline() -> None:
    guard = MotionGuard(_config())
    plan = guard.validate(_payload("cmd_late"), _measured(), 0.0)
    guard.start(plan, _measured())
    guard.observe(_measured({"joint1": 500}), 3.5)
    status = guard.status()
    assert status["state"] == "unknown"
    assert status["command_id"] == "cmd_late"


def test_stop_marks_the_active_command_stopped() -> None:
    guard = MotionGuard(_config())
    plan = guard.validate(_payload("cmd_stop"), _measured(), 0.0)
    guard.start(plan, _measured())
    status = guard.mark_stopped(_measured({"joint1": 512}), 0.4)
    assert status["state"] == "stopped"
    assert status["last_stop"]["detail"].startswith("hold command published")


def test_config_rejects_servo_id_mismatch() -> None:
    raw = _raw_config()
    raw["joints"]["joint1"]["servo_id"] = 5
    with pytest.raises(MotionConfigError):
        motion_config_from_dict(raw)


def test_config_rejects_inverted_pulse_range() -> None:
    raw = _raw_config()
    raw["joints"]["joint3"]["min_pulse"] = 930
    raw["joints"]["joint3"]["max_pulse"] = 500
    with pytest.raises(MotionConfigError):
        motion_config_from_dict(raw)


def test_config_rejects_oversized_step_cap() -> None:
    raw = _raw_config()
    raw["safety"]["max_step_rad"] = 0.5
    with pytest.raises(MotionConfigError):
        motion_config_from_dict(raw)
