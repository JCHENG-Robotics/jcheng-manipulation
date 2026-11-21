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


def test_safety_stop_latches_and_clears() -> None:
    guard = MotionGuard(_config())
    guard.set_safety_stop(True)
    assert guard.status()["safety_stop"] is True
    with pytest.raises(MotionRejected) as caught:
        guard.validate(_payload("cmd_safety"), _measured(), 0.0)
    assert caught.value.code == "SAFETY_STOP"
    assert caught.value.http_status == 409

    guard.set_safety_stop(False)
    assert guard.status()["safety_stop"] is False
    assert guard.validate(_payload("cmd_cleared"), _measured(), 0.0).command_id == (
        "cmd_cleared"
    )


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


# --------------------------------------------------------------- pose (G7)
def _pose_raw_config() -> dict:
    raw = _raw_config()
    raw["mode"] = "pose"
    raw["safety"].update(
        {
            "max_step_rad": 3.61,
            "max_velocity_rad_s": 0.5,
            "single_joint_only": False,
            "timeout_margin_s": 3.0,
        }
    )
    raw["gripper"] = {
        "joint": "r_joint",
        "servo_id": 1,
        "min_pulse": 170,
        "max_pulse": 700,
        "init_pulse": 700,
        "speed_raw_per_s": 120.0,
        "min_duration_s": 1.0,
        "max_duration_s": 6.0,
        "completion": "timed",
    }
    return raw


def _pose_config():
    return motion_config_from_dict(_pose_raw_config())


def _pose_measured(gripper: int = 170) -> dict:
    measured = _measured()
    measured["r_joint"] = {
        "raw_position": gripper,
        "sample_age_ms": 5,
        "received_at": "2026-11-19T00:00:00Z",
    }
    return measured


def _pose_payload(command_id: str, targets: dict[str, int], duration: float = 2.0) -> dict:
    config = _pose_config()
    joints = []
    for joint, pulse in targets.items():
        if joint == "r_joint":
            joints.append(
                {
                    "joint": joint,
                    "servo_id": 1,
                    "target_pulse": pulse,
                    "target_rad": config.gripper.pulse_to_rad(pulse),
                }
            )
        else:
            joints.append(_target(config, joint, pulse))
    return {
        "command_id": command_id,
        "duration_s": duration,
        "joints": joints,
    }


def test_pose_accepts_five_arm_joints() -> None:
    guard = MotionGuard(_pose_config())
    plan = guard.validate(
        _pose_payload(
            "pose-1",
            {
                "joint1": 520,
                "joint2": 500,
                "joint3": 510,
                "joint4": 520,
                "joint5": 500,
            },
        ),
        _pose_measured(),
        0.0,
    )
    assert plan.mode == "pose"
    assert {target.joint for target in plan.targets} == set(ARM_JOINTS)
    assert all(target.completion == "position" for target in plan.targets)


def test_pose_rejects_partial_joint_set() -> None:
    guard = MotionGuard(_pose_config())
    with pytest.raises(MotionRejected) as caught:
        guard.validate(
            _pose_payload("pose-2", {"joint1": 520}), _pose_measured(), 0.0
        )
    assert caught.value.code == "MOTION_BAD_REQUEST"


def test_pose_rejects_velocity_above_the_cap() -> None:
    guard = MotionGuard(_pose_config())
    with pytest.raises(MotionRejected) as caught:
        guard.validate(
            _pose_payload(
                "pose-3",
                {
                    "joint1": 930,
                    "joint2": 600,
                    "joint3": 500,
                    "joint4": 500,
                    "joint5": 500,
                },
                duration=1.0,
            ),
            _pose_measured(),
            0.0,
        )
    assert caught.value.code == "MOTION_VELOCITY_EXCEEDED"


def test_pose_gripper_is_validated_and_timed() -> None:
    guard = MotionGuard(_pose_config())
    plan = guard.validate(
        _pose_payload("grip-1", {"r_joint": 700}, duration=5.0),
        _pose_measured(),
        0.0,
    )
    assert len(plan.targets) == 1
    assert plan.targets[0].joint == "r_joint"
    assert plan.targets[0].completion == "timed"

    guard.start(plan, _pose_measured())
    # With bus verification the command cache does not complete even a timed
    # gripper; the post-interpolation bus read supplies the register.
    for at in (4.0, 4.1, 4.2, 5.1, 5.2, 5.3):
        guard.observe(_pose_measured(), at)
    assert guard.status()["state"] == "executing"
    status = guard.mark_verified(
        {"r_joint": {"raw_position": 645, "sample_age_ms": 0}},
        6.2,
        read_ok=True,
    )
    assert status["state"] == "completed"
    assert status["verification"] == "bus"
    assert status["actual"]["r_joint"]["raw_position"] == 645


def test_pose_gripper_speed_is_capped() -> None:
    guard = MotionGuard(_pose_config())
    with pytest.raises(MotionRejected) as caught:
        guard.validate(
            _pose_payload("grip-2", {"r_joint": 700}, duration=1.0),
            _pose_measured(),
            0.0,
        )
    assert caught.value.code == "MOTION_VELOCITY_EXCEEDED"


def test_pose_rejects_mixed_gripper_and_arm() -> None:
    guard = MotionGuard(_pose_config())
    with pytest.raises(MotionRejected) as caught:
        guard.validate(
            _pose_payload("mixed", {"joint1": 520, "r_joint": 700}),
            _pose_measured(),
            0.0,
        )
    assert caught.value.code == "MOTION_BAD_REQUEST"


def test_pose_config_requires_velocity_and_gripper() -> None:
    raw = _pose_raw_config()
    del raw["safety"]["max_velocity_rad_s"]
    with pytest.raises(MotionConfigError):
        motion_config_from_dict(raw)

    raw = _pose_raw_config()
    del raw["gripper"]
    with pytest.raises(MotionConfigError):
        motion_config_from_dict(raw)


def test_pose_config_rejects_single_joint_only() -> None:
    raw = _pose_raw_config()
    raw["safety"]["single_joint_only"] = True
    with pytest.raises(MotionConfigError):
        motion_config_from_dict(raw)


def test_step_mode_keeps_the_narrow_step_cap() -> None:
    raw = _raw_config()
    raw["mode"] = "step"
    raw["safety"]["max_step_rad"] = 3.61
    with pytest.raises(MotionConfigError):
        motion_config_from_dict(raw)


# --------------------------------------------- command-cache vs bus (2026-11-21)
def _pose_plan(guard: MotionGuard):
    return guard.validate(
        _pose_payload(
            "pose-bus",
            {
                "joint1": 520,
                "joint2": 500,
                "joint3": 510,
                "joint4": 520,
                "joint5": 500,
            },
        ),
        _pose_measured(),
        0.0,
    )


def _actual_for(plan, *, offset: int = 0) -> dict:
    return {
        target.joint: {
            "raw_position": target.target_pulse + offset,
            "sample_age_ms": 0,
            "received_at": "2026-11-21T00:00:00Z",
        }
        for target in plan.targets
    }


def test_pose_is_not_completed_from_the_command_cache() -> None:
    guard = MotionGuard(_pose_config())
    plan = _pose_plan(guard)
    guard.start(plan, _pose_measured())
    cache_at_target = {
        target.joint: {
            "raw_position": target.target_pulse,
            "sample_age_ms": 0,
            "received_at": "2026-11-21T00:00:00Z",
        }
        for target in plan.targets
    }
    for at in (0.2, 0.4, 0.6, 1.0, 2.0):
        guard.observe(cache_at_target, at)
    status = guard.status()
    assert status["state"] == "executing"
    assert status["verification"] == "pending"
    assert status["measured_truth"] == "commanded"


def test_pose_completes_from_a_bus_read_within_tolerance() -> None:
    guard = MotionGuard(_pose_config())
    plan = _pose_plan(guard)
    guard.start(plan, _pose_measured())
    status = guard.mark_verified(_actual_for(plan, offset=8), 2.0, read_ok=True)
    assert status["state"] == "completed"
    assert status["verification"] == "bus"
    assert status["actual"]["joint1"]["raw_position"] == 528


def test_pose_bus_read_outside_tolerance_is_unknown() -> None:
    guard = MotionGuard(_pose_config())
    plan = _pose_plan(guard)
    guard.start(plan, _pose_measured())
    status = guard.mark_verified(_actual_for(plan, offset=20), 2.0, read_ok=True)
    assert status["state"] == "unknown"
    assert status["verification"] == "failed"
    assert "outside" in (status["verification_reason"] or "")


def test_pose_bus_read_failure_is_unknown() -> None:
    guard = MotionGuard(_pose_config())
    plan = _pose_plan(guard)
    guard.start(plan, _pose_measured())
    status = guard.mark_verified(
        {}, 2.0, read_ok=False, reason="service unavailable"
    )
    assert status["state"] == "unknown"
    assert status["verification"] == "failed"
    assert status["verification_reason"] == "service unavailable"


def test_pose_config_requires_bus_verification() -> None:
    raw = _pose_raw_config()
    raw["safety"]["verification"] = False
    with pytest.raises(MotionConfigError):
        motion_config_from_dict(raw)


def test_timed_gripper_records_the_bus_register() -> None:
    guard = MotionGuard(_pose_config())
    plan = guard.validate(
        _pose_payload("grip-bus", {"r_joint": 700}, duration=5.0),
        _pose_measured(),
        0.0,
    )
    guard.start(plan, _pose_measured())
    status = guard.mark_verified(
        {"r_joint": {"raw_position": 645, "sample_age_ms": 0}},
        5.2,
        read_ok=True,
    )
    assert status["state"] == "completed"
    assert status["verification"] == "bus"
    assert status["actual"]["r_joint"]["raw_position"] == 645
