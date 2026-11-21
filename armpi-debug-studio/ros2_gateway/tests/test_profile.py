"""Gateway checks for the dense S-curve profile payload (no ROS)."""

from __future__ import annotations

import pytest

from armpi_debug_gateway.motion import (
    ARM_JOINTS,
    MotionGuard,
    MotionRejected,
    motion_config_from_dict,
)

TICK = 0.004188790204786391
START = 500
BOUNDS = {
    "joint1": (6, 70, 930, 1),
    "joint2": (5, 342, 810, 1),
    "joint3": (4, 500, 930, -1),
    "joint4": (3, 120, 880, 1),
    "joint5": (2, 120, 870, 1),
}


def _raw_config(*, profile: bool = True) -> dict:
    raw = {
        "schema_version": 1,
        "mapping_version": "test-mapping",
        "radians_per_tick": TICK,
        "mode": "pose",
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
            "max_step_rad": 4.0,
            "min_duration_s": 1.0,
            "max_duration_s": 10.0,
            "feedback_max_age_ms": 500,
            "tolerance_pulses": 3,
            "settle_samples": 3,
            "timeout_margin_s": 3.0,
            "single_joint_only": False,
            "verification": True,
            "verification_settle_s": 0.5,
            "max_velocity_rad_s": 0.5,
        },
        "gripper": {
            "joint": "r_joint",
            "servo_id": 1,
            "init_pulse": 700,
            "min_pulse": 170,
            "max_pulse": 700,
            "speed_raw_per_s": 100,
            "min_duration_s": 1.0,
            "max_duration_s": 6.0,
            "completion": "timed",
        },
    }
    if profile:
        raw["safety"]["profile"] = {
            "max_velocity_rad_s": 4.0,
            "max_accel_rad_s2": 20.0,
            "min_segment_s": 0.025,
            "max_segment_s": 0.5,
            "max_points": 400,
            "max_total_duration_s": 6.0,
            "start_tolerance_pulses": 20,
            "gripper_speed_raw_per_s": 900.0,
            "gripper_min_duration_s": 0.25,
            "gripper_settle_s": 0.6,
        }
    return raw


def _guard(*, profile: bool = True) -> MotionGuard:
    return MotionGuard(motion_config_from_dict(_raw_config(profile=profile)))


def _measured(pulses: dict[str, int] | None = None, age_ms: int = 5) -> dict:
    pulses = pulses or {}
    measured = {
        joint: {
            "raw_position": pulses.get(joint, START),
            "sample_age_ms": age_ms,
            "received_at": "2026-11-21T00:00:00Z",
        }
        for joint in ARM_JOINTS
    }
    measured["r_joint"] = {
        "raw_position": pulses.get("r_joint", 170),
        "sample_age_ms": age_ms,
        "received_at": "2026-11-21T00:00:00Z",
    }
    return measured


def _config(config_guard: MotionGuard, joint: str, pulse: int) -> dict:
    limit = config_guard.config.joints[joint]
    return {
        "joint": joint,
        "servo_id": limit.servo_id,
        "target_pulse": pulse,
        "target_rad": limit.pulse_to_rad(pulse),
    }


def _point(guard: MotionGuard, pulses: dict[str, int], duration: float = 0.05) -> dict:
    return {
        "duration_s": duration,
        "joints": [
            _config(guard, joint, pulses.get(joint, START)) for joint in ARM_JOINTS
        ],
    }


def _profile_payload(
    guard: MotionGuard,
    points: list[dict],
    endpoint: dict[str, int],
    *,
    duration: float | None = None,
    fast: bool = False,
) -> dict:
    total = sum(float(point["duration_s"]) for point in points)
    return {
        "command_id": "cmd_profile",
        "duration_s": total if duration is None else duration,
        "joints": [
            _config(guard, joint, endpoint.get(joint, START))
            for joint in ARM_JOINTS
        ],
        "profile": {"points": points},
        "fast": fast,
    }


def _ramp(guard: MotionGuard, values: list[int]) -> list[dict]:
    return [_point(guard, {"joint1": value}) for value in values]


def test_valid_profile_is_planned_with_the_summed_duration() -> None:
    guard = _guard()
    values = [510, 520, 530, 540, 548]
    payload = _profile_payload(
        guard, _ramp(guard, values), {"joint1": 548}
    )
    plan = guard.validate(payload, _measured(), 10.0)
    assert plan.points is not None
    assert len(plan.points) == 5
    assert plan.duration_s == pytest.approx(0.25)
    assert plan.targets[0].target_pulse == 548
    assert plan.profile is not None
    assert plan.settle_s == pytest.approx(0.5)


def test_profile_total_must_match_the_registered_duration() -> None:
    guard = _guard()
    payload = _profile_payload(
        guard, _ramp(guard, [510, 520]), {"joint1": 520}
    )
    # The guard uses the segment sum, so a forged duration_s cannot shorten
    # the completion deadline.
    plan = guard.validate(payload, _measured(), 0.0)
    assert plan.duration_s == pytest.approx(0.10)


def test_velocity_cap_rejects_a_fast_segment() -> None:
    guard = _guard()
    payload = _profile_payload(
        guard,
        _ramp(guard, [600, 700]),
        {"joint1": 700},
    )
    with pytest.raises(MotionRejected) as caught:
        guard.validate(payload, _measured(), 0.0)
    assert caught.value.code == "MOTION_PROFILE_VELOCITY"


def test_endpoint_must_match_the_last_point() -> None:
    guard = _guard()
    payload = _profile_payload(
        guard, _ramp(guard, [510, 520]), {"joint1": 548}
    )
    with pytest.raises(MotionRejected) as caught:
        guard.validate(payload, _measured(), 0.0)
    assert caught.value.code == "MOTION_BAD_REQUEST"
    assert "last point" in caught.value.args[0]


def test_method_requires_the_profile_block() -> None:
    guard = _guard(profile=False)
    payload = _profile_payload(
        guard, _ramp(guard, [510, 520]), {"joint1": 520}
    )
    with pytest.raises(MotionRejected) as caught:
        guard.validate(payload, _measured(), 0.0)
    assert caught.value.code == "MOTION_PROFILE_DISABLED"


def test_segment_longer_than_the_cap_is_rejected() -> None:
    guard = _guard()
    payload = _profile_payload(
        guard, [_point(guard, {"joint1": 510}, duration=1.0)], {"joint1": 510}
    )
    with pytest.raises(MotionRejected) as caught:
        guard.validate(payload, _measured(), 0.0)
    assert caught.value.code == "MOTION_BAD_REQUEST"


def test_fast_gripper_accepts_the_measured_timings() -> None:
    guard = _guard()
    gripper = guard.config.gripper
    assert gripper is not None
    payload = {
        "command_id": "cmd_close_pre",
        "duration_s": 0.35,
        "fast": True,
        "joints": [
            {
                "joint": "r_joint",
                "servo_id": 1,
                "target_pulse": 318,
                "target_rad": gripper.pulse_to_rad(318),
            }
        ],
    }
    plan = guard.validate(payload, _measured({"r_joint": 170}), 0.0)
    assert plan.fast is True
    assert plan.settle_s == pytest.approx(0.6)
    assert plan.duration_s == pytest.approx(0.35)

    slow = dict(payload)
    slow.pop("fast")
    with pytest.raises(MotionRejected):
        guard.validate(slow, _measured({"r_joint": 170}), 0.0)


def test_profile_cannot_carry_a_gripper_target() -> None:
    guard = _guard()
    gripper = guard.config.gripper
    assert gripper is not None
    payload = {
        "command_id": "cmd_mixed",
        "duration_s": 0.05,
        "joints": [
            {
                "joint": "r_joint",
                "servo_id": 1,
                "target_pulse": 318,
                "target_rad": gripper.pulse_to_rad(318),
            }
        ],
        "profile": {"points": [_point(guard, {"joint1": 510})]},
    }
    with pytest.raises(MotionRejected):
        guard.validate(payload, _measured({"r_joint": 170}), 0.0)
