from __future__ import annotations

from dataclasses import replace
from typing import Any

import pytest

from app.adapters.base import HardwareMotionDisabled
from app.adapters.gateway import GatewayError
from app.adapters.hardware import HardwareAdapter
from app.domain.enums import CommandStage, ErrorCode, Mode, StopState, Truth
from app.domain.mapping import MappingConfig
from app.domain.models import MotionTarget
from app.persistence.store import EventStore
from app.safety.validation import validate_targets
from app.service import RobotService


def _rad(mapping: MappingConfig, joint: str, pulse: int) -> float:
    return mapping.joints[joint].raw_to_rad(pulse)


class FakeGateway:
    base_url = "http://fake-gateway"
    token = "token"

    def __init__(self, *, motion_health: bool = True) -> None:
        self.motion_health = motion_health
        self.health = {
            "status": "ok",
            "ros_ready": True,
            "battery_ready": True,
            "camera_ready": True,
            "joints_ready": True,
            "measured_ready": True,
            "motion_enabled": motion_health,
        }
        self.state: dict[str, Any] = {}
        self.motion_status: dict[str, Any] = {
            "state": "idle",
            "command_id": None,
            "tolerance_pulses": 3,
            "measured": {},
        }
        self.posts: list[tuple[str, dict[str, Any]]] = []
        self.fail: GatewayError | None = None

    def get_json(self, path: str) -> dict[str, Any]:
        if path == "/health":
            return dict(self.health)
        if path == "/state":
            return self.state
        if path == "/motion/status":
            return dict(self.motion_status)
        raise GatewayError(f"unexpected GET {path}")

    def post_json(self, path: str, body: dict[str, Any]) -> dict[str, Any]:
        self.posts.append((path, body))
        if self.fail is not None:
            raise self.fail
        if path == "/motion/dispatch":
            return {"status": "dispatched", "command_id": body["command_id"]}
        if path == "/motion/stop":
            return {"status": "stop_dispatched", "command_id": body.get("command_id")}
        raise GatewayError(f"unexpected POST {path}")


def _state(mapping: MappingConfig, *, measured_pulse: int = 500) -> dict[str, Any]:
    joints = {}
    measured = {}
    for name in mapping.joints:
        pulse = measured_pulse if name == "joint1" else 500
        joints[name] = {
            "position_rad": _rad(mapping, name, 500),
            "target_rad": _rad(mapping, name, 500),
            "truth": "commanded",
            "validity": "valid",
            "sample_age_ms": 5,
        }
        measured[name] = {
            "position_rad": _rad(mapping, name, pulse),
            "raw_position": pulse,
            "truth": "measured",
            "validity": "valid",
            "sample_age_ms": 5,
        }
    return {
        "joints": joints,
        "measured": measured,
        "measured_ready": True,
        "ros_time_ns": 1,
    }


def _adapter(
    mapping: MappingConfig,
    clock,
    *,
    supervised: bool = True,
) -> tuple[HardwareAdapter, FakeGateway]:
    adapter = HardwareAdapter(
        mapping,
        clock=clock,
        gateway_url="http://fake-gateway",
        gateway_token="token",
        motion_enabled=True,
        supervised=supervised,
    )
    gateway = FakeGateway()
    gateway.state = _state(mapping)
    adapter._gateway = gateway
    return adapter, gateway


def _one(mapping: MappingConfig, joint: str = "joint1", rad: float = 0.05) -> list[dict]:
    outcome = validate_targets(
        mapping,
        [MotionTarget(joint, rad, 1.0)],
        current_targets={joint: 0.0},
    )
    assert outcome.valid, [issue.code for issue in outcome.errors]
    return outcome.normalized


def test_supervised_capabilities_are_narrower(mapping, clock) -> None:
    adapter, _ = _adapter(mapping, clock)
    caps = adapter.capabilities
    assert caps.hardware_motion is True
    assert caps.supervised_motion is True
    assert caps.single_joint_only is True
    assert caps.max_step_rad < mapping.safety.max_step_rad
    assert caps.min_duration_s >= 1.0
    assert caps.motion_mode == "supervised"


def test_snapshot_prefers_measured_positions(mapping, clock) -> None:
    adapter, _ = _adapter(mapping, clock)
    state = adapter.snapshot()
    joint1 = next(item for item in state.joints if item.name == "joint1")
    assert joint1.truth is Truth.MEASURED
    assert joint1.position_rad == pytest.approx(_rad(mapping, "joint1", 500))
    assert joint1.target_rad == pytest.approx(_rad(mapping, "joint1", 500))
    assert state.capabilities.hardware_motion is True
    assert not any(
        fault.code == "GATEWAY_MOTION_DISABLED" for fault in state.faults
    )


def test_dispatch_then_completed_events(mapping, clock) -> None:
    adapter, gateway = _adapter(mapping, clock)
    targets = _one(mapping)
    result = adapter.dispatch(targets, "cmd_1")
    assert result.status is CommandStage.DISPATCHED
    path, body = gateway.posts[-1]
    assert path == "/motion/dispatch"
    assert body["command_id"] == "cmd_1"
    assert body["supervised"] is True
    assert body["joints"][0]["target_pulse"] == targets[0]["target_raw"]

    assert adapter.snapshot().connection_state.value == "Executing"
    gateway.motion_status = {
        "state": "completed",
        "command_id": "cmd_1",
        "tolerance_pulses": 3,
        "measured": {"joint1": {"raw_position": targets[0]["target_raw"]}},
    }
    clock.advance(0.2)
    adapter.snapshot()
    events = adapter.drain_events()
    stages = [event["stage"] for event in events]
    assert "Observed" in stages
    assert "Completed" in stages
    assert adapter.snapshot().connection_state.value == "ReadOnly"


def test_dispatch_rejection_is_failed(mapping, clock) -> None:
    adapter, gateway = _adapter(mapping, clock)
    gateway.fail = GatewayError(
        "step too large", status=409, code="MOTION_STEP_EXCEEDED"
    )
    result = adapter.dispatch(_one(mapping), "cmd_2")
    assert result.status is CommandStage.FAILED
    assert result.error_code == "MOTION_STEP_EXCEEDED"


def test_dispatch_network_failure_is_unknown(mapping, clock) -> None:
    adapter, gateway = _adapter(mapping, clock)
    gateway.fail = GatewayError("connection reset")
    result = adapter.dispatch(_one(mapping), "cmd_3")
    assert result.status is CommandStage.UNKNOWN
    assert result.error_code == ErrorCode.GATEWAY_UNAVAILABLE.value
    assert adapter._active is None


def test_supervised_multi_joint_is_rejected_by_adapter(mapping, clock) -> None:
    adapter, _ = _adapter(mapping, clock)
    targets = _one(mapping, "joint1") + _one(mapping, "joint2")
    result = adapter.dispatch(targets, "cmd_4")
    assert result.status is CommandStage.FAILED
    assert result.error_code == ErrorCode.SUPERVISED_SINGLE_JOINT.value


def test_unknown_outcome_latches_motion(mapping, clock) -> None:
    adapter, gateway = _adapter(mapping, clock)
    adapter.dispatch(_one(mapping), "cmd_5")
    gateway.motion_status = {"state": "unknown", "command_id": "cmd_5"}
    clock.advance(0.2)
    state = adapter.snapshot()
    assert any(
        fault.code == "MOTION_OUTCOME_UNKNOWN" for fault in state.faults
    )
    result = adapter.dispatch(_one(mapping), "cmd_6")
    assert result.status is CommandStage.FAILED
    assert result.error_code == ErrorCode.MOTION_OUTCOME_UNKNOWN.value


def test_stop_publishes_a_hold_request(mapping, clock) -> None:
    adapter, gateway = _adapter(mapping, clock)
    adapter.dispatch(_one(mapping), "cmd_7")
    result = adapter.stop("cmd_7")
    assert result.state is StopState.STOP_DISPATCHED
    assert "physical stop is not confirmed" in result.detail
    assert gateway.posts[-1][0] == "/motion/stop"


def test_gateway_motion_disabled_blocks_dispatch(mapping, clock) -> None:
    adapter, gateway = _adapter(mapping, clock)
    gateway.health["motion_enabled"] = False
    state = adapter.snapshot()
    assert state.capabilities.hardware_motion is False
    assert any(
        fault.code == "GATEWAY_MOTION_DISABLED" for fault in state.faults
    )
    with pytest.raises(HardwareMotionDisabled):
        adapter.dispatch(_one(mapping), "cmd_8")


def test_supervised_service_executes_through_the_gate(mapping, clock, settings) -> None:
    """The full preview→execute chain passes with the acknowledgement present."""
    hardware = replace(
        settings,
        mode=Mode.HARDWARE,
        hardware_motion_enabled=True,
        supervised_motion=True,
        gateway_url="http://fake-gateway",
        gateway_token="token",
        db_path=settings.db_path.with_name("supervised-motion.db"),
    )
    adapter, gateway = _adapter(mapping, clock)
    store = EventStore(hardware.db_path)
    service = RobotService(hardware, mapping, adapter, store, clock=clock)
    try:
        lease = service.acquire_lease("session-1", 60)
        targets = [MotionTarget("joint1", 0.05, 1.0)]
        preview = service.preview("session-1", targets)
        assert preview["valid"], preview
        response = service.execute(
            session_id="session-1",
            lease_id=lease["lease_id"],
            preview_token=preview["preview_token"],
            command_id=preview["command_id"],
            targets=targets,
            idempotency_key="idem-1",
        )
        assert response["status"] == CommandStage.DISPATCHED.value
        assert gateway.posts[-1][0] == "/motion/dispatch"
        assert gateway.posts[-1][1]["supervised"] is True
    finally:
        service.close()


def test_service_gate_rejects_without_acknowledgement(mapping, clock, settings) -> None:
    """Without the acknowledgement the unverified mapping stays blocking."""
    hardware = replace(
        settings,
        mode=Mode.HARDWARE,
        hardware_motion_enabled=False,
        supervised_motion=False,
        gateway_url="http://fake-gateway",
        gateway_token="token",
        db_path=settings.db_path.with_name("unacked-motion.db"),
    )
    adapter, _ = _adapter(mapping, clock)
    adapter.motion_enabled = False
    adapter.capabilities = replace(adapter.capabilities, hardware_motion=False)
    store = EventStore(hardware.db_path)
    service = RobotService(hardware, mapping, adapter, store, clock=clock)
    try:
        lease = service.acquire_lease("session-2", 60)
        targets = [MotionTarget("joint1", 0.05, 1.0)]
        preview = service.preview("session-2", targets)
        assert preview["valid"], preview
        with pytest.raises(Exception) as caught:
            service.execute(
                session_id="session-2",
                lease_id=lease["lease_id"],
                preview_token=preview["preview_token"],
                command_id=preview["command_id"],
                targets=targets,
                idempotency_key="idem-2",
            )
        assert getattr(caught.value, "code", None) == "GATE_REJECTED"
    finally:
        service.close()


def test_motion_disabled_adapter_stays_read_only(mapping, clock) -> None:
    adapter = HardwareAdapter(
        mapping,
        clock=clock,
        gateway_url="http://fake-gateway",
        gateway_token="token",
        motion_enabled=False,
    )
    gateway = FakeGateway()
    gateway.state = _state(mapping)
    adapter._gateway = gateway
    state = adapter.snapshot()
    assert state.capabilities.hardware_motion is False
    assert any(
        fault.code == "HARDWARE_MOTION_DISABLED" for fault in state.faults
    )
    with pytest.raises(HardwareMotionDisabled):
        adapter.dispatch(_one(mapping), "cmd_9")
