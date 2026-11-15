"""Hardware adapter placeholder for G1.

This class deliberately has no rclpy import and no serial access. G1 is a
read-only discovery scope; the ROS 2 gateway (E7) is not implemented yet.
"""

from __future__ import annotations

import time
from typing import Any, Callable

from ..domain.enums import (
    CommandStage,
    ConnectionState,
    ErrorCode,
    FaultCode,
    FaultLevel,
    Mode,
    SourceLabel,
    StopState,
    Truth,
    Validity,
)
from ..domain.mapping import MappingConfig
from ..domain.models import (
    Capabilities,
    ConnectionReport,
    Fault,
    JointState,
    RobotState,
    SegmentStatus,
    StopResult,
    utc_now_iso,
)
from .base import DispatchResult, HardwareMotionDisabled, RobotAdapter


class HardwareAdapter(RobotAdapter):
    mode = Mode.HARDWARE
    source_label = SourceLabel.LIVE

    def __init__(self, mapping: MappingConfig, *, clock: Callable[[], float] | None = None) -> None:
        self.mapping = mapping
        self._clock = clock or time.monotonic
        self.capabilities = Capabilities(
            hardware_motion=False,
            hardware_read=False,
            mock_faults=False,
            replay=False,
            min_duration_s=mapping.safety.min_duration_s,
            max_duration_s=mapping.safety.max_duration_s,
            max_step_rad=mapping.safety.max_step_rad,
        )
        self._not_implemented = Fault(
            fault_id="hardware-gateway-not-implemented",
            level=FaultLevel.WARNING,
            code=FaultCode.HARDWARE_MOTION_DISABLED.value,
            message="G1 hardware gateway is not implemented; motion is disabled",
            source="hardware_adapter",
            first_seen=utc_now_iso(),
            last_seen=utc_now_iso(),
            remediation="Implement E7 read-only gateway; hardware motion requires Board approval",
        )

    def tick(self) -> None:
        return None

    def snapshot(self) -> RobotState:
        joints = [
            JointState(
                name=name,
                servo_id=joint.servo_id,
                target_rad=None,
                target_raw=None,
                position_rad=None,
                raw_position=None,
                truth=Truth.UNKNOWN,
                validity=Validity.UNAVAILABLE,
                sample_age_ms=None,
                source="hardware_gateway_unavailable",
                limit_min_rad=joint.min_rad,
                limit_max_rad=joint.max_rad,
                limit_verified=joint.verified,
            )
            for name, joint in self.mapping.joints.items()
        ]
        self._not_implemented.last_seen = utc_now_iso()
        return RobotState(
            robot_id="armpi-ultra-01",
            mode=self.mode,
            connection_state=ConnectionState.DISCONNECTED,
            source_label=self.source_label,
            control_owner=None,
            received_at=utc_now_iso(),
            monotonic_ms=self._clock() * 1000.0,
            ros_time_ns=None,
            joints=joints,
            faults=[self._not_implemented],
            mapping_version=self.mapping.mapping_version,
            limit_version=self.mapping.verification_status,
            mapping_verified=self.mapping.verified,
            capabilities=self.capabilities,
            stop_state=None,
            last_command_id=None,
        )

    def connection_report(self) -> ConnectionReport:
        segments = [
            SegmentStatus("serial", "unknown", "not probed", "G1 does not open the serial port"),
            SegmentStatus("driver", "unknown", "not probed", "E7 gateway not implemented"),
            SegmentStatus("gateway", "offline", "ROS 2 gateway not implemented", "no rclpy process"),
            SegmentStatus("feedback", "unavailable", "no gateway", "no feedback source"),
        ]
        return ConnectionReport(
            segments=segments,
            serial_available=False,
            driver_online=False,
            gateway_online=False,
            feedback_fresh=False,
            notes=[
                "Hardware Mode is read-only discovery in G1.",
                "No serial port was opened and no ROS node was started.",
            ],
        )

    def dispatch(self, targets: list[dict[str, Any]], command_id: str) -> DispatchResult:
        raise HardwareMotionDisabled(
            "Hardware motion is disabled in G1; implement the read-only gateway first"
        )

    def stop(self, command_id: str) -> StopResult:
        return StopResult(
            state=StopState.STOP_UNKNOWN,
            command_id=command_id,
            detail="hardware gateway is not implemented",
            evidence="no controller acknowledgement is available",
            requested_at=utc_now_iso(),
        )

    def drain_events(self) -> list[dict[str, Any]]:
        return []

    def reset(self) -> None:
        # Never perform a hardware side effect from a mock reset endpoint.
        return None
