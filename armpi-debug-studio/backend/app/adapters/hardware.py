"""Hardware adapter for the G1 read-only gateway.

When `ARMPI_GATEWAY_URL` is configured, this adapter reads live battery, camera,
and joint-command state from the on-robot `armpi_debug_gateway`. It never opens
a serial port and never writes to ROS. Motion remains disabled in this phase.
"""

from __future__ import annotations

import time
from typing import Any, Callable

from ..domain.enums import (
    BatteryStatus,
    CommandStage,
    ConnectionState,
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
    BatteryState,
    CameraStatus,
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
from .gateway import GatewayClient, GatewayError


class HardwareAdapter(RobotAdapter):
    mode = Mode.HARDWARE
    source_label = SourceLabel.LIVE

    def __init__(
        self,
        mapping: MappingConfig,
        *,
        clock: Callable[[], float] | None = None,
        gateway_url: str | None = None,
        gateway_token: str | None = None,
    ) -> None:
        self.mapping = mapping
        self._clock = clock or time.monotonic
        self._gateway = (
            GatewayClient(gateway_url, token=gateway_token) if gateway_url else None
        )
        self.capabilities = Capabilities(
            hardware_motion=False,
            hardware_read=bool(gateway_url),
            mock_faults=False,
            replay=False,
            min_duration_s=mapping.safety.min_duration_s,
            max_duration_s=mapping.safety.max_duration_s,
            max_step_rad=mapping.safety.max_step_rad,
        )
        self._health_cache: dict[str, Any] | None = None
        self._health_at = -1e9
        self._state_cache: dict[str, Any] | None = None
        self._state_at = -1e9
        self._not_implemented = Fault(
            fault_id="hardware-motion-disabled",
            level=FaultLevel.WARNING,
            code=FaultCode.HARDWARE_MOTION_DISABLED.value,
            message=(
                "Hardware motion is disabled; the gateway is read-only"
                if gateway_url
                else "ARMPI_GATEWAY_URL is not configured; motion is disabled"
            ),
            source="hardware_adapter",
            first_seen=utc_now_iso(),
            last_seen=utc_now_iso(),
            remediation="Complete joint-mapping verification and obtain Board approval before motion",
        )

    # ------------------------------------------------------------------ gateway
    def _health(self) -> dict[str, Any] | None:
        if self._gateway is None:
            return None
        now = self._clock()
        if self._health_at >= 0 and now - self._health_at < 2.0:
            return self._health_cache
        try:
            data: dict[str, Any] | None = self._gateway.get_json("/health")
        except GatewayError:
            data = None
        self._health_cache = data
        self._health_at = now
        return data

    def _state_payload(self) -> dict[str, Any] | None:
        if self._gateway is None:
            return None
        now = self._clock()
        if self._state_at >= 0 and now - self._state_at < 0.2:
            return self._state_cache
        try:
            data: dict[str, Any] | None = self._gateway.get_json("/state")
        except GatewayError:
            data = None
        self._state_cache = data
        self._state_at = now
        return data

    # ------------------------------------------------------------------ reads
    @staticmethod
    def _optional_int(value: Any) -> int | None:
        return int(value) if value is not None else None

    @staticmethod
    def _optional_float(value: Any) -> float | None:
        return float(value) if value is not None else None

    def _battery_unavailable(self, note: str) -> BatteryState:
        return BatteryState(
            available=False,
            voltage_mv=None,
            voltage_v=None,
            percent_estimate=None,
            cell_count=2,
            status=BatteryStatus.UNAVAILABLE,
            truth=Truth.UNKNOWN,
            validity=Validity.UNAVAILABLE,
            source="hardware_gateway",
            topic="/ros_robot_controller/battery",
            received_at=utc_now_iso(),
            sample_age_ms=None,
            notes=[note],
        )

    def _camera_unavailable(self, note: str) -> CameraStatus:
        return CameraStatus(
            available=False,
            truth=Truth.UNKNOWN,
            validity=Validity.UNAVAILABLE,
            source="hardware_gateway",
            topic="/depth_cam/rgb/image_raw",
            width=None,
            height=None,
            encoding=None,
            fps=None,
            last_frame_at=None,
            sample_age_ms=None,
            frame_count=0,
            notes=[note],
        )

    def battery(self) -> BatteryState:
        if self._gateway is None:
            return self._battery_unavailable("ARMPI_GATEWAY_URL is not configured")
        try:
            data = self._gateway.get_json("/battery")
        except GatewayError as exc:
            return self._battery_unavailable(str(exc))
        try:
            available = bool(data.get("available", data.get("voltage_mv") is not None))
            voltage_mv = self._optional_int(data.get("voltage_mv"))
            voltage_v = self._optional_float(data.get("voltage_v"))
            if voltage_v is None and voltage_mv is not None:
                voltage_v = round(voltage_mv / 1000.0, 3)
            status = BatteryStatus(data.get("status", "unavailable"))
            validity = Validity(data.get("validity", "valid" if available else "unavailable"))
            truth = Truth(data.get("truth", "measured" if available else "unknown"))
            return BatteryState(
                available=available,
                voltage_mv=voltage_mv,
                voltage_v=voltage_v,
                percent_estimate=self._optional_float(data.get("percent_estimate")),
                cell_count=int(data.get("cell_count", 2)),
                status=status,
                truth=truth,
                validity=validity,
                source=str(data.get("source", "armpi_debug_gateway")),
                topic=str(data.get("topic", "/ros_robot_controller/battery")),
                received_at=str(data.get("received_at") or utc_now_iso()),
                sample_age_ms=self._optional_int(data.get("sample_age_ms")),
                notes=[str(item) for item in data.get("notes", [])],
            )
        except (TypeError, ValueError, KeyError) as exc:
            return self._battery_unavailable(f"invalid gateway battery payload: {exc}")

    def camera_status(self) -> CameraStatus:
        if self._gateway is None:
            return self._camera_unavailable("ARMPI_GATEWAY_URL is not configured")
        try:
            data = self._gateway.get_json("/camera/status")
        except GatewayError as exc:
            return self._camera_unavailable(str(exc))
        try:
            available = bool(data.get("available", False))
            return CameraStatus(
                available=available,
                truth=Truth(data.get("truth", "measured" if available else "unknown")),
                validity=Validity(data.get("validity", "valid" if available else "unavailable")),
                source=str(data.get("source", "armpi_debug_gateway")),
                topic=str(data.get("topic", "/depth_cam/rgb/image_raw")),
                width=self._optional_int(data.get("width")),
                height=self._optional_int(data.get("height")),
                encoding=str(data["encoding"]) if data.get("encoding") is not None else None,
                fps=self._optional_float(data.get("fps")),
                last_frame_at=str(data["last_frame_at"]) if data.get("last_frame_at") else None,
                sample_age_ms=self._optional_int(data.get("sample_age_ms")),
                frame_count=int(data.get("frame_count", 0)),
                notes=[str(item) for item in data.get("notes", [])],
            )
        except (TypeError, ValueError, KeyError) as exc:
            return self._camera_unavailable(f"invalid gateway camera payload: {exc}")

    def camera_snapshot(self) -> tuple[bytes, str] | None:
        if self._gateway is None:
            return None
        try:
            return self._gateway.get_bytes("/camera/snapshot")
        except GatewayError:
            return None

    # ------------------------------------------------------------------ state
    def tick(self) -> None:
        return None

    def _joint_from_gateway(
        self,
        name: str,
        item: dict[str, Any] | None,
    ) -> JointState:
        joint = self.mapping.joints[name]
        if not isinstance(item, dict) or item.get("position_rad") is None:
            return JointState(
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
        try:
            position = float(item["position_rad"])
            target = float(item["target_rad"]) if item.get("target_rad") is not None else position
            raw = (
                int(item["raw_position"])
                if item.get("raw_position") is not None
                else int(round(joint.rad_to_raw(position)))
            )
            return JointState(
                name=name,
                servo_id=joint.servo_id,
                target_rad=target,
                target_raw=int(round(joint.rad_to_raw(target))),
                position_rad=position,
                raw_position=raw,
                truth=Truth(item.get("truth", "commanded")),
                validity=Validity(item.get("validity", "valid")),
                sample_age_ms=self._optional_int(item.get("sample_age_ms")),
                source=str(item.get("source", "gateway:/state")),
                limit_min_rad=joint.min_rad,
                limit_max_rad=joint.max_rad,
                limit_verified=joint.verified,
            )
        except (TypeError, ValueError, KeyError):
            return JointState(
                name=name,
                servo_id=joint.servo_id,
                target_rad=None,
                target_raw=None,
                position_rad=None,
                raw_position=None,
                truth=Truth.UNKNOWN,
                validity=Validity.UNAVAILABLE,
                sample_age_ms=None,
                source="hardware_gateway_invalid_payload",
                limit_min_rad=joint.min_rad,
                limit_max_rad=joint.max_rad,
                limit_verified=joint.verified,
            )

    def snapshot(self) -> RobotState:
        health = self._health()
        online = health is not None
        joints_payload: dict[str, Any] = {}
        if online:
            state_payload = self._state_payload()
            raw_joints = (state_payload or {}).get("joints")
            if isinstance(raw_joints, dict):
                joints_payload = raw_joints

        faults: list[Fault] = []
        self._not_implemented.last_seen = utc_now_iso()
        faults.append(self._not_implemented)
        if self._gateway is not None and not online:
            now = utc_now_iso()
            faults.append(
                Fault(
                    fault_id="gateway-offline",
                    level=FaultLevel.WARNING,
                    code="GATEWAY_OFFLINE",
                    message=f"gateway not reachable at {self._gateway.base_url}",
                    source="hardware_adapter",
                    first_seen=now,
                    last_seen=now,
                    remediation="Start armpi_debug_gateway on the robot or fix ARMPI_GATEWAY_URL",
                )
            )

        return RobotState(
            robot_id="armpi-ultra-01",
            mode=self.mode,
            connection_state=(
                ConnectionState.READ_ONLY if online else ConnectionState.DISCONNECTED
            ),
            source_label=self.source_label,
            control_owner=None,
            received_at=utc_now_iso(),
            monotonic_ms=self._clock() * 1000.0,
            ros_time_ns=(
                self._optional_int((self._state_payload() or {}).get("ros_time_ns"))
                if online
                else None
            ),
            joints=[
                self._joint_from_gateway(name, joints_payload.get(name))
                for name in self.mapping.joints
            ],
            faults=faults,
            mapping_version=self.mapping.mapping_version,
            limit_version=self.mapping.verification_status,
            mapping_verified=self.mapping.verified,
            capabilities=self.capabilities,
            stop_state=None,
            last_command_id=None,
        )

    def connection_report(self) -> ConnectionReport:
        health = self._health()
        if self._gateway is None:
            segments = [
                SegmentStatus("serial", "unknown", "not probed", "G1 does not open the serial port"),
                SegmentStatus("driver", "unknown", "not probed", "gateway not configured"),
                SegmentStatus("gateway", "offline", "ARMPI_GATEWAY_URL is not configured", "no gateway client"),
                SegmentStatus("feedback", "unavailable", "no gateway", "no feedback source"),
            ]
            return ConnectionReport(
                segments=segments,
                serial_available=False,
                driver_online=False,
                gateway_online=False,
                feedback_fresh=False,
                notes=[
                    "Hardware Mode is read-only in this phase.",
                    "No serial port was opened and no ROS node was started by this backend.",
                ],
            )

        online = health is not None
        ros_ready = bool(health.get("ros_ready")) if health else False
        battery_ready = bool(health.get("battery_ready")) if health else False
        camera_ready = bool(health.get("camera_ready")) if health else False
        joints_ready = bool(health.get("joints_ready")) if health else False
        segments = [
            SegmentStatus("serial", "unknown", "owned by the vendor stack", "gateway is the only serial owner"),
            SegmentStatus(
                "driver",
                "online" if ros_ready else "offline",
                "vendor ROS stack via gateway",
                "GET /health ros_ready",
            ),
            SegmentStatus(
                "gateway",
                "online" if online else "offline",
                f"{self._gateway.base_url}",
                "GET /health",
            ),
            SegmentStatus(
                "feedback",
                "online" if (joints_ready or camera_ready or battery_ready) else "unavailable",
                "battery/camera/joint state via gateway",
                "GET /state, /battery, /camera/status",
            ),
        ]
        return ConnectionReport(
            segments=segments,
            serial_available=False,
            driver_online=ros_ready,
            gateway_online=online,
            feedback_fresh=joints_ready,
            notes=[
                "Gateway is read-only; motion endpoints are NOT_IMPLEMENTED.",
                "joint_states is a command echo, not encoder feedback.",
            ],
        )

    def dispatch(self, targets: list[dict[str, Any]], command_id: str) -> DispatchResult:
        raise HardwareMotionDisabled(
            "Hardware motion is disabled; complete mapping verification and obtain Board approval"
        )

    def stop(self, command_id: str) -> StopResult:
        return StopResult(
            state=StopState.STOP_UNKNOWN,
            command_id=command_id,
            detail="hardware gateway is read-only; stop is not implemented",
            evidence="no controller acknowledgement is available",
            requested_at=utc_now_iso(),
        )

    def drain_events(self) -> list[dict[str, Any]]:
        return []

    def reset(self) -> None:
        # Never perform a hardware side effect from a mock reset endpoint.
        return None
