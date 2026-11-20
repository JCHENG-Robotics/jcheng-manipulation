"""Hardware adapter for the read-only gateway and supervised motion path.

When `ARMPI_GATEWAY_URL` is configured, this adapter reads live battery, camera,
and joint state from the on-robot `armpi_debug_gateway`. It never opens a
serial port and never talks to the vendor driver directly.

Motion is disabled unless the backend was started with
``ARMPI_HARDWARE_MOTION=1``. For unverified limits that flag is only accepted
together with the fixed ``ARMPI_SUPERVISED_MOTION_ACK`` phrase; the adapter then
enforces single-joint commands, a smaller step and a longer duration floor, and
the gateway independently re-validates the command against measured pulses.
"""

from __future__ import annotations

import time
from dataclasses import replace
from typing import Any, Callable

from ..domain.enums import (
    BatteryStatus,
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
from ..domain.motion import EffectiveMotionLimits, effective_motion_limits
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
        motion_enabled: bool = False,
        supervised: bool = False,
    ) -> None:
        self.mapping = mapping
        self._clock = clock or time.monotonic
        self._gateway = (
            GatewayClient(gateway_url, token=gateway_token) if gateway_url else None
        )
        self.motion_enabled = bool(motion_enabled and gateway_url)
        self.supervised = bool(supervised and self.motion_enabled)
        self.motion_limits: EffectiveMotionLimits = effective_motion_limits(
            mapping, supervised=self.supervised
        )
        self.capabilities = Capabilities(
            hardware_motion=self.motion_enabled,
            hardware_read=bool(gateway_url),
            mock_faults=False,
            replay=True,
            min_duration_s=self.motion_limits.min_duration_s,
            max_duration_s=self.motion_limits.max_duration_s,
            max_step_rad=self.motion_limits.max_step_rad,
            supervised_motion=self.supervised,
            single_joint_only=self.motion_limits.single_joint_only,
            motion_mode=(
                self.motion_limits.mode_label
                if self.motion_enabled
                else "read-only"
            ),
        )
        self._health_cache: dict[str, Any] | None = None
        self._health_at = -1e9
        self._state_cache: dict[str, Any] | None = None
        self._state_at = -1e9
        self._tf_cache: dict[str, Any] | None = None
        self._tf_at = -1e9
        self._motion_status_at = -1e9
        self._motion_status_cache: dict[str, Any] | None = None
        self._active: dict[str, Any] | None = None
        self._events: list[dict[str, Any]] = []
        self._motion_latched: Fault | None = None
        self._last_command_id: str | None = None
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

    def _camera_unavailable(
        self, note: str, *, topic: str = "/depth_cam/rgb/image_raw"
    ) -> CameraStatus:
        return CameraStatus(
            available=False,
            truth=Truth.UNKNOWN,
            validity=Validity.UNAVAILABLE,
            source="hardware_gateway",
            topic=topic,
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

    def _camera_from_gateway(
        self, path: str, *, default_topic: str, label: str
    ) -> CameraStatus:
        if self._gateway is None:
            return self._camera_unavailable(
                "ARMPI_GATEWAY_URL is not configured", topic=default_topic
            )
        try:
            data = self._gateway.get_json(path)
        except GatewayError as exc:
            return self._camera_unavailable(str(exc), topic=default_topic)
        try:
            available = bool(data.get("available", False))
            return CameraStatus(
                available=available,
                truth=Truth(data.get("truth", "measured" if available else "unknown")),
                validity=Validity(data.get("validity", "valid" if available else "unavailable")),
                source=str(data.get("source", "armpi_debug_gateway")),
                topic=str(data.get("topic", default_topic)),
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
            return self._camera_unavailable(
                f"invalid gateway {label} payload: {exc}", topic=default_topic
            )

    def camera_status(self) -> CameraStatus:
        return self._camera_from_gateway(
            "/camera/status",
            default_topic="/depth_cam/rgb/image_raw",
            label="camera",
        )

    def camera_snapshot(self) -> tuple[bytes, str] | None:
        if self._gateway is None:
            return None
        try:
            return self._gateway.get_bytes("/camera/snapshot")
        except GatewayError:
            return None

    def aux_camera_status(self) -> CameraStatus:
        return self._camera_from_gateway(
            "/aux-camera/status", default_topic="uvc:/dev/video4", label="aux camera"
        )

    def aux_camera_snapshot(self) -> tuple[bytes, str] | None:
        if self._gateway is None:
            return None
        try:
            return self._gateway.get_bytes("/aux-camera/snapshot")
        except GatewayError:
            return None

    def tf_snapshot(self) -> dict[str, Any] | None:
        if self._gateway is None:
            return None
        now = self._clock()
        if self._tf_cache is not None and now - self._tf_at < 0.5:
            return self._tf_cache
        try:
            data = self._gateway.get_json("/tf")
        except GatewayError:
            data = None
        self._tf_cache = data
        self._tf_at = now
        return data

    # ------------------------------------------------------------------ state
    def tick(self) -> None:
        return None

    def _unavailable_joint(self, name: str, source: str) -> JointState:
        joint = self.mapping.joints[name]
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
            source=source,
            limit_min_rad=joint.min_rad,
            limit_max_rad=joint.max_rad,
            limit_verified=joint.verified,
        )

    def _joint_from_gateway(
        self,
        name: str,
        item: dict[str, Any] | None,
        measured: dict[str, Any] | None = None,
    ) -> JointState:
        """Build a joint from the commanded echo and measured servo position.

        The measured stream is the only valid feedback for motion. When it is
        absent the joint falls back to the commanded echo and is labelled
        ``commanded``; the safety gate then blocks hardware motion.
        """
        joint = self.mapping.joints[name]
        if not isinstance(item, dict) or item.get("position_rad") is None:
            if not isinstance(measured, dict) or measured.get("position_rad") is None:
                return self._unavailable_joint(name, "hardware_gateway_unavailable")
            item = None
        try:
            if isinstance(measured, dict) and measured.get("position_rad") is not None:
                position = float(measured["position_rad"])
                target = (
                    float(item["target_rad"])
                    if isinstance(item, dict) and item.get("target_rad") is not None
                    else position
                )
                raw = (
                    int(measured["raw_position"])
                    if measured.get("raw_position") is not None
                    else int(round(joint.rad_to_raw(position)))
                )
                truth = Truth.MEASURED
                validity = Validity(measured.get("validity", "valid"))
                age = self._optional_int(measured.get("sample_age_ms"))
                source = str(measured.get("source", "gateway:/state#measured"))
            else:
                assert isinstance(item, dict)
                position = float(item["position_rad"])
                target = (
                    float(item["target_rad"])
                    if item.get("target_rad") is not None
                    else position
                )
                raw = (
                    int(item["raw_position"])
                    if item.get("raw_position") is not None
                    else int(round(joint.rad_to_raw(position)))
                )
                truth = Truth(item.get("truth", "commanded"))
                validity = Validity(item.get("validity", "valid"))
                age = self._optional_int(item.get("sample_age_ms"))
                source = str(item.get("source", "gateway:/state"))
            return JointState(
                name=name,
                servo_id=joint.servo_id,
                target_rad=target,
                target_raw=int(round(joint.rad_to_raw(target))),
                position_rad=position,
                raw_position=raw,
                truth=truth,
                validity=validity,
                sample_age_ms=age,
                source=source,
                limit_min_rad=joint.min_rad,
                limit_max_rad=joint.max_rad,
                limit_verified=joint.verified,
            )
        except (TypeError, ValueError, KeyError):
            return self._unavailable_joint(name, "hardware_gateway_invalid_payload")

    def _motion_ready(self) -> bool:
        if not self.motion_enabled:
            return False
        health = self._health()
        return bool(health and health.get("motion_enabled"))

    def _motion_disabled_fault(self) -> Fault:
        now = utc_now_iso()
        return Fault(
            fault_id="gateway-motion-disabled",
            level=FaultLevel.FAULT,
            code=FaultCode.GATEWAY_MOTION_DISABLED.value,
            message=(
                "backend hardware motion is enabled but the gateway reports "
                "motion_enabled=false"
            ),
            source="hardware_adapter",
            first_seen=now,
            last_seen=now,
            remediation=(
                "start the supervised gateway instance with --enable-motion "
                "and a motion limits file, or disable ARMPI_HARDWARE_MOTION"
            ),
        )

    def snapshot(self) -> RobotState:
        now_monotonic = self._clock()
        self._poll_motion(now_monotonic)
        health = self._health()
        online = health is not None
        joints_payload: dict[str, Any] = {}
        measured_payload: dict[str, Any] = {}
        state_payload: dict[str, Any] = {}
        if online:
            state_payload = self._state_payload() or {}
            raw_joints = state_payload.get("joints")
            if isinstance(raw_joints, dict):
                joints_payload = raw_joints
            raw_measured = state_payload.get("measured")
            if isinstance(raw_measured, dict):
                measured_payload = raw_measured

        faults: list[Fault] = []
        if self.motion_enabled:
            if online and not self._motion_ready():
                faults.append(self._motion_disabled_fault())
        else:
            self._not_implemented.last_seen = utc_now_iso()
            faults.append(self._not_implemented)
        if self._motion_latched is not None:
            self._motion_latched.last_seen = utc_now_iso()
            faults.append(self._motion_latched)
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

        capabilities = replace(
            self.capabilities,
            hardware_motion=self._motion_ready(),
        )
        return RobotState(
            robot_id="armpi-ultra-01",
            mode=self.mode,
            connection_state=(
                ConnectionState.EXECUTING
                if self._active is not None
                else (
                    ConnectionState.READ_ONLY
                    if online
                    else ConnectionState.DISCONNECTED
                )
            ),
            source_label=self.source_label,
            control_owner=None,
            received_at=utc_now_iso(),
            monotonic_ms=now_monotonic * 1000.0,
            ros_time_ns=(
                self._optional_int(state_payload.get("ros_time_ns"))
                if online
                else None
            ),
            joints=[
                self._joint_from_gateway(
                    name,
                    joints_payload.get(name),
                    measured_payload.get(name),
                )
                for name in self.mapping.joints
            ],
            faults=faults,
            mapping_version=self.mapping.mapping_version,
            limit_version=self.mapping.verification_status,
            mapping_verified=self.mapping.verified,
            capabilities=capabilities,
            stop_state=None,
            last_command_id=self._active["command_id"] if self._active else self._last_command_id,
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
        measured_ready = bool(health.get("measured_ready")) if health else False
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
            feedback_fresh=measured_ready if self.motion_enabled else joints_ready,
            notes=(
                [
                    "Supervised gateway motion is enabled for this backend instance.",
                    "joint_states is a command echo; servo_states is the measured position source.",
                ]
                if self.motion_enabled
                else [
                    "Gateway is read-only; motion endpoints answer MOTION_DISABLED.",
                    "joint_states is a command echo, not encoder feedback.",
                ]
            ),
        )

    # ------------------------------------------------------------------ motion
    def _event(
        self,
        stage: CommandStage,
        command_id: str | None,
        result: str,
        error_code: str | None = None,
        details: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        return {
            "stage": stage.value,
            "command_id": command_id,
            "source": "hardware_adapter",
            "result": result,
            "error_code": error_code,
            "details": details or {},
            "monotonic_ms": self._clock() * 1000.0,
        }

    def _finish_unknown(
        self,
        reason: str,
        *,
        error_code: str | None = None,
        details: dict[str, Any] | None = None,
    ) -> None:
        """Record an unconfirmable motion and latch further motion.

        The arm may still be moving when the gateway stops answering, so the
        adapter never guesses ``Completed``. Recovery requires an operator
        inspection and a backend restart; the latch is intentionally not
        cleared by a normal stop.
        """
        active = self._active
        command_id = active["command_id"] if active else self._last_command_id
        payload = dict(details or {})
        if self._motion_status_cache:
            payload.setdefault("last_gateway_status", self._motion_status_cache)
        self._events.append(
            self._event(
                CommandStage.UNKNOWN,
                command_id,
                reason,
                error_code or ErrorCode.MOTION_OUTCOME_UNKNOWN.value,
                payload,
            )
        )
        now = utc_now_iso()
        self._motion_latched = Fault(
            fault_id="motion-outcome-unknown",
            level=FaultLevel.FAULT,
            code=FaultCode.MOTION_OUTCOME_UNKNOWN.value,
            message=(
                "a hardware motion outcome could not be confirmed; motion is "
                "latched until the backend is restarted after inspection"
            ),
            source="hardware_adapter",
            first_seen=now,
            last_seen=now,
            remediation=(
                "check the arm and the vendor stack, then restart the backend; "
                "do not retry the command"
            ),
            details={"command_id": command_id, "reason": reason},
        )
        self._active = None

    def _poll_motion(self, now: float) -> None:
        active = self._active
        if active is None or self._gateway is None:
            return
        if self._motion_status_at >= 0 and now - self._motion_status_at < 0.1:
            return
        try:
            status = self._gateway.get_json("/motion/status")
        except GatewayError as exc:
            self._finish_unknown(
                f"cannot confirm motion outcome: {exc}",
                error_code=ErrorCode.GATEWAY_UNAVAILABLE.value,
            )
            return
        self._motion_status_cache = status
        self._motion_status_at = now
        state = str(status.get("state") or "unknown")
        status_command = status.get("command_id")
        if status_command and status_command != active["command_id"]:
            self._finish_unknown(
                "gateway reports a different active command",
                details={"gateway_command_id": status_command},
            )
            return
        if state == "completed":
            measured = status.get("measured") or {}
            self._events.append(
                self._event(
                    CommandStage.OBSERVED,
                    active["command_id"],
                    "measured servo positions reached the target band",
                    None,
                    {"measured": measured, "targets": active["targets"]},
                )
            )
            self._events.append(
                self._event(
                    CommandStage.COMPLETED,
                    active["command_id"],
                    "gateway confirmed measured positions within tolerance",
                    None,
                    {
                        "measured": measured,
                        "duration_s": active["duration_s"],
                        "tolerance_pulses": status.get("tolerance_pulses"),
                    },
                )
            )
            self._active = None
            self._last_command_id = active["command_id"]
            return
        if state == "stopped":
            measured = status.get("measured") or {}
            self._events.append(
                self._event(
                    CommandStage.OBSERVED,
                    active["command_id"],
                    "motion interrupted by a hold command; target not reached",
                    None,
                    {"measured": measured, "stopped": True},
                )
            )
            self._active = None
            self._last_command_id = active["command_id"]
            return
        if state in {"unknown", "rejected", "disabled"}:
            self._finish_unknown(
                f"gateway reports motion state {state!r}",
                details={"gateway": status},
            )
            return
        if now > active["deadline"]:
            self._finish_unknown(
                "motion did not confirm within duration + margin",
                details={
                    "duration_s": active["duration_s"],
                    "deadline_margin_s": active["deadline_margin_s"],
                    "gateway": status,
                },
            )

    def dispatch(self, targets: list[dict[str, Any]], command_id: str) -> DispatchResult:
        if not self.motion_enabled:
            raise HardwareMotionDisabled(
                "Hardware motion is disabled; complete mapping verification and obtain Board approval"
            )
        if not self._motion_ready():
            raise HardwareMotionDisabled(
                "gateway does not advertise motion; start the supervised gateway instance"
            )
        if self._motion_latched is not None:
            return DispatchResult(
                CommandStage.FAILED,
                ErrorCode.MOTION_OUTCOME_UNKNOWN.value,
                "a previous motion outcome is unconfirmed; restart the backend after inspection",
            )
        if self._active is not None:
            return DispatchResult(
                CommandStage.FAILED,
                ErrorCode.ALREADY_EXECUTING.value,
                "a hardware motion is already executing",
            )
        if not targets:
            return DispatchResult(
                CommandStage.FAILED,
                ErrorCode.NO_TARGETS.value,
                "no targets to dispatch",
            )
        if self.motion_limits.single_joint_only and len(targets) != 1:
            return DispatchResult(
                CommandStage.FAILED,
                ErrorCode.SUPERVISED_SINGLE_JOINT.value,
                "supervised hardware motion accepts exactly one joint",
            )

        durations = {round(float(item["duration_s"]), 6) for item in targets}
        if len(durations) != 1:
            return DispatchResult(
                CommandStage.FAILED,
                ErrorCode.BAD_REQUEST.value,
                "hardware dispatch uses a single duration for every joint",
                {"durations_s": sorted(durations)},
            )
        duration = durations.pop()
        if duration < self.motion_limits.min_duration_s - 1e-9:
            return DispatchResult(
                CommandStage.FAILED,
                ErrorCode.DURATION_RANGE.value,
                f"duration {duration} s is below the motion floor "
                f"{self.motion_limits.min_duration_s} s",
            )

        joints = []
        for item in targets:
            name = str(item["joint"])
            joint = self.mapping.joints.get(name)
            if joint is None:
                return DispatchResult(
                    CommandStage.FAILED,
                    ErrorCode.UNKNOWN_JOINT.value,
                    f"unknown joint: {name}",
                )
            target_rad = float(item["target_rad"])
            target_raw = int(item["target_raw"])
            expected_raw = joint.rad_to_raw(target_rad)
            if abs(expected_raw - target_raw) > 1.0:
                return DispatchResult(
                    CommandStage.FAILED,
                    ErrorCode.BAD_REQUEST.value,
                    f"{name}: target_raw {target_raw} disagrees with "
                    f"rad_to_raw({target_rad:.6f}) = {expected_raw:.3f}",
                )
            low, high = sorted((joint.min_raw, joint.max_raw))
            if not low <= target_raw <= high:
                return DispatchResult(
                    CommandStage.FAILED,
                    ErrorCode.OUT_OF_LIMIT.value,
                    f"{name}: pulse {target_raw} outside [{low}, {high}]",
                )
            joints.append(
                {
                    "joint": name,
                    "servo_id": joint.servo_id,
                    "target_pulse": target_raw,
                    "target_rad": target_rad,
                }
            )

        payload = {
            "command_id": command_id,
            "client": "armpi-debug-studio-backend",
            "supervised": self.supervised,
            "duration_s": duration,
            "joints": joints,
        }
        if self._gateway is None:  # pressed by the motion_enabled guard above
            raise HardwareMotionDisabled("no gateway client configured")
        try:
            response = self._gateway.post_json("/motion/dispatch", payload)
        except GatewayError as exc:
            if exc.status is not None and 400 <= exc.status < 500:
                return DispatchResult(
                    CommandStage.FAILED,
                    exc.code or ErrorCode.BAD_REQUEST.value,
                    str(exc),
                    {"gateway_status": exc.status, "gateway_code": exc.code},
                )
            # The POST may have reached the gateway before the connection
            # failed; never report Failed and never retry.
            return DispatchResult(
                CommandStage.UNKNOWN,
                exc.code or ErrorCode.GATEWAY_UNAVAILABLE.value,
                f"motion dispatch outcome is unknown: {exc}",
                {"gateway_status": exc.status, "gateway_code": exc.code},
            )

        now = self._clock()
        deadline_margin_s = 2.0
        self._active = {
            "command_id": command_id,
            "targets": joints,
            "duration_s": duration,
            "started_at": now,
            "deadline": now + duration + deadline_margin_s,
            "deadline_margin_s": deadline_margin_s,
            "stop_requested": False,
        }
        self._last_command_id = command_id
        return DispatchResult(
            CommandStage.DISPATCHED,
            None,
            f"gateway published {len(joints)} joint target(s) for {duration} s",
            {"gateway": response},
        )

    def stop(self, command_id: str) -> StopResult:
        if not self.motion_enabled:
            return StopResult(
                state=StopState.STOP_UNKNOWN,
                command_id=command_id,
                detail="hardware gateway is read-only; stop is not implemented",
                evidence="no controller acknowledgement is available",
                requested_at=utc_now_iso(),
            )
        if self._gateway is None:
            return StopResult(
                state=StopState.STOP_UNKNOWN,
                command_id=command_id,
                detail="no gateway client configured",
                evidence="no request was sent",
                requested_at=utc_now_iso(),
            )
        try:
            response = self._gateway.post_json(
                "/motion/stop",
                {
                    "command_id": command_id,
                    "client": "armpi-debug-studio-backend",
                },
            )
        except GatewayError as exc:
            return StopResult(
                state=StopState.STOP_UNKNOWN,
                command_id=command_id,
                detail=f"hold command could not be confirmed: {exc}",
                evidence="no gateway acknowledgement",
                requested_at=utc_now_iso(),
            )
        if self._active is not None:
            self._active["stop_requested"] = True
        return StopResult(
            state=StopState.STOP_DISPATCHED,
            command_id=command_id,
            detail=(
                "hold command published at the last measured pulses; "
                "physical stop is not confirmed"
            ),
            evidence=str(response),
            requested_at=utc_now_iso(),
        )

    def drain_events(self) -> list[dict[str, Any]]:
        events, self._events = self._events, []
        return events

    def reset(self) -> None:
        # Never perform a hardware side effect from a mock reset endpoint.
        return None
