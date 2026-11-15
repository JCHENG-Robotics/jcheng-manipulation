"""Safety gate orchestration.

The supervisor is deliberately independent of FastAPI and of the adapter. It
takes a state snapshot and returns a structured report with blocking checks.
"""

from __future__ import annotations

from ..domain.enums import (
    CommandStage,
    ConnectionState,
    ErrorCode,
    FaultLevel,
    Severity,
    SourceLabel,
    StopState,
    Validity,
)
from ..domain.mapping import MappingConfig
from ..domain.models import (
    ConnectionReport,
    RobotState,
    SafetyCheck,
    SafetyReport,
    utc_now_iso,
)
from .lease import LeaseManager


class SafetySupervisor:
    def __init__(self, mapping: MappingConfig, leases: LeaseManager) -> None:
        self.mapping = mapping
        self.leases = leases

    def evaluate(
        self,
        *,
        state: RobotState,
        connection: ConnectionReport,
        lease_id: str | None = None,
        session_id: str | None = None,
        require_lease: bool = True,
        executing: bool = False,
    ) -> SafetyReport:
        checks: list[SafetyCheck] = []

        def add(
            check_id: str,
            label: str,
            passed: bool,
            detail: str,
            *,
            blocking: bool = True,
            severity: Severity = Severity.FAULT,
        ) -> None:
            checks.append(
                SafetyCheck(
                    check_id=check_id,
                    label=label,
                    passed=passed,
                    blocking=blocking,
                    detail=detail,
                    severity=severity,
                )
            )

        add(
            "mode_supported",
            "运行模式",
            True,
            f"mode={state.mode.value}",
            severity=Severity.INFO,
        )
        add(
            "gateway_online",
            "网关在线",
            connection.gateway_online,
            "mock adapter (no ROS process)" if state.mode.value == "mock" else "gateway probe",
        )
        add(
            "driver_online",
            "Driver 在线",
            connection.driver_online,
            "mock driver (no vendor node)" if state.mode.value == "mock" else "driver probe",
        )

        if state.mode.value == "mock":
            feedback_ok = True
            feedback_detail = "simulated data; not hardware feedback"
        else:
            ages = [j.sample_age_ms for j in state.joints if j.sample_age_ms is not None]
            feedback_ok = bool(ages) and max(ages) <= self.mapping.safety.feedback_max_age_ms
            feedback_detail = (
                f"max sample age {max(ages)} ms (limit {self.mapping.safety.feedback_max_age_ms} ms)"
                if ages
                else "no feedback samples"
            )
        add(
            "feedback_fresh",
            "反馈新鲜度",
            feedback_ok,
            feedback_detail,
            blocking=state.mode.value != "mock",
            severity=Severity.WARNING if state.mode.value == "mock" else Severity.FAULT,
        )

        add(
            "mapping_loaded",
            "关节映射",
            bool(self.mapping.mapping_version and self.mapping.joints),
            f"version={self.mapping.mapping_version}",
            severity=Severity.INFO,
        )

        mapping_verified = self.mapping.verified
        add(
            "mapping_verified",
            "限位验证",
            mapping_verified,
            "provisional limits accepted for Mock; blocked for hardware motion",
            blocking=state.mode.value != "mock",
            severity=Severity.WARNING,
        )

        if require_lease:
            lease = self.leases.check(lease_id, session_id)
            add(
                "control_lease",
                "控制权租约",
                lease.ok,
                lease.detail,
                severity=Severity.FAULT,
            )
        else:
            add(
                "control_lease",
                "控制权租约",
                True,
                "not required for preview",
                blocking=False,
                severity=Severity.INFO,
            )

        active = [f for f in state.faults if f.level in (FaultLevel.FAULT, FaultLevel.CRITICAL)]
        add(
            "no_active_fault",
            "活动故障",
            not active,
            "no active fault" if not active else "; ".join(f.code for f in active),
        )

        add(
            "not_executing",
            "执行互锁",
            not executing,
            "idle" if not executing else "motion is already executing",
        )

        hardware_motion = state.capabilities.hardware_motion
        add(
            "hardware_motion_enabled",
            "硬件运动能力",
            state.mode.value == "mock" or hardware_motion,
            "mock only" if state.mode.value == "mock" else "hardware_motion=false (G1 read-only)",
        )

        unavailable = [
            j.name
            for j in state.joints
            if j.validity in (Validity.INVALID, Validity.UNAVAILABLE)
        ]
        add(
            "joints_available",
            "关节数据可用",
            not unavailable,
            "all joints available" if not unavailable else f"unavailable: {', '.join(unavailable)}",
        )

        stop_clear = state.stop_state not in (
            StopState.STOP_REQUESTED,
            StopState.STOP_DISPATCHED,
            StopState.STOP_UNKNOWN,
        )
        add(
            "stop_state_clear",
            "停止状态",
            stop_clear,
            "no stop in progress" if stop_clear else f"stop_state={state.stop_state.value if state.stop_state else None}",
        )

        blockers = [c.check_id for c in checks if not c.passed and c.blocking]
        warnings = [c.check_id for c in checks if not c.passed and not c.blocking]
        return SafetyReport(
            ready=not blockers,
            checks=checks,
            blockers=blockers,
            warnings=warnings,
            mode=state.mode,
            generated_at=utc_now_iso(),
        )
