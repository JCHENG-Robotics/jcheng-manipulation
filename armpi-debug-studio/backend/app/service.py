"""Application service: orchestration, safety gates, audit, and state.

This layer is framework-agnostic. It is called by FastAPI handlers and can be
tested directly with a fake clock.
"""

from __future__ import annotations

import hashlib
import json
import platform
import threading
import time
import uuid
from typing import Any, Callable

from .adapters.base import HardwareMotionDisabled, RobotAdapter
from .config import Settings
from .domain.enums import (
    CommandStage,
    ConnectionState,
    ErrorCode,
    FaultCode,
    FaultLevel,
    Mode,
    SourceLabel,
    StopState,
)
from .domain.mapping import MappingConfig
from .domain.models import (
    CommandRecord,
    Event,
    Fault,
    Health,
    MotionTarget,
    PreviewResult,
    RobotState,
    StopResult,
    jsonable,
    utc_now_iso,
)
from .domain.state_machine import transition
from .errors import ServiceError, StorageError
from .persistence.store import EventStore
from .safety.lease import LeaseConflict, LeaseManager
from .safety.supervisor import SafetySupervisor
from .safety.tokens import PreviewTokenManager
from .safety.validation import validate_targets

IDEMPOTENCY_TTL_S = 300

KNOWN_DEFECTS = [
    {
        "id": "B-01",
        "summary": "get_torque_state calls a non-existent method",
        "source": "ros_robot_controller_node.py:243",
        "impact": "torque-state diagnostic raises AttributeError",
    },
    {
        "id": "B-02",
        "summary": "button publisher uses an unimported PacketReportKeyEvents",
        "source": "ros_robot_controller_node.py:291",
        "impact": "NameError when the button path is exercised",
    },
    {
        "id": "B-03",
        "summary": "/joint_controller callback parameter type mismatch",
        "source": "controller_manager.py:91-95",
        "impact": "that control entry point is unusable",
    },
    {
        "id": "B-04",
        "summary": "FollowJointTrajectory action sends no feedback",
        "source": "joint_trajectory_action_controller.py",
        "impact": "no progress evidence during trajectory execution",
    },
    {
        "id": "B-05",
        "summary": "trajectory action succeeds without checking actual position",
        "source": "joint_trajectory_action_controller.py:173",
        "impact": "success does not prove the arm reached the target",
    },
    {
        "id": "B-06",
        "summary": "joint_states is the last commanded pulse, not sensor feedback",
        "source": "controller_manager.py:97-114",
        "impact": "must be labelled truth=commanded, not measured",
    },
]


class RobotService:
    def __init__(
        self,
        settings: Settings,
        mapping: MappingConfig,
        adapter: RobotAdapter,
        store: EventStore,
        *,
        clock: Callable[[], float] | None = None,
    ) -> None:
        self.settings = settings
        self.mapping = mapping
        self.adapter = adapter
        self.store = store
        self._clock = clock or time.monotonic
        self.leases = LeaseManager(self._clock)
        self.tokens = PreviewTokenManager(self._clock)
        self.supervisor = SafetySupervisor(mapping, self.leases)
        self._lock = threading.RLock()
        self._storage_error: str | None = None
        self._idempotency: dict[str, dict[str, Any]] = {}

    # ------------------------------------------------------------------ helpers
    def _storage_fault(self) -> Fault:
        now = utc_now_iso()
        return Fault(
            fault_id="storage-write-failed",
            level=FaultLevel.CRITICAL,
            code=FaultCode.STORAGE_WRITE_FAILED.value,
            message="event store write failed; motion is locked out until repaired",
            source="persistence",
            first_seen=now,
            last_seen=now,
            remediation="check DB_PATH permissions/disk and restart the backend",
        )

    def _drain_adapter_events(self) -> None:
        for raw in self.adapter.drain_events():
            event = Event(
                event_id=raw.get("event_id") or f"evt_{uuid.uuid4().hex}",
                command_id=raw.get("command_id"),
                stage=str(raw.get("stage", "Unknown")),
                wall_time=raw.get("wall_time") or utc_now_iso(),
                monotonic_ms=float(raw.get("monotonic_ms", self._clock() * 1000.0)),
                source=str(raw.get("source", "adapter")),
                result=str(raw.get("result", "")),
                error_code=raw.get("error_code"),
                details=raw.get("details") or {},
                schema_version=int(raw.get("schema_version", 1)),
            )
            try:
                self._append(event)
            except ServiceError:
                # _append already latched the storage fault. Never let a
                # completion event crash the state endpoint.
                continue

    def _state(self) -> RobotState:
        state = self.adapter.snapshot()
        self._drain_adapter_events()
        state.control_owner = self.leases.owner()
        if self._storage_error:
            state.faults.append(self._storage_fault())
        return state

    def tick(self) -> None:
        """Advance the adapter and persist adapter-originated events."""
        try:
            self._state()
        except ServiceError:
            return

    def _connection(self):
        return self.adapter.connection_report()

    def _current_targets(self, state: RobotState) -> dict[str, float]:
        return {
            joint.name: (joint.target_rad if joint.target_rad is not None else 0.0)
            for joint in state.joints
        }

    def _event(
        self,
        stage: CommandStage | str,
        command_id: str | None,
        source: str,
        result: str,
        error_code: str | None = None,
        details: dict[str, Any] | None = None,
    ) -> Event:
        return Event(
            event_id=f"evt_{uuid.uuid4().hex}",
            command_id=command_id,
            stage=stage.value if isinstance(stage, CommandStage) else str(stage),
            wall_time=utc_now_iso(),
            monotonic_ms=self._clock() * 1000.0,
            source=source,
            result=result,
            error_code=error_code,
            details=details or {},
        )

    def _append(self, event: Event) -> None:
        try:
            self.store.append_event(event)
        except StorageError as exc:
            self._storage_error = str(exc)
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "event store write failed; motion is locked out",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc

    def _save(self, record: CommandRecord) -> None:
        try:
            self.store.upsert_command(record)
        except StorageError as exc:
            self._storage_error = str(exc)
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "command store write failed; motion is locked out",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc

    def _load_record(self, command_id: str) -> CommandRecord | None:
        try:
            raw = self.store.get_command(command_id)
        except StorageError as exc:
            self._storage_error = str(exc)
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "command store read failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        if raw is None:
            return None
        return CommandRecord(
            command_id=raw["command_id"],
            session_id=raw["session_id"],
            status=CommandStage(raw["status"]),
            targets=raw["targets"],
            created_at=raw["created_at"],
            updated_at=raw["updated_at"],
            error_code=raw["error_code"],
            details=raw["details"],
            stage_history=raw["stage_history"],
        )

    @staticmethod
    def _record_dict(record: CommandRecord) -> dict[str, Any]:
        return jsonable(record)

    def _token_targets(self, normalized: list[dict[str, Any]]) -> list[dict[str, Any]]:
        return [
            {
                "joint": item["joint"],
                "from_rad": item["from_rad"],
                "target_rad": item["target_rad"],
                "duration_s": item["duration_s"],
                "target_raw": item["target_raw"],
            }
            for item in normalized
        ]

    def _preview_payload(
        self, command_id: str, session_id: str, normalized: list[dict[str, Any]]
    ) -> dict[str, Any]:
        return {
            "command_id": command_id,
            "session_id": session_id,
            "mapping_version": self.mapping.mapping_version,
            "mapping_hash": self.mapping.mapping_hash,
            "targets": self._token_targets(normalized),
        }

    @staticmethod
    def _hash(value: Any) -> str:
        canonical = json.dumps(value, sort_keys=True, separators=(",", ":"), default=str)
        return hashlib.sha256(canonical.encode("utf-8")).hexdigest()

    def _idempotency_get(self, key: str, request_hash: str) -> dict[str, Any] | None:
        now = self._clock()
        expired = [k for k, v in self._idempotency.items() if v["expires"] <= now]
        for k in expired:
            self._idempotency.pop(k, None)
        cached = self._idempotency.get(key)
        if cached is None:
            return None
        if cached["request_hash"] != request_hash:
            raise ServiceError(
                ErrorCode.IDEMPOTENCY_CONFLICT.value,
                "Idempotency-Key was reused with a different request",
                status_code=409,
            )
        return cached["response"]

    def _idempotency_put(self, key: str, request_hash: str, response: dict[str, Any]) -> None:
        self._idempotency[key] = {
            "request_hash": request_hash,
            "expires": self._clock() + IDEMPOTENCY_TTL_S,
            "response": response,
        }

    # -------------------------------------------------------------------- reads
    def health(self) -> dict[str, Any]:
        state = self._state()
        return jsonable(
            Health(
                status="ok" if not self._storage_error else "degraded",
                mode=self.settings.mode,
                version=self.settings.api_version,
                schema_version=self.settings.schema_version,
                source_label=state.source_label,
                capabilities=state.capabilities,
            )
        )

    def get_state(self) -> dict[str, Any]:
        return jsonable(self._state())

    def connection_report(self) -> dict[str, Any]:
        return jsonable(self._connection())

    def safety_report(
        self,
        *,
        lease_id: str | None = None,
        session_id: str | None = None,
        require_lease: bool = False,
    ) -> dict[str, Any]:
        state = self._state()
        report = self.supervisor.evaluate(
            state=state,
            connection=self._connection(),
            lease_id=lease_id,
            session_id=session_id,
            require_lease=require_lease,
            executing=state.connection_state is ConnectionState.EXECUTING,
        )
        return jsonable(report)

    def diagnostics(self) -> dict[str, Any]:
        state = self._state()
        connection = self._connection()
        safety = self.supervisor.evaluate(
            state=state,
            connection=connection,
            require_lease=False,
            executing=state.connection_state is ConnectionState.EXECUTING,
        )
        mock_faults = (
            self.adapter.active_faults()
            if self.settings.mode is Mode.MOCK and hasattr(self.adapter, "active_faults")
            else []
        )
        return {
            "mode": self.settings.mode.value,
            "connection": jsonable(connection),
            "mapping": self.mapping.to_public_dict(),
            "safety": jsonable(safety),
            "capabilities": jsonable(state.capabilities),
            "known_defects": KNOWN_DEFECTS,
            "probe_available": self.settings.mode is Mode.MOCK,
            "environment": {
                "robot_id": self.settings.robot_id,
                "backend_version": self.settings.api_version,
                "python": platform.python_version(),
                "mapping_version": self.mapping.mapping_version,
                "mapping_hash": self.mapping.mapping_hash,
                "mapping_verified": self.mapping.verified,
                "limit_version": self.mapping.verification_status,
                "bind_host": self.settings.bind_host,
                "hardware_motion_enabled": self.settings.hardware_motion_enabled,
            },
            "mock_faults": mock_faults,
            "storage_error": self._storage_error,
        }

    def events(
        self,
        *,
        limit: int = 200,
        stage: str | None = None,
        source: str | None = None,
        command_id: str | None = None,
        after_seq: int = 0,
    ) -> dict[str, Any]:
        try:
            items = self.store.list_events(
                limit=limit,
                stage=stage,
                source=source,
                command_id=command_id,
                after_seq=after_seq,
            )
            last_seq = self.store.last_seq()
        except StorageError as exc:
            self._storage_error = str(exc)
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "event store read failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        return {"items": items, "last_seq": last_seq, "schema_version": 1}

    def export_csv(self) -> str:
        try:
            return self.store.export_csv()
        except StorageError as exc:
            self._storage_error = str(exc)
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "event export failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc

    # -------------------------------------------------------------- control ops
    def acquire_lease(self, session_id: str, ttl_s: int | None = None) -> dict[str, Any]:
        ttl = ttl_s or self.mapping.safety.lease_ttl_s
        with self._lock:
            try:
                lease = self.leases.acquire(session_id, ttl)
            except LeaseConflict as exc:
                raise ServiceError(
                    ErrorCode.LEASE_CONFLICT.value,
                    "control lease is held by another session",
                    status_code=409,
                    details={"owner": exc.owner},
                ) from exc
            try:
                self._append(
                    self._event(
                        "LeaseAcquired",
                        None,
                        "api",
                        "control lease acquired",
                        None,
                        {"session_id": session_id, "ttl_s": ttl, "lease_id": lease.lease_id},
                    )
                )
            except ServiceError:
                self.leases.release(lease.lease_id, session_id)
                raise
            return {
                "lease_id": lease.lease_id,
                "session_id": lease.session_id,
                "acquired_at": lease.acquired_at,
                "expires_at": lease.expires_at,
                "ttl_s": lease.ttl_s,
            }

    def heartbeat(self, lease_id: str, session_id: str, ttl_s: int | None = None) -> dict[str, Any]:
        with self._lock:
            check = self.leases.heartbeat(lease_id, session_id, ttl_s)
            if not check.ok:
                raise ServiceError(
                    check.code or ErrorCode.NO_LEASE.value,
                    check.detail,
                    status_code=403,
                )
            self._append(
                self._event(
                    "LeaseHeartbeat",
                    None,
                    "api",
                    "control lease renewed",
                    None,
                    {"session_id": session_id, "lease_id": lease_id},
                )
            )
            assert check.lease is not None
            return {
                "lease_id": check.lease.lease_id,
                "session_id": check.lease.session_id,
                "expires_at": check.lease.expires_at,
                "ttl_s": check.lease.ttl_s,
            }

    def release_lease(self, lease_id: str, session_id: str) -> dict[str, Any]:
        with self._lock:
            check = self.leases.release(lease_id, session_id)
            if not check.ok:
                raise ServiceError(
                    check.code or ErrorCode.LEASE_CONFLICT.value,
                    check.detail,
                    status_code=409,
                )
            storage_error = None
            try:
                self._append(
                    self._event(
                        "LeaseReleased",
                        None,
                        "api",
                        "control lease released",
                        None,
                        {"session_id": session_id, "lease_id": lease_id},
                    )
                )
            except ServiceError as exc:
                storage_error = exc.details
            return {"released": True, "storage_error": storage_error}

    # ------------------------------------------------------------------- motion
    def preview(self, session_id: str, targets: list[MotionTarget]) -> dict[str, Any]:
        with self._lock:
            command_id = f"cmd_{uuid.uuid4().hex[:16]}"
            state = self._state()
            outcome = validate_targets(
                self.mapping,
                targets,
                current_targets=self._current_targets(state),
            )
            self._append(
                self._event(
                    CommandStage.REQUESTED,
                    command_id,
                    "api",
                    "preview requested",
                    None,
                    {
                        "kind": "preview",
                        "session_id": session_id,
                        "targets": [jsonable(t) for t in targets],
                    },
                )
            )

            if not outcome.valid:
                record = CommandRecord(
                    command_id=command_id,
                    session_id=session_id,
                    status=CommandStage.FAILED,
                    targets=outcome.normalized,
                    created_at=utc_now_iso(),
                    updated_at=utc_now_iso(),
                    error_code=outcome.errors[0].code,
                    details={"errors": [jsonable(e) for e in outcome.errors]},
                    stage_history=[
                        {"stage": CommandStage.REQUESTED.value, "at": utc_now_iso()},
                        {"stage": CommandStage.FAILED.value, "at": utc_now_iso()},
                    ],
                )
                self._save(record)
                self._append(
                    self._event(
                        CommandStage.FAILED,
                        command_id,
                        "api",
                        "preview rejected",
                        outcome.errors[0].code,
                        {
                            "errors": [jsonable(e) for e in outcome.errors],
                            "warnings": [jsonable(w) for w in outcome.warnings],
                        },
                    )
                )
                return jsonable(
                    PreviewResult(
                        command_id=command_id,
                        session_id=session_id,
                        valid=False,
                        targets=outcome.normalized,
                        errors=outcome.errors,
                        warnings=outcome.warnings,
                        preview_token=None,
                        expires_at=None,
                        mapping_version=self.mapping.mapping_version,
                        evaluated_at=utc_now_iso(),
                    )
                )

            payload = self._preview_payload(command_id, session_id, outcome.normalized)
            token, expires_at = self.tokens.issue(payload, self.mapping.safety.preview_ttl_s)
            record = CommandRecord(
                command_id=command_id,
                session_id=session_id,
                status=CommandStage.VALIDATED,
                targets=outcome.normalized,
                created_at=utc_now_iso(),
                updated_at=utc_now_iso(),
                details={"warnings": [jsonable(w) for w in outcome.warnings]},
                stage_history=[
                    {"stage": CommandStage.REQUESTED.value, "at": utc_now_iso()},
                    {"stage": CommandStage.VALIDATED.value, "at": utc_now_iso()},
                ],
            )
            self._save(record)
            self._append(
                self._event(
                    CommandStage.VALIDATED,
                    command_id,
                    "api",
                    "preview validated; token issued",
                    None,
                    {
                        "targets": outcome.normalized,
                        "warnings": [jsonable(w) for w in outcome.warnings],
                        "expires_at": expires_at,
                    },
                )
            )
            return jsonable(
                PreviewResult(
                    command_id=command_id,
                    session_id=session_id,
                    valid=True,
                    targets=outcome.normalized,
                    errors=[],
                    warnings=outcome.warnings,
                    preview_token=token,
                    expires_at=expires_at,
                    mapping_version=self.mapping.mapping_version,
                    evaluated_at=utc_now_iso(),
                )
            )

    def execute(
        self,
        *,
        session_id: str,
        lease_id: str,
        preview_token: str,
        command_id: str,
        targets: list[MotionTarget],
        idempotency_key: str,
    ) -> dict[str, Any]:
        if not idempotency_key or not idempotency_key.strip():
            raise ServiceError(
                ErrorCode.BAD_REQUEST.value,
                "Idempotency-Key is required",
                status_code=400,
            )
        request_hash = self._hash(
            {
                "session_id": session_id,
                "lease_id": lease_id,
                "preview_token": preview_token,
                "command_id": command_id,
                "targets": [jsonable(t) for t in targets],
            }
        )
        with self._lock:
            cached = self._idempotency_get(idempotency_key, request_hash)
            if cached is not None:
                return cached

            lease = self.leases.check(lease_id, session_id)
            if not lease.ok:
                raise ServiceError(
                    lease.code or ErrorCode.NO_LEASE.value,
                    lease.detail,
                    status_code=403,
                )

            state = self._state()
            outcome = validate_targets(
                self.mapping,
                targets,
                current_targets=self._current_targets(state),
            )
            if not outcome.valid:
                record = self._load_record(command_id)
                if record is not None and record.status is CommandStage.VALIDATED:
                    transition(
                        record,
                        CommandStage.FAILED,
                        error_code=outcome.errors[0].code,
                        details={"errors": [jsonable(e) for e in outcome.errors]},
                    )
                    self._save(record)
                    self._append(
                        self._event(
                            CommandStage.FAILED,
                            command_id,
                            "api",
                            "execute revalidation rejected",
                            outcome.errors[0].code,
                            {"errors": [jsonable(e) for e in outcome.errors]},
                        )
                    )
                raise ServiceError(
                    outcome.errors[0].code,
                    outcome.errors[0].message,
                    status_code=400,
                    details={"errors": [jsonable(e) for e in outcome.errors]},
                )

            payload = self._preview_payload(command_id, session_id, outcome.normalized)
            token_check = self.tokens.consume(preview_token, payload)
            if not token_check.ok:
                raise ServiceError(
                    token_check.code or ErrorCode.PREVIEW_INVALID.value,
                    token_check.detail,
                    status_code=403,
                )

            record = self._load_record(command_id)
            if record is None:
                raise ServiceError(
                    ErrorCode.PREVIEW_INVALID.value,
                    "command_id does not match a preview",
                    status_code=400,
                )
            if record.status is not CommandStage.VALIDATED:
                raise ServiceError(
                    ErrorCode.PREVIEW_REUSED.value,
                    f"command is already {record.status.value}",
                    status_code=409,
                )
            if record.session_id != session_id:
                raise ServiceError(
                    ErrorCode.LEASE_CONFLICT.value,
                    "command belongs to another session",
                    status_code=403,
                )

            safety = self.supervisor.evaluate(
                state=state,
                connection=self._connection(),
                lease_id=lease_id,
                session_id=session_id,
                require_lease=True,
                executing=state.connection_state is ConnectionState.EXECUTING,
            )
            if not safety.ready:
                self._append(
                    self._event(
                        CommandStage.FAILED,
                        command_id,
                        "safety",
                        "safety gate rejected execution",
                        "GATE_REJECTED",
                        {"blockers": safety.blockers, "checks": jsonable(safety.checks)},
                    )
                )
                raise ServiceError(
                    "GATE_REJECTED",
                    "safety gate rejected execution",
                    status_code=403,
                    details={"blockers": safety.blockers},
                )

            try:
                dispatch = self.adapter.dispatch(outcome.normalized, command_id)
            except HardwareMotionDisabled as exc:
                transition(
                    record,
                    CommandStage.FAILED,
                    error_code=ErrorCode.HARDWARE_MOTION_DISABLED.value,
                )
                self._save(record)
                self._append(
                    self._event(
                        CommandStage.FAILED,
                        command_id,
                        "adapter",
                        "hardware motion disabled",
                        ErrorCode.HARDWARE_MOTION_DISABLED.value,
                    )
                )
                raise ServiceError(
                    ErrorCode.HARDWARE_MOTION_DISABLED.value,
                    str(exc),
                    status_code=503,
                ) from exc

            if dispatch.status is CommandStage.FAILED:
                transition(
                    record,
                    CommandStage.FAILED,
                    error_code=dispatch.error_code,
                    details={"adapter_detail": dispatch.detail},
                )
                self._save(record)
                self._append(
                    self._event(
                        CommandStage.FAILED,
                        command_id,
                        "adapter",
                        dispatch.detail,
                        dispatch.error_code,
                        dispatch.details,
                    )
                )
                raise ServiceError(
                    dispatch.error_code or ErrorCode.INTERNAL_ERROR.value,
                    dispatch.detail,
                    status_code=503,
                )

            transition(
                record,
                CommandStage.DISPATCHED,
                details={
                    "safety": {"ready": safety.ready, "warnings": safety.warnings},
                    "adapter": dispatch.details,
                },
            )
            self._save(record)
            self._append(
                self._event(
                    CommandStage.DISPATCHED,
                    command_id,
                    "adapter",
                    dispatch.detail,
                    None,
                    dispatch.details,
                )
            )

            if dispatch.status is CommandStage.UNKNOWN:
                transition(
                    record,
                    CommandStage.UNKNOWN,
                    error_code=dispatch.error_code,
                    details={"reason": dispatch.detail},
                )
                self._save(record)
                self._append(
                    self._event(
                        CommandStage.UNKNOWN,
                        command_id,
                        "adapter",
                        "command outcome is unknown; no retry",
                        dispatch.error_code,
                        dispatch.details,
                    )
                )

            response = self._record_dict(record)
            self._idempotency_put(idempotency_key, request_hash, response)
            return response

    def stop(self, *, session_id: str, lease_id: str | None = None, command_id: str | None = None) -> dict[str, Any]:
        with self._lock:
            stop_command_id = command_id or f"stop_{uuid.uuid4().hex[:12]}"
            lease_detail = None
            if lease_id:
                lease_check = self.leases.check(lease_id, session_id)
                lease_detail = {"ok": lease_check.ok, "detail": lease_check.detail}
            result: StopResult = self.adapter.stop(stop_command_id)
            storage_error = None
            try:
                self._append(
                    self._event(
                        result.state.value,
                        stop_command_id,
                        "api",
                        result.detail,
                        None,
                        {
                            "session_id": session_id,
                            "lease": lease_detail,
                            "evidence": result.evidence,
                            "stop_state": result.state.value,
                        },
                    )
                )
            except ServiceError as exc:
                storage_error = exc.details
            return {
                **jsonable(result),
                "storage_error": storage_error,
            }

    # ---------------------------------------------------------------- mock ops
    def inject_fault(self, kind: str, joint: str | None = None) -> dict[str, Any]:
        if self.settings.mode is not Mode.MOCK:
            raise ServiceError(
                ErrorCode.NOT_IMPLEMENTED.value,
                "fault injection is only available in mock mode",
                status_code=503,
            )
        inject = getattr(self.adapter, "inject", None)
        if inject is None:
            raise ServiceError(ErrorCode.NOT_IMPLEMENTED.value, "adapter has no fault injection", status_code=503)
        try:
            fault = inject(kind, joint=joint)
        except Exception as exc:  # adapter validation
            raise ServiceError(ErrorCode.BAD_REQUEST.value, str(exc), status_code=400) from exc
        storage_error = None
        try:
            self._append(
                self._event(
                    "FaultInjected",
                    None,
                    "mock_adapter",
                    f"mock fault injected: {kind}",
                    None,
                    {"kind": kind, "joint": joint},
                )
            )
        except ServiceError as exc:
            storage_error = exc.details
        return {**jsonable(fault), "storage_error": storage_error}

    def reset_mock(self) -> dict[str, Any]:
        if self.settings.mode is not Mode.MOCK:
            raise ServiceError(
                ErrorCode.NOT_IMPLEMENTED.value,
                "mock reset is only available in mock mode",
                status_code=503,
            )
        self.adapter.reset()
        storage_error = None
        try:
            self._append(
                self._event("MockReset", None, "mock_adapter", "mock state reset")
            )
        except ServiceError as exc:
            storage_error = exc.details
        return {"reset": True, "storage_error": storage_error}

    def probe(self) -> dict[str, Any]:
        if self.settings.mode is not Mode.MOCK:
            raise ServiceError(
                ErrorCode.NOT_IMPLEMENTED.value,
                "hardware feedback probe is not implemented in G1",
                status_code=503,
            )
        state = self._state()
        results = []
        for index, joint in enumerate(state.joints):
            available = joint.validity.value in {"valid", "stale"}
            results.append(
                {
                    "joint": joint.name,
                    "servo_id": joint.servo_id,
                    "ok": available,
                    "raw_position": joint.raw_position,
                    "position_rad": joint.position_rad,
                    "duration_ms": 3 + index,
                    "truth": joint.truth.value,
                    "error": None if available else joint.validity.value,
                }
            )
        self._append(
            self._event(
                "Probe",
                None,
                "mock_adapter",
                "simulated single-shot probe",
                None,
                {"results": results},
            )
        )
        return {
            "truth": "simulated",
            "note": "Mock probe only; no servo was read.",
            "results": results,
        }

    def close(self) -> None:
        self.store.close()
