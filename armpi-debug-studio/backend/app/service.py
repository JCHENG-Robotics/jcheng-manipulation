"""Application service: orchestration, safety gates, audit, and state.

This layer is framework-agnostic. It is called by FastAPI handlers and can be
tested directly with a fake clock.
"""

from __future__ import annotations

import csv
import hashlib
import io
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
    Truth,
)
from .domain.experiments import (
    ANALYSIS_VERSION as EXPERIMENT_ANALYSIS_VERSION,
    DEMO_SIGNATURE,
    FAILURE_CODES,
    METHOD_LABELS,
    ExperimentDataError,
    analyze_experiment,
    build_demo_experiment,
    normalize_experiment,
    normalize_trial,
)
from .domain.mapping import MappingConfig
from .domain.motion import effective_motion_limits
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
from .domain.trajectory import (
    TrajectoryDataError,
    alignment_csv,
    analyze,
    build_demo_dataset,
    prepare_dataset,
    summary,
)
from .domain.visual import (
    VisualModelError,
    compose_tf_frames,
    compute_frames,
    load_urdf,
    load_visual_config,
)
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


def _mimic_source_available(
    model: Any,
    name: str,
    positions: dict[str, float],
    seen: set[str] | None = None,
) -> bool:
    """True when a mimic joint's source chain ends in a real position."""
    seen = seen or set()
    if name in seen:
        return False
    seen.add(name)
    joint = model.joints.get(name)
    if joint is None:
        return False
    if not joint.mimic:
        return name in positions
    return _mimic_source_available(model, joint.mimic, positions, seen)


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
        self.motion_limits = effective_motion_limits(
            mapping, supervised=settings.supervised_motion
        )
        self._lock = threading.RLock()
        self._storage_error: str | None = None
        self._idempotency: dict[str, dict[str, Any]] = {}
        self._visual_bundle: tuple[Any, dict[str, Any]] | None = None
        self._visual_error: list[str] | None = None

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
        """Baseline for step validation.

        For measured hardware feedback the actual position is the only honest
        baseline: two consecutive commands can each be within one step of the
        commanded target while the arm is still far behind. Mock/simulated and
        commanded-only data keep the previous target baseline.
        """
        return {
            joint.name: (
                joint.position_rad
                if joint.truth is Truth.MEASURED and joint.position_rad is not None
                else (joint.target_rad if joint.target_rad is not None else 0.0)
            )
            for joint in state.joints
        }

    def _validate_targets(self, state: RobotState, targets: list[MotionTarget]):
        return validate_targets(
            self.mapping,
            targets,
            current_targets=self._current_targets(state),
            max_step_rad=self.motion_limits.max_step_rad,
            min_duration_s=self.motion_limits.min_duration_s,
            single_joint_only=self.motion_limits.single_joint_only,
        )

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
        # ``from_rad`` is deliberately excluded: it comes from the live state
        # read and can change by small amounts between preview and execute
        # (hardware command echo). Binding it would turn a genuinely safe
        # preview into PREVIEW_PAYLOAD_MISMATCH instead of letting the safety
        # gate evaluate the command against the fresh state.
        return [
            {
                "joint": item["joint"],
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

    def battery(self) -> dict[str, Any]:
        return jsonable(self.adapter.battery())

    def camera_status(self) -> dict[str, Any]:
        return jsonable(self.adapter.camera_status())

    def camera_snapshot(self) -> tuple[bytes, str]:
        return self._snapshot(self.adapter.camera_snapshot(), "camera")

    def aux_camera_status(self) -> dict[str, Any]:
        return jsonable(self.adapter.aux_camera_status())

    def aux_camera_snapshot(self) -> tuple[bytes, str]:
        return self._snapshot(self.adapter.aux_camera_snapshot(), "aux camera")

    def _snapshot(
        self, snapshot: tuple[bytes, str] | None, label: str
    ) -> tuple[bytes, str]:
        if snapshot is None:
            raise ServiceError(
                ErrorCode.NOT_IMPLEMENTED.value,
                f"{label} snapshot is unavailable",
                status_code=503,
                details={
                    "mode": self.settings.mode.value,
                    "hint": "Configure ARMPI_GATEWAY_URL and run armpi_debug_gateway on the robot",
                },
            )
        return snapshot

    def peripherals(self) -> dict[str, Any]:
        return {
            "battery": self.battery(),
            "camera": self.camera_status(),
            "aux_camera": self.aux_camera_status(),
        }

    # ------------------------------------------------------------- visual (G4)
    def _load_visual_bundle(self) -> tuple[Any, dict[str, Any]]:
        if self._visual_bundle is not None:
            return self._visual_bundle
        if self._visual_error is not None:
            raise VisualModelError(self._visual_error)
        config = load_visual_config(self.settings.visual_model_path)
        try:
            model = load_urdf(
                self.settings.urdf_path,
                verified=config["urdf_verified"],
                source_note=config["urdf_note"],
            )
        except VisualModelError as exc:
            self._visual_error = exc.issues
            raise
        self._visual_bundle = (model, config)
        return self._visual_bundle

    @staticmethod
    def _joint_truth(state: RobotState) -> str:
        truths = {
            joint.truth.value
            for joint in state.joints
            if joint.position_rad is not None
        }
        if not truths:
            return "unknown"
        if len(truths) == 1:
            return next(iter(truths))
        return "mixed"

    @staticmethod
    def _max_sample_age(state: RobotState) -> int | None:
        ages = [
            joint.sample_age_ms
            for joint in state.joints
            if joint.sample_age_ms is not None
        ]
        return max(ages) if ages else None

    def _camera_block(
        self, camera: dict[str, Any], config: dict[str, Any]
    ) -> dict[str, Any]:
        camera_config = config["camera"]
        warnings: list[str] = []
        if not camera.get("available"):
            warnings.append("Camera is unavailable; no frame can be shown.")
        if camera.get("truth") == "simulated":
            warnings.append("SIMULATION: the camera snapshot is synthetic.")
        if not camera_config.get("available"):
            warnings.append(
                "Measured optical extrinsic is unavailable; the optical frame is not shown."
            )
        warnings.append(
            f"Image frame is {camera_config.get('body_frame')} (body). "
            f"PnP must use {camera_config.get('frame')}, not the URDF nominal frame."
        )
        return {
            **camera,
            "snapshot_url": "/api/peripherals/camera/snapshot",
            "frame": camera_config.get("body_frame"),
            "optical_frame": camera_config.get("frame"),
            "calibration": camera_config,
            "warnings": warnings,
        }

    def visual_snapshot(self) -> dict[str, Any]:
        """Read-only URDF/FK + camera view metadata; never queries live TF."""
        state = self._state()
        camera = jsonable(self.adapter.camera_status())
        config = load_visual_config(self.settings.visual_model_path)
        camera_block = self._camera_block(camera, config)
        base = {
            "schema_version": 1,
            "generated_at": utc_now_iso(),
            "mode": state.mode.value,
            "source_label": state.source_label.value,
            "robot_id": state.robot_id,
        }
        try:
            model, config = self._load_visual_bundle()
        except VisualModelError as exc:
            return {
                **base,
                "urdf": {
                    "available": False,
                    "path": str(self.settings.urdf_path),
                    "sha256": None,
                    "robot_name": None,
                    "verified": False,
                    "links": 0,
                    "joints": 0,
                    "source_note": config.get("urdf_note", ""),
                },
                "tf": {
                    "status": "invalid",
                    "source": "none",
                    "live": False,
                    "truth": "unavailable",
                    "joint_truth": self._joint_truth(state),
                    "max_sample_age_ms": self._max_sample_age(state),
                    "gateway_status": None,
                    "dynamic_edges": 0,
                    "static_edges": 0,
                    "max_tf_age_ms": None,
                    "frames": [],
                    "camera_frame": None,
                    "warnings": exc.issues,
                },
                "joints": [],
                "camera": camera_block,
                "warnings": exc.issues + config.get("warnings", []) + camera_block["warnings"],
            }

        positions = {
            joint.name: float(joint.position_rad)
            for joint in state.joints
            if joint.position_rad is not None
        }
        result = compute_frames(model, positions, config)
        joint_truth = self._joint_truth(state)
        max_age = self._max_sample_age(state)
        required_arm = set(self.mapping.joint_names())
        missing_arm = sorted(set(result["missing"]) & required_arm)
        warnings: list[str] = []
        warnings.extend(config.get("warnings", []))
        warnings.extend(model.warnings)
        warnings.extend(result["issues"])
        if not model.verified:
            warnings.append(
                "URDF model is provisional: joint limits and meshes are not "
                "hardware-verified."
            )
        if missing_arm:
            warnings.append(
                "Missing arm joint state: " + ", ".join(missing_arm)
            )
        if max_age is not None and max_age > self.mapping.safety.feedback_max_age_ms:
            warnings.append(
                f"Joint state age {max_age} ms exceeds "
                f"{self.mapping.safety.feedback_max_age_ms} ms."
            )
        if not config["camera"].get("available"):
            warnings.append(
                "Measured camera extrinsic missing; optical frame unavailable."
            )

        live_tf = None
        live_result: dict[str, Any] | None = None
        try:
            live_tf = self.adapter.tf_snapshot()
        except Exception:  # noqa: BLE001 - a gateway hiccup must not break the page
            live_tf = None
        if (
            isinstance(live_tf, dict)
            and live_tf.get("available")
            and live_tf.get("transforms")
        ):
            live_result = compose_tf_frames(live_tf, model.root_link)
            warnings.extend(live_result["issues"])
            if live_result["unresolved"]:
                warnings.append(
                    f"{len(live_result['unresolved'])} TF edge(s) are outside "
                    f"the {model.root_link} root and were ignored."
                )
            gateway_notes = live_tf.get("notes")
            if isinstance(gateway_notes, list):
                warnings.extend(str(note) for note in gateway_notes)
            warnings.append(
                "Live TF snapshot from gateway: dynamic transforms may derive "
                "from commanded joint states, not encoder feedback."
            )

        live_by_name: dict[str, dict[str, Any]] = {}
        if live_result is not None:
            live_by_name = {
                str(frame["name"]): frame for frame in live_result["frames"]
            }
        frames_out = []
        model_names = set()
        for frame in result["frames"]:
            model_names.add(frame["name"])
            live = live_by_name.get(frame["name"])
            if live is None:
                frames_out.append(frame)
                continue
            merged = {**frame, **live}
            merged["is_camera"] = bool(frame.get("is_camera") or live.get("is_camera"))
            merged["nominal"] = bool(frame.get("nominal"))
            frames_out.append(merged)
        if live_result is not None:
            for name, frame in live_by_name.items():
                if name in model_names:
                    continue
                added = dict(frame)
                added["is_camera"] = "camera" in name
                added["nominal"] = name in {
                    "camera_link",
                    "camera_optical_frame",
                    "camera_connet_link",
                }
                if name == config["camera"].get("frame"):
                    added["optical"] = True
                    added["truth"] = config["camera"].get("truth", "measured")
                    added["calibration_version"] = config["camera"].get(
                        "calibration_version", ""
                    )
                frames_out.append(added)

        camera_name = config["camera"].get("frame")
        camera_frame = next(
            (frame for frame in frames_out if frame.get("name") == camera_name),
            result["camera_frame"],
        )
        camera_block["tf_frames"] = [
            name
            for name in (
                config["camera"].get("body_frame"),
                config["camera"].get("frame"),
            )
            if name and any(frame.get("name") == name for frame in frames_out)
        ]
        if live_result is not None:
            missing_camera = [
                name
                for name in (
                    config["camera"].get("body_frame"),
                    config["camera"].get("frame"),
                )
                if name and not any(frame.get("name") == name for frame in frames_out)
            ]
            if missing_camera:
                camera_block["warnings"].append(
                    "Live TF does not include: " + ", ".join(missing_camera)
                )

        degraded = bool(missing_arm) or (
            max_age is not None
            and max_age > self.mapping.safety.feedback_max_age_ms
        )
        if live_result is None:
            tf_status = "degraded" if degraded else "model_only"
            tf_source = "urdf_fk"
            tf_truth = "model"
            warnings.append(
                "No live tf2 subscription from the gateway; frames are URDF "
                f"forward kinematics from truth={joint_truth} joint positions."
            )
        else:
            gateway_status = str(live_tf.get("status", "unknown"))
            if gateway_status == "ok" and not live_result["issues"]:
                tf_status = "live"
            elif gateway_status == "static_only":
                tf_status = "live_static_only"
            else:
                tf_status = "degraded"
            tf_source = "tf2_topics"
            tf_truth = "measured_tf"
            if gateway_status == "static_only":
                warnings.append(
                    "Only /tf_static is live; dynamic link frames fall back to "
                    "the URDF where the static graph does not reach."
                )
            if gateway_status == "stale":
                warnings.append(
                    "Live TF is stale (newest dynamic edge "
                    f"{live_tf.get('freshest_age_ms')} ms)."
                )
            missing_live = sorted(model_names - set(live_by_name))
            if missing_live:
                warnings.append(
                    f"{len(missing_live)} model frame(s) are not in live TF and "
                    "use URDF FK: " + ", ".join(missing_live[:6])
                )
        camera_block["tf_status"] = tf_status

        joint_records = []
        for name in model.movable_order:
            joint = model.joints[name]
            mapping_joint = self.mapping.get_joint(name)
            position = positions.get(name)
            position_source = "state"
            if (
                position is None
                and name in result["derived_joints"]
                and _mimic_source_available(model, name, positions)
            ):
                position = result["derived_joints"][name]
                position_source = "mimic"
            elif position is None:
                position_source = "missing"
            if mapping_joint is not None:
                lower = mapping_joint.min_rad
                upper = mapping_joint.max_rad
                limit_source = mapping_joint.limit_source
                verified = mapping_joint.verified
            else:
                lower = joint.lower
                upper = joint.upper
                limit_source = "urdf"
                verified = False
            within_limits = None
            if position is not None and lower is not None and upper is not None:
                within_limits = lower <= position <= upper
            joint_records.append(
                {
                    "name": name,
                    "type": joint.joint_type,
                    "position_rad": position,
                    "position_source": position_source,
                    "truth": joint_truth,
                    "lower_rad": lower,
                    "upper_rad": upper,
                    "urdf_lower_rad": joint.lower,
                    "urdf_upper_rad": joint.upper,
                    "velocity_limit": joint.velocity,
                    "limit_source": limit_source,
                    "verified": verified,
                    "within_limits": within_limits,
                    "mimic_of": joint.mimic,
                    "mimic_multiplier": joint.mimic_multiplier if joint.mimic else None,
                }
            )

        return {
            **base,
            "urdf": {
                "available": True,
                "path": model.source_path,
                "sha256": model.sha256,
                "robot_name": model.robot_name,
                "verified": model.verified,
                "links": len(model.links),
                "joints": len(model.movable_order),
                "source_note": model.source_note,
                "warnings": model.warnings,
            },
            "tf": {
                "status": tf_status,
                "source": tf_source,
                "live": live_result is not None,
                "truth": tf_truth,
                "joint_truth": joint_truth,
                "max_sample_age_ms": max_age,
                "gateway_status": live_tf.get("status") if live_result is not None else None,
                "dynamic_edges": live_tf.get("dynamic_edges") if live_result is not None else 0,
                "static_edges": live_tf.get("static_edges") if live_result is not None else 0,
                # Static TF edges are latched and legitimately old; use the
                # dynamic maximum so the UI does not call a live graph stale.
                "max_tf_age_ms": (
                    live_tf.get("max_dynamic_age_ms", live_tf.get("max_age_ms"))
                    if live_result is not None
                    else None
                ),
                "frames": frames_out,
                "camera_frame": camera_frame,
                "warnings": warnings,
            },
            "joints": joint_records,
            "camera": camera_block,
            "warnings": warnings + camera_block["warnings"],
        }

    def list_camera_references(self, limit: int = 50) -> dict[str, Any]:
        try:
            items = self.store.list_camera_references(limit)
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "camera reference store read failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        return {"items": items, "schema_version": 1}

    def save_camera_reference(self, note: str = "") -> dict[str, Any]:
        if not isinstance(note, str) or len(note) > 500:
            raise ServiceError(
                ErrorCode.BAD_REQUEST.value,
                "note must be a string of at most 500 chars",
                status_code=400,
            )
        visual = self.visual_snapshot()
        camera = visual["camera"]
        if not camera.get("available"):
            raise ServiceError(
                ErrorCode.BAD_REQUEST.value,
                "camera is unavailable; no reference saved",
                status_code=400,
            )
        calibration = camera.get("calibration") or {}
        record = {
            "reference_id": f"ref_{uuid.uuid4().hex[:12]}",
            "created_at": utc_now_iso(),
            "mode": visual["mode"],
            "source_label": visual["source_label"],
            "truth": camera.get("truth", "unknown"),
            "camera_topic": camera.get("topic", ""),
            "width": camera.get("width"),
            "height": camera.get("height"),
            "encoding": camera.get("encoding"),
            "fps": camera.get("fps"),
            "frame_at": camera.get("last_frame_at"),
            "sample_age_ms": camera.get("sample_age_ms"),
            "calibration_version": calibration.get("calibration_version", ""),
            "calibration_source": calibration.get("calibration_source", ""),
            "tf_status": visual["tf"]["status"],
            "tf_source": visual["tf"]["source"],
            "note": note.strip(),
            "snapshot_url": camera.get("snapshot_url", ""),
            "schema_version": 1,
        }
        try:
            inserted = self.store.insert_camera_reference(
                reference_id=record["reference_id"],
                created_at=record["created_at"],
                record=record,
            )
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "camera reference store write failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        if not inserted:
            raise ServiceError(
                ErrorCode.INTERNAL_ERROR.value,
                "camera reference insert was ignored",
                status_code=500,
            )
        storage_error = None
        try:
            self._append(
                self._event(
                    "CameraReferenceSaved",
                    None,
                    "visual",
                    "camera metadata reference saved",
                    None,
                    {
                        "reference_id": record["reference_id"],
                        "truth": record["truth"],
                        "tf_status": record["tf_status"],
                        "calibration_version": record["calibration_version"],
                        "note": record["note"],
                    },
                )
            )
        except ServiceError as exc:
            storage_error = exc.details
        return {
            **record,
            "storage_error": storage_error,
        }

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
            supervised_motion=self.settings.supervised_motion,
            motion_limits=self.motion_limits,
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
            supervised_motion=self.settings.supervised_motion,
            motion_limits=self.motion_limits,
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
                "supervised_motion": self.motion_limits.supervised,
                "motion_mode": self.motion_limits.mode_label,
                "motion_max_step_rad": self.motion_limits.max_step_rad,
                "motion_min_duration_s": self.motion_limits.min_duration_s,
                "motion_single_joint_only": self.motion_limits.single_joint_only,
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
            outcome = self._validate_targets(state, targets)
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
            outcome = self._validate_targets(state, targets)
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
                supervised_motion=self.settings.supervised_motion,
                motion_limits=self.motion_limits,
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

    # -------------------------------------------------------- trajectories (G2)
    def list_trajectories(self, limit: int = 50) -> dict[str, Any]:
        try:
            items = self.store.list_trajectories(limit)
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "trajectory store read failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        return {"items": items, "schema_version": 1}

    def import_trajectory(
        self, raw: Any, source: str = "import"
    ) -> dict[str, Any]:
        """Validate, analyze, and store an offline dataset.

        Duplicate datasets (same canonical SHA-256) return the existing record.
        This path never calls the adapter and never dispatches motion.
        """
        try:
            dataset, analysis, digest = prepare_dataset(raw)
        except TrajectoryDataError as exc:
            raise ServiceError(
                ErrorCode.BAD_REQUEST.value,
                "trajectory dataset is invalid",
                status_code=400,
                details={"issues": exc.issues},
            ) from exc

        try:
            existing = self.store.find_trajectory_by_hash(digest)
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "trajectory store read failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        if existing is not None:
            return {**existing, "duplicate": True, "storage_error": None}

        record: dict[str, Any] = {
            "trajectory_id": f"trj_{uuid.uuid4().hex[:12]}",
            "created_at": utc_now_iso(),
            "name": dataset["name"],
            "source": source,
            "schema_version": dataset["schema_version"],
            "dataset_sha256": digest,
            "analysis_version": analysis["analysis_version"],
            "dataset": dataset,
            "analysis": analysis,
        }
        summary_fields = summary(record)
        try:
            inserted = self.store.insert_trajectory(
                trajectory_id=record["trajectory_id"],
                created_at=record["created_at"],
                name=record["name"],
                source=record["source"],
                schema_version=record["schema_version"],
                dataset_sha256=digest,
                analysis_version=record["analysis_version"],
                dataset=dataset,
                analysis=analysis,
                summary=summary_fields,
            )
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "trajectory store write failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        if not inserted:
            stored = self.store.find_trajectory_by_hash(digest)
            if stored is not None:
                return {**stored, "duplicate": True, "storage_error": None}
            raise ServiceError(
                ErrorCode.INTERNAL_ERROR.value,
                "trajectory insert was ignored without a matching stored record",
                status_code=500,
            )

        storage_error = None
        try:
            self._append(
                self._event(
                    "TrajectoryImported",
                    None,
                    "trajectory",
                    f"offline dataset stored: {record['name']}",
                    None,
                    {
                        "trajectory_id": record["trajectory_id"],
                        "dataset_sha256": digest,
                        "source": source,
                        "feedback_truth": dataset["feedback"]["truth"],
                        "tracking_available": summary_fields["tracking_available"],
                    },
                )
            )
        except ServiceError as exc:
            storage_error = exc.details
        return {
            **record,
            "duplicate": False,
            "storage_error": storage_error,
        }

    def import_demo_trajectory(self) -> dict[str, Any]:
        return self.import_trajectory(build_demo_dataset(), source="demo")

    def _trajectory_or_404(self, trajectory_id: str) -> dict[str, Any]:
        try:
            record = self.store.get_trajectory(trajectory_id)
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "trajectory store read failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        if record is None:
            raise ServiceError(
                ErrorCode.NOT_FOUND.value,
                f"trajectory {trajectory_id!r} does not exist",
                status_code=404,
            )
        return record

    def get_trajectory(self, trajectory_id: str) -> dict[str, Any]:
        return self._trajectory_or_404(trajectory_id)

    def recompute_trajectory(self, trajectory_id: str) -> dict[str, Any]:
        record = self._trajectory_or_404(trajectory_id)
        fresh = analyze(record["dataset"])
        return {
            "trajectory_id": trajectory_id,
            "analysis_version": fresh["analysis_version"],
            "stored_analysis_version": record["analysis_version"],
            "matches_stored": fresh == record["analysis"],
            "analysis": fresh,
        }

    def export_trajectory_csv(
        self, trajectory_id: str, sample: str = "feedback"
    ) -> tuple[str, str]:
        record = self._trajectory_or_404(trajectory_id)
        try:
            text = alignment_csv(record["dataset"], sample)
        except ValueError as exc:
            raise ServiceError(
                ErrorCode.BAD_REQUEST.value, str(exc), status_code=400
            ) from exc
        return f"armpi-trajectory-{trajectory_id}-{sample}.csv", text

    def export_trajectory_json(self, trajectory_id: str) -> dict[str, Any]:
        return self._trajectory_or_404(trajectory_id)

    # ------------------------------------------------------ experiments (G3)
    @staticmethod
    def _experiment_summary(row: dict[str, Any]) -> dict[str, Any]:
        protocol = row["protocol"]
        n_trials = int(row["n_trials"])
        n_complete = int(row["n_complete_pairs"])
        planned = int(protocol.get("planned_pairs", 0))
        if n_trials == 0:
            status = "planned"
        elif planned > 0 and n_complete >= planned:
            status = "complete"
        else:
            status = "in_progress"
        n_baseline = int(row["n_baseline"])
        n_intervention = int(row["n_intervention"])
        return {
            "experiment_id": row["experiment_id"],
            "created_at": row["created_at"],
            "name": row["name"],
            "data_origin": row["data_origin"],
            "operator": row["operator"],
            "truth": "simulated" if row["data_origin"] == "simulated" else "measured",
            "schema_version": row.get("schema_version", 1),
            "analysis_version": row["analysis_version"],
            "planned_pairs": planned,
            "seed": protocol.get("seed"),
            "n_trials": n_trials,
            "n_complete_pairs": n_complete,
            "n_baseline": n_baseline,
            "n_intervention": n_intervention,
            "baseline_success_rate": (
                int(row["n_baseline_successes"]) / n_baseline if n_baseline else None
            ),
            "intervention_success_rate": (
                int(row["n_intervention_successes"]) / n_intervention
                if n_intervention
                else None
            ),
            "status": status,
        }

    def list_experiments(self, limit: int = 50) -> dict[str, Any]:
        try:
            rows = self.store.list_experiments(limit)
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "experiment store read failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        return {
            "items": [self._experiment_summary(row) for row in rows],
            "failure_codes": FAILURE_CODES,
            "methods": METHOD_LABELS,
            "schema_version": 1,
        }

    def _experiment_or_404(self, experiment_id: str) -> dict[str, Any]:
        try:
            record = self.store.get_experiment(experiment_id)
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "experiment store read failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        if record is None:
            raise ServiceError(
                ErrorCode.NOT_FOUND.value,
                f"experiment {experiment_id!r} does not exist",
                status_code=404,
            )
        return record

    def _experiment_detail(self, record: dict[str, Any]) -> dict[str, Any]:
        try:
            trials = self.store.list_trials(record["experiment_id"])
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "trial store read failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        return {
            "experiment": record,
            "trials": trials,
            "analysis": analyze_experiment(record, trials),
        }

    def create_experiment(self, raw: Any) -> dict[str, Any]:
        try:
            definition = normalize_experiment(raw)
        except ExperimentDataError as exc:
            raise ServiceError(
                ErrorCode.BAD_REQUEST.value,
                "experiment definition is invalid",
                status_code=400,
                details={"issues": exc.issues},
            ) from exc
        record = {
            "experiment_id": f"exp_{uuid.uuid4().hex[:12]}",
            "created_at": utc_now_iso(),
            "analysis_version": EXPERIMENT_ANALYSIS_VERSION,
            **definition,
        }
        try:
            inserted = self.store.insert_experiment(
                experiment_id=record["experiment_id"],
                created_at=record["created_at"],
                signature=None,
                definition=record,
            )
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "experiment store write failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        if not inserted:
            raise ServiceError(
                ErrorCode.INTERNAL_ERROR.value,
                "experiment insert was ignored",
                status_code=500,
            )
        storage_error = None
        try:
            self._append(
                self._event(
                    "ExperimentCreated",
                    None,
                    "experiment",
                    f"experiment created: {record['name']}",
                    None,
                    {
                        "experiment_id": record["experiment_id"],
                        "data_origin": record["data_origin"],
                        "planned_pairs": record["protocol"]["planned_pairs"],
                    },
                )
            )
        except ServiceError as exc:
            storage_error = exc.details
        return {
            **self._experiment_detail(record),
            "duplicate": False,
            "storage_error": storage_error,
        }

    def import_demo_experiment(self) -> dict[str, Any]:
        try:
            existing = self.store.find_experiment_by_signature(DEMO_SIGNATURE)
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "experiment store read failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        if existing is not None:
            return {
                **self._experiment_detail(existing),
                "duplicate": True,
                "storage_error": None,
            }

        definition, raw_trials = build_demo_experiment()
        record = {
            "experiment_id": f"exp_{uuid.uuid4().hex[:12]}",
            "created_at": utc_now_iso(),
            "analysis_version": EXPERIMENT_ANALYSIS_VERSION,
            **definition,
        }
        try:
            inserted = self.store.insert_experiment(
                experiment_id=record["experiment_id"],
                created_at=record["created_at"],
                signature=DEMO_SIGNATURE,
                definition=record,
            )
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "experiment store write failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        if not inserted:
            existing = self.store.find_experiment_by_signature(DEMO_SIGNATURE)
            if existing is not None:
                return {
                    **self._experiment_detail(existing),
                    "duplicate": True,
                    "storage_error": None,
                }
            raise ServiceError(
                ErrorCode.INTERNAL_ERROR.value,
                "demo experiment insert was ignored",
                status_code=500,
            )

        for raw in raw_trials:
            self._record_trial(record, raw, audit=False)
        storage_error = None
        try:
            self._append(
                self._event(
                    "ExperimentSeeded",
                    None,
                    "experiment",
                    "simulated demo experiment seeded",
                    None,
                    {
                        "experiment_id": record["experiment_id"],
                        "n_trials": len(raw_trials),
                        "data_origin": "simulated",
                    },
                )
            )
        except ServiceError as exc:
            storage_error = exc.details
        return {
            **self._experiment_detail(record),
            "duplicate": False,
            "storage_error": storage_error,
        }

    def get_experiment(self, experiment_id: str) -> dict[str, Any]:
        return self._experiment_detail(self._experiment_or_404(experiment_id))

    def _record_trial(
        self,
        experiment: dict[str, Any],
        raw: Any,
        *,
        audit: bool = True,
    ) -> dict[str, Any]:
        try:
            existing = self.store.list_trials(experiment["experiment_id"])
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "trial store read failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        try:
            trial = normalize_trial(raw, experiment, existing)
        except ExperimentDataError as exc:
            raise ServiceError(
                ErrorCode.BAD_REQUEST.value,
                "trial is invalid",
                status_code=400,
                details={"issues": exc.issues},
            ) from exc
        if trial["trajectory_id"]:
            try:
                trajectory = self.store.get_trajectory(trial["trajectory_id"])
            except StorageError as exc:
                raise ServiceError(
                    ErrorCode.STORAGE_WRITE_FAILED.value,
                    "trajectory store read failed",
                    status_code=503,
                    details={"storage_error": str(exc)},
                ) from exc
            if trajectory is None:
                raise ServiceError(
                    ErrorCode.BAD_REQUEST.value,
                    "trial is invalid",
                    status_code=400,
                    details={
                        "issues": [
                            {
                                "path": "trajectory_id",
                                "message": "trajectory dataset does not exist",
                            }
                        ]
                    },
                )
        record = {
            **trial,
            "trial_id": f"trl_{uuid.uuid4().hex[:12]}",
            "experiment_id": experiment["experiment_id"],
            "created_at": utc_now_iso(),
        }
        try:
            inserted = self.store.insert_trial(
                trial_id=record["trial_id"],
                experiment_id=record["experiment_id"],
                created_at=record["created_at"],
                trial=record,
            )
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "trial store write failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        if not inserted:
            raise ServiceError(
                ErrorCode.CONFLICT.value,
                "pair/method or pair/trial_order is already recorded",
                status_code=409,
            )
        if audit:
            self._append(
                self._event(
                    "TrialRecorded",
                    None,
                    "experiment",
                    f"trial recorded: {record['pair_id']} / {record['method']}",
                    record["failure_code"],
                    {
                        "experiment_id": record["experiment_id"],
                        "trial_id": record["trial_id"],
                        "pair_id": record["pair_id"],
                        "method": record["method"],
                        "grasp_success": record["grasp_success"],
                        "failure_code": record["failure_code"],
                        "trajectory_id": record["trajectory_id"],
                        "truth": record["truth"],
                    },
                )
            )
        return record

    def record_trial(self, experiment_id: str, raw: Any) -> dict[str, Any]:
        experiment = self._experiment_or_404(experiment_id)
        return self._record_trial(experiment, raw, audit=True)

    def experiment_analysis(self, experiment_id: str) -> dict[str, Any]:
        experiment = self._experiment_or_404(experiment_id)
        try:
            trials = self.store.list_trials(experiment_id)
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "trial store read failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        return analyze_experiment(experiment, trials)

    def export_experiment_csv(self, experiment_id: str) -> tuple[str, str]:
        record = self._experiment_or_404(experiment_id)
        try:
            trials = self.store.list_trials(experiment_id)
        except StorageError as exc:
            raise ServiceError(
                ErrorCode.STORAGE_WRITE_FAILED.value,
                "trial store read failed",
                status_code=503,
                details={"storage_error": str(exc)},
            ) from exc
        buffer = io.StringIO()
        writer = csv.writer(buffer, lineterminator="\n")
        writer.writerow(
            [
                "experiment_id",
                "trial_id",
                "pair_id",
                "trial_order",
                "method",
                "fixture",
                "grasp_success",
                "continuous_error_mm",
                "rotation_error_deg",
                "failure_code",
                "failure_detail",
                "duration_s",
                "measurement_source",
                "trajectory_id",
                "truth",
                "artifacts_json",
                "operator_notes",
                "target_json",
                "created_at",
                "schema_version",
            ]
        )
        for trial in trials:
            writer.writerow(
                [
                    experiment_id,
                    trial["trial_id"],
                    trial["pair_id"],
                    trial["trial_order"],
                    trial["method"],
                    trial["fixture"],
                    1 if trial["grasp_success"] else 0,
                    "" if trial["continuous_error_mm"] is None else trial["continuous_error_mm"],
                    "" if trial["rotation_error_deg"] is None else trial["rotation_error_deg"],
                    trial["failure_code"] or "",
                    trial["failure_detail"],
                    "" if trial["duration_s"] is None else trial["duration_s"],
                    trial["measurement_source"],
                    trial["trajectory_id"] or "",
                    trial["truth"],
                    json.dumps(trial["artifacts"], sort_keys=True),
                    trial["operator_notes"],
                    json.dumps(trial["target"], sort_keys=True),
                    trial["created_at"],
                    trial["schema_version"],
                ]
            )
        return f"armpi-experiment-{experiment_id}.csv", buffer.getvalue()

    def export_experiment_json(self, experiment_id: str) -> dict[str, Any]:
        record = self._experiment_or_404(experiment_id)
        detail = self._experiment_detail(record)
        return {
            "schema_version": 1,
            "exported_at": utc_now_iso(),
            "experiment": detail["experiment"],
            "trials": detail["trials"],
            "analysis": detail["analysis"],
        }

    def close(self) -> None:
        self.store.close()
