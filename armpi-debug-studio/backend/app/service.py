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
from dataclasses import replace
from pathlib import Path
from typing import Any, Callable

import numpy as np

from .adapters.base import AdapterError, HardwareMotionDisabled, RobotAdapter
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
from .domain.approach import plan_approach
from .domain.grasp_fit import fit as grasp_fit, validate_target as validate_grasp_target
from .domain.locate import cube_box_present, cube_point, segment_floor_cube
from .domain.profile import ProfileParams, plan_move, validate_params as validate_profile_params
from .domain.task import build_task_plan
from .domain.takeover import (
    ARM_JOINTS,
    TakeoverConfig,
    load_takeover_config,
)
from .domain.models import (
    CommandRecord,
    Event,
    Fault,
    Health,
    MotionTarget,
    PreviewResult,
    RobotState,
    StopResult,
    ValidationIssue,
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
from .safety.pose_gate import PoseGate, mapping_targets_valid
from .safety.supervisor import SafetySupervisor
from .safety.tokens import PreviewTokenManager
from .safety.validation import validate_targets

IDEMPOTENCY_TTL_S = 300

# The gateway is a separate process/package; keep its evidence notes intact but
# show the operator a Chinese explanation. Unknown notes pass through unchanged.
_GATEWAY_NOTE_LABELS = {
    "TF is published by robot nodes. Dynamic transforms may be derived "
    "from commanded joint states, not encoder feedback.": (
        "TF 由机器人节点发布；动态变换可能由指令关节状态推导，不是编码器反馈。"
    ),
    "Only /tf_static has been received; dynamic links are not live.": (
        "只收到 /tf_static；动态连杆不是实时值。"
    ),
    "No /tf or /tf_static message has been received.": (
        "尚未收到 /tf 或 /tf_static 消息。"
    ),
}


def _localize_gateway_note(note: str) -> str:
    if note in _GATEWAY_NOTE_LABELS:
        return _GATEWAY_NOTE_LABELS[note]
    prefix = "Newest dynamic TF edge is "
    suffix = " ms old."
    if note.startswith(prefix) and note.endswith(suffix):
        age = note[len(prefix) : -len(suffix)]
        return f"最新动态 TF 边已过期 {age} ms。"
    return note

KNOWN_DEFECTS = [
    {
        "id": "B-01",
        "summary": "get_torque_state 调用不存在的方法",
        "source": "ros_robot_controller_node.py:243",
        "impact": "扭矩诊断抛 AttributeError",
    },
    {
        "id": "B-02",
        "summary": "按键发布者引用未导入的 PacketReportKeyEvents",
        "source": "ros_robot_controller_node.py:291",
        "impact": "触发按键路径时抛 NameError",
    },
    {
        "id": "B-03",
        "summary": "/joint_controller 回调参数类型不匹配",
        "source": "controller_manager.py:91-95",
        "impact": "该控制入口不可用",
    },
    {
        "id": "B-04",
        "summary": "FollowJointTrajectory action 不发送反馈",
        "source": "joint_trajectory_action_controller.py",
        "impact": "轨迹执行期间没有进度证据",
    },
    {
        "id": "B-05",
        "summary": "轨迹 action 未检查实际位置就报告成功",
        "source": "joint_trajectory_action_controller.py:173",
        "impact": "成功不代表机械臂到达目标",
    },
    {
        "id": "B-06",
        "summary": "joint_states 是最后下发的脉冲，不是传感器反馈",
        "source": "controller_manager.py:97-114",
        "impact": "必须标记为 truth=commanded，而不是 measured",
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
        takeover: TakeoverConfig | None = None,
    ) -> None:
        self.settings = settings
        self.mapping = mapping
        self.adapter = adapter
        self.store = store
        self._clock = clock or time.monotonic
        self.leases = LeaseManager(self._clock)
        self.tokens = PreviewTokenManager(self._clock)
        self.supervisor = SafetySupervisor(mapping, self.leases)
        self.takeover = (
            takeover
            if takeover is not None
            else (
                load_takeover_config(settings.takeover_config_path)
                if settings.motion_scope == "pose"
                else None
            )
        )
        self.motion_limits = effective_motion_limits(
            mapping,
            supervised=settings.supervised_motion,
            takeover=self.takeover,
        )
        self._pose_gate: PoseGate | None = None
        self._last_task_plan: dict[str, Any] | None = None
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
            message="事件存储写入失败；修复前运动被锁定",
            source="persistence",
            first_seen=now,
            last_seen=now,
            remediation="检查 DB_PATH 权限/磁盘并重启后端",
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
        """Baseline for step/pose validation.

        For measured hardware feedback the actual position is the only honest
        baseline: two consecutive commands can each be within one step of the
        commanded target while the arm is still far behind. Mock/simulated and
        commanded-only data keep the previous target baseline.
        """
        targets = {
            joint.name: (
                joint.position_rad
                if joint.truth is Truth.MEASURED and joint.position_rad is not None
                else (joint.target_rad if joint.target_rad is not None else 0.0)
            )
            for joint in state.joints
        }
        gripper = state.gripper
        if gripper is not None and gripper.position_rad is not None:
            targets[gripper.joint] = gripper.position_rad
        return targets

    def _fast_gripper(self):
        """The fast task gripper contract, or None when the fast path is off."""
        if self.takeover is None or self.takeover.task is None:
            return None
        fast = self.takeover.task.fast
        if not fast.enabled:
            return None
        return replace(
            self.takeover.gripper,
            speed_raw_per_s=fast.gripper_speed_raw_per_s,
            min_duration_s=fast.gripper_min_duration_s,
            max_duration_s=max(self.takeover.gripper.max_duration_s, 5.0),
        )

    def _validate_targets(
        self,
        state: RobotState,
        targets: list[MotionTarget],
        *,
        profile: ProfileParams | None = None,
        fast: bool = False,
    ):
        min_duration = self.motion_limits.min_duration_s
        max_velocity = self.motion_limits.max_velocity_rad_s
        gripper = self.takeover.gripper if self.takeover is not None else None
        if profile is not None:
            # A dense profile is validated endpoint-by-endpoint against the
            # profile's own peak velocity; its total can be shorter than the
            # single-command duration floor.
            min_duration = 0.0
            max_velocity = profile.vmax_rad_s
        elif fast:
            # Fast task gripper steps use the measured 0.35-0.8 s raw durations.
            min_duration = 0.0
        if fast or profile is not None:
            fast_gripper = self._fast_gripper()
            if fast_gripper is not None:
                gripper = fast_gripper
        outcome = validate_targets(
            self.mapping,
            targets,
            current_targets=self._current_targets(state),
            max_step_rad=self.motion_limits.max_step_rad,
            min_duration_s=min_duration,
            max_duration_s=self.motion_limits.max_duration_s,
            single_joint_only=self.motion_limits.single_joint_only,
            scope="pose" if self.motion_limits.takeover else "step",
            max_velocity_rad_s=max_velocity,
            gripper=gripper,
        )
        if self.motion_limits.takeover and self.takeover is not None:
            gripper_name = self.takeover.gripper.joint
            if any((target.joint or "").strip() == gripper_name for target in targets):
                gripper = state.gripper
                if (
                    gripper is None
                    or not gripper.available
                    or gripper.position_rad is None
                    or gripper.truth
                    not in (
                        Truth.MEASURED,
                        Truth.MEASURED_ON_DEMAND,
                        Truth.COMMANDED,
                        Truth.SIMULATED,
                    )
                ):
                    outcome.errors.append(
                        self._issue(
                            ErrorCode.FEEDBACK_STALE,
                            "gripper measured position is unavailable; a gripper "
                            "command needs measured feedback for its baseline",
                            gripper_name,
                        )
                    )
        return outcome

    def _check_profile(
        self,
        state: RobotState,
        normalized: list[dict[str, Any]],
        profile: ProfileParams,
        outcome,
    ) -> None:
        """Validate the fast profile against the contract and the live start."""
        if self.takeover is None or self.takeover.task is None:
            outcome.errors.append(
                self._issue(
                    "PROFILE_DISABLED",
                    "the fast profile needs a task block in takeover.yaml",
                )
            )
            return
        fast = self.takeover.task.fast
        if not fast.enabled:
            outcome.errors.append(
                self._issue(
                    "PROFILE_DISABLED",
                    "task.fast.enabled is false in configs/takeover.yaml",
                )
            )
            return
        try:
            validate_profile_params(profile)
        except ValueError as exc:
            outcome.errors.append(self._issue(ErrorCode.BAD_REQUEST, str(exc)))
            return
        if profile.vmax_rad_s > fast.vmax_rad_s + 1e-9:
            outcome.errors.append(
                self._issue(
                    "PROFILE_LIMIT",
                    f"profile vmax {profile.vmax_rad_s} rad/s exceeds the "
                    f"contract's {fast.vmax_rad_s} rad/s",
                )
            )
            return
        if profile.amax_rad_s2 > fast.amax_rad_s2 + 1e-9:
            outcome.errors.append(
                self._issue(
                    "PROFILE_LIMIT",
                    f"profile amax {profile.amax_rad_s2} rad/s^2 exceeds the "
                    f"contract's {fast.amax_rad_s2} rad/s^2",
                )
            )
            return
        target = {item["joint"]: float(item["target_rad"]) for item in normalized}
        if set(target) != set(ARM_JOINTS):
            outcome.errors.append(
                self._issue(
                    ErrorCode.POSE_JOINT_SET,
                    "a dense profile needs exactly the five arm joints",
                )
            )
            return
        start = self._current_targets(state)
        try:
            generated = plan_move(
                {name: float(start[name]) for name in ARM_JOINTS},
                target,
                profile,
            )
        except (KeyError, ValueError) as exc:
            outcome.errors.append(
                self._issue(
                    "PROFILE_INVALID",
                    f"the profile could not be built: {exc}",
                )
            )
            return
        if generated.total_s > fast.max_total_duration_s + 1e-9:
            outcome.errors.append(
                self._issue(
                    "PROFILE_LIMIT",
                    f"profile total {generated.total_s:.3f} s exceeds the "
                    f"contract's {fast.max_total_duration_s:.3f} s",
                )
            )
            return
        if generated.segments > fast.max_points:
            outcome.errors.append(
                self._issue(
                    "PROFILE_LIMIT",
                    f"profile has {generated.segments} segments; the cap is "
                    f"{fast.max_points}",
                )
            )
            return
        # Show the real S-curve time in the preview instead of the plan-time
        # estimate (the live start differs by a few pulses after every move).
        for item in normalized:
            item["duration_s"] = generated.total_s

    def _build_profile_points(
        self,
        state: RobotState,
        normalized: list[dict[str, Any]],
        profile: ProfileParams,
    ) -> list[dict[str, Any]]:
        """Build the concrete waypoints from the fresh start pose."""
        start = self._current_targets(state)
        target = {item["joint"]: float(item["target_rad"]) for item in normalized}
        generated = plan_move(
            {name: float(start[name]) for name in ARM_JOINTS},
            target,
            profile,
        )
        by_joint = {item["joint"]: item for item in normalized}
        points: list[dict[str, Any]] = []
        for duration, pose in zip(generated.durations_s, generated.points):
            targets = []
            for name in ARM_JOINTS:
                joint = self.mapping.joints[name]
                targets.append(
                    {
                        "joint": name,
                        "target_rad": float(pose[name]),
                        "target_raw": int(round(joint.rad_to_raw(float(pose[name])))),
                    }
                )
            points.append({"duration_s": float(duration), "targets": targets})
        # The gateway requires the last waypoint to equal the previewed
        # endpoint exactly; use the validated raw pulses there.
        for item in points[-1]["targets"]:
            source = by_joint[item["joint"]]
            item["target_rad"] = float(source["target_rad"])
            item["target_raw"] = int(source["target_raw"])
        return points

    def _issue(self, code: ErrorCode | str, message: str, joint: str | None = None, **details):
        code_value = code.value if isinstance(code, ErrorCode) else str(code)
        return ValidationIssue(code=code_value, message=message, joint=joint, details=details)

    def _pose_gate_for(self) -> PoseGate:
        if self._pose_gate is None:
            model, _config = self._load_visual_bundle()
            if self.takeover is None:  # guarded by callers
                raise VisualModelError(["takeover contract is not loaded"])
            self._pose_gate = PoseGate(self.takeover, model)
        return self._pose_gate

    def _pose_check(
        self,
        state: RobotState,
        normalized: list[dict[str, Any]],
    ) -> tuple[dict[str, Any] | None, list[Any]]:
        """Run the FK gate for a five-joint pose.

        Returns ``(pose_details, errors)``. The gripper is not a pose and the
        gate never runs for it.
        """
        if self.takeover is None or self.motion_limits.scope != "pose":
            return None, []
        target = {item["joint"]: float(item["target_rad"]) for item in normalized}
        if set(target) != set(self.mapping.joints):
            return None, []
        try:
            gate = self._pose_gate_for()
        except VisualModelError as exc:
            return None, [
                self._issue(
                    ErrorCode.FK_UNAVAILABLE,
                    "URDF/FK is unavailable; the pose gate fails closed: "
                    + "; ".join(exc.issues),
                )
            ]
        start = self._current_targets(state)
        try:
            outcome = gate.check(start, target)
        except Exception as exc:  # noqa: BLE001 - fail closed, never crash a preview
            return None, [
                self._issue(
                    ErrorCode.FK_UNAVAILABLE,
                    f"pose gate failed to evaluate the path: {exc}",
                )
            ]
        details = outcome.to_public_dict()
        errors = [
            self._issue(
                violation.code,
                violation.message,
                violation.joint,
                **violation.details,
            )
            for violation in outcome.violations
        ]
        if not outcome.start_compliant:
            details["warning"] = (
                "the measured start is outside the gate envelope; it is exempt "
                "so the arm can be commanded back, but check the arm first"
            )
        return details, errors

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

    def _token_targets(
        self,
        normalized: list[dict[str, Any]],
        *,
        profile: ProfileParams | None = None,
    ) -> list[dict[str, Any]]:
        # ``from_rad`` is deliberately excluded: it comes from the live state
        # read and can change by small amounts between preview and execute
        # (hardware command echo). Binding it would turn a genuinely safe
        # preview into PREVIEW_PAYLOAD_MISMATCH instead of letting the safety
        # gate evaluate the command against the fresh state.
        # A profile duration is also excluded: it is computed from that live
        # start, so it changes by a few milliseconds between preview and
        # execute even when the target is identical.
        return [
            {
                "joint": item["joint"],
                "target_rad": item["target_rad"],
                "target_raw": item["target_raw"],
                **(
                    {}
                    if profile is not None
                    else {"duration_s": item["duration_s"]}
                ),
            }
            for item in normalized
        ]

    def _preview_payload(
        self,
        command_id: str,
        session_id: str,
        normalized: list[dict[str, Any]],
        *,
        profile: ProfileParams | None = None,
        fast: bool = False,
    ) -> dict[str, Any]:
        payload = {
            "command_id": command_id,
            "session_id": session_id,
            "mapping_version": self.mapping.mapping_version,
            "mapping_hash": self.mapping.mapping_hash,
            "targets": self._token_targets(normalized, profile=profile),
        }
        if profile is not None:
            payload["profile"] = profile.to_public_dict()
        if fast:
            payload["fast"] = True
        return payload

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
            warnings.append("相机不可用；无法显示画面。")
        if camera.get("truth") == "simulated":
            warnings.append("模拟：相机快照为合成画面。")
        if not camera_config.get("available"):
            warnings.append(
                "缺少实测光学外参；不显示光学框架。"
            )
        warnings.append(
            f"图像框架是 {camera_config.get('body_frame')}（机身）。"
            f"PnP 必须使用 {camera_config.get('frame')}，不能用 URDF 标称框架。"
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
                "URDF 模型为暂定：关节限位与网格尚未经过硬件验证。"
            )
        if missing_arm:
            warnings.append(
                "缺少臂关节状态：" + ", ".join(missing_arm)
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
                    f"有 {len(live_result['unresolved'])} 条 TF 边不在 "
                    f"{model.root_link} 根节点下，已忽略。"
                )
            gateway_notes = live_tf.get("notes")
            if isinstance(gateway_notes, list):
                warnings.extend(
                    _localize_gateway_note(str(note)) for note in gateway_notes
                )
            warnings.append(
                "来自网关的实时 TF 快照：动态变换可能由指令关节状态推导，"
                "不是编码器反馈。"
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
                "网关没有实时 tf2 订阅；框架来自 URDF 前向运动学"
                f"（关节真值={joint_truth} 位置）。"
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
                    "只有 /tf_static 是实时的；静态图未覆盖的动态连杆框架"
                    "回退到 URDF。"
                )
            if gateway_status == "stale":
                warnings.append(
                    "实时 TF 已过期（最新动态边 "
                    f"{live_tf.get('freshest_age_ms')} ms）。"
                )
            missing_live = sorted(model_names - set(live_by_name))
            if missing_live:
                warnings.append(
                    f"有 {len(missing_live)} 个模型框架不在实时 TF 中，"
                    "使用 URDF FK：" + ", ".join(missing_live[:6])
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
            motion_scope=self.motion_limits.scope,
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
            motion_scope=self.motion_limits.scope,
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
                "motion_scope": self.motion_limits.scope,
                "motion_max_step_rad": self.motion_limits.max_step_rad,
                "motion_max_velocity_rad_s": self.motion_limits.max_velocity_rad_s,
                "motion_min_duration_s": self.motion_limits.min_duration_s,
                "motion_single_joint_only": self.motion_limits.single_joint_only,
                "gripper_control": self.motion_limits.gripper_enabled,
                "takeover_config": (
                    self.takeover.config_hash if self.takeover is not None else None
                ),
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
    def preview(
        self,
        session_id: str,
        targets: list[MotionTarget],
        *,
        profile: ProfileParams | None = None,
        fast: bool = False,
    ) -> dict[str, Any]:
        with self._lock:
            command_id = f"cmd_{uuid.uuid4().hex[:16]}"
            state = self._state()
            outcome = self._validate_targets(
                state, targets, profile=profile, fast=fast
            )
            if outcome.valid and profile is not None:
                self._check_profile(state, outcome.normalized, profile, outcome)
            pose_details: dict[str, Any] | None = None
            if outcome.valid:
                pose_details, pose_errors = self._pose_check(state, outcome.normalized)
                if pose_errors:
                    outcome.errors.extend(pose_errors)
                elif pose_details and pose_details.get("warning"):
                    outcome.warnings.append(
                        self._issue(
                            "START_OUTSIDE_GATE",
                            pose_details["warning"],
                        )
                    )
                if pose_details and any(
                    joint.truth is Truth.COMMANDED for joint in state.joints
                ):
                    outcome.warnings.append(
                        self._issue(
                            "START_ECHO",
                            "the start pose is the vendor command cache (the "
                            "actual registers can differ by ~8 pulses / 0.03 rad); "
                            "the move is bus-verified after completion",
                        )
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
                    details={
                        "errors": [jsonable(e) for e in outcome.errors],
                        "pose": pose_details,
                    },
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
                        pose=pose_details,
                    )
                )

            payload = self._preview_payload(
                command_id,
                session_id,
                outcome.normalized,
                profile=profile,
                fast=fast,
            )
            token, expires_at = self.tokens.issue(payload, self.mapping.safety.preview_ttl_s)
            record = CommandRecord(
                command_id=command_id,
                session_id=session_id,
                status=CommandStage.VALIDATED,
                targets=outcome.normalized,
                created_at=utc_now_iso(),
                updated_at=utc_now_iso(),
                details={
                    "warnings": [jsonable(w) for w in outcome.warnings],
                    "pose": pose_details,
                },
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
                        "pose": pose_details,
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
                    pose=pose_details,
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
        profile: ProfileParams | None = None,
        fast: bool = False,
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
                "profile": profile.to_public_dict() if profile else None,
                "fast": bool(fast),
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
            outcome = self._validate_targets(
                state, targets, profile=profile, fast=fast
            )
            if outcome.valid and profile is not None:
                self._check_profile(state, outcome.normalized, profile, outcome)
            pose_details: dict[str, Any] | None = None
            if outcome.valid:
                pose_details, pose_errors = self._pose_check(state, outcome.normalized)
                if pose_errors:
                    outcome.errors.extend(pose_errors)
            if not outcome.valid:
                record = self._load_record(command_id)
                if record is not None and record.status is CommandStage.VALIDATED:
                    transition(
                        record,
                        CommandStage.FAILED,
                        error_code=outcome.errors[0].code,
                        details={
                            "errors": [jsonable(e) for e in outcome.errors],
                            "pose": pose_details,
                        },
                    )
                    self._save(record)
                    self._append(
                        self._event(
                            CommandStage.FAILED,
                            command_id,
                            "api",
                            "execute revalidation rejected",
                            outcome.errors[0].code,
                            {
                                "errors": [jsonable(e) for e in outcome.errors],
                                "pose": pose_details,
                            },
                        )
                    )
                raise ServiceError(
                    outcome.errors[0].code,
                    outcome.errors[0].message,
                    status_code=400,
                    details={
                        "errors": [jsonable(e) for e in outcome.errors],
                        "pose": pose_details,
                    },
                )

            payload = self._preview_payload(
                command_id,
                session_id,
                outcome.normalized,
                profile=profile,
                fast=fast,
            )
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
                motion_scope=self.motion_limits.scope,
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
                if profile is not None:
                    points = self._build_profile_points(
                        state, outcome.normalized, profile
                    )
                    dispatch = self.adapter.dispatch_profile(
                        outcome.normalized, points, command_id
                    )
                else:
                    dispatch = self.adapter.dispatch(
                        outcome.normalized, command_id, fast=fast
                    )
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
                    "pose": pose_details,
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
                    {**dict(dispatch.details or {}), "pose": pose_details},
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

    # ------------------------------------------------------------ chassis
    def chassis_pulse(
        self,
        *,
        session_id: str,
        lease_id: str,
        vx: float,
        vy: float,
        duty: float,
        duration_s: float,
    ) -> dict[str, Any]:
        """Pulse the open-loop chassis toward the cube, then re-measure.

        Only allowed with the arm at the task view: the pulse is computed from
        that view's measurement, and the next locate must see the same scene.
        The gateway caps each pulse; the caller is responsible for the loop.
        """
        takeover = self.takeover
        task = takeover.task if takeover is not None else None
        approach = task.approach if task is not None else None
        if approach is None or not approach.enabled:
            raise ServiceError(
                ErrorCode.NOT_IMPLEMENTED.value,
                "the chassis approach is disabled in the takeover contract",
                status_code=503,
            )
        if not (0.0 < float(duty) <= approach.duty):
            raise ServiceError(
                ErrorCode.BAD_REQUEST.value,
                f"duty must be in (0, {approach.duty}]",
                status_code=400,
            )
        if not (
            approach.min_duration_s
            <= float(duration_s)
            <= approach.max_duration_s
        ):
            raise ServiceError(
                ErrorCode.BAD_REQUEST.value,
                f"duration must be in "
                f"[{approach.min_duration_s}, {approach.max_duration_s}] s",
                status_code=400,
            )
        if self.settings.mode is Mode.MOCK:
            return {
                "status": "Completed",
                "verification": "simulated",
                "session_id": session_id,
                "pulse": {
                    "vx": float(vx),
                    "vy": float(vy),
                    "duty": float(duty),
                    "duration_s": float(duration_s),
                },
            }
        if (
            self.settings.mode is not Mode.HARDWARE
            or not self.settings.hardware_motion_enabled
        ):
            raise ServiceError(
                ErrorCode.HARDWARE_MOTION_DISABLED.value,
                "chassis motion requires hardware mode and ARMPI_HARDWARE_MOTION=1",
                status_code=403,
            )
        with self._lock:
            lease = self.leases.check(lease_id, session_id)
            if not lease.ok:
                raise ServiceError(
                    lease.code or ErrorCode.NO_LEASE.value,
                    lease.detail,
                    status_code=403,
                )
            state = self._state()
            view = takeover.preset(task.view_preset) if task is not None else None
            if view is None:
                raise ServiceError(
                    ErrorCode.NOT_IMPLEMENTED.value,
                    f"task view preset {task.view_preset!r} is missing",
                    status_code=503,
                )
            current = {
                joint.name: joint.position_rad
                for joint in state.joints
                if joint.position_rad is not None
            }
            if set(current) != set(ARM_JOINTS):
                raise ServiceError(
                    "TASK_STATE_UNAVAILABLE",
                    "not all five arm joints have a position",
                    status_code=409,
                )
            deviation = max(
                abs(float(current[name]) - float(view.joints[name]))
                for name in ARM_JOINTS
            )
            if deviation > 0.12:
                raise ServiceError(
                    "TASK_NOT_AT_VIEW",
                    "the arm is not at the task view pose; prepare the view first",
                    status_code=409,
                )
            payload = {
                "vx": float(vx),
                "vy": float(vy),
                "duty": float(duty),
                "duration_s": float(duration_s),
            }
            try:
                response = self.adapter.chassis_pulse(payload)
            except AdapterError as exc:
                raise ServiceError(
                    exc.code,
                    str(exc),
                    status_code=502,
                    details=exc.details,
                ) from exc
            return {
                "status": "Completed",
                "verification": (response or {}).get(
                    "verification", "open_loop"
                ),
                "session_id": session_id,
                "pulse": payload,
                "gateway": response,
            }

    # ------------------------------------------------------------ pose preset
    def motion_presets(self) -> dict[str, Any]:
        """Return the takeover contract and the hardware-validated poses.

        Every preset is checked statically against the joint limits and the FK
        gate. A path to a preset is still re-checked at preview/execute time
        from the measured start, because the straight interpolation can differ.
        """
        state = self._state()
        base: dict[str, Any] = {
            "schema_version": 1,
            "generated_at": utc_now_iso(),
            "scope": self.motion_limits.scope,
            "available": self.takeover is not None,
            "contract": (
                self.takeover.to_public_dict() if self.takeover is not None else None
            ),
            "presets": [],
            "current": {
                "joints": {
                    joint.name: joint.position_rad
                    for joint in state.joints
                    if joint.position_rad is not None
                },
                "gripper": jsonable(state.gripper) if state.gripper is not None else None,
                "tcp_m": None,
                "measured": all(
                    joint.truth in (Truth.MEASURED, Truth.MEASURED_ON_DEMAND)
                    for joint in state.joints
                ),
                "source": (
                    "measured_on_demand"
                    if any(
                        joint.truth is Truth.MEASURED_ON_DEMAND
                        for joint in state.joints
                    )
                    else (
                        "commanded"
                        if any(joint.truth is Truth.COMMANDED for joint in state.joints)
                        else "mixed"
                    )
                ),
            },
            "gate_error": None,
        }
        if self.takeover is None:
            return base

        gate: PoseGate | None
        try:
            gate = self._pose_gate_for()
        except VisualModelError as exc:
            gate = None
            base["gate_error"] = exc.issues

        current_joints = base["current"]["joints"]
        if gate is not None and set(current_joints) == set(self.mapping.joints):
            tcp = gate.tcp(current_joints)
            base["current"]["tcp_m"] = (
                [round(value, 6) for value in tcp] if tcp is not None else None
            )

        presets: list[dict[str, Any]] = []
        for preset in self.takeover.presets:
            bad = mapping_targets_valid(self.mapping, preset.joints)
            raw = {}
            for name, value in preset.joints.items():
                joint = self.mapping.get_joint(name)
                raw[name] = int(round(joint.rad_to_raw(value))) if joint else None
            entry = {
                **preset.to_public_dict(),
                "target_raw": raw,
                "limit_ok": not bad,
                "bad_limits": bad,
                "pose_check": (
                    gate.check_pose(preset.joints).to_public_dict()
                    if gate is not None
                    else None
                ),
            }
            presets.append(entry)
        base["presets"] = presets
        return base

    # ---------------------------------------------------------------- G8 task
    MOCK_TASK_POINT = (
        0.22690956575501345,
        -0.01390503513063539,
        -0.05001748059159114,
    )

    def _task_context(self) -> tuple[TakeoverConfig, Any]:
        takeover = self.takeover
        if takeover is None or takeover.task is None:
            raise ServiceError(
                ErrorCode.NOT_IMPLEMENTED.value,
                "task takeover requires a takeover contract with a task block",
                status_code=503,
            )
        if self.motion_limits.scope != "pose":
            raise ServiceError(
                ErrorCode.NOT_IMPLEMENTED.value,
                "task takeover requires ARMPI_MOTION_SCOPE=pose",
                status_code=503,
            )
        return takeover, takeover.task

    def _task_tcp(self, joints: list[float]) -> "np.ndarray":
        model, _config = self._load_visual_bundle()
        frames = compute_frames(
            model,
            {name: float(joints[index]) for index, name in enumerate(ARM_JOINTS)},
            {},
        )["frames"]
        for frame in frames:
            if frame["name"] == "end_effector_link":
                return np.asarray(frame["position_m"], dtype=np.float64)
        raise VisualModelError(["end_effector_link is missing from the URDF FK"])

    def _task_link4(self, joints: dict[str, float]) -> "np.ndarray":
        model, _config = self._load_visual_bundle()
        frames = compute_frames(model, dict(joints), {})["frames"]
        for frame in frames:
            if frame["name"] == "link4":
                matrix = np.eye(4)
                matrix[:3, :3] = np.asarray(
                    frame["rotation"], dtype=np.float64
                ).reshape(3, 3)
                matrix[:3, 3] = np.asarray(
                    frame["position_m"], dtype=np.float64
                )
                return matrix
        raise VisualModelError(["link4 is missing from the URDF FK"])

    @staticmethod
    def _task_hand_eye(task: Any) -> "np.ndarray":
        path = Path(task.hand_eye_path)
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
            matrix = np.asarray(
                data["hand_eye"]["methods"]["TSAI"]["T_link4_camera"],
                dtype=np.float64,
            )
        except (OSError, KeyError, TypeError, json.JSONDecodeError) as exc:
            raise ServiceError(
                ErrorCode.FK_UNAVAILABLE.value,
                f"hand-eye calibration is unusable: {exc}",
                status_code=503,
            ) from exc
        if matrix.shape != (4, 4) or not np.all(np.isfinite(matrix)):
            raise ServiceError(
                ErrorCode.FK_UNAVAILABLE.value,
                "hand-eye T_link4_camera is not a finite 4x4 matrix",
                status_code=503,
            )
        return matrix

    def _task_invalid(
        self,
        code: str,
        message: str,
        *,
        view_joints: dict[str, float] | None = None,
        locate: dict[str, Any] | None = None,
        fit: dict[str, Any] | None = None,
        current: dict[str, float] | None = None,
        simulated: bool = False,
        extra_errors: list[dict[str, Any]] | None = None,
        approach: dict[str, Any] | None = None,
    ) -> dict[str, Any]:
        errors = [
            {"label": "task", "code": code, "message": message, "joint": None}
        ]
        errors.extend(extra_errors or [])
        return {
            "schema_version": 1,
            "valid": False,
            "errors": errors,
            "warnings": [],
            "simulated": simulated,
            "steps": [],
            "view_joints": view_joints or {},
            "current_joints": current or {},
            "locate": locate or {},
            "fit": fit,
            "approach": approach,
            "plan_id": None,
        }

    def _task_pose_basis(
        self,
        state: RobotState,
        view: Any,
    ) -> tuple[dict[str, float], dict[str, int | None], str]:
        """Return the joint basis for the locate FK and its pulse source.

        The vendor command cache at the view can differ from the actual
        registers by ~0.03 rad (~9 mm at 0.3 m). Prefer the gateway's last bus
        read when it is at the same view; otherwise use the cache and warn.
        """
        cache = {
            joint.name: float(joint.position_rad)
            for joint in state.joints
            if joint.position_rad is not None
        }
        raw = {
            joint.name: joint.raw_position for joint in state.joints
        }
        if self.settings.mode is Mode.MOCK:
            return cache, raw, "simulated"
        try:
            actual = self.adapter.last_actual()
        except Exception:  # noqa: BLE001 - the cache remains usable
            actual = None
        if isinstance(actual, dict):
            rad = {}
            for name in ARM_JOINTS:
                item = actual.get(name)
                if not isinstance(item, dict) or item.get("position_rad") is None:
                    rad = {}
                    break
                rad[name] = float(item["position_rad"])
            if rad:
                deviations = {
                    name: abs(rad[name] - float(view.joints[name]))
                    for name in ARM_JOINTS
                }
                if max(deviations.values()) <= 0.12:
                    return (
                        rad,
                        {
                            name: (
                                int(actual[name]["raw_position"])
                                if isinstance(actual[name], dict)
                                and actual[name].get("raw_position") is not None
                                else raw.get(name)
                            )
                            for name in ARM_JOINTS
                        },
                        "measured_on_demand",
                    )
        measured = all(
            joint.truth in (Truth.MEASURED, Truth.MEASURED_ON_DEMAND)
            for joint in state.joints
        )
        return cache, raw, "measured_on_demand" if measured else "commanded"

    def task_plan(self) -> dict[str, Any]:
        """Locate the floor cube and build the validated task plan.

        Read-only: no motion command is issued. The arm must already be at the
        task view preset; the cloud and joint positions are read at that pose.
        """
        takeover, task = self._task_context()
        state = self._state()
        simulated = self.settings.mode is Mode.MOCK
        view = takeover.preset(task.view_preset)
        if view is None:
            raise ServiceError(
                ErrorCode.NOT_IMPLEMENTED.value,
                f"task view preset {task.view_preset!r} is missing",
                status_code=503,
            )
        if state.connection_state is ConnectionState.EXECUTING:
            raise ServiceError(
                ErrorCode.ALREADY_EXECUTING.value,
                "motion is executing; wait for it to finish before locating",
                status_code=409,
            )
        if (
            self.settings.mode is Mode.HARDWARE
            and not state.capabilities.hardware_motion
        ):
            raise ServiceError(
                ErrorCode.HARDWARE_MOTION_DISABLED.value,
                "task takeover requires hardware motion and pose scope",
                status_code=503,
            )
        current = {
            joint.name: joint.position_rad
            for joint in state.joints
            if joint.position_rad is not None
        }
        if set(current) != set(ARM_JOINTS):
            return self._task_invalid(
                "TASK_STATE_UNAVAILABLE",
                "not all five arm joints have a position",
                view_joints=view.joints,
                current=current,
                simulated=simulated,
            )
        offsets = {
            name: abs(float(current[name]) - float(view.joints[name]))
            for name in ARM_JOINTS
        }
        at_view = max(offsets.values()) <= 0.12
        if not at_view:
            return self._task_invalid(
                "TASK_NOT_AT_VIEW",
                "the arm is not at the task view pose; prepare the view first",
                view_joints=view.joints,
                current=current,
                simulated=simulated,
            )
        basis, raw_pulses, source = self._task_pose_basis(state, view)
        if simulated:
            source = "simulated"
        report: dict[str, Any]
        point: "np.ndarray"
        meta: dict[str, Any] = {}
        if simulated:
            source = "simulated"
            point = np.asarray(self.MOCK_TASK_POINT, dtype=np.float64)
            report = {
                "schema_version": 1,
                "mode": "simulated_locate",
                "success": True,
                "cloud": {"simulated": True},
                "selected": {
                    "grasp_point_base": [float(value) for value in point],
                    "top": {"centroid": [float(value) for value in point]},
                },
                "segmentation": {"clusters": 0, "cluster_details": []},
            }
        else:
            cloud = self.adapter.pointcloud()
            if cloud is None or cloud.get("points") is None:
                raise ServiceError(
                    ErrorCode.POINTCLOUD_UNAVAILABLE.value,
                    "the gateway has no registered point cloud; start the pose "
                    "gateway with --pointcloud-topic on 8766",
                    status_code=503,
                    details={"cloud": (cloud or {}).get("meta")},
                )
            meta = cloud["meta"] if isinstance(cloud.get("meta"), dict) else {}
            points = cloud["points"]
            age = meta.get("sample_age_ms")
            if isinstance(age, (int, float)) and age > 1500:
                raise ServiceError(
                    ErrorCode.POINTCLOUD_STALE.value,
                    f"the point cloud is {age:.0f} ms old",
                    status_code=409,
                    details={"cloud": meta},
                )
            try:
                hand_eye = self._task_hand_eye(task)
                link4 = self._task_link4(basis)
                report = segment_floor_cube(
                    points,
                    link4 @ hand_eye,
                    task.locate,
                    cloud_meta={
                        "frame_id": meta.get("frame_id"),
                        "topic": meta.get("topic"),
                        "sample_age_ms": age,
                        "layout": meta.get("layout"),
                    },
                )
            except VisualModelError as exc:
                return self._task_invalid(
                    ErrorCode.FK_UNAVAILABLE.value,
                    "URDF FK is unavailable: " + "; ".join(exc.issues),
                    view_joints=view.joints,
                    current=current,
                    simulated=simulated,
                )
            measured, accepted = cube_point(report)
            if measured is None or not accepted:
                reason = (
                    report.get("rejection", {}).get("reason")
                    if isinstance(report.get("rejection"), dict)
                    else report.get("error")
                ) or "no usable cube point"
                return self._task_invalid(
                    "LOCATE_FAILED",
                    f"the segmenter did not return a usable grasp point: {reason}",
                    view_joints=view.joints,
                    locate=report,
                    current=current,
                    simulated=simulated,
                )
            point = measured
        correction = np.asarray(task.correction_mm, dtype=np.float64) / 1000.0
        target = point + correction
        try:
            fit_result = grasp_fit(
                target,
                self._task_tcp,
                j2=task.grasp_j2,
                j3=task.grasp_j3,
                j5=task.grasp_j5,
            )
        except Exception as exc:  # noqa: BLE001 - report, never crash the plan
            return self._task_invalid(
                "FIT_FAILED",
                f"the deep-family fit failed: {exc}",
                view_joints=view.joints,
                locate=report,
                current=current,
                simulated=simulated,
            )
        reasons = validate_grasp_target(
            target,
            fit_result,
            task.max_xy_error_mm,
            (task.min_reach_m, task.max_reach_m),
        )
        if reasons:
            approach = plan_approach(point, task.approach)
            if approach is not None:
                return self._task_invalid(
                    "APPROACH_REQUIRED",
                    "the cube is outside the direct-grasp band ("
                    + "; ".join(reasons)
                    + "); pulse the chassis and re-locate",
                    view_joints=view.joints,
                    locate=report,
                    fit=fit_result,
                    current=current,
                    simulated=simulated,
                    approach=approach,
                )
            return self._task_invalid(
                "FIT_REJECTED",
                "; ".join(reasons),
                view_joints=view.joints,
                locate=report,
                fit=fit_result,
                current=current,
                simulated=simulated,
            )
        try:
            gate = self._pose_gate_for()
        except VisualModelError as exc:
            return self._task_invalid(
                ErrorCode.FK_UNAVAILABLE.value,
                "URDF FK is unavailable: " + "; ".join(exc.issues),
                view_joints=view.joints,
                locate=report,
                fit=fit_result,
                current=current,
                simulated=simulated,
            )
        gripper = state.gripper
        start_gripper = (
            gripper.position_rad
            if gripper is not None and gripper.position_rad is not None
            else takeover.gripper.open_rad
        )
        locate_summary = {
            k: report.get(k)
            for k in ("cloud", "floor", "segmentation", "selected", "rejection")
            if k in report
        }
        locate_summary["success"] = bool(report.get("success"))
        locate_summary["previous"] = {
            name: {
                "rad": round(float(basis[name]), 6),
                "raw": raw_pulses.get(name),
            }
            for name in ARM_JOINTS
        }
        hand_eye_summary = {
            "method": "TSAI",
            "path": task.hand_eye_path,
            "frame": "link4 -> camera optical (depth_camera_link layout)",
        }
        plan = build_task_plan(
            takeover=takeover,
            mapping=self.mapping,
            limits=self.motion_limits,
            gate=gate,
            fit_result=fit_result,
            point=[float(value) for value in point],
            start_joints=basis,
            start_gripper_rad=float(start_gripper),
            source=source,
            locate_summary=locate_summary,
            hand_eye_summary=hand_eye_summary,
            simulated=simulated,
        )
        recovered = (report.get("selected") or {}).get("top_recovered")
        if isinstance(recovered, dict):
            plan["warnings"].append(
                {
                    "label": "locate",
                    "code": "TOP_CLIPPED_RECOVERED",
                    "message": (
                        "the min-range sphere clipped the near top edge; the "
                        "cube centre was rebuilt from the far edge with the "
                        f"measured {recovered.get('cube_size_m', 0.0) * 1000:.1f} mm cube size "
                        f"(centroid bias {recovered.get('centroid_bias_mm', 0.0):.1f} mm, "
                        f"clipped side {recovered.get('clipped_side')})"
                    ),
                }
            )
        plan["plan_id"] = f"task_{uuid.uuid4().hex[:16]}"
        plan["generated_at"] = utc_now_iso()
        plan["cloud"] = meta
        with self._lock:
            self._last_task_plan = {
                "plan": plan,
                "point": [float(value) for value in point],
                "target": [float(value) for value in target],
                "view": dict(view.joints),
                "source": source,
                "simulated": simulated,
            }
        return plan

    def task_verify(
        self,
        plan_id: str | None = None,
        *,
        expect_view: bool = True,
    ) -> dict[str, Any]:
        """Check the cube box at the view after the lift.

        Returns ``held`` together with the segmentation report. A failed
        segmentation is reported, never silently converted to a held claim.
        """
        takeover, task = self._task_context()
        with self._lock:
            stored = self._last_task_plan
        if stored is None:
            raise ServiceError(
                "TASK_NOT_PLANNED",
                "no task plan exists; locate and plan first",
                status_code=409,
            )
        plan = stored["plan"]
        if plan_id and plan.get("plan_id") != plan_id:
            raise ServiceError(
                "TASK_PLAN_STALE",
                "the supplied plan_id is not the current plan",
                status_code=409,
            )
        state = self._state()
        simulated = bool(stored.get("simulated"))
        view = takeover.preset(task.view_preset)
        current = {
            joint.name: joint.position_rad
            for joint in state.joints
            if joint.position_rad is not None
        }
        if expect_view and view is not None:
            if set(current) == set(ARM_JOINTS):
                offsets = {
                    name: abs(
                        float(current[name]) - float(view.joints[name])
                    )
                    for name in ARM_JOINTS
                }
                if max(offsets.values()) > 0.12:
                    return {
                        "held": None,
                        "verified": False,
                        "code": "TASK_NOT_AT_VIEW",
                        "message": "the arm is not at the view pose; the "
                        "grip check needs the view",
                        "point": stored["point"],
                        "simulated": simulated,
                    }
        basis, _raw, basis_source = (
            self._task_pose_basis(state, view)
            if view is not None
            else (
                {name: float(value) for name, value in current.items()},
                {},
                "commanded",
            )
        )
        if simulated:
            return {
                "held": True,
                "verified": False,
                "simulated": True,
                "code": "SIMULATED_VERIFY",
                "message": "mock mode assumes the grip; no camera was read",
                "point": stored["point"],
                "clusters": 0,
                "report": {"mode": "simulated_verify"},
                "source": "simulated",
            }
        cloud = self.adapter.pointcloud()
        if cloud is None or cloud.get("points") is None:
            raise ServiceError(
                ErrorCode.POINTCLOUD_UNAVAILABLE.value,
                "the gateway has no registered point cloud for the grip check",
                status_code=503,
            )
        meta = cloud["meta"] if isinstance(cloud.get("meta"), dict) else {}
        age = meta.get("sample_age_ms")
        if isinstance(age, (int, float)) and age > 1500:
            raise ServiceError(
                ErrorCode.POINTCLOUD_STALE.value,
                f"the grip-check cloud is {age:.0f} ms old",
                status_code=409,
            )
        try:
            hand_eye = self._task_hand_eye(task)
            link4 = self._task_link4(basis)
            report = segment_floor_cube(
                cloud["points"],
                link4 @ hand_eye,
                task.locate,
                cloud_meta={
                    "frame_id": meta.get("frame_id"),
                    "topic": meta.get("topic"),
                    "sample_age_ms": age,
                    "layout": meta.get("layout"),
                },
            )
        except VisualModelError as exc:
            raise ServiceError(
                ErrorCode.FK_UNAVAILABLE.value,
                "URDF FK is unavailable: " + "; ".join(exc.issues),
                status_code=503,
            ) from exc
        held = not cube_box_present(
            report,
            stored["point"],
            task.verify_radius_m,
            task.verify_min_height_m,
        )
        clusters = int(report.get("segmentation", {}).get("clusters") or 0)
        warning = None
        if not report.get("success") and clusters == 0:
            warning = (
                "segmentation returned no clusters; 'held' is the phase-3 box "
                "rule, not a positive grip observation"
            )
        return {
            "held": held,
            "verified": True,
            "simulated": False,
            "point": stored["point"],
            "clusters": clusters,
            "report": report,
            "source": basis_source,
            "warning": warning,
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
                "硬件反馈探针尚未实现；请用姿态网关执行后的 GetBusServoState 校验",
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
