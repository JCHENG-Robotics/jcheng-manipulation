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
from dataclasses import replace
from typing import Any, Callable

from .adapters.base import RobotAdapter
from .application.experiment_ops import ExperimentOperations
from .application.motion_ops import MotionOperations
# Re-exported for the existing task tests; the implementation now lives in
# application/task_ops.py.
from .application.task_ops import (
    TaskOperations,
    _locate_rejection_text,
)
from .application.trajectory_ops import TrajectoryOperations
from .config import Settings
from .domain.enums import (
    CommandStage,
    ConnectionState,
    ErrorCode,
    FaultCode,
    FaultLevel,
    Mode,
    Truth,
)
from .domain.mapping import MappingConfig
from .domain.motion import effective_motion_limits
from .domain.profile import ProfileParams, plan_move, validate_params as validate_profile_params
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
    RobotState,
    ValidationIssue,
    jsonable,
    utc_now_iso,
)
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
from .safety.pose_gate import PoseGate
from .safety.supervisor import SafetySupervisor
from .safety.tokens import PreviewTokenManager
from .safety.validation import validate_targets

__all__ = ["RobotService", "_locate_rejection_text"]

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


class RobotService(
    TaskOperations, ExperimentOperations, TrajectoryOperations, MotionOperations
):
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
        now = self._clock()
        expired = [k for k, v in self._idempotency.items() if v["expires"] <= now]
        for k in expired:
            self._idempotency.pop(k, None)
        self._idempotency[key] = {
            "request_hash": request_hash,
            "expires": now + IDEMPOTENCY_TTL_S,
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
        self._last_task_plan = None
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

    def close(self) -> None:
        self.store.close()
