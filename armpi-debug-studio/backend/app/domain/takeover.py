"""G7 pose-takeover contract.

The G6 write path was deliberately tiny: one arm joint, 23 pulses, one second
minimum. G7 keeps the same "preview -> confirm -> execute" flow but lifts the
scope to one **whole five-joint pose** plus one **gripper** command. This module
holds the limits that only make sense for a pose command, and a small library
of poses that have actually run on the hardware.

This file has no ROS, FastAPI or adapter imports so the safety core, the
service and the tests can share it.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml

from .locate import LocateParams

# Enabling a multi-joint write surface needs an action in the launch command,
# not a checkbox in the UI. The phrase is intentionally different from the G6
# single-joint acknowledgement.
TAKEOVER_ACK = "I_AM_AT_THE_ROBOT_WITH_A_CLEAR_WORKSPACE"

ARM_JOINTS: tuple[str, ...] = ("joint1", "joint2", "joint3", "joint4", "joint5")
GRIPPER_JOINT = "r_joint"
GRIPPER_SERVO_ID = 1
GRIPPER_INIT_RAW = 700


class TakeoverConfigError(ValueError):
    """The takeover contract file is missing or malformed."""


@dataclass(frozen=True)
class GripperLimit:
    """One gripper servo, measured with the vendor init-700 convention."""

    joint: str
    servo_id: int
    init_raw: int
    open_raw: int
    close_raw: int
    measured_min_raw: int
    measured_max_raw: int
    speed_raw_per_s: float
    min_duration_s: float
    max_duration_s: float
    completion: str = "timed"

    def raw_to_rad(self, raw: float) -> float:
        return (float(raw) - self.init_raw) * self.radians_per_tick

    def rad_to_raw(self, rad: float) -> float:
        return self.init_raw + float(rad) / self.radians_per_tick

    def pulse_to_rad(self, pulse: int) -> float:
        return self.raw_to_rad(pulse)

    def rad_to_pulse(self, rad: float) -> float:
        return self.rad_to_raw(rad)

    @property
    def radians_per_tick(self) -> float:
        # The vendor uses one uniform 240 deg / 1000 tick assumption for all
        # six bus servos; the arm mapping carries the same value.
        return 240.0 / 360.0 * (2.0 * math.pi) / 1000.0

    @property
    def open_rad(self) -> float:
        return self.raw_to_rad(self.open_raw)

    @property
    def close_rad(self) -> float:
        return self.raw_to_rad(self.close_raw)

    @property
    def min_rad(self) -> float:
        return min(self.open_rad, self.close_rad)

    @property
    def max_rad(self) -> float:
        return max(self.open_rad, self.close_rad)

    @property
    def measured_min_rad(self) -> float:
        return self.raw_to_rad(self.measured_min_raw)

    @property
    def measured_max_rad(self) -> float:
        return self.raw_to_rad(self.measured_max_raw)

    def suggested_duration_s(self, delta_raw: float) -> float:
        scaled = abs(float(delta_raw)) / self.speed_raw_per_s
        return min(
            self.max_duration_s,
            max(self.min_duration_s, math.ceil(scaled * 10.0) / 10.0),
        )

    def to_public_dict(self) -> dict[str, Any]:
        return {
            "joint": self.joint,
            "servo_id": self.servo_id,
            "init_raw": self.init_raw,
            "open_raw": self.open_raw,
            "close_raw": self.close_raw,
            "measured_min_raw": self.measured_min_raw,
            "measured_max_raw": self.measured_max_raw,
            "open_rad": round(self.open_rad, 6),
            "close_rad": round(self.close_rad, 6),
            "min_rad": round(self.min_rad, 6),
            "max_rad": round(self.max_rad, 6),
            "speed_raw_per_s": self.speed_raw_per_s,
            "min_duration_s": self.min_duration_s,
            "max_duration_s": self.max_duration_s,
            "completion": self.completion,
        }


@dataclass(frozen=True)
class WorkspaceBox:
    frame: str
    enabled: bool
    lower: tuple[float, float, float]
    upper: tuple[float, float, float]

    def contains(self, point: tuple[float, float, float]) -> bool:
        return all(
            self.lower[index] <= point[index] <= self.upper[index]
            for index in range(3)
        )

    def to_public_dict(self) -> dict[str, Any]:
        return {
            "frame": self.frame,
            "enabled": self.enabled,
            "lower": list(self.lower),
            "upper": list(self.upper),
        }


@dataclass(frozen=True)
class TablePlane:
    frame: str
    enabled: bool
    normal: tuple[float, float, float]
    offset: float
    clearance_m: float

    def height(self, point: tuple[float, float, float]) -> float:
        """Signed distance from the plane (positive above it)."""
        return (
            self.normal[0] * point[0]
            + self.normal[1] * point[1]
            + self.normal[2] * point[2]
            + self.offset
        )

    def to_public_dict(self) -> dict[str, Any]:
        return {
            "frame": self.frame,
            "enabled": self.enabled,
            "normal": list(self.normal),
            "offset": self.offset,
            "clearance_m": self.clearance_m,
        }


@dataclass(frozen=True)
class ChassisGuard:
    """Measured rule: do not yaw a low arm across the chassis."""

    enabled: bool
    low_tcp_z_m: float
    max_low_yaw_rad: float

    def violation(
        self,
        tcp: tuple[float, float, float],
        joint1_rad: float,
    ) -> str | None:
        if not self.enabled:
            return None
        if tcp[2] >= self.low_tcp_z_m:
            return None
        if abs(joint1_rad) <= self.max_low_yaw_rad:
            return None
        return (
            f"tcp_z={tcp[2]:.3f} m is below {self.low_tcp_z_m:.3f} m with "
            f"joint1={joint1_rad:.3f} rad (limit ±{self.max_low_yaw_rad:.3f})"
        )

    def to_public_dict(self) -> dict[str, Any]:
        return {
            "enabled": self.enabled,
            "low_tcp_z_m": self.low_tcp_z_m,
            "max_low_yaw_rad": self.max_low_yaw_rad,
        }


@dataclass(frozen=True)
class PosePreset:
    preset_id: str
    label: str
    label_en: str
    joints: dict[str, float]
    verified_on_hardware: bool
    source: str

    def to_public_dict(self) -> dict[str, Any]:
        return {
            "id": self.preset_id,
            "label": self.label,
            "label_en": self.label_en,
            "joints": {name: round(value, 6) for name, value in self.joints.items()},
            "verified_on_hardware": self.verified_on_hardware,
            "source": self.source,
        }


@dataclass(frozen=True)
class TaskConfig:
    """G8 task contract: locate + the validated deep grasp family."""

    hand_eye_path: str
    correction_mm: tuple[float, float, float]
    view_preset: str
    lift_preset: str
    grasp_j2: float
    grasp_j3: float
    grasp_j5: float
    pre_grasp_offset_rad: float
    preclose_rad: float
    grip_rad: float
    grip_preclose_duration_s: float
    grip_duration_s: float
    release_open_duration_s: float
    verify_radius_m: float
    verify_min_height_m: float
    min_reach_m: float
    max_reach_m: float
    max_xy_error_mm: float
    locate: LocateParams

    def to_public_dict(self) -> dict[str, Any]:
        return {
            "hand_eye_path": self.hand_eye_path,
            "correction_mm": list(self.correction_mm),
            "view_preset": self.view_preset,
            "lift_preset": self.lift_preset,
            "grasp_j2": self.grasp_j2,
            "grasp_j3": self.grasp_j3,
            "grasp_j5": self.grasp_j5,
            "pre_grasp_offset_rad": self.pre_grasp_offset_rad,
            "preclose_rad": self.preclose_rad,
            "grip_rad": self.grip_rad,
            "grip_preclose_duration_s": self.grip_preclose_duration_s,
            "grip_duration_s": self.grip_duration_s,
            "release_open_duration_s": self.release_open_duration_s,
            "verify_radius_m": self.verify_radius_m,
            "verify_min_height_m": self.verify_min_height_m,
            "min_reach_m": self.min_reach_m,
            "max_reach_m": self.max_reach_m,
            "max_xy_error_mm": self.max_xy_error_mm,
            "locate": (
                self.locate.to_public_dict()
                if hasattr(self.locate, "to_public_dict")
                else self.locate
            ),
        }


@dataclass(frozen=True)
class TakeoverConfig:
    schema_version: int
    scope: str
    max_velocity_rad_s: float
    min_duration_s: float
    max_duration_s: float
    path_samples: int
    completion_tolerance_pulses: int
    completion_settle_samples: int
    timeout_margin_s: float
    gripper: GripperLimit
    workspace: WorkspaceBox
    table_plane: TablePlane
    chassis_guard: ChassisGuard
    presets: tuple[PosePreset, ...]
    task: TaskConfig | None = None
    source_path: str = ""
    config_hash: str = ""

    def preset(self, preset_id: str) -> PosePreset | None:
        for item in self.presets:
            if item.preset_id == preset_id:
                return item
        return None

    def to_public_dict(self) -> dict[str, Any]:
        return {
            "schema_version": self.schema_version,
            "scope": self.scope,
            "max_velocity_rad_s": self.max_velocity_rad_s,
            "min_duration_s": self.min_duration_s,
            "max_duration_s": self.max_duration_s,
            "path_samples": self.path_samples,
            "completion_tolerance_pulses": self.completion_tolerance_pulses,
            "completion_settle_samples": self.completion_settle_samples,
            "timeout_margin_s": self.timeout_margin_s,
            "gripper": self.gripper.to_public_dict(),
            "workspace": self.workspace.to_public_dict(),
            "table_plane": self.table_plane.to_public_dict(),
            "chassis_guard": self.chassis_guard.to_public_dict(),
            "presets": [item.to_public_dict() for item in self.presets],
            "task": (
                self.task.to_public_dict() if self.task is not None else None
            ),
            "config_hash": self.config_hash,
        }


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise TakeoverConfigError(message)


def _triple(value: Any, name: str) -> tuple[float, float, float]:
    _require(
        isinstance(value, (list, tuple)) and len(value) == 3,
        f"{name} must have 3 values",
    )
    values = tuple(float(item) for item in value)
    _require(all(math.isfinite(item) for item in values), f"{name} must be finite")
    return values  # type: ignore[return-value]


def load_takeover_config(path: str | Path) -> TakeoverConfig:
    path = Path(path)
    if not path.is_file():
        raise TakeoverConfigError(f"takeover config not found: {path}")
    try:
        raw = yaml.safe_load(path.read_text(encoding="utf-8"))
    except yaml.YAMLError as exc:  # pragma: no cover - malformed user file
        raise TakeoverConfigError(f"invalid YAML in {path}: {exc}") from exc
    _require(isinstance(raw, dict), "takeover config root must be a mapping")

    schema_version = int(raw.get("schema_version", 1))
    _require(schema_version == 1, f"unsupported takeover schema_version: {schema_version}")
    scope = str(raw.get("scope", "pose")).strip().lower()
    _require(scope == "pose", f"takeover scope must be 'pose', got {scope!r}")

    velocity = float(raw.get("max_velocity_rad_s", 0.5))
    _require(0.05 <= velocity <= 1.2, "max_velocity_rad_s must be in [0.05, 1.2]")
    min_duration = float(raw.get("min_duration_s", 1.0))
    max_duration = float(raw.get("max_duration_s", 10.0))
    _require(0.5 <= min_duration <= max_duration <= 30.0, "invalid duration range")
    path_samples = int(raw.get("path_samples", 21))
    _require(2 <= path_samples <= 201, "path_samples must be 2..201")
    tolerance = int(raw.get("completion_tolerance_pulses", 3))
    settle = int(raw.get("completion_settle_samples", 3))
    _require(0 <= tolerance <= 10, "completion_tolerance_pulses must be 0..10")
    _require(1 <= settle <= 10, "completion_settle_samples must be 1..10")
    margin = float(raw.get("timeout_margin_s", 3.0))
    _require(0.0 <= margin <= 10.0, "timeout_margin_s must be 0..10")

    gripper_raw = raw.get("gripper")
    _require(isinstance(gripper_raw, dict), "gripper must be a mapping")
    _require(
        str(gripper_raw.get("joint", GRIPPER_JOINT)) == GRIPPER_JOINT,
        f"gripper.joint must be {GRIPPER_JOINT}",
    )
    _require(
        int(gripper_raw.get("servo_id", GRIPPER_SERVO_ID)) == GRIPPER_SERVO_ID,
        f"gripper.servo_id must be {GRIPPER_SERVO_ID}",
    )
    gripper = GripperLimit(
        joint=GRIPPER_JOINT,
        servo_id=GRIPPER_SERVO_ID,
        init_raw=int(gripper_raw.get("init_raw", GRIPPER_INIT_RAW)),
        open_raw=int(gripper_raw["open_raw"]),
        close_raw=int(gripper_raw["close_raw"]),
        measured_min_raw=int(gripper_raw["measured_min_raw"]),
        measured_max_raw=int(gripper_raw["measured_max_raw"]),
        speed_raw_per_s=float(gripper_raw.get("speed_raw_per_s", 120.0)),
        min_duration_s=float(gripper_raw.get("min_duration_s", 1.0)),
        max_duration_s=float(gripper_raw.get("max_duration_s", 6.0)),
        completion=str(gripper_raw.get("completion", "timed")),
    )
    _require(gripper.init_raw == GRIPPER_INIT_RAW, "gripper.init_raw must be 700")
    _require(
        0 <= gripper.measured_min_raw < gripper.open_raw < gripper.close_raw,
        "gripper open must be above the measured open stop and below close",
    )
    _require(gripper.close_raw <= 1000, "gripper close must be <= 1000")
    _require(
        gripper.measured_max_raw <= gripper.close_raw,
        "measured gripper close stop must be inside the command",
    )
    _require(gripper.speed_raw_per_s > 0, "gripper speed must be positive")
    _require(
        0.5 <= gripper.min_duration_s <= gripper.max_duration_s <= 30.0,
        "invalid gripper duration range",
    )
    _require(
        gripper.completion in {"timed", "position"},
        "gripper completion must be 'timed' or 'position'",
    )

    workspace_raw = raw.get("workspace")
    _require(isinstance(workspace_raw, dict), "workspace must be a mapping")
    lower = _triple(workspace_raw.get("lower"), "workspace.lower")
    upper = _triple(workspace_raw.get("upper"), "workspace.upper")
    _require(
        all(lower[i] < upper[i] for i in range(3)),
        "workspace lower must be below upper",
    )
    workspace = WorkspaceBox(
        frame=str(workspace_raw.get("frame", "base_link")),
        enabled=bool(workspace_raw.get("enabled", True)),
        lower=lower,
        upper=upper,
    )

    table_raw = raw.get("table_plane")
    _require(isinstance(table_raw, dict), "table_plane must be a mapping")
    normal = _triple(table_raw.get("normal"), "table_plane.normal")
    norm = math.sqrt(sum(value * value for value in normal))
    _require(norm > 0, "table_plane.normal must be non-zero")
    _require(normal[2] > 0, "table_plane.normal must point up (+z)")
    clearance = float(table_raw.get("clearance_m", 0.01))
    _require(clearance >= 0, "table_plane.clearance_m must be non-negative")
    table = TablePlane(
        frame=str(table_raw.get("frame", "base_link")),
        enabled=bool(table_raw.get("enabled", True)),
        normal=normal,
        offset=float(table_raw.get("offset", 0.0)),
        clearance_m=clearance,
    )
    _require(
        workspace.frame == table.frame,
        "workspace and table_plane must use the same frame",
    )

    chassis_raw = raw.get("chassis_guard") or {}
    _require(isinstance(chassis_raw, dict), "chassis_guard must be a mapping")
    chassis = ChassisGuard(
        enabled=bool(chassis_raw.get("enabled", True)),
        low_tcp_z_m=float(chassis_raw.get("low_tcp_z_m", 0.09)),
        max_low_yaw_rad=float(chassis_raw.get("max_low_yaw_rad", 0.20)),
    )
    _require(0.0 <= chassis.max_low_yaw_rad <= 0.5, "chassis yaw limit must be 0..0.5")

    presets_raw = raw.get("presets") or []
    _require(isinstance(presets_raw, list), "presets must be a list")
    presets: list[PosePreset] = []
    seen: set[str] = set()
    for item in presets_raw:
        _require(isinstance(item, dict), "each preset must be a mapping")
        preset_id = str(item.get("id", "")).strip()
        _require(bool(preset_id), "preset id is required")
        _require(preset_id not in seen, f"duplicate preset id: {preset_id}")
        seen.add(preset_id)
        joints_raw = item.get("joints")
        _require(isinstance(joints_raw, dict), f"{preset_id}: joints must be a mapping")
        _require(
            set(joints_raw) == set(ARM_JOINTS),
            f"{preset_id}: joints must be exactly {', '.join(ARM_JOINTS)}",
        )
        joints = {name: float(joints_raw[name]) for name in ARM_JOINTS}
        _require(
            all(math.isfinite(value) for value in joints.values()),
            f"{preset_id}: joint values must be finite",
        )
        presets.append(
            PosePreset(
                preset_id=preset_id,
                label=str(item.get("label", preset_id)),
                label_en=str(item.get("label_en", preset_id)),
                joints=joints,
                verified_on_hardware=bool(item.get("verified_on_hardware", False)),
                source=str(item.get("source", "")),
            )
        )

    # ------------------------------------------------------------- G8 task
    task_raw = raw.get("task")
    task: TaskConfig | None = None
    if task_raw is not None:
        _require(isinstance(task_raw, dict), "task must be a mapping")
        hand_eye = str(task_raw.get("hand_eye", "")).strip()
        _require(bool(hand_eye), "task.hand_eye is required")
        hand_eye_path = (path.parent / hand_eye).resolve()
        _require(
            hand_eye_path.is_file(),
            f"task.hand_eye not found: {hand_eye_path}",
        )
        correction = _triple(task_raw.get("correction_mm"), "task.correction_mm")
        view_preset = str(task_raw.get("view_preset", "view_table"))
        lift_preset = str(task_raw.get("lift_preset", "lift_150"))
        preset_ids = {item.preset_id for item in presets}
        _require(
            view_preset in preset_ids,
            f"task.view_preset {view_preset!r} is not a preset",
        )
        _require(
            lift_preset in preset_ids,
            f"task.lift_preset {lift_preset!r} is not a preset",
        )
        locate_raw = task_raw.get("locate") or {}
        _require(isinstance(locate_raw, dict), "task.locate must be a mapping")
        x_range_raw = locate_raw.get("x_range_m", [0.15, 0.32])
        _require(
            isinstance(x_range_raw, (list, tuple)) and len(x_range_raw) == 2,
            "task.locate.x_range_m must have 2 values",
        )
        x_range = (float(x_range_raw[0]), float(x_range_raw[1]))
        _require(x_range[0] < x_range[1], "task.locate.x_range_m must ascend")
        locate = LocateParams(
            min_range_m=float(locate_raw.get("min_range_m", 0.15)),
            max_range_m=float(locate_raw.get("max_range_m", 0.60)),
            floor_threshold_m=float(locate_raw.get("floor_threshold_m", 0.004)),
            floor_margin_m=float(locate_raw.get("floor_margin_m", 0.006)),
            voxel_m=float(locate_raw.get("voxel_m", 0.005)),
            min_cluster_points=int(locate_raw.get("min_cluster_points", 30)),
            slab_m=float(locate_raw.get("slab_m", 0.005)),
            min_top_extent_m=float(locate_raw.get("min_top_extent_m", 0.035)),
            ransac_iterations=int(locate_raw.get("ransac_iterations", 800)),
            ransac_points=int(locate_raw.get("ransac_points", 40000)),
            x_range_m=x_range,
            y_range_m=float(locate_raw.get("y_range_m", 0.09)),
            max_xy_m=float(locate_raw.get("max_xy_m", 0.09)),
            min_height_m=float(locate_raw.get("min_height_m", 0.020)),
            max_height_m=float(locate_raw.get("max_height_m", 0.070)),
            seed=int(locate_raw.get("seed", 0)),
            cube_size_m=float(locate_raw.get("cube_size_m", 0.0)),
            recovery_min_visible_m=float(
                locate_raw.get("recovery_min_visible_m", 0.012)
            ),
            recovery_intact_extent_m=float(
                locate_raw.get("recovery_intact_extent_m", 0.030)
            ),
            recovery_edge_tol_m=float(
                locate_raw.get("recovery_edge_tol_m", 0.002)
            ),
            recovery_range_band_m=float(
                locate_raw.get("recovery_range_band_m", 0.0025)
            ),
            recovery_range_std_m=float(
                locate_raw.get("recovery_range_std_m", 0.0015)
            ),
        )
        _require(locate.min_range_m > 0, "task.locate.min_range_m must be positive")
        _require(
            locate.min_range_m < locate.max_range_m,
            "task.locate range must ascend",
        )
        _require(locate.voxel_m > 0, "task.locate.voxel_m must be positive")
        _require(
            locate.min_cluster_points >= 1,
            "task.locate.min_cluster_points must be >= 1",
        )
        _require(
            locate.min_height_m < locate.max_height_m,
            "task.locate height range must ascend",
        )
        grasp_j2 = float(task_raw.get("grasp_j2", -0.6618))
        grasp_j3 = float(task_raw.get("grasp_j3", -1.80117))
        grasp_j5 = float(task_raw.get("grasp_j5", 0.0))
        pre_offset = float(task_raw.get("pre_grasp_offset_rad", 0.15))
        preclose = float(task_raw.get("preclose_rad", -1.6))
        grip = float(task_raw.get("grip_rad", -0.63))
        _require(0.0 <= pre_offset <= 0.5, "task.pre_grasp_offset_rad must be 0..0.5")
        _require(
            gripper.min_rad <= preclose <= gripper.max_rad,
            "task.preclose_rad must be inside the gripper range",
        )
        _require(
            gripper.min_rad <= grip <= gripper.max_rad,
            "task.grip_rad must be inside the gripper range",
        )
        grip_pre_dur = float(task_raw.get("grip_preclose_duration_s", 1.3))
        grip_dur = float(task_raw.get("grip_duration_s", 2.0))
        release_dur = float(task_raw.get("release_open_duration_s", 3.5))
        for name, value in (
            ("grip_preclose_duration_s", grip_pre_dur),
            ("grip_duration_s", grip_dur),
            ("release_open_duration_s", release_dur),
        ):
            _require(
                gripper.min_duration_s <= value <= gripper.max_duration_s,
                f"task.{name} must be inside the gripper duration range",
            )
        verify_radius = float(task_raw.get("verify_radius_m", 0.04))
        verify_height = float(task_raw.get("verify_min_height_m", 0.01))
        min_reach = float(task_raw.get("min_reach_m", 0.10))
        max_reach = float(task_raw.get("max_reach_m", 0.30))
        max_xy = float(task_raw.get("max_xy_error_mm", 3.0))
        _require(verify_radius > 0, "task.verify_radius_m must be positive")
        _require(verify_height >= 0, "task.verify_min_height_m must be >= 0")
        _require(0 <= min_reach < max_reach, "task reach range must ascend")
        _require(max_xy >= 0, "task.max_xy_error_mm must be >= 0")
        task = TaskConfig(
            hand_eye_path=str(hand_eye_path),
            correction_mm=correction,
            view_preset=view_preset,
            lift_preset=lift_preset,
            grasp_j2=grasp_j2,
            grasp_j3=grasp_j3,
            grasp_j5=grasp_j5,
            pre_grasp_offset_rad=pre_offset,
            preclose_rad=preclose,
            grip_rad=grip,
            grip_preclose_duration_s=grip_pre_dur,
            grip_duration_s=grip_dur,
            release_open_duration_s=release_dur,
            verify_radius_m=verify_radius,
            verify_min_height_m=verify_height,
            min_reach_m=min_reach,
            max_reach_m=max_reach,
            max_xy_error_mm=max_xy,
            locate=locate,
        )

    canonical = {
        "schema_version": schema_version,
        "scope": scope,
        "max_velocity_rad_s": velocity,
        "min_duration_s": min_duration,
        "max_duration_s": max_duration,
        "path_samples": path_samples,
        "completion_tolerance_pulses": tolerance,
        "completion_settle_samples": settle,
        "timeout_margin_s": margin,
        "gripper": gripper.to_public_dict(),
        "workspace": workspace.to_public_dict(),
        "table_plane": table.to_public_dict(),
        "chassis_guard": chassis.to_public_dict(),
        "presets": [item.to_public_dict() for item in presets],
        "task": task.to_public_dict() if task is not None else None,
    }
    config_hash = hashlib.sha256(
        json.dumps(canonical, sort_keys=True, separators=(",", ":")).encode("utf-8")
    ).hexdigest()

    return TakeoverConfig(
        schema_version=schema_version,
        scope=scope,
        max_velocity_rad_s=velocity,
        min_duration_s=min_duration,
        max_duration_s=max_duration,
        path_samples=path_samples,
        completion_tolerance_pulses=tolerance,
        completion_settle_samples=settle,
        timeout_margin_s=margin,
        gripper=gripper,
        workspace=workspace,
        table_plane=table,
        chassis_guard=chassis,
        presets=tuple(presets),
        task=task,
        source_path=str(path),
        config_hash=config_hash,
    )
