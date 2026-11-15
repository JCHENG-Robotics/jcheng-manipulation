"""Strict, cwd-independent controller configuration selection for Gate 2."""

from __future__ import annotations

import hashlib
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import yaml
from yaml.nodes import MappingNode, SequenceNode
from yaml.tokens import AliasToken, AnchorToken, TagToken

PROFILE_FILES = {"default": "arm_controllers.yaml", "gain_0p1": "arm_controllers_gain_0p1.yaml", "gain_0p2": "arm_controllers_gain_0p2.yaml"}
_FORBIDDEN_PATH_CHARS = ("\x00", "\n", "\r", "$")


class ControllerConfigError(ValueError):
    """A user-visible controller configuration validation failure."""


@dataclass(frozen=True)
class ControllerConfig:
    profile: str
    original_input: str
    share_directory: Path
    logical_path: Path
    realpath: Path
    sha256: str


def _reject_yaml_features(text: str) -> None:
    try:
        tokens = list(yaml.scan(text))
        documents = list(yaml.compose_all(text))
    except yaml.YAMLError as exc:
        raise ControllerConfigError(f"invalid YAML: {exc}") from exc
    if any(isinstance(token, (AnchorToken, AliasToken, TagToken)) for token in tokens):
        raise ControllerConfigError("YAML anchors, aliases, and tags are not allowed")
    if len(documents) != 1 or documents[0] is None:
        raise ControllerConfigError("exactly one non-empty YAML document is required")

    def visit(node: Any) -> None:
        if isinstance(node, MappingNode):
            keys: set[str] = set()
            for key, value in node.value:
                if not isinstance(key.value, str) or key.value in keys:
                    raise ControllerConfigError(f"duplicate or non-string YAML key: {key.value!r}")
                keys.add(key.value)
                visit(value)
        elif isinstance(node, SequenceNode):
            for item in node.value:
                visit(item)
    visit(documents[0])


def _load_mapping(path: Path) -> tuple[dict[str, Any], bytes]:
    try:
        raw = path.read_bytes()
    except OSError as exc:
        raise ControllerConfigError(f"cannot read controller config {path}: {exc}") from exc
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise ControllerConfigError(f"controller config is not UTF-8: {path}") from exc
    _reject_yaml_features(text)
    try:
        data = yaml.safe_load(text)
    except yaml.YAMLError as exc:
        raise ControllerConfigError(f"invalid YAML: {exc}") from exc
    if not isinstance(data, dict):
        raise ControllerConfigError("controller config root must be a mapping")
    return data, raw


def _validate_common(data: dict[str, Any]) -> None:
    try:
        manager = data["controller_manager"]["ros__parameters"]
        params = data["arm_controller"]["ros__parameters"]
    except (KeyError, TypeError) as exc:
        raise ControllerConfigError("missing required ros__parameters mapping") from exc
    if not isinstance(manager, dict) or set(manager) != {"update_rate", "joint_state_broadcaster", "arm_controller"}:
        raise ControllerConfigError("controller_manager.ros__parameters has an invalid schema")
    if type(manager["update_rate"]) is not int or manager["update_rate"] != 100:
        raise ControllerConfigError("controller_manager.update_rate must be integer 100")
    if manager["joint_state_broadcaster"] != {"type": "joint_state_broadcaster/JointStateBroadcaster"}:
        raise ControllerConfigError("invalid joint_state_broadcaster declaration")
    if manager["arm_controller"] != {"type": "joint_trajectory_controller/JointTrajectoryController"}:
        raise ControllerConfigError("invalid arm_controller declaration")
    expected = {"joints", "command_interfaces", "state_interfaces", "open_loop_control", "allow_nonzero_velocity_at_trajectory_start"}
    if not isinstance(params, dict) or set(params) != expected:
        raise ControllerConfigError("arm_controller.ros__parameters has an invalid schema")
    if params["joints"] != ["joint1", "joint2"] or params["command_interfaces"] != ["position"] or params["state_interfaces"] != ["position", "velocity"]:
        raise ControllerConfigError("invalid arm controller joint/interface configuration")
    if type(params["open_loop_control"]) is not bool or params["open_loop_control"] is not False:
        raise ControllerConfigError("open_loop_control must be boolean false")
    if type(params["allow_nonzero_velocity_at_trajectory_start"]) is not bool or params["allow_nonzero_velocity_at_trajectory_start"] is not True:
        raise ControllerConfigError("allow_nonzero_velocity_at_trajectory_start must be boolean true")


def _validate_schema(data: dict[str, Any], profile: str) -> None:
    root = {"controller_manager", "arm_controller"}
    if profile == "default":
        if set(data) != root:
            raise ControllerConfigError("default controller config must contain only controller_manager and arm_controller")
        _validate_common(data)
        return
    if set(data) != root | {"/gz_ros_control"}:
        raise ControllerConfigError("V2 controller config must contain controller_manager, arm_controller, and /gz_ros_control only")
    _validate_common(data)
    try:
        node = data["/gz_ros_control"]
        gain = node["ros__parameters"]["position_proportional_gain"]
    except (KeyError, TypeError) as exc:
        raise ControllerConfigError("V2 profile requires /gz_ros_control position_proportional_gain") from exc
    if not isinstance(node, dict) or set(node) != {"ros__parameters"} or not isinstance(node["ros__parameters"], dict) or set(node["ros__parameters"]) != {"position_proportional_gain"}:
        raise ControllerConfigError("/gz_ros_control has an invalid schema")
    expected_gain = 0.1 if profile == "gain_0p1" else 0.2
    if type(gain) is not float or gain != expected_gain:
        raise ControllerConfigError(f"position_proportional_gain must be float {expected_gain}")


def resolve_controller_config(package_share: str, profile: str, controller_config: str = "") -> ControllerConfig:
    """Resolve and validate a controller YAML without using cwd or basename heuristics."""
    if profile not in PROFILE_FILES:
        raise ControllerConfigError(f"unsupported controller_profile {profile!r}; allowed: {sorted(PROFILE_FILES)}")
    try:
        share = Path(package_share).resolve(strict=True)
        config_root = (share / "config").resolve(strict=True)
    except OSError as exc:
        raise ControllerConfigError(f"cannot resolve package share {package_share!r}: {exc}") from exc
    original = controller_config
    if controller_config:
        if controller_config != controller_config.strip() or any(c in controller_config for c in _FORBIDDEN_PATH_CHARS):
            raise ControllerConfigError("controller_config contains forbidden whitespace/control/$ characters")
        supplied = Path(controller_config).expanduser()
        logical = supplied if supplied.is_absolute() else config_root / supplied
    else:
        logical = config_root / PROFILE_FILES[profile]
    try:
        resolved = logical.resolve(strict=True)
    except OSError as exc:
        raise ControllerConfigError(f"controller_config does not resolve: {logical}: {exc}") from exc
    if not resolved.is_file():
        raise ControllerConfigError(f"controller_config is not a regular file: {resolved}")
    data, raw = _load_mapping(resolved)
    _validate_schema(data, profile)
    return ControllerConfig(profile, original, share, logical, resolved, hashlib.sha256(raw).hexdigest())
