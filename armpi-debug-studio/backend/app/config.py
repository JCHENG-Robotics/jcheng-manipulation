"""Runtime configuration for the G1 backend."""

from __future__ import annotations

import os
from dataclasses import dataclass
from pathlib import Path

from .domain.enums import Mode
from .domain.mapping import MappingConfig, load_mapping

BACKEND_ROOT = Path(__file__).resolve().parents[1]
PROJECT_ROOT = BACKEND_ROOT.parent
DEFAULT_MAPPING = PROJECT_ROOT / "configs" / "joint_mapping.yaml"


def _env_bool(name: str, default: bool) -> bool:
    raw = os.getenv(name)
    if raw is None:
        return default
    return raw.strip().lower() in {"1", "true", "yes", "on"}


@dataclass(frozen=True)
class Settings:
    mode: Mode
    robot_id: str
    bind_host: str
    port: int
    db_path: Path
    mapping_path: Path
    frontend_dist: Path
    log_level: str
    hardware_motion_enabled: bool
    api_version: str = "0.1.0"
    schema_version: int = 1

    @classmethod
    def from_env(cls, *, mode: str | None = None, db_path: str | None = None) -> "Settings":
        raw_mode = (mode or os.getenv("ARMPI_MODE", "mock")).strip().lower()
        if raw_mode not in {Mode.MOCK.value, Mode.HARDWARE.value}:
            raise ValueError(f"ARMPI_MODE must be 'mock' or 'hardware', got {raw_mode!r}")
        resolved_mode = Mode(raw_mode)

        env_mode = os.getenv("ARMPI_MODE", "mock").strip().lower()
        # Hardware motion is impossible in mock mode. In hardware mode it still
        # requires an explicit opt-in; G1 defaults to false.
        motion_flag = _env_bool("ARMPI_HARDWARE_MOTION", False)
        hardware_motion_enabled = resolved_mode is Mode.HARDWARE and motion_flag

        return cls(
            mode=resolved_mode,
            robot_id=os.getenv("ARMPI_ROBOT_ID", "armpi-ultra-01"),
            bind_host=os.getenv("BIND_HOST", "127.0.0.1"),
            port=int(os.getenv("PORT", "8000")),
            db_path=Path(db_path or os.getenv("DB_PATH", str(PROJECT_ROOT / "data" / "studio.db"))),
            mapping_path=Path(os.getenv("ARMPI_JOINT_MAPPING", str(DEFAULT_MAPPING))),
            frontend_dist=Path(os.getenv("FRONTEND_DIST", str(PROJECT_ROOT / "frontend" / "dist"))),
            log_level=os.getenv("LOG_LEVEL", "info"),
            hardware_motion_enabled=hardware_motion_enabled,
        )

    def load_mapping(self) -> MappingConfig:
        return load_mapping(self.mapping_path)
