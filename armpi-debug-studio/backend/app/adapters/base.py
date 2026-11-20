"""Robot adapter boundary.

The backend only talks to an adapter. Mock and hardware are separate
implementations; the hardware adapter has no motion write path in G1.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any

from ..domain.enums import CommandStage, Mode, SourceLabel
from ..domain.models import (
    BatteryState,
    CameraStatus,
    Capabilities,
    ConnectionReport,
    RobotState,
    StopResult,
)


class HardwareMotionDisabled(RuntimeError):
    """Raised by the hardware adapter when motion is requested in G1."""


class AdapterError(RuntimeError):
    """Structured adapter failure."""

    def __init__(self, code: str, message: str, details: dict[str, Any] | None = None) -> None:
        super().__init__(message)
        self.code = code
        self.details = details or {}


@dataclass
class DispatchResult:
    status: CommandStage
    error_code: str | None
    detail: str
    details: dict[str, Any] = field(default_factory=dict)


class RobotAdapter(ABC):
    mode: Mode
    source_label: SourceLabel
    capabilities: Capabilities

    @abstractmethod
    def tick(self) -> None:
        """Advance simulated time or poll the hardware boundary."""

    @abstractmethod
    def snapshot(self) -> RobotState:
        """Return the latest state, clearly labelled with its truth source."""

    @abstractmethod
    def connection_report(self) -> ConnectionReport:
        """Return four-segment connection evidence."""

    @abstractmethod
    def battery(self) -> BatteryState:
        """Return the latest battery sample, or an explicit unavailable state."""

    @abstractmethod
    def camera_status(self) -> CameraStatus:
        """Return camera health, or an explicit unavailable state."""

    @abstractmethod
    def camera_snapshot(self) -> tuple[bytes, str] | None:
        """Return (image bytes, media type), or None when no frame is available."""

    def aux_camera_status(self) -> CameraStatus:
        """Return the off-arm UVC camera health; unavailable by default."""
        return CameraStatus(
            available=False,
            truth=Truth.UNKNOWN,
            validity=Validity.UNAVAILABLE,
            source="adapter",
            topic="uvc:disabled",
            width=None,
            height=None,
            encoding=None,
            fps=None,
            last_frame_at=None,
            sample_age_ms=None,
            frame_count=0,
            notes=["aux camera is not supported by this adapter"],
        )

    def aux_camera_snapshot(self) -> tuple[bytes, str] | None:
        """Return the off-arm UVC frame, or None when unavailable."""
        return None

    def tf_snapshot(self) -> dict[str, Any] | None:
        """Return a read-only TF snapshot when the adapter supports one."""
        return None

    @abstractmethod
    def dispatch(self, targets: list[dict[str, Any]], command_id: str) -> DispatchResult:
        """Send a validated target set. Hardware must reject in G1."""

    @abstractmethod
    def stop(self, command_id: str) -> StopResult:
        """Request a stop. The result is evidence-based, never an emergency stop."""

    @abstractmethod
    def drain_events(self) -> list[dict[str, Any]]:
        """Return adapter events (observed/completed/unknown) for the audit log."""

    @abstractmethod
    def reset(self) -> None:
        """Reset mock faults/motion. Hardware must not implement side effects."""
