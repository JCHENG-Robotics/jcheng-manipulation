"""Robot adapter boundary.

The backend only talks to an adapter. Mock and hardware are separate
implementations; the hardware adapter has no motion write path in G1.
"""

from __future__ import annotations

from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any

from ..domain.enums import CommandStage, Mode, SourceLabel
from ..domain.models import Capabilities, ConnectionReport, RobotState, StopResult


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
