"""REST and WebSocket routes.

The API never exposes a serial port or arbitrary ROS call. It exposes named
business actions with structured validation results.
"""

from __future__ import annotations

import asyncio
from typing import Any

from fastapi import APIRouter, Header, Query, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import PlainTextResponse
from pydantic import BaseModel, Field

from ..domain.models import MotionTarget
from ..errors import ServiceError
from ..service import RobotService

router = APIRouter(prefix="/api")


class TargetIn(BaseModel):
    joint: str = Field(min_length=1, max_length=64)
    target_rad: float
    duration_s: float


class PreviewIn(BaseModel):
    session_id: str = Field(min_length=1, max_length=128)
    targets: list[TargetIn] = Field(min_length=1, max_length=12)


class ExecuteIn(PreviewIn):
    lease_id: str = Field(min_length=1, max_length=128)
    preview_token: str = Field(min_length=1, max_length=256)
    command_id: str = Field(min_length=1, max_length=128)
    idempotency_key: str | None = Field(default=None, max_length=256)


class LeaseIn(BaseModel):
    session_id: str = Field(min_length=1, max_length=128)
    ttl_s: int | None = Field(default=None, ge=1, le=3600)


class HeartbeatIn(LeaseIn):
    lease_id: str = Field(min_length=1, max_length=128)


class ReleaseIn(BaseModel):
    lease_id: str = Field(min_length=1, max_length=128)
    session_id: str = Field(min_length=1, max_length=128)


class StopIn(BaseModel):
    session_id: str = Field(min_length=1, max_length=128)
    lease_id: str | None = Field(default=None, max_length=128)
    command_id: str | None = Field(default=None, max_length=128)


class FaultIn(BaseModel):
    kind: str = Field(min_length=1, max_length=64)
    joint: str | None = Field(default=None, max_length=64)


def _service(request: Request) -> RobotService:
    return request.app.state.service


def _targets(items: list[TargetIn]) -> list[MotionTarget]:
    return [
        MotionTarget(joint=item.joint, target_rad=item.target_rad, duration_s=item.duration_s)
        for item in items
    ]


@router.get("/health")
def health(request: Request) -> dict[str, Any]:
    return _service(request).health()


@router.get("/state")
def state(request: Request) -> dict[str, Any]:
    return _service(request).get_state()


@router.get("/connection")
def connection(request: Request) -> dict[str, Any]:
    return _service(request).connection_report()


@router.get("/safety")
def safety(
    request: Request,
    lease_id: str | None = Query(default=None),
    session_id: str | None = Query(default=None),
    require_lease: bool = Query(default=False),
) -> dict[str, Any]:
    return _service(request).safety_report(
        lease_id=lease_id, session_id=session_id, require_lease=require_lease
    )


@router.get("/diagnostics")
def diagnostics(request: Request) -> dict[str, Any]:
    return _service(request).diagnostics()


@router.post("/diagnostics/probe")
def probe(request: Request) -> dict[str, Any]:
    return _service(request).probe()


@router.post("/control/acquire")
def acquire(request: Request, body: LeaseIn) -> dict[str, Any]:
    return _service(request).acquire_lease(body.session_id, body.ttl_s)


@router.post("/control/heartbeat")
def heartbeat(request: Request, body: HeartbeatIn) -> dict[str, Any]:
    return _service(request).heartbeat(body.lease_id, body.session_id, body.ttl_s)


@router.post("/control/release")
def release(request: Request, body: ReleaseIn) -> dict[str, Any]:
    return _service(request).release_lease(body.lease_id, body.session_id)


@router.post("/motion/preview")
def preview(request: Request, body: PreviewIn) -> dict[str, Any]:
    return _service(request).preview(body.session_id, _targets(body.targets))


@router.post("/motion/execute")
def execute(
    request: Request,
    body: ExecuteIn,
    idempotency_header: str | None = Header(default=None, alias="Idempotency-Key"),
) -> dict[str, Any]:
    key = body.idempotency_key or idempotency_header
    if not key:
        raise ServiceError(
            "BAD_REQUEST",
            "Idempotency-Key header or body.idempotency_key is required",
            status_code=400,
        )
    return _service(request).execute(
        session_id=body.session_id,
        lease_id=body.lease_id,
        preview_token=body.preview_token,
        command_id=body.command_id,
        targets=_targets(body.targets),
        idempotency_key=key,
    )


@router.post("/motion/stop")
def stop(request: Request, body: StopIn) -> dict[str, Any]:
    return _service(request).stop(
        session_id=body.session_id,
        lease_id=body.lease_id,
        command_id=body.command_id,
    )


@router.get("/events")
def events(
    request: Request,
    limit: int = Query(default=200, ge=1, le=2000),
    stage: str | None = Query(default=None),
    source: str | None = Query(default=None),
    command_id: str | None = Query(default=None),
    after_seq: int = Query(default=0, ge=0),
) -> dict[str, Any]:
    return _service(request).events(
        limit=limit,
        stage=stage,
        source=source,
        command_id=command_id,
        after_seq=after_seq,
    )


@router.get("/events/export.csv", response_class=PlainTextResponse)
def export_events(request: Request) -> PlainTextResponse:
    csv_text = _service(request).export_csv()
    return PlainTextResponse(
        content=csv_text,
        media_type="text/csv",
        headers={"Content-Disposition": 'attachment; filename="armpi-events.csv"'},
    )


@router.post("/mock/faults")
def inject_fault(request: Request, body: FaultIn) -> dict[str, Any]:
    return _service(request).inject_fault(body.kind, body.joint)


@router.post("/mock/reset")
def reset_mock(request: Request) -> dict[str, Any]:
    return _service(request).reset_mock()


@router.websocket("/ws/state")
async def ws_state(websocket: WebSocket) -> None:
    service: RobotService = websocket.app.state.service
    await websocket.accept()
    try:
        while True:
            await websocket.send_json(service.get_state())
            await asyncio.sleep(0.1)
    except WebSocketDisconnect:
        return
    except Exception:
        with_suppress = getattr(websocket, "close", None)
        if with_suppress is not None:
            try:
                await websocket.close()
            except Exception:
                pass


@router.websocket("/ws/events")
async def ws_events(websocket: WebSocket) -> None:
    service: RobotService = websocket.app.state.service
    try:
        after = int(websocket.query_params.get("after", "0"))
    except ValueError:
        after = 0
    await websocket.accept()
    try:
        while True:
            payload = service.events(after_seq=after, limit=100)
            items = sorted(payload["items"], key=lambda item: item["seq"])
            for item in items:
                await websocket.send_json(item)
                after = max(after, int(item["seq"]))
            await asyncio.sleep(0.25)
    except WebSocketDisconnect:
        return
    except Exception:
        try:
            await websocket.close()
        except Exception:
            pass
