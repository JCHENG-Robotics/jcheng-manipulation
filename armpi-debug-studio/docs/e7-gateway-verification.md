# E7 read-only gateway — hardware verification

Date: 2026-11-17

Scope: real ArmPi Ultra Pi 5, direct Ethernet, read-only gateway + backend
`hardware` mode + Dashboard. **No motion was dispatched.**

## 1. Environment

| Item | Value |
|---|---|
| Robot | Pi 5, `eth0=10.42.0.218/24`, vendor ROS 2 Humble stack running |
| ROS runtime | vendor container `ArmPiUltra` (`ros:humble`, host network) |
| Laptop | studio backend `127.0.0.1:8000`; direct link `10.42.0.1/24` |
| Gateway | `armpi_debug_gateway` bound to `10.42.0.218:8765`, token required |

The container has OpenCV 4.10 and numpy 1.26 but no `cv_bridge`. The gateway
therefore used its numpy fallback (`ros2_gateway/armpi_debug_gateway/imaging.py`)
to decode the `bgr8` RGB stream. This was the first live exercise of that path.

## 2. Gateway results

```text
GET /health            status=ok; ros/battery/camera/joints ready; motion_enabled=false
GET /battery           7424 mV; 51.2%; 2S; truth=measured; validity=valid
GET /camera/status     640x400 bgr8; 8.5 fps; truth=measured
GET /camera/snapshot   JPEG 640x400; 20415 bytes
GET /state             5 joints + gripper; truth=commanded
GET /health (no token) 401
POST /motion/dispatch  501 NOT_IMPLEMENTED
```

Raw responses: [`evidence/studio_gateway/20261117/`](../../../evidence/studio_gateway/20261117/).

## 3. Backend hardware mode

Started with:

```bash
export ARMPI_MODE=hardware
export ARMPI_GATEWAY_URL=http://10.42.0.218:8765
export ARMPI_GATEWAY_TOKEN='<session token>'
export DB_PATH=/tmp/armpi_studio_hardware_20261117.db
uvicorn app.main:create_app --factory --host 127.0.0.1 --port 8000
```

Results:

```text
GET /api/health        mode=hardware; source_label=LIVE; hardware_motion=false; hardware_read=true
GET /api/peripherals   battery + camera measured/valid; 7.424 V; 640x400; 8.5 fps
GET /api/connection    driver/gateway/feedback online; serial unknown (vendor-owned)
GET /api/state         joints commanded/valid; limits provisional
GET /api/safety        blockers: mapping_verified, hardware_motion_enabled
POST /api/motion/preview   valid; single-use token issued
POST /api/motion/execute   GATE_REJECTED; same two blockers
```

The execute request carried a valid lease, a valid preview token and an
`Idempotency-Key`; the backend still rejected it before any adapter dispatch.
The gateway has no motion endpoint, so two independent layers kept the arm
still.

## 4. Dashboard (browser)

The built frontend was served by FastAPI and opened at
`http://127.0.0.1:8000/`:

- Header: `Source LIVE`, `Connection ReadOnly`, `Feedback age` in milliseconds.
- Camera · RGB: `measured`, `valid`, 640×400, `bgr8`, ~5.8–8.5 fps, a live
  workspace frame (not the Mock SVG).
- Battery · 2S: `measured`, `ok`, 7.42 V, 51%, source
  `armpi_debug_gateway`, `valid`.
- Connection chain: driver/gateway/feedback `online`.
- Safety gate: 2 blockers (`mapping_verified`, `hardware_motion_enabled`).
- Joint table: 5 joints, `commanded`/`valid`, limits marked provisional.
- Active faults: `HARDWARE_MOTION_DISABLED`.

The browser DOM snapshot and a screenshot were taken during the run. The
screenshot contains the live workspace image and is **not** committed; the
numerical evidence and the exact UI text are in the evidence directory.

## 5. What this does NOT prove

- No hardware motion: execute is blocked, and the gateway has no write path.
- No encoder feedback: `/controller_manager/joint_states` is a command echo.
- No validated joint limits or mapping (`provisional`).
- No persistent deployment: the gateway was started manually with
  `docker exec`; it will not survive a reboot/container restart.
- No Playwright suite: the UI check was a one-off browser session, not a
  committed automated test.
- No authentication review: the gateway uses a shared token and was bound only
  to the direct-Ethernet address. Do not expose it on a shared/public network.

## 6. Reproduce

See
[`evidence/studio_gateway/20261117/README.md`](../../../evidence/studio_gateway/20261117/README.md)
for the copy/start commands and the full payload list. Rebuild the frontend
(`npm run build`) before starting the backend if `frontend/dist` is stale.
