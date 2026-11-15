# ArmPi Debug Studio

A safe, evidence-first debugging console for the ArmPi Ultra 5-DoF arm.

**Current milestone: G1-Mock vertical slice (not PRD G1 acceptance).** The Mock path is runnable
end-to-end. Hardware mode is a read-only placeholder and deliberately rejects
all motion. The vendor SDK/Driver are referenced read-only and are **not**
copied into this directory.

> Status note (2026-11-15): this directory is the G1 **Mock** slice imported into
> the main repository. It is **not** a PRD G1 pass: E7 (ROS 2 read-only gateway)
> and supervised hardware acceptance are not implemented. See
> `docs/audit/2026-11-15/PROJECT_STATUS_AUDIT.md` and `DECISION_REQUIRED.md`.

## Status

| Area | Status | Evidence |
|---|---|---|
| Mock adapter + fault injection | Implemented | `backend/app/adapters/mock.py`, 41 backend tests |
| Backend REST + WebSocket | Implemented | `backend/app/api/routes.py` |
| Safety core (validation/gate/lease/token/idempotency) | Implemented | `backend/app/safety/`, `backend/tests/test_safety.py` |
| SQLite append-only event log + CSV export | Implemented | `backend/app/persistence/store.py` |
| Dashboard / Joint Control / Diagnostics / Event Timeline / Settings | Implemented | `frontend/src/pages/` |
| Frontend type-check + production build | Verified | `tsc -b && vite build` |
| Mock HTTP end-to-end | Verified | preview → execute → Completed; see below |
| E7 ROS 2 read-only gateway | **Not implemented** | `backend/app/adapters/hardware.py` |
| Hardware motion | **Disabled by design** | `capabilities.hardware_motion=false` |
| Gripper (`r_joint`) | **Excluded from G1** | `configs/joint_mapping.yaml` |
| Trajectory Inspector / Experiment Lab / 3D / camera | **Not implemented** | Settings → Roadmap |

No hardware was accessed while building or testing this slice: no serial port,
no ROS node, no robot motion.

## Architecture

```text
Browser (React + TS)
   │ HTTP / WebSocket
   ▼
FastAPI backend
   ├── SafetySupervisor / LeaseManager / PreviewTokenManager
   ├── MockRobotAdapter (G1; no ROS, no serial)
   ├── HardwareAdapter (G1 stub; motion raises)
   └── SQLite event store (append-only, WAL)
```

The browser never talks to ROS or a serial port. The backend exposes named
business actions with structured validation results, not raw device commands.

## Layout

```text
armpi-debug-studio/
├── backend/
│   ├── app/
│   │   ├── api/            REST + WebSocket routes
│   │   ├── adapters/       mock and hardware-stub implementations
│   │   ├── domain/         enums, models, state machine, mapping
│   │   ├── persistence/    SQLite event store
│   │   ├── safety/         validation, supervisor, lease, preview token
│   │   ├── config.py
│   │   ├── service.py
│   │   └── main.py
│   └── tests/              41 unit/API tests
├── configs/
│   └── joint_mapping.yaml  provisional vendor-derived mapping
└── frontend/
    ├── src/api/            typed client + zod contract checks
    ├── src/components/     AppShell and shared UI primitives
    ├── src/pages/          five G1 pages
    └── src/store/          zustand state + persisted display preferences
```

## Quick start (Mock mode; no ROS, no hardware)

> **Path note:** the commands below are relative to `armpi-debug-studio/`.
> If your prompt already ends in `armpi-debug-studio`, run `cd backend`, not
> `cd armpi-debug-studio/backend`. The Python project file is
> `backend/pyproject.toml`; the Node project file is `frontend/package.json`.

### Backend

```bash
cd backend
python3 -m venv .venv
. .venv/bin/activate
pip install -e '.[dev]'
uvicorn app.main:create_app --factory --host 127.0.0.1 --port 8000
```

Useful environment variables:

| Variable | Default | Meaning |
|---|---|---|
| `ARMPI_MODE` | `mock` | `mock` or `hardware` |
| `DB_PATH` | `../data/studio.db` | SQLite event store |
| `ARMPI_JOINT_MAPPING` | `../configs/joint_mapping.yaml` | mapping file |
| `BIND_HOST` | `127.0.0.1` | do not expose publicly in G1 |
| `ARMPI_HARDWARE_MOTION` | `false` | must stay false in G1 |

### Frontend

```bash
cd frontend
npm install
npm run dev
```

Open <http://127.0.0.1:5173>. The page must display `SIMULATION` and Mock data.
There is no serial access, no ROS node, and no hardware write path.

For a production build:

```bash
cd frontend
npm run build
```

If `frontend/dist/index.html` exists, the FastAPI app serves it at `/`.

### Tests

```bash
cd backend
. .venv/bin/activate
pytest
```

## Access from another device

> Replace `<user>` and `<M70Q_TAILSCALE_IP>` with your own SSH user and host.

G1 has **no login/authentication**. Do not expose the backend or the Vite dev
server to the public internet. Prefer SSH tunneling or a private Tailscale
network.

### Recommended: SSH tunnel, one port

On the machine running the backend:

```bash
cd frontend
npm install
npm run build
cd ../backend
. .venv/bin/activate
uvicorn app.main:create_app --factory --host 127.0.0.1 --port 8000
```

On the other device:

```bash
ssh -L 8000:127.0.0.1:8000 <user>@<M70Q_TAILSCALE_IP>
```

Open <http://127.0.0.1:8000> on that device. FastAPI serves the built frontend
and the API on the same origin. If the backend was already running before
`npm run build`, restart it so the static mount is created.

### Remote Vite dev server over Tailscale/LAN

Keep the backend on localhost and expose only the Vite dev server:

```bash
# terminal 1: backend
cd backend
. .venv/bin/activate
uvicorn app.main:create_app --factory --host 127.0.0.1 --port 8000

# terminal 2: frontend
cd frontend
npm run dev -- --host 0.0.0.0
```

On another device on the same tailnet/LAN, open
`http://<M70Q_TAILSCALE_IP>:5173`. Vite proxies `/api` and `/ws` to the remote
backend. Allow TCP 5173 only on the trusted network.

### Frontend on the other device

If Node is not installed on the robot/remote host, run Vite on the other device:

```bash
# other device: tunnel the backend port
ssh -L 8000:127.0.0.1:8000 <user>@<M70Q_TAILSCALE_IP>

# other device, second terminal: copy the frontend once and run it
scp -r <user>@<M70Q_TAILSCALE_IP>:~/projects/ros2-industrial-arm-learning/armpi-debug-studio/frontend .
cd frontend
npm install
npm run dev
```

Open <http://127.0.0.1:5173> on that device. Vite proxies to the tunneled
`127.0.0.1:8000`.

### Direct binding (trusted LAN/Tailscale only)

If you explicitly want the backend reachable without a tunnel:

```bash
uvicorn app.main:create_app --factory --host 0.0.0.0 --port 8000
```

Then open `http://<remote-ip>:8000` after building the frontend. Allow TCP 8000
in the firewall only for the trusted interface. Never port-forward it on the
public internet; there is no authentication in G1.

If you serve the frontend from a different origin and call the API directly
instead of through the Vite proxy, the G1 CORS allowlist only includes
`localhost:5173`. Use one of the tunnel/proxy recipes above, or add an explicit
CORS/authentication review before exposing it.

## API surface

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/health` | mode, source label, capabilities |
| GET | `/api/state` | latest `RobotState` |
| GET | `/api/connection` | four-segment connection evidence |
| GET | `/api/safety` | safety gate checks |
| GET | `/api/diagnostics` | mapping, defects, environment, probe availability |
| POST | `/api/diagnostics/probe` | simulated single-shot probe (Mock only) |
| POST | `/api/control/acquire` | acquire the single control lease |
| POST | `/api/control/heartbeat` | renew the lease |
| POST | `/api/control/release` | release the lease |
| POST | `/api/motion/preview` | validate target(s) and issue a single-use token |
| POST | `/api/motion/execute` | lease + token + safety gate → dispatch |
| POST | `/api/motion/stop` | software stop request; four-state result |
| GET | `/api/events` | append-only audit events |
| GET | `/api/events/export.csv` | CSV export with schema version |
| POST | `/api/mock/faults` | inject a Mock fault |
| POST | `/api/mock/reset` | clear Mock faults/state |
| WS | `/api/ws/state` | state stream (10 Hz) |
| WS | `/api/ws/events` | event stream |

`POST /api/motion/execute` requires an `Idempotency-Key` header (or body
`idempotency_key`) and the `command_id` + `preview_token` returned by preview.

## Safety invariants

- The backend never opens a serial port and never imports `rclpy`.
- Hardware Mode advertises `hardware_motion=false`; execute requests are
  rejected with `GATE_REJECTED`.
- Every execute requires a valid control lease, a valid single-use preview
  token, and all safety checks passing.
- The state machine never produces `Acknowledged`; the vendor SDK has no ACK.
- `Dispatched → Completed` is illegal; observed state must be explicit.
- Events are append-only. A storage failure latches a critical fault and locks
  out further motion.
- The UI says **停止请求** (stop request), never **急停** (emergency stop).
- Mock values are labelled `SIMULATION` and `truth=simulated` everywhere.

## Verification performed (2026-11-14)

```text
backend:  41 passed
frontend: tsc -b && vite build → success
HTTP E2E: health ok; state SIMULATION/5 joints; lease; preview valid;
          execute → Dispatched; idempotent replay → same command;
          after 1.2 s → joint1 position 0.1 rad, truth=simulated;
          events Requested/Validated/Dispatched/Observed/Completed;
          empty_feedback → all joints unavailable; reset → valid;
          stop → StopObserved with "not a hardware emergency stop";
          CSV export contains event_id.
```

These are Mock/simulation results. **They are not hardware validation.** No
ArmPi device was connected, no serial port was opened, and no motion command
was sent to a robot.

## Known limitations

1. E7 ROS 2 read-only gateway is not implemented; Hardware Mode cannot read
   real state yet.
2. Hardware motion is disabled; the vendor driver defects (no ACK, blocking
   reads, `joint_states` command echo) are not repaired.
3. The joint mapping and limits are provisional (`configs/joint_mapping.yaml`).
   They are not mechanical measurements.
4. Mock API is unauthenticated and bound to localhost by default. Do not expose
   it on a LAN/public interface without a separate security review.
5. Frontend has no automated Playwright suite yet; backend tests and a production
   build were run.
6. G2–G4 (trajectory, experiments, 3D/camera) are not implemented and have no
   routes or buttons in the UI.

## Next steps

1. E7: read-only ROS 2 gateway (`CapabilityProbe`, `StateReader`, watchdog,
   `NOT_IMPLEMENTED` for dispatch/stop).
2. Board decisions on hardware mapping verification and controlled single-joint
   tests.
3. E8: Playwright E2E + contract tests for the frontend.
4. G2 offline trajectory recording/replay before any hardware trajectory work.
