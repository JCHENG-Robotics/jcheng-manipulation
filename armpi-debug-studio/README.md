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

> Status note (2026-11-17, later): camera/battery and the read-only gateway are
> verified on the real robot over direct Ethernet, and the gateway now runs as
> an enabled systemd service (clean SIGTERM stop/start verified; reboot test
> deferred). `pytest` reports **48 passed**, `npm run build` succeeds, the
> Playwright mock E2E suite reports **7 passed**, gateway `/battery`,
> `/camera/status`, `/camera/snapshot` and `/state` return live data, and the
> backend `hardware` mode rejects execute with `GATE_REJECTED`. See
> [`docs/e7-gateway-verification.md`](docs/e7-gateway-verification.md) and
> [`frontend/e2e/README.md`](frontend/e2e/README.md). Hardware motion remains
> disabled.

> Status note (2026-11-18): the **G2 Trajectory Inspector** is implemented for
> offline JSON datasets. Planned / commanded / feedback streams are stored
> separately, metrics are recomputable from CSV, and replay is client-side only.
> The bundled demo is explicitly `simulated`; no hardware trajectory has been
> captured. Backend `pytest` reports **56 passed**, the Playwright mock E2E suite
> reports **8 passed**. See
> [`docs/g2-trajectory-inspector.md`](docs/g2-trajectory-inspector.md) and
> [`evidence/studio_g2/20261118/`](../evidence/studio_g2/20261118/README.md).

> Status note (2026-11-18, later): the **G3 Experiment Lab** is implemented for
> offline experiment definitions and append-only paired trials, with failure
> codes, G2 dataset references, CSV/JSON export and the Phase 3 paired analysis
> (seeded bootstrap, sign test, McNemar). The bundled demo is `simulated`.
> Backend `pytest` reports **65 passed**, the Playwright mock E2E suite reports
> **9 passed**. See [`docs/g3-experiment-lab.md`](docs/g3-experiment-lab.md) and
> [`evidence/studio_g3/20261118/`](../evidence/studio_g3/20261118/README.md).

> Status note (2026-11-18, latest): the **G4 Visual Workbench** is implemented
> as a read-only URDF forward-kinematics view plus optional gateway `/tf`,
> camera/calibration/TF warnings and metadata-only camera references. It has no
> motion path and never enters the safety chain. Backend `pytest` reports
> **74 passed**, the Playwright mock E2E suite reports **10 passed**. See
> [`docs/g4-visual-workbench.md`](docs/g4-visual-workbench.md) and
> [`evidence/studio_g4/20261118/`](../evidence/studio_g4/20261118/README.md).

## Status

| Area | Status | Evidence |
|---|---|---|
| Mock adapter + fault injection | Implemented | `backend/app/adapters/mock.py`, 41 baseline tests + 5 peripheral tests |
| Backend REST + WebSocket | Implemented | `backend/app/api/routes.py` |
| Safety core (validation/gate/lease/token/idempotency) | Implemented | `backend/app/safety/`, `backend/tests/test_safety.py` |
| SQLite append-only event log + CSV export | Implemented | `backend/app/persistence/store.py` |
| Dashboard / Joint Control / Diagnostics / Trajectory Inspector / Experiment Lab / Event Timeline / Settings | Implemented | `frontend/src/pages/` |
| Frontend type-check + production build | Verified | `tsc -b && vite build` |
| Mock HTTP end-to-end | Verified | preview → execute → Completed; see below |
| Playwright browser E2E (mock) | **Verified (2026-11-18)** | `npm run test:e2e`, 10 passed; [`frontend/e2e/README.md`](frontend/e2e/README.md) |
| Camera + battery (Mock) | Implemented | `/api/peripherals`, Dashboard panels |
| Camera + battery (hardware) | **Verified read-only (2026-11-17)** | live RGB 640×400 `bgr8` and 7.424 V battery via gateway |
| Read-only ROS 2 gateway for camera/battery/state | **Verified read-only + persistent service (2026-11-17)** | [`docs/e7-gateway-verification.md`](docs/e7-gateway-verification.md), [`ros2_gateway/deploy/README.md`](ros2_gateway/deploy/README.md) |
| G2 Trajectory Inspector (offline) | **Implemented, offline-validated (2026-11-18)** | [`docs/g2-trajectory-inspector.md`](docs/g2-trajectory-inspector.md); 3-stream JSON import, recomputable metrics, client-side replay |
| G3 Experiment Lab (offline) | **Implemented, offline-validated (2026-11-18)** | [`docs/g3-experiment-lab.md`](docs/g3-experiment-lab.md); immutable definitions, append-only paired trials, failure codes, CSV/JSON export, paired analysis |
| G4 Visual Workbench (read-only) | **Implemented, mock/offline-validated (2026-11-18)** | [`docs/g4-visual-workbench.md`](docs/g4-visual-workbench.md); URDF FK + optional gateway `/tf`, camera/calibration/TF warnings, metadata-only references |
| Hardware motion | **Disabled by design** | `capabilities.hardware_motion=false` |
| Gripper (`r_joint`) | **Excluded from G1** | `configs/joint_mapping.yaml` |
| 3D / depth-cloud workbench | **Not implemented** | Settings → Roadmap |

No hardware was accessed while building or testing the Mock slice: no serial
port, no ROS node, no robot motion. The read-only gateway was later verified on
the robot host (2026-11-17); its battery and camera payloads are now hardware
evidence, but joint positions remain a command echo and motion remains off.

## Architecture

```text
Browser (React + TS)
   │ HTTP / WebSocket
   ▼
FastAPI backend
   ├── SafetySupervisor / LeaseManager / PreviewTokenManager
   ├── MockRobotAdapter (no ROS, no serial)
   ├── HardwareAdapter (read-only gateway client; motion disabled)
   ├── SQLite event store (append-only, WAL)
   └── HTTP client ──> armpi_debug_gateway (robot host, ROS 2)
                          ├── /battery  <- /ros_robot_controller/battery
                          ├── /camera   <- /depth_cam/rgb/image_raw
                          └── /state    <- /controller_manager/joint_states (command echo)
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
│   │   ├── domain/         enums, models, state machine, mapping, trajectory analysis
│   │   ├── persistence/    SQLite event store
│   │   ├── safety/         validation, supervisor, lease, preview token
│   │   ├── config.py
│   │   ├── service.py
│   │   └── main.py
│   └── tests/              74 unit/API tests
├── configs/
│   └── joint_mapping.yaml  provisional vendor-derived mapping
├── ros2_gateway/           read-only gateway (runs on the robot)
│   ├── armpi_debug_gateway/
│   ├── deploy/             host systemd unit, runner, token rotation
│   └── tests/              12 unit tests (no ROS needed)
└── frontend/
    ├── e2e/                Playwright suite (mock mode)
    ├── src/api/            typed client + zod contract checks
    ├── src/components/     AppShell and shared UI primitives
    ├── src/pages/          eight console pages (G1 five + G2 + G3 + G4)
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
pytest                       # 74 passed

cd ../ros2_gateway
../backend/.venv/bin/python -m pytest   # 12 passed (no ROS needed)

cd ../frontend
npm run test:e2e             # builds, then 10 Playwright tests
```

The E2E suite starts its own mock backend on `127.0.0.1:8766` with a temporary
SQLite database and serves `frontend/dist`; it never touches the robot. See
[`frontend/e2e/README.md`](frontend/e2e/README.md).

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

## Camera and battery

### Mock mode

The Dashboard shows a synthetic camera frame (`SVG`, labelled `MOCK CAMERA`) and
a 2S battery gauge around `7.65 V`. Both are explicitly `truth=simulated`.
Use Diagnostics → Mock fault injection to test:

- `battery_low` — low-voltage warning
- `battery_missing` — battery `Unavailable`
- `camera_offline` — camera `Unavailable`

Mock camera frames are generated placeholders; they are **not** camera data.

### Real hardware (read-only gateway)

Run `ros2_gateway/` on the robot host (see `ros2_gateway/README.md`). It
subscribes to:

| Data | Topic | Truth |
|---|---|---|
| Battery | `/ros_robot_controller/battery` (`std_msgs/UInt16`, mV) | `measured` |
| Camera | `/depth_cam/rgb/image_raw` (`sensor_msgs/Image`) | `measured` |
| Joint state | `/controller_manager/joint_states` | `commanded` (echo, not encoder feedback) |

Then start the backend with:

```bash
export ARMPI_MODE=hardware
export ARMPI_GATEWAY_URL=http://127.0.0.1:8765
export ARMPI_GATEWAY_TOKEN='replace-with-a-long-random-token'
uvicorn app.main:create_app --factory --host 127.0.0.1 --port 8000
```

The gateway has **no motion endpoints**: `/motion/dispatch` and `/motion/stop`
return `501 NOT_IMPLEMENTED`. `hardware_motion` stays `false`; real joint motion
still requires mapping/limit verification and Board approval.

## API surface

| Method | Path | Purpose |
|---|---|---|
| GET | `/api/health` | mode, source label, capabilities |
| GET | `/api/state` | latest `RobotState` |
| GET | `/api/connection` | four-segment connection evidence |
| GET | `/api/peripherals` | battery + camera status |
| GET | `/api/peripherals/camera/snapshot` | latest camera frame (JPEG or Mock SVG) |
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
| GET | `/api/trajectories` | offline dataset summaries (G2) |
| POST | `/api/trajectories` | validate + analyze + store a dataset (no motion) |
| POST | `/api/trajectories/demo` | deterministic simulated demo dataset |
| GET | `/api/trajectories/{id}` | dataset + stored analysis |
| POST | `/api/trajectories/{id}/recompute` | re-run analysis and compare |
| GET | `/api/trajectories/{id}/export.csv` | aligned CSV (`sample=feedback\|commanded`) |
| GET | `/api/trajectories/{id}/export.json` | dataset + analysis + metadata |
| GET | `/api/experiments` | experiment summaries + failure-code map (G3) |
| POST | `/api/experiments` | create an immutable experiment definition |
| POST | `/api/experiments/demo` | deterministic simulated 16-pair demo |
| GET | `/api/experiments/{id}` | definition + plan + trials + analysis |
| POST | `/api/experiments/{id}/trials` | append one paired trial |
| GET | `/api/experiments/{id}/analysis` | recompute the paired analysis |
| GET | `/api/experiments/{id}/export.csv` | trial rows for script recomputation |
| GET | `/api/experiments/{id}/export.json` | definition + trials + analysis |
| GET | `/api/visual` | URDF FK frames + camera/calibration/TF metadata (G4) |
| GET | `/api/visual/camera-references` | metadata-only camera references |
| POST | `/api/visual/camera-references` | save a timestamp/calibration/TF reference |
| POST | `/api/mock/faults` | inject a Mock fault |
| POST | `/api/mock/reset` | clear Mock faults/state |
| WS | `/api/ws/state` | state stream (10 Hz) |
| WS | `/api/ws/events` | event stream |

`POST /api/motion/execute` requires an `Idempotency-Key` header (or body
`idempotency_key`) and the `command_id` + `preview_token` returned by preview.

## Safety invariants

- The backend never opens a serial port and never imports `rclpy`.
- The `ros2_gateway` process only subscribes; it has no motion publisher.
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

## Hardware read-only verification (2026-11-17)

```text
backend:  46 passed (41 baseline + 5 peripheral)
frontend: tsc -b && vite build → success
gateway:  /health all ready, motion_enabled=false
          /battery 7424 mV / 51.2% / truth=measured
          /camera  640x400 bgr8, 8.5 fps, JPEG snapshot
          /state   5 joints + gripper, truth=commanded
          no token → 401; POST /motion/dispatch → 501
backend:  mode=hardware; hardware_read=true; hardware_motion=false
          /api/peripherals battery + camera measured/valid
          /api/connection driver/gateway/feedback online
          preview valid → execute GATE_REJECTED
          blockers: mapping_verified, hardware_motion_enabled
UI:       LIVE; camera measured 640x400 bgr8; battery 7.42 V / 51%;
          camera renders a live frame, not the Mock SVG
```

Evidence:
[`evidence/studio_gateway/20261117/`](../evidence/studio_gateway/20261117/) and
[`docs/e7-gateway-verification.md`](docs/e7-gateway-verification.md). The raw
camera image is private and not committed.

## G2 trajectory verification (2026-11-18, offline only)

```text
backend:  56 passed (48 existing + 8 trajectory tests)
frontend: tsc -b && vite build → success
browser:  8 Playwright mock tests passed (new: Trajectory Inspector)
demo:     deterministic simulated 5-joint dataset; feedback truth=simulated
analysis: command fidelity + tracking metrics; CSV recompute matches backend
replay:   client-side clock only; no new audit event during play/pause
```

No robot, gateway, serial port or ROS node was involved. The demo feedback is
synthetic and is not hardware validation. Evidence:
[`evidence/studio_g2/20261118/`](../evidence/studio_g2/20261118/). Contract and
metric definitions:
[`docs/g2-trajectory-inspector.md`](docs/g2-trajectory-inspector.md).

## G3 experiment verification (2026-11-18, offline only)

```text
backend:  65 passed (56 existing + 9 experiment tests)
frontend: tsc -b && vite build → success
browser:  9 Playwright mock tests passed (new: Experiment Lab)
demo:     16 simulated pairs / 32 trials; data_origin=simulated
analysis: seeded bootstrap + exact sign/McNemar; CSV recompute matches backend
warning:  binary endpoint explicitly underpowered (not significance-tested)
```

No robot, gateway, serial port or ROS node was involved. The demo is synthetic
and is not hardware validation. Evidence:
[`evidence/studio_g3/20261118/`](../evidence/studio_g3/20261118/). Contract,
failure codes and analysis definitions:
[`docs/g3-experiment-lab.md`](docs/g3-experiment-lab.md).

## G4 visual verification (2026-11-18, read-only)

```text
backend:  74 passed (65 existing + 8 visual tests + 1 cache-header test)
frontend: tsc -b && vite build → success
browser:  10 Playwright mock tests passed (new: Visual Workbench)
model:    URDF FK camera_forward mount matches SRDF note (x≈0.154, z≈0.282 m)
camera:   mock 640×400, truth=simulated; optical frame from measured hand-eye
gateway:  /tf endpoint and composition covered by no-ROS tests; live robot TF not yet verified
safety:   read-only; camera references store metadata only; no motion events
```

No robot, gateway, serial port or ROS node was involved. URDF limits remain
provisional, the camera snapshot is synthetic in mock mode, and the model view
is a read-only projection (not a live TF window). Evidence:
[`evidence/studio_g4/20261118/`](../evidence/studio_g4/20261118/). Contract and
warnings: [`docs/g4-visual-workbench.md`](docs/g4-visual-workbench.md).

## Known limitations

1. The read-only gateway now runs as an enabled host systemd service and
   stop/start with clean SIGTERM is verified. A full reboot or vendor-container
   restart is **not** tested because the vendor `startup_check` moves the arm.
   Joint positions are still the vendor command echo, not encoder feedback.
   Rotate the shared token with `deploy/rotate-token.sh` before sharing the
   robot; the script is provided but was not exercised in this run.
2. Hardware motion is disabled; there is no motion write path in the gateway.
   The vendor driver defects (no ACK, blocking reads, `joint_states` command
   echo) are not repaired.
3. The joint mapping and limits are provisional (`configs/joint_mapping.yaml`).
   Read-only bus-servo positions and torque states were captured on 2026-11-17,
   but the mechanical-limit survey (torque off, hand movement) is pending. See
   [`docs/phase2/JOINT_LIMITS.md`](../docs/phase2/JOINT_LIMITS.md).
4. Backend API is unauthenticated and bound to localhost by default. The
   gateway uses a shared token. Do not expose either on a LAN/public interface
   without a separate security review.
5. The committed Playwright suite covers mock mode only. The hardware-mode UI
   check is still a manual browser session; there are no visual-regression
   baselines.
6. The vendor container has no `cv_bridge`; the gateway uses its numpy fallback
   for `bgr8`/`rgb8`/`mono8`. Other encodings report an error frame instead of
   being guessed.
7. The gateway uses the default RMW (Fast DDS). Topics interoperate with the
   vendor CycloneDDS nodes, but services do not; the read-only gateway does not
   call services. Any future service call must run with the vendor `.zshrc`
   environment (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, `CYCLONEDDS_URI`).
8. G2 is offline-only: no hardware trajectory or rosbag2 import yet, the demo is
   synthetic, and the timebase is a declared shared start (no latency claim).
9. G3 is offline-only: no hardware trial exists, artifact references are not
   read or validated, trial records cannot be edited/voided, and the demo's
   binary endpoint is deliberately underpowered.
10. G4 has an optional gateway `/tf` snapshot, but it is covered only by no-ROS
    tests and has **not** been verified against the robot TF graph. Without it,
    frames are URDF forward kinematics from the current joint truth (commanded
    in hardware mode), the URDF is provisional, the view is an SVG projection
    rather than a mesh render, and saved camera references contain metadata
    only. The live camera still needs a real hardware-mode check.

## Next steps

1. Hardware mapping/limit verification with torque off and manual movement
   (`tools/phase2/joint_limit_survey.py --mode survey`), with the arm supported.
2. Add a guarded single-joint motion path only after Board approval; keep
   `hardware_motion=false` until then.
3. Capture a real planned/commanded/measured trajectory with the validated
   bus-servo read path; there is no ROS recorder in the Studio yet.
4. Run the first supervised paired grasp experiment only after steps 1–3 and
   size the binary endpoint from pilot discordance, not from a small demo.
5. Verify reboot/container-restart persistence when arm motion is acceptable.
6. Verify the gateway `/tf` endpoint on the robot and compare its frames with
   `tf2_echo`; only then enable live-TF claims for G4.
7. Optional: a hardware-mode Playwright run behind an explicit environment gate
   and a private gateway, never in CI.
