# Studio G4 Visual Workbench — read-only verification (2026-11-18)

> Scope: laptop only. Mock backend, temporary SQLite database, built frontend,
> and the mock 640×400 camera. No robot, gateway, serial port, ROS node or
> hardware motion was involved. The camera frame is **simulated**, the URDF is
> explicitly provisional, and the `/tf` gateway path is tested with a **fake
> node**, not the robot TF graph.

## Environment

See [`environment.txt`](environment.txt): Python 3.12.3, pytest 9.2.0, Node
v24.22.3, npm 11.19.1, Playwright 1.65.1 (Chromium), 2026-11-18.

## Results

Raw logs:

- [`backend_test_run.txt`](backend_test_run.txt) — `74 passed`
- [`playwright_test_run.txt`](playwright_test_run.txt) — `10 passed`
- [`frontend_build.txt`](frontend_build.txt) — `tsc -b && vite build` success
- `ros2_gateway` regression — `12 passed` (includes the `/tf` endpoint)

```text
backend (armpi-debug-studio/backend/tests)   74 passed
gateway (ros2_gateway/tests)                 12 passed
frontend Playwright (mock mode)              10 passed
frontend build                               tsc -b && vite build OK
```

Visual backend coverage: URDF + gripper include parsing, camera_forward FK,
mimic/missing-state behavior, cyclic URDF rejection, `/api/visual` contract,
camera-reference save/list, no motion events, missing-URDF degradation, and
metadata-only references. The Playwright spec checks the model SVG, provisional
and no-live-TF warnings, camera frame names, 11-joint limit table, reference
save flow, and that no `Requested`/`Dispatched` event is appended.

## Captured API evidence

| File | Content |
|---|---|
| [`backend_health.json`](backend_health.json) | mock mode, `hardware_motion=false`, `replay=true` |
| [`visual_snapshot.json`](visual_snapshot.json) | URDF/FK frames, 11 joints, camera + calibration + warnings |
| [`fk_check.json`](fk_check.json) | independent camera_forward FK extraction and URDF audit |
| [`camera_reference.json`](camera_reference.json) | saved metadata-only reference |
| [`camera_references.json`](camera_references.json) | reference list |
| [`events.json`](events.json) | only `CameraReferenceSaved`; no motion stage |
| [`visual_routes.json`](visual_routes.json) | only GET `/api/visual` + GET/POST references |
| [`gateway_tf_fake.json`](gateway_tf_fake.json) | `/tf` HTTP contract from a fake node (not robot data) |
| [`visual_page.png`](visual_page.png) | full-page mock UI screenshot with a saved reference |

Key values:

```text
URDF                  armpi_ultra, 17 links, 11 movable joints, verified=false
URDF SHA-256          fe7cdfb7df71129606f27818407c82e0fb08f60d3722c99144114e72cc34bb94
camera_forward        camera_connet_link = [0.154412, 0.000000, 0.281453] m
                      (matches the SRDF note x≈0.154, z≈0.282)
optical frame         camera_optical_calibrated, parent link4,
                      source=measured_hand_eye, version=handeye-tsai-20261117
nominal frame         camera_optical_frame nominal=true (not for PnP)
TF status             model_only, live=false, source=urdf_fk, joint_truth=simulated
gateway /tf           2-edge fake-node contract; live robot TF graph NOT verified
camera                mock 640×400 bgr8, truth=simulated, age≈0 ms
reference             metadata only; no image bytes stored
```

The OpenAPI visual paths are only `GET /api/visual` and
`GET/POST /api/visual/camera-references`. There is no lease, dispatch, stop or
motion endpoint, and the event log contains only `CameraReferenceSaved`.

## Reproduce

```bash
cd armpi-debug-studio/backend
.venv/bin/python -m pytest -o addopts="" -q

cd ../frontend
npm run build
npm run test:e2e

cd ../ros2_gateway
../backend/.venv/bin/python -m pytest -q   # includes the /tf endpoint
```

To reproduce the API payloads without hardware:

```bash
cd armpi-debug-studio/backend
rm -f /tmp/armpi_g4.db*
DB_PATH=/tmp/armpi_g4.db ARMPI_MODE=mock \
  .venv/bin/python -m uvicorn app.main:create_app --factory \
  --host 127.0.0.1 --port 8798 --log-level warning &

curl -s http://127.0.0.1:8798/api/visual
curl -s -X POST http://127.0.0.1:8798/api/visual/camera-references \
  -H 'Content-Type: application/json' -d '{"note":"reference"}'
```
