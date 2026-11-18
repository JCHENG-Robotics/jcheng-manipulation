# Studio G2 Trajectory Inspector — offline verification (2026-11-18)

> Scope: laptop only. Mock backend, temporary SQLite database, built frontend
> and the deterministic demo dataset. No robot, gateway, serial port, ROS node
> or hardware motion was involved. The demo feedback is **synthetic** and is
> not hardware validation.

## Environment

See [`environment.txt`](environment.txt):

| Tool | Version |
|---|---|
| Python | 3.12.3 |
| pytest | 9.2.0 |
| Node | v24.22.3 |
| npm | 11.19.1 |
| Playwright | 1.65.1 (Chromium) |
| Date | 2026-11-18 |

The working tree also contains the on-site step 1–3 changes from the same day
(`CHANGELOG.md`, hardware docs, gripper URDF/SRDF); those are separate from G2.

## Results

Raw logs:

- [`backend_test_run.txt`](backend_test_run.txt) — `56 passed, 1 warning in 1.05s`
- [`playwright_test_run.txt`](playwright_test_run.txt) — `8 passed (6.3s)`
- [`frontend_build.txt`](frontend_build.txt) — `tsc -b && vite build` success

```text
backend (armpi-debug-studio/backend/tests)   56 passed
frontend Playwright (mock mode)               8 passed
frontend build                                tsc -b && vite build OK
```

Trajectory backend coverage: exact fixed-dataset metrics, gap/dropout quality,
command-echo tracking refusal, invalid datasets with JSON paths, deterministic
demo, import/list/get/recompute, duplicate handling, CSV recomputation of every
RMSE and max error, export JSON, 404 handling, and no motion event on import.

The new Playwright test loads the demo, checks the simulated feedback label and
two charts, starts Replay (top bar becomes `REPLAY`), pauses, and asserts that
`/api/events?after_seq=<last>` is empty — replay performs no backend write.

## Captured API evidence

| File | Content |
|---|---|
| [`backend_health.json`](backend_health.json) | mock mode, `hardware_motion=false`, `replay=true` |
| [`demo_import.json`](demo_import.json) | full imported demo record + analysis |
| [`demo_analysis.json`](demo_analysis.json) | analysis-only extract (tracking, fidelity, sampling quality, notes) |
| [`trajectory_list.json`](trajectory_list.json) | summary list (one record) |
| [`recompute.json`](recompute.json) | fresh analysis equals stored (`matches_stored=true`) |
| [`demo_alignment_head.csv`](demo_alignment_head.csv) | first 12 aligned feedback rows with per-joint error |
| [`events_after_import.json`](events_after_import.json) | only `TrajectoryImported`; no motion stage |
| [`trajectory_routes.json`](trajectory_routes.json) | trajectory routes have no motion/dispatch endpoint |
| [`trajectory_page.png`](trajectory_page.png) | full-page mock UI screenshot after loading the demo (SIMULATION, no hardware) |

Key values:

```text
trajectory_id      trj_3edfb450ce92
dataset_sha256     11d5df757b794a26dd3d4bad46f46bdd61fbdb81e611f272646f330e483c2899
feedback truth     simulated
executor truth     simulated
aligned samples    195
analysis_version   g2-1
recompute          matches_stored=true
```

The trajectory OpenAPI paths are only `GET/POST /api/trajectories`,
`POST /api/trajectories/demo`, `GET /api/trajectories/{id}`,
`POST /api/trajectories/{id}/recompute`, and the two export GETs. There is no
endpoint that turns a stored dataset into a motion command.

## Reproduce

```bash
cd armpi-debug-studio/backend
.venv/bin/python -m pytest -o addopts="" -q

cd ../frontend
npm run build
npm run test:e2e
```

To reproduce the captured API payloads without hardware:

```bash
cd armpi-debug-studio/backend
rm -f /tmp/armpi_g2.db*
DB_PATH=/tmp/armpi_g2.db ARMPI_MODE=mock \
  .venv/bin/python -m uvicorn app.main:create_app --factory \
  --host 127.0.0.1 --port 8790 --log-level warning &

curl -s -X POST http://127.0.0.1:8790/api/trajectories/demo
curl -s http://127.0.0.1:8790/api/trajectories
```
