# Studio G3 Experiment Lab — offline verification (2026-11-18)

> Scope: laptop only. Mock backend, temporary SQLite database, built frontend
> and the deterministic demo experiment. No robot, gateway, serial port, ROS
> node or hardware motion was involved. Every demo trial is **simulated** and
> is not hardware validation.

## Environment

See [`environment.txt`](environment.txt): Python 3.12.3, pytest 9.2.0, Node
v24.22.3, npm 11.19.1, Playwright 1.65.1 (Chromium), 2026-11-18.

## Results

Raw logs:

- [`backend_test_run.txt`](backend_test_run.txt) — `65 passed`
- [`playwright_test_run.txt`](playwright_test_run.txt) — `9 passed`
- [`frontend_build.txt`](frontend_build.txt) — `tsc -b && vite build` success

```text
backend (armpi-debug-studio/backend/tests)   65 passed
frontend Playwright (mock mode)               9 passed
frontend build                                tsc -b && vite build OK
```

The new backend coverage includes definition/plan validation, trial validation
(success requires error, failure requires a failure code, plan/fixture/order
mismatch, duplicates, missing G2 trajectory), fixed-arithmetic analysis,
API create/record/analyze/export, CSV recomputation, demo idempotency, manual
experiments without a plan, all required failure codes, and no motion audit
stages. The Playwright test loads the demo, checks 32 trial rows, the paired
improvement/CI/sign test, the explicit underpowered-success warning and export
links, then asserts no `Requested`/`Dispatched` event was appended.

## Captured API evidence

| File | Content |
|---|---|
| [`backend_health.json`](backend_health.json) | mock mode, `hardware_motion=false`, `replay=true` |
| [`demo_import.json`](demo_import.json) | first demo import: definition + 32 trials + analysis |
| [`demo_repeat.json`](demo_repeat.json) | second import: `duplicate=true`, same experiment |
| [`experiment_list.json`](experiment_list.json) | summary + failure-code/method maps |
| [`experiment_detail.json`](experiment_detail.json) | definition + plan + trials + analysis |
| [`analysis.json`](analysis.json) | paired report only |
| [`trials_export.csv`](trials_export.csv) | 32 trial rows for independent recomputation |
| [`export.json`](export.json) | metadata JSON export |
| [`csv_recompute.json`](csv_recompute.json) | independent CSV recomputation |
| [`events_after_demo.json`](events_after_demo.json) | only `ExperimentSeeded`; no motion event |
| [`experiment_routes.json`](experiment_routes.json) | routes have no lease/motion endpoint |
| [`experiment_page.png`](experiment_page.png) | full-page mock UI screenshot after loading the demo |

Key values (all simulated):

```text
experiment_id       exp_055d9413270c
data_origin         simulated
plan                16 pairs, seed 7, fixture demo_fixture_A
trials              32 (16 baseline + 16 intervention)
complete pairs      16; 12 have both continuous errors
mean improvement    1.1607684408116101 mm (baseline - intervention)
95% bootstrap CI    [-1.43, -0.89] mm (intervention - baseline)
sign test           12 wins / 0 losses / 0 ties, p = 4.88e-4
success             68.8% -> 87.5%; McNemar p = 0.375 (underpowered warning)
```

`csv_recompute.json` reports
`recomputed_improvement_mm = 1.1607684408116101` and
`matches_within_1e-9 = true`, so the exported CSV independently reproduces the
reported continuous endpoint.

The experiment OpenAPI paths are only list/create, demo, get, trial append,
analysis and the two exports. There is no lease, dispatch, stop or motion
endpoint. The event log after the demo contains a single `ExperimentSeeded`
event.

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
rm -f /tmp/armpi_g3.db*
DB_PATH=/tmp/armpi_g3.db ARMPI_MODE=mock \
  .venv/bin/python -m uvicorn app.main:create_app --factory \
  --host 127.0.0.1 --port 8794 --log-level warning &

curl -s -X POST http://127.0.0.1:8794/api/experiments/demo
curl -s http://127.0.0.1:8794/api/experiments
```
