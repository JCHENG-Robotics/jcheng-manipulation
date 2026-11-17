# Studio E8 / gateway test run (2026-11-17)

> Scope: laptop only. Mock backend, temporary SQLite database, built frontend.
> No ROS node, gateway, serial port or hardware was accessed by these tests.

## Environment

See [`environment.txt`](environment.txt):

| Tool | Version |
|---|---|
| Python | 3.12.3 |
| pytest | 9.2.0 |
| Node | v24.22.3 |
| npm | 11.19.1 |
| Playwright | 1.65.1 (Chromium) |
| Base commit | `7e16095` |

## Results

Raw log: [`test_run_20261117.txt`](test_run_20261117.txt).

```text
backend (armpi-debug-studio/backend/tests)   48 passed
gateway (ros2_gateway/tests)                 12 passed
frontend Playwright (mock mode)               7 passed
frontend build                                tsc -b && vite build OK
```

Playwright coverage:

1. API contract: health/state/peripherals/camera snapshot/preview token.
2. Safety gate ready; mock preview issues a token.
3. Dashboard: SIMULATION source, camera frame 640×400, battery 2S, 5 joints.
4. Navigation across all five G1 pages.
5. Settings: mock mode + provisional mapping.
6. Joint Control rejects a step beyond `max_step_rad` (`STEP_EXCEEDED`).
7. Lease → preview → confirm → execute → `Completed` → lease release.

The suite also uncovered and fixed a real serving issue: a plain StaticFiles
mount returned 404 for client-side routes. `SPAStaticFiles` now falls back to
`index.html` for non-API paths; `/api/...` unknowns stay 404.

## Reproduce

```bash
cd armpi-debug-studio/backend
.venv/bin/python -m pytest -q -o addopts=""

cd ../ros2_gateway
../backend/.venv/bin/python -m pytest -q

cd ../frontend
npm run test:e2e
```
