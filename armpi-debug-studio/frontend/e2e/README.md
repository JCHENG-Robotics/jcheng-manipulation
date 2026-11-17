# Frontend E2E (Playwright)

Black-box browser tests against the **built** frontend served by the FastAPI
backend in `mock` mode. No ROS, gateway, serial port or hardware is touched.

## Run

```bash
cd armpi-debug-studio/frontend
npm install                    # once
npx playwright install chromium  # once, if the browser is missing
npm run test:e2e
```

`pretest:e2e` runs `npm run build` first, so the backend serves the current
bundle. The config starts `e2e/start-backend.sh`, which uses a temporary SQLite
database (`/tmp/armpi_studio_e2e_8766.db`) and port `8766`; override with
`E2E_PORT`. To reuse an already running mock backend, keep it on the same port
(local runs use `reuseExistingServer`; CI does not).

Artifacts:

- `playwright-report/` — HTML report (`npm run test:e2e:report`)
- `test-results/` — traces and screenshots on failure (gitignored)

## Coverage

| Spec | Checks |
|---|---|
| `dashboard.spec.ts` | SIMULATION source, camera panel renders the mock frame (640×400), battery 2S panel, 5-joint table, safety gate 0 blockers, five-page navigation, Settings mode/mapping |
| `motion-flow.spec.ts` | target beyond `max_step_rad` is rejected (`STEP_EXCEEDED`); lease → preview token → confirm dialog → execute → `Completed` → lease release |
| `api-contract.spec.ts` | `/api/health`, `/api/state`, `/api/peripherals`, camera snapshot and preview token match the frontend's zod contract |

## Not covered

- Hardware mode: camera/battery `measured` and `GATE_REJECTED` execute require the
  real gateway and a hardware-mode backend. Use the manual procedure in
  [`../../docs/e7-gateway-verification.md`](../../docs/e7-gateway-verification.md)
  or a dedicated hardware E2E run; do not point CI at the robot.
- Visual regression: no screenshot baselines are committed. The suite asserts
  text, roles and counts only.
- Fault injection flows (mock faults) beyond the safety/motion paths above.
