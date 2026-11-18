# Studio G6 — supervised motion write path (2026-11-18)

Scope: **software + read-only hardware verification only. No valid motion
command was dispatched and the arm did not move.**

Purpose: verify that the new supervised write path (backend gate + gateway
validation + measured completion) is wired correctly on the real robot while
keeping `mapping_verified=false` visible.

## Environment

- Robot: ArmPi Ultra, direct Ethernet `10.42.0.218`, vendor container
  `ArmPiUltra` running, battery healthy.
- Read-only gateway: `armpi-gateway.service` on `8765`, redeployed from this
  commit.
- Supervised gateway: a second, manual instance on `8766` with
  `--enable-motion --motion-config .../motion_limits.json`.
- Backend: `ARMPI_MODE=hardware`, `ARMPI_HARDWARE_MOTION=1`,
  `ARMPI_SUPERVISED_MOTION_ACK=I_HAVE_SUPPORT_AND_A_CLEAR_WORKSPACE`, bound to
  `127.0.0.1:8011`, connected to `8766`.
- Mapping: `eeprom-bounded-2026-11-18`, `verified=false`.
- Motion limits: 0.10 rad step cap (23 pulses = 0.096342 rad after alignment),
  1.0 s duration floor, single joint, ±3 pulse completion tolerance.

## Files

| File | Content |
|---|---|
| `backend_health.json` | supervised backend capabilities: `hardware_motion=true`, `supervised_motion=true`, `max_step_rad=0.096342`, `min_duration_s=1.0` |
| `backend_state.json` | 5/5 joints `truth=measured`, source `/controller_manager/servo_states`, age 15 ms |
| `backend_diagnostics.json` | `safety.ready=true`, no blockers, 14 checks including supervised + measured feedback |
| `preview_valid.json` | joint1 +0.05 rad / 1.5 s preview valid, token issued (`target_raw=513`) |
| `preview_multi_joint.json` | two joints rejected with `SUPERVISED_SINGLE_JOINT` |
| `preview_step_over.json` | +0.15 rad rejected with `STEP_EXCEEDED` |
| `gateway_motion_health.json` | supervised gateway `motion_enabled=true`, `motion_state=idle`, `measured_ready=true` |
| `gateway_motion_status.json` | idle; safety + per-joint limits; single joint |
| `gateway_reject_oob.txt` | pulse 950 → HTTP 400 `MOTION_OUT_OF_LIMIT` |
| `gateway_reject_step.txt` | pulse 930 → HTTP 409 `MOTION_STEP_EXCEEDED` (429 > 23 pulses) |
| `final_measured_state.json` | read-only `/state` after shutdown; used for the no-motion comparison |

## Results

```text
read-only gateway 8765
  health        status=ok motion_enabled=false measured_ready=true
  motion/status state=disabled
  dispatch      HTTP 403 MOTION_DISABLED

supervised gateway 8766
  health        status=ok motion_enabled=true motion_state=idle measured_ready=true
  out-of-limit  400 MOTION_OUT_OF_LIMIT
  step probe    409 MOTION_STEP_EXCEEDED
  shutdown      clean SIGTERM; only 8765 remained

supervised backend
  /api/health       hardware_motion=true supervised_motion=true
                    max_step_rad=0.096342174710087 min_duration_s=1.0
  /api/state        5/5 measured, source=/controller_manager/servo_states, age 15 ms
  /api/diagnostics  safety.ready=true, blockers=[]
  preview valid     joint1 +0.05 rad / 1.5 s -> valid, token issued
  preview multi     SUPERVISED_SINGLE_JOINT
  preview step      STEP_EXCEEDED (0.15 rad > 0.096342)
  measured delta    max |after - before| = 0.000000 rad
```

`backend_state.json` and `final_measured_state.json` are compared joint by
joint; the maximum difference is 0.0 rad. The two rejected gateway probes are
validated before publication, and no valid dispatch was sent.

## Regression at this revision

```text
backend pytest        92 passed   (76 before + 4 validation/supervision + 12 hardware motion)
gateway pytest        38 passed   (24 before + 14 motion guard/server)
mock Playwright       10 passed
hardware Playwright    3 passed   (read-only: LIVE, live /tf, GATE_REJECTED)
frontend build        tsc -b && vite build OK
```

## Not covered

- A first real supervised motion. That requires the operator, support and a
  clear workspace; see `docs/g6-motion-write-path.md`.
- Joint3–5 mechanical limits and `mapping_verified=true`.
- Physical e-stop, brake, independent encoder, multi-joint/trajectory
  streaming, automatic retry after an `Unknown` outcome.
