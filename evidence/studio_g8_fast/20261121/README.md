# G8 fast task path — real-robot run (2026-11-21)

Status: **PASS with real motion on the second attempt.** One
`tools/studio/g8_task_run.py --execute` planned from a live registered cloud
and ran the **7-phase fast task** — base approach (in-run, on the first
attempt), dense S-curves at 4.0 rad/s / 20 rad/s², fast gripper timings —
to grip verify and put-back. Every arm completion is a `GetBusServoState`
bus read. Operator present; no e-stop or brake is fitted, so the workspace
was clear and power was reachable.

Artifacts:

| File | Content |
|---|---|
| [`run1.json`](run1.json) | first attempt: the merged base pulse + `approach` + `grasp` passed, then `close_pre` was rejected by the gateway (`duration_s` floor 1.0 s) — the backend had planned `fast: true` but the adapter did not forward it |
| [`run2.json`](run2.json) | successful full run (7 phases, grip verify `held: true`, put-back) |
| [`after_putback_plan.json`](after_putback_plan.json) | read-only re-locate after the run, used for the landing measurement |

## Setup

| Item | Value |
|---|---|
| Robot | ArmPi Ultra at `10.42.0.218`, vendor Humble stack in `ArmPiUltra` |
| Pose gateway | `8766`, `--enable-motion`, pose contract **with `safety.profile`**, `verification=true`, `/pointcloud-topic /depth_cam/depth_registered/points`, vendor zsh/CycloneDDS environment |
| Read-only gateway | `8765`, untouched |
| Studio backend | laptop `127.0.0.1:8000`, hardware mode, `ARMPI_MOTION_SCOPE=pose`, fixed acknowledgement |
| Contract | `configs/takeover.yaml` `task.fast` (4.0/20, 0.35/0.5/0.45 s grip, 0.35 s verify settle) |
| Cube | the 39.4 mm floor cube used in the phase-3 trials |

## Attempt 1 — merged base approach worked, fast gripper flag did not

The first plan was `APPROACH_REQUIRED` (target radius 0.257 m, 2 mm outside the
0.220-0.255 m band). The **run itself** did the base pulse:

```
底盘接近 1/3: vx=0.996 vy=-0.085 duty=10.0 0.27 s (13 mm nominal)
pulse: Completed open_loop
重新规划: VALID
```

Then `approach` (actual `501/339/929/495/500`), `descend` (`502/339/930/463/500`)
and `grasp` (`501/339/930/426/500`) completed with deviations <= 5 pulses.

`close_pre` (0.35 s) was **rejected before publication**:

```
MOTION_BAD_REQUEST: duration_s must be within [1.0, 10.0]
```

The plan and preview had the fast 0.35 s step (`fast: true`), but the backend
adapter's single-command dispatch did not include the flag in the gateway
payload, so the gateway applied the slow path's 1.0 s floor. The arm stopped at
the grasp pose with the gripper open; the tool released its lease.

**Fix:** `RobotAdapter.dispatch(..., fast=False)` now carries the flag from the
service to the gateway payload (profile commands already set it). Added
`test_fast_flag_is_forwarded_to_the_gateway`; backend **136 passed**. The arm
was returned to `view_table` with the ordinary 0.5 rad/s preview/execute
(2.6 s; actual `501/554/791/122/497`, deviations <= 7 pulses) before the
second run.

## Attempt 2 — PASS

Plan (read-only, no base pulse needed this time):

| Item | Value |
|---|---|
| Cube point | `(0.220067, 0.001021, -0.047862)` m |
| Corrected target | `(0.233767, 0.004921, -0.038962)` m (radius 0.234 m) |
| Fit j1..j4 | `0.0278, -0.6618, -1.8012, -0.3853` (xy error 0.00 mm) |
| Plan | 7 phases, `fast.mode = s_curve`, 5.466 s commanded |

| # | Phase | Commands (planned) | Bus actual | Deviation |
|---|---|---|---|---|
| 1 | `approach` | profile 4.0 rad/s, 0.71 s / 15 segments | 502/338/930/494/500 | -5/-4/0/-6/0 |
| 2 | `grasp` | descend 0.6 rad/s 0.44 s, final 0.35 rad/s 0.46 s | 502/339/930/410/500 | -5/-3/0/+2/0 |
| 3 | `close` | preclose 318 (0.35 s), grip 550 (0.5 s) | r_joint 309 then 414 | timed |
| 4 | `lift` | profile 4.0 rad/s, 0.62 s / 13 segments (grasp -> view) | 504/557/794/119/497 | +4/-4/+10/-1/+1 |
| 5 | `grip_verify` | cloud at the view | `held: true`, 0 clusters | - |
| 6 | `release` | descend 1.4 rad/s 1.08 s, final 0.8 rad/s 0.25 s, open 0.45 s | 504/338/931/412/499, r_joint 174 | -3/-4/+1/+4/-1 |
| 7 | `return_view` | profile 4.0 rad/s, 0.62 s / 13 segments | 503/558/792/119/497 | +3/-3/+8/-1/+1 |

`close_pre` stopped 9 raw short of its 318 command (finger contact) and
`close_grip` at **414** (the cube stops the fingers; an empty close is
~640-645), matching the 13-step run's contact signature.

**Timing** (first preview -> last terminal event): **13.46 s** wall for
5.466 s of commanded motion. The old 13-step single-point plan was 33.56 s
wall for 23.5 s commanded. The per-phase deltas were approach ~1.2 s,
descend 0.93, grasp 1.04, close_pre 1.30, close_grip 1.39, lift 1.40, verify
~1.6, release_descend 0.98, release_final 1.35, return 1.15. The extra time
over the commanded motion is the per-command bus verification settle
(0.35 s arm / 0.6 s gripper) and event polling.

**Landing** (read-only re-locate after the run, same base position):
`(0.222289, 0.000557, -0.046798)` m, i.e. **+2.22 mm x, -0.46 mm y,
+1.06 mm z** (2.27 mm planar) against the pre-grasp point. The plan warned
`TOP_CLIPPED_RECOVERED` as usual.

## What this run proves

- The base approach is part of the run (attempt 1), and the run continues
  after a valid re-plan without an operator button.
- The dense S-curve path executed on the servos with per-segment gateway
  validation and the endpoint bus read; the 13.46 s wall time is 2.5x faster
  than the 13-step single-point plan.
- The fast gripper timings work on hardware (`close_pre` 0.35 s, `close_grip`
  0.5 s, `release_open` 0.45 s), with the recorded contact registers.
- A fast-path regression is caught safely: the rejected `close_pre` published
  nothing and the arm stayed at the grasp pose.
