# G8.1 chassis approach — hardware run (2026-11-21)

Status: **PASS.** A cube placed at **x ≈ 0.357 m** — outside the arm's
direct-grasp band and outside the old locate region — was brought into the band
by two open-loop base pulses closed by the camera, then grasped, lifted,
verified and put back by the G8 13-step task. Every arm completion is a
`GetBusServoState` bus read; every base pulse is bounded by the gateway and
re-measured before the next one.

Operator present at the robot; the workspace was clear.

## Why

The camera view covers about 0.15-0.55 m of floor, while the validated
tool-down grasp pose reaches about 0.22-0.255 m (target radius). A cube placed
anywhere else used to be rejected (`FIT_REJECTED` / `LOCATE_FAILED`). The
2026-11-19/21 trials always moved the base by hand between placements; G8.1
automates exactly that motion with the already-calibrated pulse model.

## Contract

```yaml
chassis:                  # gateway caps (one pulse)
  enabled: true
  max_duty: 10.0
  wheel_max_duty: 30.0
  speed_mm_s_per_duty: 4.9   # 48-51 mm/s at duty 10 (2026-11-19 approaches)
  min_duration_s: 0.20       # below ~0.15 s the base stays in the dead band
  max_duration_s: 2.00
task:
  locate:
    x_range_m: [0.15, 0.55]  # whole visible floor, not just the reach band
  approach:
    enabled: true
    target_point_m: [0.230, 0.0]
    tolerance_m: 0.008
    duty: 10.0
    speed_mm_s_per_duty: 4.9
    min_duration_s: 0.25
    max_duration_s: 2.0
    max_pulses: 3
    settle_s: 0.5
```

The plan aims the cube at the **middle of the verified band on the front
axis** (`0.230, 0`), not at the nearest edge: the 2 % duty-scale error then has
slack on both sides, and the tool-down `joint1 ≈ 0` family that passed 24/24
stays valid (a base translation keeps the cube on the axis; no arm yaw is
needed).

## Flow

1. `POST /api/task/plan` captures the cloud, locates the cube and fits. If the
   target is outside the band, it returns `APPROACH_REQUIRED` with
   `approach: {vx, vy, duty, duration_s, forward_mm, left_mm, ...}` instead of
   `FIT_REJECTED`.
2. The operator confirms; `POST /api/chassis/pulse` (lease required, arm at the
   view) publishes `MotorsSpeedControl` at 20 Hz for the pulse, then zeros.
   `verification: open_loop` — the base has no odometry.
3. After the settle the page re-plans (fresh cloud) and repeats until the plan
   is valid or `max_pulses` is reached.

## Run

| Step | Measured |
|---|---|
| initial cube point | `(0.3570, -0.0092, -0.0370)` m, target radius 0.371 m |
| pulse 1 | vx 0.997, vy -0.072, duty 10, **2.0 s**, nominal 98 mm |
| after pulse 1 | `(0.2631, -0.0030, -0.0395)` — moved **94 mm** |
| pulse 2 | vx 0.996, vy -0.090, duty 10, **0.68 s**, nominal 33.2 mm |
| after pulse 2 (recovered centre) | `(0.2214, 0.0024, -0.0493)` — target radius **0.2351 m** |
| grip | `close_grip` actual raw **412** (the cube stops the fingers) |
| lift | `508/541/849/200/501` |
| grip verify | `held: true`, 2 clusters in the scene |
| put-back landing | `(0.2219, 0.0017, -0.0491)`: **+0.5 mm x, -0.7 mm y, +0.2 mm z** |

13 steps, all bus-verified `Completed`: `open_gripper`, `approach`, `pre_grasp`,
`grasp`, `close_pre`, `close_grip`, `lift`, `view_verify`, `grip_verify`,
`putback_pre_grasp`, `putback_grasp`, `release_open`, `return_view`. Fit
`j1..j5 = 0.0359 / -0.6618 / -1.80117 / -0.3745 / 0`, FK xy error 0.

Raw: [`approach_task.json`](approach_task.json) (plan + steps),
[`approach_required_plan.json`](approach_required_plan.json) (the first
`APPROACH_REQUIRED` plan with the pulse), [`final_plan.json`](final_plan.json),
[`after_putback_plan.json`](after_putback_plan.json).

## Problems found live and fixed

1. **Backend timeout on the first pulse.** The pulse itself ran (the base moved
   94 mm) but the gateway handler — 3 s subscriber discovery + the pulse +
   four zero messages — exceeded the adapter's default 8 s POST timeout, and the
   error path then hit a missing `AdapterError` import (HTTP 500). The adapter
   now waits **20 s** for `/chassis/pulse`.
2. **Double-sided clip false positive.** At the aim point the top face was
   clipped by the min-range sphere and the recovery saw *two* edge candidates
   inside its 2.5 mm range band (`a_min` and `b_max`, ranges 0.2525 vs 0.2531).
   The min-range cut is a sphere, so its boundary in the top plane is
   perpendicular to the camera->edge direction; a cube edge is not. The
   recovery now also requires `|dot(radial, edge_normal)| >= 0.85`. With that
   test the same cloud recovers the centre from the far edge correctly.
3. **Stale bus basis after a gateway restart.** The gateway holds the last
   `GetBusServoState` read in memory; restarting it cleared the basis, and the
   next plan used the command cache (`START_ECHO`, the measured point ~9 mm
   farther). A 1 s view re-command refreshed the actual before the final plan.

## Limits

- The base is **open-loop** (four duties, no encoders/odometry); position is
  closed only by the camera, one pulse at a time, and a dropped pulse is
  detected by the re-measure (not by the gateway).
- The gateway caps a single pulse at duty 10 / 2 s (about 98 mm nominal); the
  page caps the loop at `max_pulses` (3). No autonomous long drive.
- The approach is a **translation only**. A cube far off the front axis is
  handled by the lateral component (the mecanum strafe); a base *yaw* is not
  implemented, so a cube behind the reachable lateral window is rejected.
- The aim point assumes the validated constant correction (`joint1 ≈ 0`); the
  approach keeps the cube on the axis rather than extending the correction to
  yawed grasps.
- The table plane for the FK gate is re-measured by every locate; the takeover
  contract's plane moves by <1 mm over the pulse range.
