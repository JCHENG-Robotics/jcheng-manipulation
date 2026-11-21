# G8 task takeover — hardware run (2026-11-21)

Status: **PASS with real motion and bus-servo verification.** One command
(`tools/studio/g8_task_run.py --execute`) planned a 13-step task from a live
registered point cloud and ran locate → grasp → 150 mm-class lift → grip verify
→ put-back → return to view on the robot. Every arm completion is a
`GetBusServoState` read of the actual registers.

Operator present at the robot; no e-stop or brake is fitted, so the workspace
was clear and power was reachable.

## Setup

| Item | Value |
|---|---|
| Robot | ArmPi Ultra at `10.42.0.218`, vendor Humble stack in `ArmPiUltra` |
| Pose gateway | `8766`, `--enable-motion`, pose contract, `verification=true`, `/pointcloud-topic /depth_cam/depth_registered/points`, started with the vendor zsh/CycloneDDS environment |
| Read-only gateway | `8765`, untouched by this run |
| Studio backend | laptop `127.0.0.1:8000`, hardware mode, `ARMPI_MOTION_SCOPE=pose`, fixed acknowledgement |
| Contract | `armpi-debug-studio/configs/takeover.yaml` (task block included) |
| Cloud capture | on-demand subprocess, ~1 s, 208k-215k points per capture |
| Cube | the 39.4 mm floor cube used in the phase-3 trials |

The arm started at the calibrated view preset (`view_table`: 0/0.2574/-1.18773/
-1.5917/-0.01676) with the gripper open (register 184).

## What blocked the first plans: the min-range sphere

The first locate attempts returned `partial_top_view`: the top slab's minimum
extent was **23.1 mm** against the 35 mm gate. The top face was not clipped by a
cube edge. The visible part was bounded by an **arc**, and the *whole* cloud had
**no valid point nearer than 0.251 m**:

- the Aurora's registered cloud reports no depth inside a min-range sphere
  (~0.25 m at this pose), so a cube whose near top edge is inside the sphere
  loses the near part of its top face;
- the far edge and both side edges were intact (side extent 40.6 mm ≈ the cube
  width), and the far edge was a straight line across the slab;
- the clipped centroid was biased **+7.8 to +12.5 mm** toward the far side, so
  the corrected target (0.259 m) fell outside the 0.220-0.255 m direct-grasp
  band and the plan was rejected rather than planning an edge-of-reach grasp;
- opening the gripper (the initial suspicion: closed fingers in the view) did
  not remove the clip; the clip is the sensor range, not an occluder.

`app/domain/locate.py` now has a gated **min-range clip recovery**
(`task.locate.cube_size_m: 0.0394`): when exactly one side of the top's
minimum-area rectangle lies on the min-range sphere (range spread <= 1.5 mm and
mean within 2.5 mm of the cloud minimum) while the opposite edge and both side
edges are intact, the centre is rebuilt as `far_edge - cube_size/2` along the
clipped axis and the mid-point along the intact axis. The plan carries a
`TOP_CLIPPED_RECOVERED` warning with the centroid bias, and the whole-top gate
is unchanged. The recovery is one-sided by construction; a top clipped on two
sides is still rejected.

Evidence: the failed pre-recovery plans (`/tmp` reports, not archived) and the
unit tests `test_segment_recovers_min_range_clipped_top`,
`test_segment_keeps_rejecting_clipped_top_without_recovery`.

## First execution attempt — aborted at `close_grip` (velocity)

The first 13-step run reached step 6 and was stopped by the backend preview:

```text
== [5/13] close_pre (grip)   Completed actual r_joint = 306   (command 318)
== [6/13] close_grip (grip)  preview REJECTED ['VELOCITY_EXCEEDED']
```

`close_pre` finished 12 raw *past* its command (the fingers stall on contact),
so the next preview measured its start from 306 instead of the planned 318:
(550-306) raw = 1.02 rad over 2.0 s = **0.51 rad/s**, just over the 0.503 rad/s
cap implied by the old 120 raw/s setting. Nothing was retried; the report was
overwritten by the successful run below.

Fix in the contract: `speed_raw_per_s` 120 -> **100** (the measured low end of
the vendor's 100-120 raw/s tracking, so every grip step has ~20 % margin) and
the configured gripper durations `close_pre` 1.3 -> **1.5 s**, `close_grip`
2.0 -> **2.6 s**, `release_open` 3.5 -> **4.0 s**. The gripper preview test now
uses 5.5 s for the full stroke.

Recovery from the abort: open the gripper in place, move back to the view
(3.0 s), re-locate with the **fresh bus actual** pose basis.

## The successful run

Plan (`first_task.json`):

| Field | Value |
|---|---|
| cube point (recovered) | `(0.224692, 0.001856, -0.048928)` m |
| corrected target | `(0.238392, 0.005756, -0.040028)` m, radius **0.2384 m** |
| deep-family fit | `j1..j5 = 0.031966 / -0.6618 / -1.80117 / -0.349541 / 0` |
| FK xy error | 6e-7 mm |
| pose basis | fresh `GetBusServoState` actual (no `START_ECHO`) |
| warnings | `TOP_CLIPPED_RECOVERED` (bias 7.8 mm, clipped side `a_min`) |

The 13 steps, bus actuals in raw pulses:

| # | step | kind | duration | actual | deviation |
|---|---|---|---|---|---|
| 1 | open_gripper | grip | 1.0 s | r_joint 184 | - |
| 2 | approach | pose | 3.2 s | 504/338/930/495/499 | -4/-4/0/-5/-1 |
| 3 | pre_grasp | pose | 1.0 s | 505/339/929/454/499 | -3/-3/-1/+2/-1 |
| 4 | grasp | pose | 1.0 s | 504/339/930/421/499 | -4/-3/0/+4/-1 |
| 5 | close_pre | grip | 1.5 s | r_joint 306 | - |
| 6 | close_grip | grip | 2.6 s | r_joint **416** | - |
| 7 | lift | pose | 1.9 s | 507/543/849/200/501 | -5/-6/+10/+1/+1 |
| 8 | view_verify | pose | 1.0 s | 503/555/793/122/497 | +3/-6/+9/+2/+1 |
| 9 | grip_verify | verify | - | `held: true`, clusters 1 | - |
| 10 | putback_pre_grasp | pose | 2.8 s | 504/341/932/446/499 | -4/-1/+2/-6/-1 |
| 11 | putback_grasp | pose | 1.0 s | 504/338/932/421/499 | -4/-4/+2/+4/-1 |
| 12 | release_open | grip | 4.0 s | r_joint 201 | - |
| 13 | return_view | pose | 2.5 s | 504/556/792/122/497 | +4/-5/+8/+2/+1 |

`close_grip` stopping at 416 (not the empty-gripper stall ~640) is consistent
with the cube blocking the fingers. The grip verify segments the view *after*
the lift and finds no cube box at the expected point -> `held: true`.

Raw: [`first_task.json`](first_task.json) (plan + every preview, execute, and
terminal event).

## Put-back landing

A read-only re-plan at the same view after the run measured the cube at
`(0.228336, 0.002086, -0.048514)` m (recovered: visible 26.9 mm, intact
40.6 mm, centroid bias 4.9 mm):

- landing offset vs the pre-grasp point: **+3.6 mm x, +0.2 mm y, +0.4 mm z**;
- the cube remained in the direct-grasp band (the new plan was `valid: true`).

Raw: [`after_putback_plan.json`](after_putback_plan.json).

## Limits of this result

- One cube, one position, one run; the put-back is to the grasp pose itself, not
  a commanded placement.
- The locate point's z follows the pose basis; the plan used the fresh bus
  actual, which moved the measured point ~8 mm versus a cache-only basis. The
  bus-verified `Completed` is the completion evidence, and `START_ECHO` warns
  when a plan had to fall back to the cache.
- The clip recovery reconstructs the centre from the far edge with the measured
  39.4 mm cube size; it is not a general occluded-cube solver and it never
  relaxes the whole-top gate.
- No chassis approach: a cube beyond the 0.220-0.255 m band is rejected, not
  reached for (G8 v1).
- No e-stop/brake; the operator watched the run.
