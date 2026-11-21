# G8 — Task takeover

Status: **implemented and verified with a real task on 2026-11-21**. One
command planned and ran a 13-step task on the floor cube — locate → grasp →
lift → grip verify → put-back → return to view — with every arm completion a
`GetBusServoState` bus read. Evidence:
[`evidence/studio_g8/20261121/`](../../evidence/studio_g8/20261121/README.md).

G8 is the first Studio scope that acts on **what the camera sees** instead of a
pose the operator typed. It is deliberately narrow: one cube on the floor in
front of the robot, the validated deep grasp family, and a put-back to the
grasp pose. It is not a planner and it does not drive the chassis.

## Flow

```text
view_table preset (operator prepares the view)
   -> POST /api/task/plan
        cloud capture (gateway subprocess, ~1 s)
        hand-eye T_link4_camera (TSAI, calibrated table-down)
        segment floor cube (RANSAC floor, components, top slab)
        constant correction (+13.7, +3.9, +8.9) mm
        deep-family fit (j2=-0.6618, j3=-1.80117, j5=0, j1/j4 solved on URDF)
        reach band 0.220-0.255 m, FK xy <= 3 mm
        build 13 steps, each validated: joint limits, velocity cap, FK gate
   -> per step: POST /api/motion/preview -> /api/motion/execute
        gateway bus-verifies each pose (12 pulses) and timed gripper step
   -> POST /api/task/verify (plan_id) after the lift
```

The 13 steps: `open_gripper`, `approach` (j4 = 0), `pre_grasp` (j4 offset
0.15 rad toward approach), `grasp` (fit), `close_pre` (raw 318), `close_grip`
(raw 550), `lift` (the 2026-11-19 pass-2 lift preset), `view_verify`,
`grip_verify` (marker), `putback_pre_grasp`, `putback_grasp`, `release_open`,
`return_view`.

## Locate and fit

- The registered cloud `/depth_cam/depth_registered/points` is captured by an
  on-demand subprocess (`cloud_capture.py`) so the multi-megabyte
  deserialization never shares the gateway's motion process. Layout: OpenCV
  optical XYZ in the driver's `depth_camera_link` frame.
- The segmenter is a pure-NumPy port of `tools/phase2/object_segment_rgbd.py`:
  RANSAC floor, plane refinement, 5 mm voxel components, region/height/size
  gates, top slab with a 35 mm minimum extent.
- **Min-range clip recovery** (2026-11-21): the Aurora reports no valid depth
  nearer than ~0.25 m at the view pose, so a cube inside that sphere loses the
  near part of its top face; the clipped centroid is biased toward the far
  edge by 8-13 mm. When exactly one side of the top's minimum-area rectangle
  lies on the min-range sphere (range spread <= 1.5 mm, mean within 2.5 mm of
  the cloud minimum) and the opposite edge and both side edges are intact, the
  centre is rebuilt as `far_edge - cube_size/2` along the clipped axis and the
  mid-point along the intact axis. Enabled by `task.locate.cube_size_m`
  (0.0394 m, the measured cube); the plan warns `TOP_CLIPPED_RECOVERED`.
  A top clipped on two sides is still rejected.
- The pose basis for the FK is a **fresh bus actual** from the gateway when it
  is recent enough and within 0.12 rad of the view; otherwise the command
  cache is used and the plan warns `START_ECHO` (up to ~8 pulses, ~8 mm at the
  TCP). The 2026-11-21 run shows the difference: the same cloud moved the
  measured point ~8 mm between a cache and a fresh-actual basis.
- The reach band is **direct grasp only** (0.220-0.255 m target radius). A
  farther cube is rejected, not reached for; the chassis approach is a G8.1
  decision.

## Safety

- Every planned pose step is validated twice: at plan time from the planned
  start, and at execute time by the ordinary preview from the **actual**
  state. A rejected preview aborts the run; nothing is retried.
- The FK gate covers the straight joint-space interpolation (21 samples) with
  the workspace box, the measured tilted table plane + 10 mm, and the low-pose
  yaw rule; the planned `approach`/`pre_grasp`/`grasp`/put-back start/end are
  all gate-checked.
- The grip verify segments the view after the lift and requires that no cube
  box remains at the expected point; a miss aborts before the put-back.
- The run needs a lease, the fixed acknowledgement, pose scope and the
  verified mapping; the operator is at the robot with a clear workspace.
- Gripper steps are timed (the fingers stall on contact) and speed-capped with
  a 20 % margin: `speed_raw_per_s: 100` against the vendor's measured
  100-120 raw/s. The 2026-11-21 first attempt was aborted at `close_grip`
  because the pre-close stalled 12 raw past its command and a 2.0 s grip
  needed 0.51 rad/s; the contract now uses 1.5/2.6/4.0 s and a 0.419 rad/s
  validation cap.

## API

| Method | Path | Purpose |
|---|---|---|
| POST | `/api/task/plan` | locate + fit + 13 steps; `valid`, `point`, `target`, `fit`, `warnings`, `plan_id` |
| POST | `/api/task/verify` | `{plan_id}`; cube box at the view after the lift |
| GET | `/api/motion/presets` | `view_table`, `lift_150`, ... from the contract |

`tools/studio/g8_task_run.py` is the operator runner: plan-only by default;
`--execute --i-have-clear-workspace` runs the steps, requires bus-verified
`Completed`, and saves a report.

## Limitations

- One cube, one position, one put-back target (the grasp pose); the landing
  scatter measured after the run is +3.6 mm x, +0.2 mm y, +0.4 mm z.
- No chassis approach; the cube must be inside the direct-grasp band.
- No collision model: the FK gate is TCP-only and nominal.
- The clip recovery is one-sided and uses the measured cube size; a two-sided
  clip, a different object or a different view pose needs new evidence.
- The measured point's z follows the pose basis; use the fresh bus actual when
  the last command was not bus-read.
