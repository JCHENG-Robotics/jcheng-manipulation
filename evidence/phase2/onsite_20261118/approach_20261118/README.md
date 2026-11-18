# Supervised object-free approach at `camera_forward` — 2026-11-18

> Status: **plan-only and supervised execution PASS.** The arm moved from the
> table-down pose to `camera_forward`, the grasp planner planned a 10 cm
> approach to the nearest reachable point on the camera optical axis, the
> safety service approved it, and the exact inspected trajectory executed
> (final TCP within 1 mm of the plan). The main finding is kinematic: the
> `camera_forward` **view** volume is not a grasp volume. At the camera height
> the arm reaches only ~0.10 m in front of the lens, so a target at 0.22 m is
> 0.124 m outside the reachable set; the plan uses the nearest reachable point.

## Setup

| Item | Value |
|---|---|
| Robot pose | `camera_forward` (`joint1..joint5 = [0, 0.419, -1.257, -0.709, 0]`) |
| Camera origin in `base_link` | **(0.1702, 0.0088, 0.2722) m** (measured-servo FK + camera_forward TSAI) |
| Camera optical axis in `base_link` | (0.9996, −0.0108, −0.0274) |
| Hand-eye | `T_link4_camera` from `evidence/calibration/20261117/calibration_servo_20261117.json` (TSAI) |
| Planner | `armpi_grasp_planner`, joint sampler, MoveIt Humble, plan-only |
| Safety | `/armpi_safety/validate_trajectory`, adopted EEPROM-bounded limits |
| Operator | at the robot, workspace clear, power switch within reach |

## Requested target and reachability

The requested object-free target was 0.22 m along the optical axis:

```text
target_requested = (0.3901, 0.0064, 0.2662) m
```

A joint-sampler search returned **0 candidates** and an offline FK grid showed
the closest reachable TCP was **0.124 m** away. At camera height
(`z ≈ 0.24–0.27 m`) the arm's maximum forward TCP is **x ≈ 0.27 m**, i.e. about
0.10 m in front of the camera. The reachable set at this height is a small
volume in front of the lens, not the 0.3–0.5 m view volume used for the
chessboard.

The target was therefore moved to the nearest reachable point on the axis:

```text
target_used = (0.268, 0.000, 0.246) m   (≈0.10 m in front of the camera)
```

## Plan-only result

Command (inside the Humble container):

```bash
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-point 0.268 0.0 0.246 \
  --max-plan 3 \
  --safety-service /armpi_safety/validate_trajectory \
  --include-trajectory \
  --json-out /tmp/approach_plan.json
```

| Stage | Count |
|---|---:|
| Candidates generated | 66 |
| Sanity | 66 |
| IK / FK | 33 / 33 |
| Collision-free | 33 |
| Plan-only OK | **3** |
| Safety approved | **3 / 3** |

Best candidate `js_0016`:

| Item | Value |
|---|---|
| Pitch / roll | 73.6° / +5.3° |
| TCP | **(0.2589, −0.0003, 0.2360) m** |
| Target offset | **13.6 mm** (inside the 20 mm sampler radius) |
| Joints | `[−0.001, −0.262, −0.001, −1.592, −0.092]` |
| Trajectory | 72 points, **7.039 s** |
| Max velocity / acceleration | 0.35 rad/s / 0.10 rad/s² |
| Safety | approved, `code=OK`, workspace checked |

The final joint4 is at the adopted EEPROM bound (`−1.5917 rad`), so the approach
is a large fold toward the camera, not a small extension.

## Supervised execution

The exact inspected trajectory was executed with
`tools/phase2/execute_plan_json.py` after a second safety validation and a
start-state check:

```text
safety: approved=True code=OK
start_state_max_delta=0.0000 rad
execute_status=4
execute_error_code=1
```

Final state:

| Joint | Command `joint_states` (rad) | Servo register (pulse) | Measured (rad) |
|---|---:|---:|---:|
| joint1 | −0.00419 | 497 | −0.01257 |
| joint2 | −0.26389 | 437 | −0.26389 |
| joint3 | −0.00419 | 508 | −0.03351 |
| joint4 | −1.59174 | **122** | −1.58336 |
| joint5 | −0.09215 | 478 | −0.09215 |

Final TCP from live TF: **(0.259, −0.001, 0.235) m**, **1 mm** from the planned
(0.2589, −0.0003, 0.2360). The servo registers confirm the pose; joint4 is
22 pulses inside the EEPROM bound (20-pulse adopted margin).

No object was grasped and no collision occurred. The camera view after the
approach is a folded, near-camera view; the raw images contain people and stay
private.

## Consequences

- A "point 0.22 m in front of the camera at camera height" cannot be reached by
  this arm. Use the table-down view (where the arm reaches the table) for
  grasp-volume work; the `camera_forward` pose remains a **viewing/calibration**
  pose.
- The grasp planner can express the near-camera approach only with a large
  fold (joint4 at its bound). If this volume is used again, add a joint-limit
  margin check to the sampler rather than accepting a bound pose.
- The table-view work is in
  [`../depth_plane_20261118.md`](../depth_plane_20261118.md).

## Files

| File | Content |
|---|---|
| `approach_plan_20261118.json` | Full plan-only report (3 trajectories, safety reports) |
| `../../../../tools/phase2/execute_plan_json.py` | Guarded executor for an inspected plan |
| `../../../../tools/phase2/real_moveit_execute.py` | Named-state plan/execute with safety veto |
| private images | `/tmp/forward_capture`, `/tmp/approach_final` (not committed) |

## Reproduction

Inside the Humble container the repo tools are mounted at `/repo_tools`:

```bash
# 1. Move to camera_forward (safety veto is required for --execute).
python3 /repo_tools/phase2/real_moveit_execute.py camera_forward \
  --execute --i-have-clear-workspace

# 2. Plan the nearest reachable point on the optical axis.
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-point 0.268 0.0 0.246 --max-plan 3 \
  --safety-service /armpi_safety/validate_trajectory \
  --include-trajectory --json-out /tmp/approach_plan.json

# 3. Dry-run the inspected trajectory, then execute it.
python3 /repo_tools/phase2/execute_plan_json.py \
  /tmp/approach_plan.json js_0016
python3 /repo_tools/phase2/execute_plan_json.py \
  /tmp/approach_plan.json js_0016 --execute --i-have-clear-workspace
```
