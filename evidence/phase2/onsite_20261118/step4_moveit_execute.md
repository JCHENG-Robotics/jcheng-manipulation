# Step 4 — real MoveIt Plan → Execute (`camera_forward`)

Date: 2026-11-18 09:41–09:50 (robot local time, UTC+8). Operator at the robot,
workspace clear, power switch within reach.

## Setup

- Robot: vendor Humble stack, live `/joint_states` 20 Hz.
- Laptop: Humble Docker container `armpi-humble`, `real_moveit.launch.py
  use_rviz:=false`, CycloneDDS on `enx00e04ca373c7`.
- Tool: `tools/phase2/real_moveit_execute.py`, which plans through
  `/plan_kinematic_path` and sends the **exact inspected trajectory** to
  `/execute_trajectory`; execution requires `--i-have-clear-workspace`.
- Scaling: velocity 0.05, acceleration 0.05.

## Plan

| Item | Value |
|---|---|
| Named state | `camera_forward`, group `arm` |
| Target | `joint1=0`, `joint2=0.419`, `joint3=-1.257`, `joint4=-0.709`, `joint5=0` |
| `error_code` | `1` (`SUCCESS`) |
| Planning time | 0.014–0.016 s |
| Waypoints | 101–102 |
| Duration | **9.988 s** |
| Predicted start | `0.837758, 0, 0, 0, 0` (matches the live boot pose) |
| Predicted final | `0.009775, 0.415304, -1.247019, -0.713450, 0.008704` |

## Execute result

| Item | Value |
|---|---|
| Action goal | accepted |
| Action status | `6` (`ABORTED`) |
| MoveIt error | `-6` (`CONTROL_FAILED`) |
| MoveIt log | `waitForExecution timed out`; `expected upper bound ... 12.485683 seconds` |
| Vendor controller log | `Controller 'arm_controller' successfully finished` immediately after the cancel |

## Actual final state

`/joint_states` (command) and `bus_servo/get_state` (servo registers):

| Joint | Target (rad) | Final `joint_states` (rad) | Error (rad) | Command pulse | Servo register | Register − command |
|---|---:|---:|---:|---:|---:|---:|
| joint1 | 0.000 | 0.008378 | +0.0084 | 502 | 507 | +5 |
| joint2 | 0.419 | 0.414690 | −0.0043 | 599 | 595 | −4 |
| joint3 | −1.257 | −1.244071 | +0.0129 | 797 | 802 | +5 |
| joint4 | −0.709 | −0.716283 | −0.0073 | 329 | 330 | +1 |
| joint5 | 0.000 | 0.008378 | +0.0084 | 502 | 502 | 0 |

All joints are within **0.013 rad (0.75°)** of the target; all servo registers
are within **5 pulses (1.2°)** of the commanded pulse.

## Operator observation

> “跑起来顺，没有卡顿和抖动，是正常，没有碰撞。”

The motion was smooth, without the freeze/lurch behavior seen in earlier
sessions, and no collision occurred.

## Diagnosis

The arm **did execute and reach the target**. The failure is a reporting/timing
race:

1. MoveIt uses `allowed_execution_duration_scaling=1.2` and
   `allowed_goal_duration_margin=0.5 s`, so a 9.988 s trajectory gets a
   12.486 s upper bound.
2. The vendor `JointTrajectoryActionController` sends one servo command per
   trajectory segment and adds per-segment overhead; 101 segments accumulated
   ~2.5 s, so it reported completion at ~12.49 s.
3. MoveIt cancelled at the deadline and returned `CONTROL_FAILED`; the vendor
   controller reported success at the same moment.

This is not a motion failure and not a controller crash. It is the same
vendor-action latency that makes MoveIt's default duration allowance too tight.

## Fix applied

`config/moveit_controllers.yaml`:

```yaml
trajectory_execution:
  allowed_execution_duration_scaling: 1.6   # was 1.2
  allowed_goal_duration_margin: 1.0         # was 0.5
```

For a 10 s trajectory this allows ~17 s, well above the observed ~12.5 s but
still bounded enough to detect a real stall. A no-op re-execution at the same
pose is used as the regression check after restarting `move_group`.

## Files

| File | Content |
|---|---|
| `tools/phase2/real_moveit_execute.py` | Guarded plan/execute client |
| `evidence/phase2/onsite_20261118/step4_moveit_execute.md` | This record |
