# Motion Write Paths and Safety Authority

> Status: **audit record, 2026-11-21.** This document enumerates every path
> that can move the arm or the chassis and states which guard protects it. It
> exists because the repository does **not** have a single motion-authority
> guard: `armpi_safety` protects the MoveIt CLI path only, while the Studio
> gateway has its own policy and the fast CLI runner bypasses both.
>
> Freeze recommendation: treat the Studio gateway as the supported operator
> path and label the CLI runners **research-only**. The Board decision is still
> pending; this document does not change behavior.

## Path inventory

| # | Path | Entry points | Guard | Limits / verification |
|---|---|---|---|---|
| 1 | MoveIt CLI | `tools/phase2/real_moveit_execute.py`, `tools/phase2/execute_plan_json.py` | `/armpi_safety/validate_trajectory` (required) | `armpi_safety/config/safety_limits.yaml`: position bounds, **1.2 rad/s**, **0.5 rad/s²**, start-state, workspace, table plane, duration/points |
| 2 | Fast CLI | `tools/phase2/fast_motion.py`, `tools/phase3/fast_motion_e2e.py` | **None from `armpi_safety`**; the S-curve profile and a measured bus-servo settle are the only guards | `vmax=4.0 rad/s`, `amax=20 rad/s²`; per-move bus settle (≤3 counts in 0.12 s) |
| 3 | Studio G6 step | Studio backend → gateway `/motion/dispatch`; gateway must run with `--enable-motion` | Gateway motion policy: single-joint step cap, duration floor, lease/token, per-command limit check, bus verification (±8 pulses) | Limits come from `joint_mapping.yaml`; `takeover.yaml` sets the pose-mode velocity cap |
| 4 | Studio G7 pose | Studio backend → gateway pose scope; G7 presets/editor | Gateway pose gate: FK workspace/table, joint/preset checks, velocity cap, bus verification | `takeover.yaml`: `max_velocity_rad_s: 0.5`; TCP-only, no self-collision model |
| 5 | Studio G8 task / fast | Studio backend → `/api/task/plan` + `/api/motion/execute`; gateway profile | Gateway profile policy: peak joint velocity, acceleration, segment length, max points, total duration, start tolerance, bus verification; base pulse caps | `takeover.yaml` `task.profile`: **4.0 rad/s / 20 rad/s²**, descent 0.6, final 0.35, release descent 1.4; `max_points 400`; `max_total_duration_s 6.0` |
| 6 | Chassis pulse | Gateway `/chassis/pulse`; only with the chassis block enabled | Gateway chassis policy: duty cap, wheel cap, duration window, finite values, direction signs | G8.1 deployment caps duty at **10** and duration at **2.0 s**; open loop, no odometry |

## What this means

- **`armpi_safety` is not the guard for Studio motion.** G6/G7/G8 use the
  gateway policy. The gateway is the only path with a bus-verified completion
  read.
- **The fast CLI is outside `armpi_safety`.** `fast_motion.py` sends dense
  trajectories directly to `/arm_controller/follow_joint_trajectory`. Its
  4.0/20 envelope would be rejected by `armpi_safety` (1.2/0.5) if it were on
  that path; it is not, by design.
- **`safety_stop` has no consumer.** `armpi_safety` publishes a transient-local
  `std_msgs/Bool` on `safety_stop`, but no executor subscribes. Treat it as
  **advisory** until an executor treats it as authoritative.
- **No path has a self-collision model or an e-stop.** The checks are TCP-only
  and nominal-FK; the physical stop is power/support and a person.
- **The table plane is base-relative.** Any chassis pulse invalidates the
  measured plane; re-measure or re-localize before table work at a new base
  pose.

## Freeze rules

1. Every new motion feature must name its path, guard and limit source in this
   table before merge.
2. The supported path for the baseline is the Studio gateway (G6/G7/G8). CLI
   runners are research-only and must not be presented as production.
3. No release note may say "the safety veto covers all motion". Say which path
   was used and which guard applied.
4. Before `v1.0.0`, either wire `safety_stop` into an executor or rename it to
   an advisory signal; do not keep the "fail-safe" wording.
5. Any change to `safety_limits.yaml` or `takeover.yaml` must pass
   `python3 tools/verify_config_consistency.py` and a supervised smoke test.

## Inspection commands

```bash
# which entry points call the veto?
grep -rn "validate_trajectory" tools ros2_ws/src

# which entry points talk to the arm action directly?
grep -rn "follow_joint_trajectory" tools ros2_ws/src armpi-debug-studio

# who subscribes to the stop flag?
grep -rn "safety_stop" tools ros2_ws/src armpi-debug-studio

# limit/correction consistency
python3 tools/verify_config_consistency.py
```
