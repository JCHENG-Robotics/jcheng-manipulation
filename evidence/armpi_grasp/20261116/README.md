# Grasp Candidate Search — 2026-11-16

> Scope: **mock only.** ROS 2 Jazzy, x86_64, `mock_components`, KDL IK,
> Pilz `PTP`, plan-only. No ArmPi hardware, camera or execution was used.

## Environment

| Item | Value |
|---|---|
| Host | Ubuntu 24.04, x86_64 |
| ROS | Jazzy |
| MoveIt launch | `armpi_ultra_moveit_config demo.launch.py` with `use_rviz:=false`, `ros2_control_hardware_type:=mock_components` |
| IK | KDL, full-pose (`position_only_ik: false`) |
| Pipeline | `pilz_industrial_motion_planner`, `PTP` |
| Execution | none; `planning_options.plan_only = true` |

## Commands

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

# positive control: FK-exact Cartesian candidate
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-joints 0.3 0.9 -1.8 -1.7 0.4 \
  --max-plan 3 --json-out cartesian_exact_fk.json

# joint-space sampler at a model-reachable target point
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-point 0.1382 0.0225 0.0896 --sampler joint \
  --max-plan 3 --json-out joint_sampler.json

# negative control: unreachable point
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-point 0.60 0.0 0.30 --sampler joint \
  --json-out unreachable.json
```

## Results

| Check | Generated | Sanity OK | IK OK | FK OK | Collision-free | Planned | Exit |
|---|---:|---:|---:|---:|---:|---:|---|
| Cartesian exact FK | 54 | 54 | 1 | 1 | 1 | 1 | 0 |
| Joint sampler | 80 | 80 | 40 | 40 | 26 | 3 | 0 |
| Unreachable point | 0 | 0 | 0 | 0 | 0 | 0 | 1 |

The Cartesian run's only successful candidate is the exact FK pose
`p030.83_a+017.94_r-022.35_s000`, with 48 trajectory points. The joint sampler's
three planned candidates are `js_0243` (4.7 mm target offset, 93 points),
`js_0256` (4.8 mm, 87 points) and `js_0255` (4.8 mm, 79 points).

In the joint-sampler run, all 40 exact-target refinements failed IK while all
40 FK-consistent sampled poses solved IK. Fourteen sampled poses self-collided;
26 were collision-free.

## Files

| File | Content |
|---|---|
| `cartesian_exact_fk.json` | 54 Cartesian candidates, per-stage results, exact FK flag |
| `joint_sampler.json` | 80 joint-sampled candidates, sampled seeds, target offsets, planning results |
| `unreachable.json` | zero-candidate negative control |
| `moveit_headless_20261116.log` | `move_group`, controllers and mock hardware startup |

## Interpretation

- The pipeline is wired correctly: a known reachable pose is recovered and
  planned.
- A fixed target point does **not** admit arbitrary tool orientations: 53 of 54
  Cartesian neighbours failed IK. This is the 5-DoF constraint, not a planner
  defect.
- Joint-space sampling is the practical generator: every sampled pose is
  FK-consistent, so IK succeeds; the remaining filters are collision and
  planning.
- The reachable sample set is narrow. Sampled pitches went down to about 7.4°
  from vertical; no perfectly vertical top-down pose was found. The best plan
  (`js_0243`) used `joint5 = -2.09 rad`, i.e. a joint limit; the other two used
  1.81 and 1.51 rad.
- The only `[ERROR]` line in the MoveIt log is the expected
  `No 3D sensor plugin(s) defined for octomap updates`; there is no camera.
  The KDL root-inertia warning is the known Phase 0 non-blocking issue.

## Limitations

- No table, object or camera exists in the mock planning scene. Collision
  checking is self-collision only.
- The TCP offset (0.074 561 2 m) and all joint limits are provisional.
- Plan-only: no controller call, no execution, no grasp verification.
- The target point is model-generated, not camera-generated. This is not
  visual grasping.

See [`docs/phase2/GRASP_CANDIDATE_SEARCH.md`](../../../docs/phase2/GRASP_CANDIDATE_SEARCH.md)
for the method, conventions and next steps.
