# Joint-limit margin — 2026-11-18

Status: **fixed and mock-verified.** No hardware motion; the real robot stack
was not touched. This closes leftover issue 1 from the 2026-11-18 onsite
session ("grasp sampler needs a joint-limit margin").

## Origin

The table-alignment plan-only run
([depth_plane_20261118.md](depth_plane_20261118.md)) produced exactly one
`PLAN_OK` candidate (`js_0016`) and it sat on a joint bound:

```text
joints = [-0.001, -0.662, -1.401, -0.192, -0.092]
joint2 = -0.662 rad is exactly the adopted low bound -0.6618
```

It was not executed. The recorder also showed that several 2026-11-16 mock
plans used `joint5 = -2.09 rad`, the old vendor URDF bound, so the problem was
not specific to this target.

## Root cause

Two independent paths could produce a pose on a limit:

1. `ArmKinematics.sample_candidates` built its grid over
   `[lower, upper]` **including the exact endpoints**, so the first row of
   every axis was the adopted bound itself.
2. MoveIt IK clamps its answer to the `joint_limits.yaml` bounds, so a
   Cartesian candidate or a refined sample could still come back exactly on a
   bound even when the request was interior.

A third, related defect: the default IK seed `(0, 0.7309, -1.6127, -1.6591, 0)`
is the SRDF `init_pose`, whose `joint4 = -1.6591` is outside the adopted low
bound `-1.5917`, so the first IK seed was itself invalid under the adopted
limits.

## Fix

New pure-Python module
`ros2_ws/src/armpi_grasp_planner/armpi_grasp_planner/joint_limits.py`:

- `DEFAULT_JOINT_LIMIT_MARGIN_RAD = 0.03` rad (1.7°, about 7 servo pulses,
  ~3.6× the 0.008 rad worst-case tracking error measured by G5);
- `shrink_limits`, `check_joint_margin`, `clamp_joints`.

Wiring:

- **Sampler**: keeps the historical grid and drops values inside the margin;
  the margin boundary is added as an extra sample. Filtering alone lost up to
  a full 0.2 rad step at the joint2 bound and removed every candidate for the
  table target; anchoring the whole grid at `lower + margin` shifted every
  sample and also lost targets. The boundary-preserving filter gives
  `joint2 = -0.6318` as the nearest admissible row.
- **Evaluator**: after IK and before FK, an IK solution closer than the margin
  to any bound becomes `JOINT_LIMIT_MARGIN` (counted in
  `counts.margin_rejected`). Seeds are clamped into the margin-shrunk range
  (`counts.seeds_clamped`).
- **CLI**: `--joint-limit-margin` (default `0.03`, `0.0` reproduces the old
  search); the JSON report records the margin, the limits and the per-candidate
  `joint_margin` block.

A 0.05 rad default was tried first and rejected: joint2 has a long lever arm,
so 0.05 rad moves the TCP by about 30 mm and no candidate remained within the
30 mm sampling radius of the 9 cm table target. 0.03 rad keeps that target
searchable.

## Tests

`cd ros2_ws/src/armpi_grasp_planner && python3 -m pytest test -q`:

```text
34 passed
```

New coverage: margin math and the exact-bound regression
(`test_joint_limits.py`), sampler grid phase/boundary and margin filtering
(`test_reachability.py`), evaluator `JOINT_LIMIT_MARGIN` rejection and seed
clamping without MoveIt (`test_evaluator_margin.py`). The flake8/pep257
linters are part of the same suite and pass.

The old Phase 0 positive control `(0.3, 0.9, -1.8, -1.7, 0.4)` is no longer
usable for the sampler: its joint4 is outside the adopted bound. Tests now use
`(0.3, 0.9, -1.6, -1.3, 0.4)` → TCP `(0.1775, 0.0348, 0.1630)`.

## Mock verification

Environment: this laptop, ROS 2 Jazzy, isolated `ROS_DOMAIN_ID=42`,
`mock_components`, KDL IK, Pilz `PTP`, plan-only. The real MoveIt graph in the
`armpi-humble` container was left untouched. Worktree source was used via
`PYTHONPATH`; the installed MoveIt config symlinks to the source tree, so the
adopted limits and SRDF were current.

Target: `(0.34, 0.0, 0.009)` m, `--sampling-radius 0.03`, joint sampler,
`--max-plan 3`.

| Run | Margin | Generated | IK/FK | Collision-free | Plans | Result |
|---|---:|---:|---:|---:|---:|---|
| Control | 0.00 | 44 | 22 / 22 | 2 | 0 (`PLAN_FAIL`) | reproduces the old candidate set |
| Fixed | 0.03 | 24 | 12 / 12 | 0 | 0 | no admissible plan |

Control evidence: both collision-free candidates (`js_0016`, `js_0017`) have
`joint2 = -0.6618` exactly; `check_joint_margin(..., 0.03)` reports
`min_margin_rad = 0.0`, so the new evaluator path would reject them. Fixed-run
evidence: the nearest admissible grid row is `joint2 = -0.6318`; its 12
sampled poses are within 28.8 mm of the target but all self-collide in this
mock scene, and the refined exact-target candidates fail IK. On this mock the
table target therefore has **no margin-safe plan**, which is the correct
outcome for a target that was only reachable on the bound; it must move inward
or upward (or the grid/margin must be revisited) before a real table grasp.

Reports: [table_margin_000.json](joint_limit_margin_20261118/table_margin_000.json),
[table_margin_003.json](joint_limit_margin_20261118/table_margin_003.json).

Caveat: the mock planner failed the two collision-free control plans
(`PLAN_FAIL`) where the real Humble stack had planned one successfully, so the
plan counts are **not** hardware evidence. The margin rejection is independent
of the planner and is proven by the candidate joints in the control report.

## What remains

- The real table-alignment target still has no executable candidate; do not
  execute `js_0016`.
- A margin-safe table grasp needs the target moved inward/upward, a finer
  sampling step, or a reduced margin with a written justification.
- The SRDF `init_pose` named state itself is still outside the adopted limits
  (`joint4 = -1.6591`); only the planning seed is clamped, the vendor named
  state was not changed.
- Hardware joint limits remain partially measured; this margin does not change
  that and does not enable hardware motion.
