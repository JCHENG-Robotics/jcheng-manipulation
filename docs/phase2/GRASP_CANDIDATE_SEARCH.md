# 5-DoF Constrained Grasp Candidate Search

> Status: **Implemented, mock-validated.** No hardware was used. All runs are
> plan-only with `mock_components`; nothing is executed.
>
> Plan references: `MASTER_PLAN_V3_PRIVATE.md` §3, §4, §8 and §17.

## 1. Why this exists

The ArmPi Ultra is a **5-DoF** arm. A generic 6D pose goal is overconstrained:
at a fixed tool point the reachable tool orientations form a thin manifold, not
a 3-D neighbourhood. Phase 0 showed that Pilz `PTP` can plan one
FK-generated constrained goal, but that does not answer the Phase 2 question:

> Given a target point, which grasps are actually reachable and plannable?

`armpi_grasp_planner` answers that question with two generators:

1. a **Cartesian family** over the documented grasp variables, used as a
   diagnostic and as an exact-FK positive control;
2. a **joint-space reachability sampler**, which samples joint values, keeps
   FK poses near the target, and verifies them with MoveIt. This is the
   practical generator for a vision-supplied target point.

The vendor code is not modified. The package is project-specific code.

## 2. Tool-frame conventions

The conventions come from the URDF, not from a generic end-effector standard:

| Quantity | Definition |
|---|---|
| Approach axis | `gripper_link` **+z** |
| Finger-opening axis | `gripper_link` **y** (finger joints rotate about x and are offset along y) |
| Tool x axis | `y × z` |
| TCP | `end_effector_link`, **0.074 561 2 m** from `gripper_link` along +z |
| `pitch` | angle between the approach axis and straight down; 0° is a vertical top-down approach |
| `approach_yaw` | azimuth of the tilt direction about `base_link` +z |
| `roll` | rotation of the fingers about the approach axis |
| `standoff` | pre-grasp offset back along the approach axis |

The TCP offset is **nominal**: it is derived from the URDF fixed joints
(`end_effector_joint` is 0.08 m above `link5`; `gripper_link` is 0.005 438 8 m
above `link5`). The real fingertip point must be measured in Phase 1.

`roll = 0` puts the fingers along the horizontal reference direction
`(-sin(approach_yaw), cos(approach_yaw), 0)`. To align the fingers with an
object yaw, set `approach_yaw` to that object yaw for a nominal grasp.

The parameters reproduce any tool orientation exactly, including FK-derived
poses; the decomposition function is unit-tested for a round trip.

## 3. Package layout

```text
ros2_ws/src/armpi_grasp_planner/
├── armpi_grasp_planner/
│   ├── geometry.py        # tool-frame math, quaternion conversion, decomposition
│   ├── candidates.py      # Cartesian candidate family, policy sanity checks
│   ├── reachability.py    # PyKDL FK chain + joint-space sampler
│   ├── evaluator.py       # MoveIt 2 IK/FK/collision/plan pipeline
│   └── cli.py             # grasp_candidate_check entry point
└── test/                  # 15 tests: geometry, grid, sanity, FK, sampler
```

`geometry.py` and `candidates.py` are pure Python. `reachability.py` needs
`PyKDL`, `urdf_parser_py` and `xacro`; `evaluator.py` needs MoveIt 2.

The local KDL chain was checked against MoveIt's `/compute_fk` link by link at
`(0.3, 0.9, -1.8, -1.7, 0.4)`: **0.0 mm** difference for all arm links,
`gripper_link` and `end_effector_link`. The evaluator still re-checks every
candidate with MoveIt, so a model mismatch cannot silently pass.

## 4. Generation modes

### 4.1 Cartesian family (`--sampler cartesian`)

Generates the product of:

```text
pitch list × approach-yaw list × roll list × stand-off list
```

with the TCP at the target point (minus the stand-off along the approach
axis). This is the family documented in
[`ARMPI_5DOF_CONSTRAINTS.md`](../phase0/ARMPI_5DOF_CONSTRAINTS.md).

With `--target-joints J1..J5`, the target TCP and orientation come from FK;
the exact FK candidate is appended and marked `is_exact_fk`. That is the
positive control for the whole pipeline.

### 4.2 Joint-space sampler (`--sampler joint`)

Samples `joint1…joint4` over their URDF limits, computes FK, and keeps poses
whose TCP is within `--sampling-radius` of the target and whose pitch is within
`--max-pitch`. For each kept pose, `joint5` is sampled for roll. Two candidates
are produced per pose:

| Source | TCP | Purpose |
|---|---|---|
| `joint_sample_refined` | exactly the target | if IK succeeds, the grasp point is exact |
| `joint_sample` | the sampled FK TCP (within the radius) | FK-consistent fallback; IK should succeed from its own joint values |

The sampled joint values are passed to IK **first** as a seed. The evaluator
then tries the default seeds (SRDF `init_pose`, zero, Phase 0 pose). The
`seed_kind` field records whether the sampled or a default seed solved it.

This mode is the one to use with a real target point. It is also why the
sampler reports `target_offset_m`: a fallback grasp may be a few millimetres
away from the requested point.

## 5. Screening pipeline

Each candidate goes through the stages in master-plan order:

| Stage | Check | Failure status |
|---|---|---|
| 1 | Policy sanity: pitch ≤ `--max-pitch`; optional object width ≤ gripper opening | `SANITY_FAIL` |
| 2 | Full-pose IK (`/compute_ik`, `avoid_collisions=false`) | `IK_FAIL` |
| 3 | FK verification of the IK solution (position ≤ 1 mm, angle ≤ 1°) | `FK_MISMATCH` |
| 4 | State validity (`/check_state_validity`) | `COLLISION` |
| 5 | Plan-only Pilz `PTP` (`/move_action`, `plan_only=true`) | `PLAN_FAIL` / `PLAN_OK` |

Only candidates that pass stages 1–4 are planned, up to `--max-plan`.
Duplicate joint solutions are planned once (`SKIPPED_DUPLICATE_SOLUTION`).
Candidates that pass collision but are not planned are `COLLISION_FREE`.

No trajectory is executed. There is no controller call and no hardware path.

## 6. Running the check

Terminal 1 — headless MoveIt mock:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch armpi_ultra_moveit_config demo.launch.py \
  use_rviz:=false ros2_control_hardware_type:=mock_components \
  use_sim_time:=false use_gazebo:=false use_real:=false
```

Terminal 2 — the three checks:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
cd /path/to/jcheng-manipulation

# 1. Cartesian positive control: exact FK pose
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-joints 0.3 0.9 -1.8 -1.7 0.4 \
  --max-plan 3 --json-out /tmp/cartesian.json

# 2. Joint-space sampler at a model-reachable target point
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-point 0.1382 0.0225 0.0896 --sampler joint \
  --max-plan 3 --json-out /tmp/joint.json

# 3. Negative control: an unreachable point
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-point 0.60 0.0 0.30 --sampler joint \
  --json-out /tmp/unreachable.json
```

Expected: checks 1 and 2 end with `RESULT PASS`; check 3 generates 0 candidates
and exits 1. The JSON report contains per-candidate IK attempts, FK errors,
collisions, planning results, source and sampled joints.

## 7. Mock results (2026-11-16)

Environment: ROS 2 Jazzy, x86_64, `mock_components`, KDL IK, Pilz `PTP`,
plan-only. Evidence:
[`evidence/armpi_grasp/20261116/`](../../evidence/armpi_grasp/20261116/).

| Check | Generated | IK OK | FK OK | Collision-free | Planned | Exit |
|---|---:|---:|---:|---:|---:|---|
| Cartesian exact FK | 54 | 1 | 1 | 1 | 1 | 0 (`PASS`) |
| Joint sampler | 80 | 40 | 40 | 26 | 3 | 0 (`PASS`) |
| Unreachable point | 0 | 0 | 0 | 0 | 0 | 1 (`FAIL`) |

Details:

- **Cartesian**: the only successful candidate is the exact FK pose
  `p030.83_a+017.94_r-022.35_s000` (pitch 30.83°, yaw 17.94°, roll -22.35°),
  `PLAN_OK`, 48 trajectory points. Every ±10–15° neighbour failed IK.
- **Joint sampler**: 40 exact-target refinements all failed IK; 40 FK-consistent
  sampled poses all solved IK. 14 of the 40 self-collided; 26 were
  collision-free. The three planned candidates were `js_0243` (4.7 mm off
  target, 93 points), `js_0256` (4.8 mm, 87 points) and `js_0255` (4.8 mm,
  79 points). All used the sampled seed.
- **Unreachable**: the sampler found no FK pose within 20 mm of
  `(0.60, 0.0, 0.30)`, so no IK or planning was attempted.

## 8. What this run shows about the 5-DoF mechanism

1. **A known reachable pose is recovered exactly.** The Cartesian positive
   control proves the IK → FK → collision → plan pipeline is wired correctly.
2. **Orientation is not freely selectable at a fixed point.** Perturbing the
   exact tool orientation by ±10–15° produced 53 IK failures out of 54. This
   is the expected 5-DoF behaviour, not a planner bug.
3. **Exact-target refinements from nearby samples all failed.** The sampled
   orientations were reachable a few millimetres away, but not at the exact
   point. A practical grasp search must therefore sample the reachable set and
   report the residual target offset, or move the target to a reachable grasp
   point; it cannot assume an arbitrary 6D orientation at the requested point.
4. **The reachable sample set is narrow.** Among FK-consistent sampled poses,
   pitches went down to about **7.4°** from vertical; a perfectly vertical
   top-down pose was not found. A 2.99° exact-target refinement exists in the
   search, but it failed IK and was rejected. This is consistent with the
   joint-limit analysis in Phase 0: the tool axis cannot quite reach straight
   down.
5. **Self-collision matters.** The 37–38° pitch family collided with itself in
   the mock scene, while the 15° family was collision-free. Collision checking
   must stay in the loop; a pure IK-success count is misleading.
6. **Some plans sit on joint limits.** The best plan (`js_0243`) uses
   `joint5 = -2.09 rad`; the other two use 1.81 and 1.51 rad. Limit behaviour,
   backlash and the real fingertip point must be measured on hardware before
   these candidates are trusted.

## 9. Limitations

- No hardware, no serial link, no camera, no gripper feedback, no execution.
- Collision checking is **self-collision only**: the mock planning scene has no
  table, object or camera. A grasp that is collision-free here can still collide
  with a real table or object.
- The TCP offset and all joint limits are provisional.
- IK is KDL. The vendor analytic IK is not available in this x86 workspace.
- Planning uses Pilz `PTP` from the current mock state. `LIN` is known to fail
  on constrained 5-DoF goals; other planners were not evaluated here.
- The grasp candidates are not yet tied to an object model. There is no
  finger-width/object-width feasibility check unless `--object-width` and
  `--gripper-max-opening` are supplied.
- The target point in the sampler evidence comes from the robot model, not from
  a camera. This is **not** visual grasping.

## 10. Next steps

1. **Phase 1 hardware baseline**: measure joint limits, the real TCP point and
   the `link4 → camera` transform. Re-run the exact-FK control with measured
   values.
2. **Scene and object model**: add a table and a target object to the planning
   scene; check finger–object and link–table collisions, not only self-collision.
3. **Fiducial target pose**: implement the AprilTag/fiducial node and feed its
   target point into `--target-point --sampler joint`.
4. **Execution and verification**: after hardware safety checks, execute the
   best candidate with conservative velocity limits and verify the grasp.
5. **Candidate-strategy study**: compare Cartesian-family yield, joint-sampler
   yield, planning time and target offset on the same target points. This
   becomes part of the Phase 2/3 evidence, not a claim about success rates.
