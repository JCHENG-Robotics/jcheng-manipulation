# 5-DoF Constrained Grasp Candidate Search

> Status: **Implemented; the runs in this document are mock-validated and
> plan-only.** The same planner was later used on hardware for the five real
> grasp passes (2026-11-19); those results are in
> [`evidence/phase2/onsite_20261119/`](../../evidence/phase2/onsite_20261119/)
> and are not part of this mock record.
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
│   ├── joint_limits.py    # joint-limit margin, seed clamping (pure Python)
│   ├── evaluator.py       # MoveIt 2 IK/FK/collision/plan pipeline
│   └── cli.py             # grasp_candidate_check entry point
└── test/                  # geometry, grid, sanity, FK, sampler, margin,
                           # evaluator-wiring tests
```

`geometry.py`, `candidates.py` and `joint_limits.py` are pure Python.
`reachability.py` needs `PyKDL`, `urdf_parser_py` and `xacro`;
`evaluator.py` needs MoveIt 2.

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
`--max-pitch`. Values within `--joint-limit-margin` of either limit are dropped
and the margin boundary is added as an extra sample; the historical grid is
otherwise unchanged, so shrinking the range does not shift every sample by a
fraction of a step. For each kept pose, `joint5` is sampled for roll. Two
candidates are produced per pose:

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
| 3 | Joint-limit margin: every joint ≥ `--joint-limit-margin` inside its limit | `JOINT_LIMIT_MARGIN` |
| 4 | FK verification of the IK solution (position ≤ 1 mm, angle ≤ 1°) | `FK_MISMATCH` |
| 5 | State validity (`/check_state_validity`) | `COLLISION` |
| 6 | Plan-only Pilz `PTP` (`/move_action`, `plan_only=true`) | `PLAN_FAIL` / `PLAN_OK` |

Only candidates that pass stages 1–5 are planned, up to `--max-plan`.
Duplicate joint solutions are planned once (`SKIPPED_DUPLICATE_SOLUTION`).
Candidates that pass collision but are not planned are `COLLISION_FREE`.

No trajectory is executed. There is no controller call and no hardware path.

### 5.1 Joint-limit margin

The adopted limits are already conservative command limits (servo EEPROM
limits minus 20 pulses; see [JOINT_LIMITS.md](JOINT_LIMITS.md)), but a pose
**on** a limit is still a bad plan: IK has zero headroom, servo tracking error
can push the joint into the stop, and the pose cannot be re-used if a later
correction moves it slightly. The planner therefore keeps an additional
margin, default **0.03 rad** (1.7°, about 7 servo pulses; 3.6× the 0.008 rad
worst-case tracking error measured by the G5 recorder):

- the joint sampler drops grid values inside the margin and adds the margin
  boundary as a sample;
- the evaluator rejects an IK solution closer than the margin as
  `JOINT_LIMIT_MARGIN` **before** FK and collision checking;
- IK seeds are clamped into the margin-shrunk range, because the SRDF
  `init_pose` seed predates the adopted limits (`joint4 = −1.6591`, low bound
  `−1.5917`). `counts.seeds_clamped` records how many seeds moved.

The trigger was the 2026-11-18 table-alignment run: its only plan had
`joint2 = −0.6618`, exactly the adopted low bound, and was not executed
([depth_plane_20261118.md](../../evidence/phase2/onsite_20261118/depth_plane_20261118.md)).
On the isolated Jazzy mock, the control run with `--joint-limit-margin 0.0`
reproduced the two collision-free candidates at `joint2 = −0.6618`; with the
default margin those candidates are no longer generated and the preferred
plans are gone (the 9 cm table target has no margin-safe, collision-free
candidate at this grid resolution). The target therefore needs to move inward
or the margin/grid needs revisiting before a table grasp; the boundary plan is
not silently accepted.

## 6. Running the check

For a **table** grasp, publish the table into the planning scene first and keep
the publisher alive; `/check_state_validity` and the plan-only call then include
it. The safety service's TCP-only table-plane check is separate.

```bash
python3 tools/phase2/table_collision.py \
  --plane-json evidence/phase2/onsite_20261118/depth_plane_optical_20261118.json
```

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

# 1. Cartesian positive control: exact FK pose inside the adopted limits
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-joints 0.3 0.9 -1.6 -1.3 0.4 \
  --max-plan 3 --json-out /tmp/cartesian.json

# 2. Joint-space sampler at the same model-reachable target point
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-point 0.1775 0.0348 0.1630 --sampler joint \
  --max-plan 3 --json-out /tmp/joint.json

# 3. Negative control: an unreachable point
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-point 0.60 0.0 0.30 --sampler joint \
  --json-out /tmp/unreachable.json
```

Expected: checks 1 and 2 end with `RESULT PASS`; check 3 generates 0 candidates
and exits 1. The JSON report contains per-candidate IK attempts, FK errors,
joint-limit margins, collisions, planning results, source and sampled joints.
`--joint-limit-margin 0.0` disables the margin for a historical comparison;
the Phase 0 pose `0.3 0.9 -1.8 -1.7 0.4` is no longer a valid positive
control because its `joint4 = −1.7` is outside the adopted low bound
`−1.5917`.

## 7. Mock results (2026-11-16)

These results were recorded **before** the EEPROM-bounded limits and the
joint-limit margin existed, with the old vendor URDF limits and the Phase 0
control pose. They are kept as the historical mock baseline, not as a current
expected outcome.

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
   `joint5 = -2.09 rad`; the other two use 1.81 and 1.51 rad. That outcome is
   now impossible: the adopted limits and the joint-limit margin reject such a
   candidate as `JOINT_LIMIT_MARGIN` (§5.1). Limit behaviour, backlash and the
   real fingertip point must still be measured on hardware before these
   candidates are trusted.

## 9. Limitations

- No hardware, no serial link, no camera, no gripper feedback, no execution.
- Collision checking is **self-collision only unless the table collision object
  is published** (`tools/phase2/table_collision.py`). Without it the mock scene
  has no table, object or camera, and a grasp that is collision-free here can
  still collide with a real table or object.
- At the time of these mock runs the TCP offset and joint limits were
  provisional. The joint limits are now verified for the operating envelope
  (2026-11-19); the TCP offset remains a model value measured on hardware.
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
