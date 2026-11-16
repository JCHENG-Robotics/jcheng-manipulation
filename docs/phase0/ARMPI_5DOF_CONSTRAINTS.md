# ArmPi Ultra 5-DoF Constraints and Grasp Policy

> Status: **model-level analysis; not hardware-validated.**
> Applies to MASTER_PLAN_V3 §3, §4 and §7.

## 1. Degrees of freedom

ArmPi Ultra is modelled as a **5-DoF arm plus one gripper joint**:

| Joint | Role | Model range (provisional) |
|---|---|---|
| `joint1` | base yaw / arm working-plane direction | -2.09 … 2.09 rad |
| `joint2` | shoulder pitch | -2.09 … 2.09 rad |
| `joint3` | elbow pitch | -2.09 … 0.0 rad |
| `joint4` | wrist pitch | -2.09 … 2.09 rad |
| `joint5` | wrist roll about the tool axis | -2.09 … 2.09 rad |
| `r_joint` | gripper (mimic chain) | -1.57 … 1.257 rad |

`joint1` and `joint5` both contribute to tool yaw, but they are not
interchangeable: `joint1` rotates the whole arm plane, `joint5` rotates the tool
around its own axis.

## 2. What the arm cannot do

- It cannot reach an arbitrary 6D end-effector pose. A 6-DoF task needs six
  independent joint axes; this arm has five.
- The tool orientation is fully determined once `joint1…joint5` are fixed.
  There is no free wrist-yaw joint independent of the arm plane.
- A vertical approach is not available everywhere. Approach pitch and working
  radius are coupled through `joint2…joint4`.
- A straight-line Cartesian path that holds a fixed orientation may leave the
  reachable set even when both endpoints are reachable. In the 2026-11-16 test,
  Pilz `PTP` planned a reachable 6D goal while Pilz `LIN` failed with
  `NO_IK_SOLUTION` on the same goal.

## 3. Feasible grasp family (initial scope)

Search inside a constrained set, not over arbitrary poses:

```text
top-down and limited side grasps
+ approach pitch candidates
+ tool yaw candidates (joint5; joint1 sets the working plane)
+ small approach offsets along the tool axis
```

For each candidate:

```text
IK feasibility
→ orientation validity (tool axis / finger direction)
→ collision validity
→ trajectory validity
```

A 6D pose that happens to be reachable is not automatically a good grasp: the
gripper fingers must approach the object without collision and the tool axis
must match the intended grasp direction.

## 4. IK policy

| Use | Configuration | Allowed? |
|---|---|---|
| Reachability: “can the arm reach this point?” | `kinematics_reachability.yaml` (`position_only_ik: true`) | Yes, reachability only |
| Final grasp target | `kinematics.yaml` (`position_only_ik: false`) | Yes |
| Final grasp target | position-only IK | **No** — orientation can be physically unusable |

The checked-in default is `position_only_ik: false`
(`ros2_ws/src/armpi_ultra_moveit_config/config/kinematics.yaml`). The
reachability profile is a separate file and must be swapped in deliberately.

## 5. Camera geometry

The camera is mounted on `link4`, **before** `joint5`:

```text
base_link → link1 → link2 → link3 → link4
                                      ├── camera_connet_link → camera_link → camera_optical_frame
                                      └── joint5 → link5 → gripper_link
```

Consequences:

- The calibration transform of interest is `link4 → camera`, not
  `gripper → camera`.
- `joint5` rotates the gripper, **not the camera**. Wrist roll therefore gives
  no extra camera roll diversity.
- Calibration pose collection must deliberately vary arm elevation, viewing
  direction, distance and image position. Nearly coplanar poses are not
  acceptable.
- The real `link4 → camera` extrinsic is unmeasured; the checked-in frames are
  nominal (identity mount + REP-103 optical rotation).

## 6. Planning behaviour observed on 2026-11-16 (mock hardware)

| Goal | Planner | Result |
|---|---|---|
| FK-exact 6D pose on `gripper_link` | Pilz `PTP` | SUCCESS, 66 trajectory points |
| Same pose, tool roll rotated +20° | Pilz `PTP` | FAILURE / no trajectory |
| FK-exact 6D pose on `gripper_link` | OMPL `RRTConnect` | FAILURE: “Unable to sample any valid states for goal tree” |
| Same goal | Pilz `LIN` | `NO_IK_SOLUTION` |

Interpretation:

- `PTP` is the usable primitive for endpoint-constrained 5-DoF goals: it solves
  IK at the goal and plans the joint-space path.
- OMPL's goal sampler is not usable as the constrained-goal entry point with
  KDL on this 5-DoF model. Do not read its failure as “the target is
  unreachable”; `/compute_ik` succeeds from random seeds for the same pose.
- `LIN` failure is expected for a 5-DoF arm when a fixed-orientation straight
  line leaves the reachable set. This is a task-space limitation, not a bug.

Evidence: `evidence/armpi_moveit/phase0_20261116/`.

## 7. Consequences for later phases

- Grasp candidate generation must be part of the project, not delegated to a
  generic 6D pose goal.
- Calibration validation poses must be chosen for the camera's actual kinematic
  diversity.
- The mobile base can extend the reachable set, but it does not add arm DOF.
- Mobile-base docking error must be budgeted separately from arm error.
