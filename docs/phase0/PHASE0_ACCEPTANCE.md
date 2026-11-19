# Phase 0 Acceptance — ArmPi Model and Constrained Planning

> Date: 2026-11-16
> Plan: `MASTER_PLAN_V3_PRIVATE.md` §5 (Phase 0) and §26
> Scope: simulation / `mock_components` only. **No real ArmPi hardware was
> used.** Phase 0 checks model coherence and planning configuration; it cannot
> validate mechanical limits, servo torque, the camera or the gripper.
>
> **Update (2026-11-17):** Phase 1/2 later measured the real camera optical
> frame (`camera_optical_calibrated`, 2.94 mm / 0.39° hand-eye); see
> [`docs/hardware/camera_frames.md`](../hardware/camera_frames.md). This
> acceptance remains the mock-scope record for Phase 0.
>
> **Update (2026-11-19):** the hardware items that Phase 0 could not test have
> since been exercised: adopted EEPROM-bounded joint limits, a re-measured
> 2.22 rad gripper stroke, real MoveIt execution, and 3/5 real
> pick–lift–place attempts on a 39.4 mm cube. The Phase 0 acceptance itself is
> still a mock-scope record; see the repository README for current status.

## 1. Acceptance checklist (MASTER_PLAN_V3 §5)

| # | Acceptance item | Status | Evidence / limit |
|---|---|---|---|
| 1 | ArmPi loads in RViz | **Pass (mock)** | `rviz_load_20261116.log`: `rviz2` loaded `armpi_ultra` and KDL via the MoveIt demo; `display.launch.py` was rewritten to load the same model without the vendor `need_compile` trap (`display_headless_20261116.log`) |
| 2 | TF tree is coherent | **Pass (model)** | `xacro` + `check_urdf`: 17 links, 16 joints, single root `base_link`; `tf2_echo base_link camera_optical_frame` returns the expected chain in the display launch |
| 3 | Joint types are corrected | **Pass** | `joint1…joint5` are `revolute`; the stale vendor URDF with `joint1`/`r_joint` fixed is archived at `docs/archive/armpi_ultra.urdf.stale` |
| 4 | Camera optical frame exists | **Pass (nominal)** | `camera_connet_link → camera_link → camera_optical_frame`; REP-103 rotation; real extrinsic later measured as `camera_optical_calibrated` (2026-11-17, see hardware docs) |
| 5 | MoveIt2 can plan constrained targets | **Pass (mock)** | Pilz `PTP` planned a 6D-constrained goal on `gripper_link` (position + orientation): `SUCCESS`, 66 trajectory points. A +20° tool-roll perturbation failed, so the orientation constraint is enforced |
| 6 | 5-DoF limitations are documented | **Pass** | `docs/phase0/ARMPI_5DOF_CONSTRAINTS.md` |
| 7 | Simulated limits are explicitly marked provisional where not verified | **Pass** | URDF block + per-joint `PROVISIONAL` comments; `joint_limits.yaml` header; `docs/hardware/measured_joint_limits.md` |

## 2. Reproduce

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

# model checks
xacro ros2_ws/src/armpi_ultra_description/urdf/armpi_ultra.xacro > /tmp/armpi.urdf
check_urdf /tmp/armpi.urdf

# headless MoveIt (mock)
ros2 launch armpi_ultra_moveit_config demo.launch.py \
  use_rviz:=false ros2_control_hardware_type:=mock_components \
  use_sim_time:=false use_gazebo:=false use_real:=false

# model-only RViz path (use_rviz:=true to open RViz)
ros2 launch armpi_ultra_description display.launch.py use_gui:=false use_rviz:=false

# constrained target (PTP; plan-only, no execution)
python3 tools/phase0/constrained_plan_check.py \
  --pipeline pilz_industrial_motion_planner --planner PTP

# negative control: same pose, tool roll rotated +20°
python3 tools/phase0/constrained_plan_check.py \
  --pipeline pilz_industrial_motion_planner --planner PTP --rotate-x-deg 20
```

Expected: the first command prints `error_code=1 (SUCCESS)` and a non-zero
trajectory point count; the negative control prints a failure and zero points.

## 3. Evidence index

`evidence/armpi_moveit/phase0_20261116/`:

| File | Content |
|---|---|
| `phase0_ptp_ok.json` | reachable 6D goal, `SUCCESS`, 66 points |
| `phase0_ptp_rot20.json` | +20° roll control, `FAILURE`, 0 points |
| `moveit_headless_20261116.log` | `move_group` + controllers, headless launch |
| `rviz_load_20261116.log` | RViz started and loaded the model |
| `display_headless_20261116.log` | `display.launch.py` model publisher, no RViz |
| `check_urdf_20261116.txt` | final `check_urdf` output |

Earlier model/planning evidence: `evidence/armpi_moveit/MOVEIT_SMOKE_20261115.md`.

## 4. Known non-blocking issues

1. **KDL root inertia warning** — `base_link` has inertia and is the URDF root;
   KDL warns but plans. A dummy root would remove the warning; not required for
   Phase 0.
2. **Octomap monitor error** — no 3D sensor is configured. `sensors_3d.yaml`
   now has `sensors: []`; `move_group` still logs “No 3D sensor plugin(s)
   defined”. No camera exists on the dev host; planning is unaffected.
3. **OMPL goal sampling fails for the 6D goal** — `Unable to sample any valid
   states for goal tree`. `/compute_ik` solves the same pose from 20/20 random
   seeds, and Pilz `PTP` plans it. Use PTP for endpoint-constrained goals until
   an IK/planner combination is evaluated further. See
   `ARMPI_5DOF_CONSTRAINTS.md` §6.
4. **`position_only_ik` default changed** — the arm now uses full-pose KDL IK.
   RViz interactive dragging of arbitrary 6D poses may fail more often; that is
   the intended 5-DoF behaviour. Use `config/kinematics_reachability.yaml` for
   reachability-only experiments.
5. **No hardware** — no `/dev/rrc`, camera, gripper feedback, base or real
   motion. **Superseded after Phase 1:** the live robot, camera, gripper and
   supervised motion were exercised on 2026-11-17/19.

## 5. Gap against the master plan

| Plan area | State on 2026-11-19 | Blocker |
|---|---|---|
| Phase 0 Panda freeze | Complete: `panda-baseline-v1` tag; 8-week README/roadmap archived; Phase II work archived, not merged | — |
| Phase 0 ArmPi model | Complete for model/mock scope (see checklist) | — |
| Phase 0 constrained planning | Complete for endpoint-constrained PTP; candidate search is Phase 2 | — |
| Phase 1 hardware baseline | **Captured; limits partially measured; controller board probed** | joint1 low, joint3 positive and joint5 stops unmeasured; IMU/encoders/e-stop absent |
| Phase 2 grasp candidate search | **Implemented; mock-validated and used on hardware** | `docs/phase2/GRASP_CANDIDATE_SEARCH.md`; mock search fed all five real passes |
| Phase 2 fiducial localization | **Implemented; synthetic + real-background validated; physical tag scale check** | `docs/phase2/FIDUCIAL_TARGET_LOCALIZATION.md`; physical tag scale 0.67 % lateral |
| Phase 2 fiducial → grasp bridge | **Implemented, mock E2E validated** (2026-11-17) | `docs/phase2/FIDUCIAL_GRASP_BRIDGE.md`; real grasps used the planner directly, not this bridge |
| Phase 2 calibration + real visual grasp | **Calibration validated; 3/5 real pick–lift–place attempts** | Passes 1–2 and 5 succeeded; passes 3–4 measured the low-configuration error; pass 5 used a tag correction |
| Phase 3 Error Budget v1 | **Framework + partial budget implemented**; measured hand-eye/fiducial terms only, required repeatability/backlash/execution/gripper/fixture terms unmeasured; paired-trial framework simulated only | Hardware measurements and a physical tolerance experiment |
| Host/DDS benchmark | **Read-only snapshot measured** (2026-11-17 Session 4); one laptop/Pi run, no load sweep | Repeated runs under controlled load |
| Bonus mobile base | **Open-loop only** | No encoders/IMU/odometry; purchase/extension decision open |

Phase 1 started on 2026-11-17. Hardware-dependent items are promoted only
where the corresponding evidence exists; see the repository README for the
current per-workstream status.

## 6. Changes made on 2026-11-16

- Arm group in `armpi_ultra.srdf` is now a chain `base_link → gripper_link`, so
  the IK tip is the grasp tool frame rather than the last actuated link.
- `kinematics.yaml`: `position_only_ik: false`; solver timeout 0.1 s.
  Added `kinematics_reachability.yaml` for reachability-only use.
- Every URDF arm limit and `effort="1000"` now carries a `PROVISIONAL` comment.
- `sensors_3d.yaml`: removed the non-existent Kinect/octomap updater config.
- `demo.launch.py`: added `use_rviz` (default `true`) for headless checks.
- Rewrote the vendor `display.launch.py`, `rviz.launch.py` and
  `robot_description.launch.py` (removed the `need_compile` requirement and the
  hardcoded `/home/ubuntu` / `mentorpi_description` references). The description
  package's flake8/pep257 tests now pass.
- Added `tools/phase0/constrained_plan_check.py` and the evidence above.
