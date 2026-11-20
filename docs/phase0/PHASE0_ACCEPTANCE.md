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
> since been exercised: a formal five-joint stop survey with
> `mapping_verified=true` for the operating envelope, a re-measured 2.22 rad
> gripper stroke, real MoveIt execution, 3/5 first-series pick–lift–place
> attempts, and a pre-registered 24-pair trial that raised grasp success from
> 12/24 to 24/24.
>
> **Update (2026-11-20):** this file is now the single acceptance record for
> the capstone as well as the Phase 0 mock scope. §1 is the current verdict;
> §2 remains the original Phase 0 checklist. The README, ROADMAP and CHANGELOG
> were synchronized to the same status on 2026-11-20, including the D435i front
> camera, fast motion (through the in-view and far return-home runs), completed
> error budget and pre-registered 24-site trial.

## 1. Current acceptance summary (2026-11-20)

Hardware results are as of 2026-11-20. Verdicts use the repository convention:
**Pass** = implemented and backed by a command, log or data file in
`evidence/`; **Partial** = the framework exists but required terms are still
unmeasured; **Not accepted** = outside the demonstrated scope.

| Area | Verdict | Basis / limit |
|---|---|---|
| Phase 0 model and constrained planning | **Pass (mock scope)** | 17 links / 16 joints, single root; Pilz `PTP` 6D goal `SUCCESS` with 66 points; +20° tool-roll negative control fails; see §2 |
| Hardware baseline | **Pass** | Live Pi/ROS/servo/TF/camera baseline; controller board probed (battery, 6 bus servos, 4 open-loop motor duties, buzzer, LEDs; no IMU, encoders, e-stop or onboard gamepad); chassis-front D435i on USB 3.0 with depth/colour services |
| Joint limits | **Pass (operating envelope)** | All five joints hand-surveyed 2026-11-19 with torque released; adopted EEPROM − 20 pulses, joint2 high tightened to 775 and reloaded into the live move_group/safety/Studio stack on 2026-11-20; `mapping_verified=true`; stops are pose-dependent self-collision limits |
| Hand–eye calibration | **Pass** | Wrist 14-pose chessboard 2.94 mm / 0.39°, held-out 3.29 mm / 0.43°; table-down 15-pose re-fit 2.7 mm / 0.42°; independent physical tag scale check (0.67 % lateral); front D435i intrinsics 903.5 px / RMS 0.371 px and chassis extrinsic 31.6° pitch |
| Front camera (D435i) | **Pass with limits** | Chassis-front USB 3.0; depth 640×480 Z16 ~62 % valid / 10.6 Hz depth-only; `armpi-rs-depth` + `armpi-ext-cam` boot with `NRestarts=0`; IMU unavailable; depth starts ~0.2 m (grasp band colour-only); collision box protects MoveIt plans only |
| Fiducial localization | **Pass** | 5/5 synthetic (max 1.82 mm / 1.49°), 15/15 semi-synthetic on real frames, 0 false detections on 8 real frames; 4.87 % table-forward bias recorded |
| Real visual grasping | **Pass** | First series 3/5 on the unmarked 39.4 mm cube; pass 5 with the in-configuration correction; pre-registered 24-pair trial 12/24 → 24/24 (McNemar p = 0.00049); dual-camera run PASS, D435i agreed to 2.58 mm x/y and verified grip + put-back |
| Fast motion | **Pass on four real-cube runs at 4.0/20** | Jerk-limited S-curve with measured bus-servo settle; grasp+put-back **11.67 s → 8.94 s**; 4.0/20 defaults; duty-20 base 100.2 mm/s; aligned auto-base landing 2.22 mm; later in-view/far runs grasped in 4.77/4.76 s and `--return-home` reversed a 245.3 mm approach while holding the cube (11.06 mm landing vs the fitted target, open-loop return) |
| Low-configuration model | **Pass inside the measured envelope** | Fixed-tag constant (+11.6, +4.0, +8.5) mm, grasp-validated correction (+13.7, +3.9, +8.9) mm; joint-offset and `link4` models fitted and rejected; yaw extension measured to `j1 ≈ 0.5 rad` |
| Reliability intervention | **Pass** | The constant correction is the intervention; paired trial: baseline 12/24 vs intervention 24/24, paired placement improvement 2.19 mm (p = 0.0063); four protocol deviations documented |
| 24-site follow-up | **Pre-registered, not run** | 24 pairs / 48 trials, four radii × six bearings, seed-17 plan and layout committed before data; runner and 48-record dry run tested; hardware session pending |
| Error budget | **Complete; bound gates fail** | All required sources measured (2026-11-20 adds execution 5.386 mm / 1.495° max, placement 11.514 mm max / 3.080 mm mean, fixture 14.815 mm radial max); translation RSS 13.325 mm / worst 24.014 mm vs 6 mm; rotation RSS 1.858° (pass) / worst 3.322° (fail) vs the provisional 2°; empirical corrected placement mean 3.080 mm (RMS 4.082) is inside 6 mm; fixture-inclusive RSS 19.926 mm |
| Safety layer | **Pass with constraints** | Watchdog, joint/velocity/acceleration/start-state/duration/workspace checks, table-plane clearance and trajectory veto exercised; D435i collision box (MoveIt only); clock-sync timer holds the Pi ~2 s ahead; **no e-stop or brake**, all motion supervised |
| Debug Studio | **Pass G1–G6** | 93 backend + 47 gateway tests; 10 mock + 3 hardware Playwright; G5 moving capture; G6 supervised single-joint path verified without motion; D435i aux feed on the Dashboard |
| Mobile base | **Not accepted** | Four open-loop motor duties, no encoders/IMU/odometry; any base move invalidates the table calibration; one duty-20 aligned auto-base run at (0.2316, −0.0025) m |

**Scope verdict.** The capstone claim is accepted for the measured fixed-base
pick–lift–place task on this robot, cube and fixture: the pipeline works, the
dominant error is kinematic, and the correction produces a statistically
significant improvement in a pre-registered paired trial. The front D435i and
fast motion are accepted as validated with the stated limits; the 24-site
follow-up is accepted as **pre-registered only**, not as a result. The error
budget is now **complete** but fails its bound gates (translation RSS
13.325 mm, rotation worst 3.322°), while the empirical corrected placement mean
(3.080 mm) fits; the result is **not** a population reliability estimate, an
autonomous-operation claim, or a contact-rich manipulation claim. The mobile
base is outside the accepted scope.

## 2. Acceptance checklist (MASTER_PLAN_V3 §5)

| # | Acceptance item | Status | Evidence / limit |
|---|---|---|---|
| 1 | ArmPi loads in RViz | **Pass (mock)** | `rviz_load_20261116.log`: `rviz2` loaded `armpi_ultra` and KDL via the MoveIt demo; `display.launch.py` was rewritten to load the same model without the vendor `need_compile` trap (`display_headless_20261116.log`) |
| 2 | TF tree is coherent | **Pass (model)** | `xacro` + `check_urdf`: 17 links, 16 joints, single root `base_link`; `tf2_echo base_link camera_optical_frame` returns the expected chain in the display launch |
| 3 | Joint types are corrected | **Pass** | `joint1…joint5` are `revolute`; the stale vendor URDF with `joint1`/`r_joint` fixed is archived at `docs/archive/armpi_ultra.urdf.stale` |
| 4 | Camera optical frame exists | **Pass (nominal)** | `camera_connet_link → camera_link → camera_optical_frame`; REP-103 rotation; real extrinsic later measured as `camera_optical_calibrated` (2026-11-17, see hardware docs) |
| 5 | MoveIt2 can plan constrained targets | **Pass (mock)** | Pilz `PTP` planned a 6D-constrained goal on `gripper_link` (position + orientation): `SUCCESS`, 66 trajectory points. A +20° tool-roll perturbation failed, so the orientation constraint is enforced |
| 6 | 5-DoF limitations are documented | **Pass** | `docs/phase0/ARMPI_5DOF_CONSTRAINTS.md` |
| 7 | Simulated limits are explicitly marked provisional where not verified | **Pass** | URDF block + per-joint `PROVISIONAL` comments; `joint_limits.yaml` header; `docs/hardware/measured_joint_limits.md` |

## 3. Reproduce

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

## 4. Evidence index

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

## 5. Known non-blocking issues

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

## 6. Gap against the master plan (current state, 2026-11-20)

| Plan area | State on 2026-11-20 | Blocker / next |
|---|---|---|
| Phase 0 Panda freeze | Complete: `panda-baseline-v1` tag; 8-week README/roadmap archived; Phase II work archived, not merged | — |
| Phase 0 ArmPi model | Complete for model/mock scope (see checklist) | — |
| Phase 0 constrained planning | Complete for endpoint-constrained PTP; candidate search is Phase 2 | — |
| Phase 1 hardware baseline | **Captured; all five joints formally surveyed; board probed; D435i added** | Stops are pose-dependent self-collision limits; IMU/encoders/e-stop absent; D435i depth starts ~0.2 m and its IMU is unavailable |
| Phase 2 grasp candidate search | **Implemented; mock-validated and used on hardware** | `docs/phase2/GRASP_CANDIDATE_SEARCH.md`; mock search fed all five real passes |
| Phase 2 fiducial localization | **Implemented; synthetic + real-background validated; physical tag scale check** | `docs/phase2/FIDUCIAL_TARGET_LOCALIZATION.md`; physical tag scale 0.67 % lateral |
| Phase 2 fiducial → grasp bridge | **Implemented, mock E2E validated** (2026-11-17) | `docs/phase2/FIDUCIAL_GRASP_BRIDGE.md`; real grasps used the planner directly, not this bridge |
| Phase 2 calibration + real visual grasp | **Calibration validated; 3/5 first series; 24/24 with the correction in the paired trial; dual-camera PASS; fast motion validated** | Yawed grasp test open; the cube must first be rotated to face the gripper; fast motion has four real-cube runs at 4.0/20, including a far-cube return-home run with an open-loop (centimetre-level) home return |
| Phase 3 Error Budget v1 | **Framework + completed budget**; all sources measured 2026-11-20; bound gates fail (translation RSS 13.325 mm, rotation worst 3.322°) | The empirical placement mean (3.080 mm) is inside 6 mm; the 2° rotation gate is still provisional |
| Phase 3 paired trial | **Complete (2026-11-19)**: 24 pairs, baseline 12/24 vs intervention 24/24, McNemar p = 0.00049; paired placement improvement 2.19 mm (p = 0.0063) | 24-site follow-up pre-registered 2026-11-20, not run; then a second intervention on the dominant placement term |
| Operations | **Clock sync, live limit reload and reboot test PASS (2026-11-20)** | Timer needs the laptop user manager (or linger); D435i services have strict ordering rules |
| Host/DDS benchmark | **Read-only snapshot measured** (2026-11-17 Session 4); one laptop/Pi run, no load sweep | Repeated runs under controlled load |
| Documentation | **Synchronized 2026-11-20** across README, README.zh-CN, ROADMAP, CHANGELOG and this report | Keep §1 and the ROADMAP status table aligned |
| Bonus mobile base | **Open-loop only** | No encoders/IMU/odometry; base move invalidates the table calibration; one duty-20 auto-approach run |

Phase 1 started on 2026-11-17. Hardware-dependent items are promoted only
where the corresponding evidence exists. §1 and this table are the single
acceptance view; the README, ROADMAP and CHANGELOG carry the same status.

## 7. Changes made on 2026-11-16

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
