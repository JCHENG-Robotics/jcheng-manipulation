# Changelog

All notable changes to this project are documented here. Format based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased] — 2026-11-16 Phase 0 acceptance

### Added

- `docs/phase0/PHASE0_ACCEPTANCE.md` — Phase 0 checklist, evidence, known issues
  and gap against the master plan.
- `docs/phase0/ARMPI_5DOF_CONSTRAINTS.md` — 5-DoF task-space limits, grasp
  candidate policy and `link4 → camera` calibration consequence.
- `docs/phase0/README.md` — index; historical audits marked as outdated status.
- `tools/phase0/constrained_plan_check.py` — plan-only 6D-constrained goal check
  with a +roll negative control.
- `config/kinematics_reachability.yaml` — explicit position-only profile.
- `evidence/armpi_moveit/phase0_20261116/` — plan JSON, MoveIt log and RViz log.

### Changed

- `kinematics.yaml`: `position_only_ik: false` (full-pose IK is the default);
  solver timeout 0.1 s. Position-only IK is now a separate, opt-in profile.
- `armpi_ultra.srdf`: `arm` group is a chain `base_link → gripper_link`, so the
  IK tip is the grasp tool frame.
- `demo.launch.py`: added `use_rviz` (default `true`) for headless runs.
- Rewrote the vendor `armpi_ultra_description` launch files
  (`display.launch.py`, `rviz.launch.py`, `robot_description.launch.py`):
  removed the `need_compile` requirement and the hardcoded `/home/ubuntu` /
  `mentorpi_description` paths. `colcon test` for the package now passes
  (12 tests, 0 failures, 1 skipped).
- URDF arm limits and `effort="1000"` now carry explicit `PROVISIONAL` comments.
- `sensors_3d.yaml`: removed the non-existent Kinect/octomap updater config.
- README and ROADMAP now report the verified Phase 0 status.

### Verified (mock only)

- Pilz `PTP` planned an FK-exact 6D goal on `gripper_link` (`SUCCESS`, 66
  trajectory points, plan-only).
- The same goal with tool roll rotated +20° failed, confirming the orientation
  constraint is enforced.
- RViz loaded the `armpi_ultra` model and KDL plugin.
- `armpi_ultra_description` tests: 12 tests, 0 failures, 1 skipped.
- `xacro` + `check_urdf`: 17 links, 16 joints, single root.

### Known limitations

- OMPL goal sampling fails on the 6D goal with KDL; Pilz `PTP` is the current
  constrained-goal primitive.
- Pilz `LIN` fails on the same goal (`NO_IK_SOLUTION`) because a
  fixed-orientation straight line leaves the 5-DoF reachable set.
- No hardware validation; limits, camera extrinsic and gripper remain
  unmeasured.

## [Unreleased] — 2026-11-15 merge

### Added

- ArmPi Ultra 5-DoF description and MoveIt config under `ros2_ws/src/`
  (imported from the R9000P `armpi_sim_ws`; vendor baseline, project wrapper).
- `camera_link` and `camera_optical_frame` on `link4` (nominal/REP-103;
  extrinsic not measured).
- M2.2 Gate 2 rework1: `arm_description.controller_config` helper, strict
  schema/path validation, 0.1/0.2 gain profiles, stage watchdogs, offline tests.
- `armpi-debug-studio/` G1-Mock source (FastAPI + React; no hardware motion).
- `docs/audit/2026-11-15/` source inventory, reconciliation, status, merge plan,
  decision log.
- `docs/hardware/` baseline templates; `docs/phase0/`; `docs/armpi/` PRD + G0.
- `evidence/m2_2_v2/` bags/analysis/snapshots and `evidence/armpi_moveit/`.

### Changed

- `ROADMAP.md` replaced with `ROADMAP_PUBLIC.md` content; old 8-week roadmap and
  README archived under `docs/archive/`.
- `joint3` upper limit is `0.0`; `r_joint` upper limit is `1.257` (provisional
  intersection of URDF and vendor software ranges).
- `kinematics.yaml` uses KDL (TRAC-IK not installed); `position_only_ik: true`
  remains reachability-only.
- `initial_positions.yaml` uses the vendor startup arm pose; `r_joint` is a
  simulation placeholder because the real startup value is outside the model.

### Fixed

- Gripper joints now have acceleration limits; the previous
  `l_in_joint` `AddTimeOptimalParameterization` failure is not reproduced.
- Stale `armpi_ultra.urdf` (joint1/r_joint `fixed`) moved to
  `docs/archive/armpi_ultra.urdf.stale`; xacro is the only model source.

### Verified (simulation/Mock only)

- M2.2 `default`, `gain_0p1`, `gain_0p2`: Gazebo trajectory goals returned
  `error_code=0` (2026-11-15).
- ArmPi MoveIt: KDL loaded; joint-space `/move_action` plan returned
  `error_code.val=1` (mock_components).
- Studio: 41 backend tests pass; frontend `tsc -b && vite build` passes.

### Not verified

- Any real ArmPi hardware, serial port, joint limit, camera, gripper, base.
- 5-DoF position/pose-constrained grasp planning and hand-eye calibration.
- Error Budget v1, repeatability/backlash, task-tolerance envelope.

## [2026-11-10] — M2.2 controller study

- Recorded four MCAP bags (trajectory tracking / settling / gain 0.2) on the
  2-DOF `simple_arm` in Gazebo.

## [2026-11-08] — Panda baseline

- `panda-baseline-v1` annotated tag on `435f8ec` (Merge JER-17), pushed to
  GitHub.
- Phase II Week 1 branch remains unmerged; see
  `docs/archive/phase2-week01.md`.

## [0.1.0] — Simulated Pick-and-Place MVP

- Eight-week ROS 2/Panda learning project: C++ warm-up, ROS communication,
  URDF/Gazebo/ros2_control, MoveIt 2, GDB/rosbag2, final pick-and-place demo.
  See `docs/archive/README_8WEEK.md` and `docs/weekly-log/`.
