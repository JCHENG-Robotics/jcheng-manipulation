# Changelog

All notable changes to this project are documented here. Format based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased] — 2026-11-17 Phase 2 hand-eye validation

### Added

- `tools/phase2/chessboard_capture.py`: bus-servo position readback through
  `/ros_robot_controller/bus_servo/get_state`; records `measured_joints` and
  raw pulses alongside the cached commands. Adds `--base-pose`,
  `--servo-reads`, `--settle`, `--warmup` and `--repeats` (3-frame median),
  IPPE + LM board PnP, and a `--check-only` stationary mode.
- `evidence/calibration/20261117/calibration_servo_20261117.json`,
  `dataset_servo_20261117.json` and `validation_servo_20261117.json` — validated
  14-pose hand-eye result, measured-joint dataset and independent checks.

### Changed

- `tools/phase2/chessboard_hand_eye.py` prefers `measured_joints` for link4 FK,
  treats the vendor static TF as informational (body vs optical frame), and
  checks a held-out `final_base` sample. The pass gate is now board scatter
  ≤5 mm / ≤2°, intrinsics RMS ≤1 px and the held-out base.

### Verified (real hardware, 2026-11-17)

- Hand-eye `T_link4_camera` = `[-0.0348, -0.0091, 0.0586] m`, RPY
  `[0.951, -0.058, -89.789]°`; board scatter **2.94 mm / 0.39°**, held-out
  final base **3.29 mm / 0.43°**, 2-fold CV 2.5–3.2 mm / 0.31–0.43°;
  chessboard intrinsic refinement fx 418.31 vs driver 418.05, RMS 0.060 px.
- The measured bus-servo positions differ from the cached commands by up to
  2.16° (RMS 1.13°); using them reduced the board scatter from 13.4 mm to
  2.9 mm. The ~89.34° static-TF difference is the expected body/optical roll.

## [Unreleased] — 2026-11-17 Studio E8 + persistent gateway

### Added

- `armpi-debug-studio/ros2_gateway/deploy/` — host systemd unit
  (`armpi-gateway.service`), `docker exec` runner, token file support,
  `install-gateway.sh` and `rotate-token.sh`. The unit waits for the vendor
  container, restarts on failure and stops the in-container gateway with
  SIGTERM.
- `armpi-debug-studio/ros2_gateway/armpi_debug_gateway/auth.py` and
  `tests/` — token precedence (`--token` > `--token-file` > env) and 12 unit
  tests that run without ROS.
- `armpi-debug-studio/frontend/playwright.config.ts` and `frontend/e2e/` —
  Playwright browser suite: Dashboard panels, five-page navigation, preview
  rejection, lease → preview → confirm → execute → `Completed`, and served API
  contract checks. `npm run test:e2e` builds first and starts a temporary mock
  backend on port 8766.
- `armpi-debug-studio/backend/tests/test_spa.py` and `SPAStaticFiles` — serves
  `index.html` for client-side routes while keeping unknown `/api/...` paths
  404.
- `tools/phase2/joint_limit_survey.py` — safe read-only bus-servo survey plus
  an interactive torque-off mechanical-limit survey with automatic torque
  restore. `docs/phase2/JOINT_LIMITS.md` and
  `evidence/phase2/joint_limits_20261117/` document the read-only evidence and
  the pending physical procedure.

### Changed

- Gateway `main.py`: SIGTERM/SIGINT set a stop event, the HTTP server runs in a
  worker thread, and rclpy/HTTP shut down cleanly. Binding to a non-loopback
  address without a token now fails closed (exit 2).
- `armpi-debug-studio/README.md` status, limitations and next steps updated for
  the persistent gateway and committed E2E suite; top-level READMEs report the
  new test counts and joint-limit status.

### Verified (real hardware, 2026-11-17)

- Persistent gateway installed on the Pi host: `armpi-gateway.service` is
  `enabled` and `active`, bound to `10.42.0.218:8765`. `/health`,
  `/battery` (7424 mV), `/camera/status` (640×400 `bgr8`) and token 401 were
  re-checked through the service. `systemctl stop` produced a clean SIGTERM
  (`received signal 15`), released port 8765 and removed the in-container
  process; `systemctl start` restored `/health` 200. A hardware-mode backend
  using the service reported `LIVE` / 7.424 V / 640×400, and execute returned
  **403 `GATE_REJECTED`** with `mapping_verified` and
  `hardware_motion_enabled`; no motion was dispatched.
- Read-only bus-servo state: joint1–joint5 pulses `497 / 642 / 808 / 331 / 500`,
  torque registers all `1`, battery 7424 mV; all inside the provisional ranges.
  **No torque was disabled and no mechanical limit was measured.**
- Test runs: backend **48 passed**, gateway **12 passed**, Playwright **7
  passed**, `npm run build` succeeds.

### Known limitations (this increment)

- A full reboot and a vendor-container restart were **not** tested; both run
  the vendor `startup_check`, which moves the arm. The unit is enabled and the
  stop/start path is verified.
- The token rotation script is provided but was not exercised on the robot.
- Mechanical joint limits remain provisional; the torque-off survey is pending
  a supported arm.
- The gateway uses the default RMW (Fast DDS). Topics interoperate with the
  vendor CycloneDDS stack, but services do not; the read-only gateway subscribes
  only. Survey/service work must source the vendor `.zshrc`.

## [Unreleased] — 2026-11-17 Phase 1 hardware baseline

### Added

- `tools/humble_moveit/` — Humble MoveIt/RViz Docker environment for the real
  robot: `Dockerfile`, `run_humble.sh`, `setup_humble_ws.sh`, CycloneDDS
  configs and README. The robot stays Humble; the laptop runs a matching Humble
  container instead of Jazzy.
- `ros2_ws/src/armpi_ultra_moveit_config/launch/real_moveit.launch.py` —
  move_group + RViz only; deliberately no mock controllers,
  `robot_state_publisher` or static TF, so the real robot owns state and
  actions.
- `tools/phase1/camera_stream_check.py` — sensor-QoS RGB/depth rate, encoding
  and depth-value check; optional RGB frame save.
- `docs/hardware/` — live Phase 1 baseline: inventory, ROS environment, driver
  interfaces, live/model TF, camera mount/intrinsics, vendor baseline.
- `evidence/phase1/20261117/` — raw host/ROS/camera/TF/pose logs and one real
  RGB frame.

### Changed

- `armpi_ultra_moveit_config`: added the `camera_forward` arm named state
  (camera optical +z axis horizontal forward; FK-verified at x≈0.154 m,
  z≈0.282 m) and bound the SRDF and controller mapping explicitly in
  `real_moveit.launch.py`. Documented the named-state Plan & Execute workflow
  for the real robot.
- `armpi_ultra_moveit_config/config/armpi_ultra.srdf`: added
  `disable_collisions` for the vendor finger-finger and palm-finger mesh
  overlaps that made the planner report the start state as self-colliding while
  the physical gripper was open.
- Private vendor `servo_controller.launch.py` (not in this repository):
  removed `name='controller_manager'`, which also renamed the internal
  `ServoManager` and created duplicate nodes/action servers.
- Corrected the live DDS topology in the Phase 1 docs: robot `wlan0` ↔ laptop
  `wlp4s0` (office WiFi); direct Ethernet is an SSH fallback, not the current
  CycloneDDS interface. Recorded that the camera launch can be killed and must
  be checked/restarted with the persistent `docker exec -d ... exec` form, and
  that a running Humble container keeps the old bind-mounted
  `cyclonedds-laptop.xml` inode, so it must be restarted after an interface
  change.

### Verified (real hardware, 2026-11-17)

- One `/controller_manager`, one `/servo_manager`, one
  `/arm_controller/follow_joint_trajectory` server and one MoveIt client;
  `/joint_states` at ~20 Hz; one supervised `reset_servo` motion.
- `camera_forward` planned successfully from the live robot pose
  (`error_code=1`, 0.015 s, 96 waypoints) with the real-robot
  `real_moveit.launch.py`. **Plan-only; no execute goal was sent.** Evidence:
  `evidence/armpi_moveit/real_camera_forward_20261117/`.
- Deptrum Aurora 930 opens and streams: RGB 8.0 Hz, 640×400, `bgr8`; depth
  8.2 Hz, 640×400, `mono16` in millimetres; RGB/depth `camera_info` captured.
- Live TF: `base_link → rgb_camera_link` and `base_link → depth_camera_link`
  resolve; the project `camera_optical_frame` does **not** exist on the robot.
- Host: Raspberry Pi 5, Debian 12, kernel 6.6.20; battery 7.651 V; 48.8 °C;
  `throttled=0x0`.

### Known limitations

- Joint limits and servo offsets are **not measured**; only command snapshots
  exist. `joint_states`/`servo_states` are cached commands, not encoder
  feedback. The separate bus-servo `get_state` service does read the hardware
  position and is used by the Phase 2 hand-eye calibration.
- `/ros_robot_controller/imu_raw` has a publisher but produced no message.
- ~~The camera hand-eye (`link4 → camera`) extrinsic is the vendor static TF,
  not a measured calibration~~ — superseded by the Phase 2 hand-eye validation
  above (2.94 mm / 0.39° board scatter). Intrinsics are driver-reported and
  independently refined to 0.060 px RMS.
- The camera enumerates at USB 2.0 (480 Mbps) and the RGB stream is ~8 Hz.
- A transient camera open failure (`0x21003`) occurred before a clean power
  cycle and was not reproduced afterwards.
- Root filesystem is 95% full; battery is 7.651 V.
- No real fiducial detection, calibration, visual grasp or automated trial has
  been performed.

## [Unreleased] — 2026-11-16 Fiducial target localization

### Added

- `ros2_ws/src/armpi_fiducial_localizer/` — AprilTag/ArUco 36h11 detection
  and 6D pose estimation:
  - `detector.py`: `cv2.aruco` detection, `solvePnP(IPPE_SQUARE)`, reprojection
    filtering and quaternion output, OpenCV 4.6/4.7-compatible API selection;
  - `node.py`: image + `CameraInfo` → `target_pose` (`PoseStamped`), optional
    TF transform to `base_link`;
  - `synthetic.py` / `synthetic_publisher.py`: hardware-free marker rendering
    and ROS image stream;
  - `cli.py`: synthetic accuracy check with JSON and overlay images.
- `launch/fiducial_localizer.launch.py`.
- `tools/phase2/fiducial_ros_check.py` — one-shot pose comparison for ROS
  integration checks.
- `docs/phase2/FIDUCIAL_TARGET_LOCALIZATION.md` and updated Phase 2 index.
- `evidence/fiducial/20261116/` — synthetic accuracy, ROS camera-frame and
  `base_link` checks, node/publisher/TF logs, overlay images.

### Verified (synthetic only)

- 5/5 synthetic poses detected; max translation 1.82 mm, mean 0.67 mm; max
  rotation 1.49°, mean 0.64°; max reprojection 0.32 px.
- ROS node + static TF: `camera_optical_frame` and `base_link` checks both
  passed with 0.44 mm position error and 0.03° angle error.
- `armpi_fiducial_localizer`: 11 tests pass; flake8 and PEP 257 pass.

### Known limitations

- No real camera, lens, lighting, intrinsics or `link4 → camera` extrinsic.
  Synthetic accuracy is not real camera metrology.
- `marker_size_m` must match the printed marker edge length.
- Single marker, single frame, no temporal filtering or occlusion handling.
- The published pose is the marker centre, not an object grasp-point offset.

## [Unreleased] — 2026-11-16 Phase 2 grasp candidate search

### Added

- `ros2_ws/src/armpi_grasp_planner/` — 5-DoF grasp candidate generation and
  MoveIt 2 screening:
  - Cartesian candidate family (pitch, approach yaw, roll, stand-off) with an
    exact-FK positive-control mode;
  - joint-space reachability sampler (PyKDL + URDF) that keeps FK-consistent
    poses near a target point and can refine them to the exact target;
  - staged pipeline: policy sanity → full-pose IK → FK verification →
    state validity → plan-only Pilz `PTP`;
  - `grasp_candidate_check` CLI and JSON report.
- `docs/phase2/GRASP_CANDIDATE_SEARCH.md` and `docs/phase2/README.md`.
- `evidence/armpi_grasp/20261116/` — Cartesian control, joint-sampler run,
  unreachable negative control, and the headless MoveIt log.

### Verified (mock only)

- Cartesian exact-FK control: 54 candidates, the FK-exact candidate returned
  `PLAN_OK` with 48 trajectory points; the ±10–15° neighbours failed IK.
- Joint sampler at `(0.1382, 0.0225, 0.0896)`: 80 candidates → 40 IK OK →
  40 FK OK → 26 collision-free → 3 `PLAN_OK`.
- Unreachable point `(0.60, 0.0, 0.30)`: 0 candidates, exit 1.
- `armpi_grasp_planner`: 15 tests pass; flake8 and PEP 257 pass.
- Local KDL FK matches MoveIt `/compute_fk` to 0.0 mm per link at the checked
  configuration.

### Known limitations

- No hardware, camera, scene objects or execution. Collision checking is
  self-collision only.
- TCP offset and joint limits are provisional.
- The joint sampler's target point is model-generated, not camera-generated;
  this is not visual grasping.
- The best plan (`js_0243`) sits on `joint5 = -2.09 rad`, a joint limit; limit
  behaviour is unmeasured.

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
