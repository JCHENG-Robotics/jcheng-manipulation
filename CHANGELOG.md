# Changelog

All notable changes to this project are documented here. Format based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased] — 2026-11-18 depth table plane, supervised approach, link/DDS fixes

### Added

- **Studio G6 supervised motion write path** — gateway `/motion/dispatch` and
  `/motion/stop`, a pure-Python `MotionGuard` that revalidates pulse limits,
  step, duration, measured-feedback age and single-command occupancy, and a
  backend `HardwareAdapter` dispatch path gated by `ARMPI_HARDWARE_MOTION=1`
  plus the fixed `ARMPI_SUPERVISED_MOTION_ACK` phrase while limits are
  unverified. Effective supervised limits: one joint, 23 pulses (0.096342 rad),
  duration ≥ 1.0 s; completion from measured `servo_states`; an unconfirmable
  outcome latches `MOTION_OUTCOME_UNKNOWN` until a backend restart. Verified on
  the real robot with no motion (measured delta 0.0 rad); see
  `armpi-debug-studio/docs/g6-motion-write-path.md` and
  `evidence/studio_g6/20261118/`.
- `tools/export_gateway_motion_config.py` — validates and exports the adopted
  pulse limits as the gateway's motion JSON.
- `armpi-debug-studio/tools/start-supervised-backend.sh` — explicit operator
  launcher for the supervised backend.
- `armpi-debug-studio/ros2_gateway/deploy/supervised-motion.md` — robot-side
  supervised gateway procedure, including the PID-based shutdown (a `pkill -f`
  pattern that starts with `--` silently fails).
- `tools/phase2/depth_plane_fit.py` — read-only RANSAC + SVD table-plane fit
  from `/depth_cam/depth_registered/points`. Composes `base_link` from the
  **measured bus-servo joints** and the hand-eye; records the driver's cloud
  frame-label/optical-layout mismatch. It found a **147,626-inlier plane,
  0.734 mm RMS**, normal **3.66°** from `base_link` +z.
- `tools/phase2/execute_plan_json.py` — guarded executor for an inspected
  `grasp_candidate_check` trajectory: re-validates with the safety service,
  checks the start state against `/joint_states`, and refuses execution unless
  `--execute --i-have-clear-workspace` is passed.
- `evidence/phase2/onsite_20261118/approach_20261118/` — supervised object-free
  approach at `camera_forward`: 3/3 safety-approved plans, best plan executed,
  final TCP **1 mm** from the plan.
- `evidence/phase2/onsite_20261118/depth_plane_20261118.md` — depth plane,
  215 mm ruler check, RGB board comparison, tag absolute-z caveat, and the
  table-alignment plan-only result.
- SRDF `table_down` named state (measured bus-servo pose).
- `armpi_grasp_planner/joint_limits.py` — pure-Python joint-limit margin
  policy (`DEFAULT_JOINT_LIMIT_MARGIN_RAD = 0.03`) plus `--joint-limit-margin`.
- `evidence/phase2/onsite_20261118/joint_limit_margin_20261118.md` and its
  two mock plan-only reports — closes the "sampler needs a joint-limit margin"
  leftover from the table-alignment run.
- `tools/phase2/table_collision.py` — publishes the measured table plane as an
  oriented `table_surface` collision object on `/collision_object` (top face on
  the plane, 1 Hz republish, `--remove`); pure geometry functions are tested.
- `evidence/phase2/onsite_20261118/table_envelope_20261118.md` — table-envelope
  analysis, service matrix and planning-scene check.
- `tools/phase2/chassis_jog.py` — chassis link status (`--status`), predicted
  motor values (`--dry-run`), and a supervised `/cmd_vel` pulse behind `--yes`
  with hard caps (0.30 m/s, 0.60 rad/s, 3 s). It publishes at 20 Hz so the node
  watchdog does not fire mid-pulse, sends an explicit zero, verifies the echoed
  command is zero, and writes a JSON record.
- `evidence/phase2/onsite_20261118/chassis_link_20261118.md` — zero-motion
  chassis verification: `/cmd_vel` → `mecanum_chassis_node` → four-motor
  `MotorsSpeedControl` (all 0.0), battery 7691 mV. Records the open issues
  (rps-vs-duty unit mismatch, no odometry/IMU, no `/cmd_vel` safety coverage,
  vendor teleop topic mismatch) and the ground-test protocol.
- `docs/hardware/patches/controller_mecanum_watchdog.patch` and
  `docs/hardware/vendor_baseline.md` item 5 — the deployed chassis node is a
  hardened fork of the vendor `mecanum.py` (0.5 s `/cmd_vel` watchdog at
  20 Hz, zero duty on timeout and shutdown; vendor `9c54a5d5…` → deployed
  `c599517a…`).
- `tools/phase2/rgbd_snapshot.py` — same-stamp RGB + registered cloud + raw
  depth/IR + joints snapshot for the Humble container (numpy only; no cv2).
- `tools/phase2/rgbd_board_check.py` and `rgbd_tag_check.py` — offline
  same-frame RGB-D plane comparisons for a chessboard and a matte AprilTag.
- `evidence/phase2/onsite_20261118/rgbd_same_surface_20261118.md` and its
  reports — closes the board-thickness leftover.

### Changed

- **The wireless link is now the default.** Robot `wlan0` rejoins the open
  office SSID `tjyx_office` (`10.51.128.226/23`, NM profile
  `tjyx_office-robot` recreated, AP `HW-5401EF08` autoconnect off); the robot's
  CycloneDDS binds `wlan0` (wired backup `.bak_eth0_20261118`), the laptop's
  binds `wlp4s0`, and the gateway binds `10.51.128.226`. Verified SSH, DDS
  (`/joint_states`, `/battery` 8289 mV, `/tf` 12.5 Hz) and gateway HTTP. The
  core restart left the arm unmoved; the camera launch was restarted (6.5 fps
  in-container) but cross-host image topics drop to ~2 Hz over WiFi — use the
  gateway `/camera/snapshot` or run capture tools inside the container.
  `eth0` stays attached as the fallback. Log:
  `evidence/phase2/onsite_20261118/wireless_switch_20261118.md`.
- **Hardware joint state now reports measured positions.** The gateway
  subscribes to `/controller_manager/servo_states` and `/state` carries both
  the command echo and a `measured` block; the backend uses the measured pulse
  as `position_rad`/`truth=measured`, while `target_rad` still comes from the
  echo. Step validation uses the measured position, so consecutive commands
  cannot each be "one step from the last target" while the arm lags. The
  read-only hardware Playwright expectation was updated from `commanded` to
  `measured`.
- **Grasp candidates can no longer sit on an adopted joint bound.** The
  joint-space sampler drops grid values inside the margin and adds the margin
  boundary; the evaluator rejects a boundary IK solution as
  `JOINT_LIMIT_MARGIN` before FK/collision; IK seeds are clamped into the
  shrunk range (the SRDF `init_pose` seed's `joint4 = −1.6591` is outside the
  adopted `−1.5917` low bound). `--joint-limit-margin 0.0` reproduces the old
  search. The 2026-11-18 table-alignment target now has no margin-safe plan,
  which is the intended outcome for a target reachable only on the bound.
- **Table grasps are no longer vetoed by the flat z bound.** The safety policy
  has a calibrated `table_plane` check (measured normal/offset, 10 mm
  clearance, code `TABLE_CLEARANCE`); the workspace lower z is now a coarse
  hard floor at −0.10 m. A 120k-sample FK search found the lowest reachable TCP
  at z ≈ −0.0445 m (33 mm above the table), so the old 0.00 bound blocked the
  whole reachable table band. The live `armpi_safety` node must be restarted to
  load the new config. The table collision object is still required for
  link/gripper geometry and must be running during planning/execution.
- **The 6.1 mm RGB-board-vs-depth-table number is not the board thickness.**
  The board is **11 mm** (calipered) and the chessboard is invisible to the
  depth camera (high-contrast/glossy board → depth holes filled with the table
  plane). A same-frame matte-AprilTag test measured a **−4.25 / −4.65 mm
  same-surface RGB-vs-depth gap** at the table view (0.37° / 0.76°); the RGB
  absolute z is biased −3 to −5 mm. The depth table plane remains the
  absolute-z reference; use RGB XY + depth z with a ≥5 mm vertical margin. The
  earlier "RGB and depth agree within the board thickness" conclusion is
  superseded.
- **Corrected the absolute table height.** The depth plane gives
  **z = −0.0787 m** under the camera and the 215 mm gripper-fingertip ruler
  check **−0.0784 m** (1.3 mm apart). The RGB board top is 6.1 mm above the
  depth plane (board thickness), so the previous "~40–50 mm RGB depth bias"
  conclusion (based on a −0.024 m chassis estimate) is **superseded**; the
  table is at **z ≈ −0.078 m**. The tag's absolute z is separately biased
  6–13 mm low; its lateral 0.67 % scale result stands.
- Safety limits now use the adopted EEPROM-bounded joint ranges, and both the
  named-state executor and the plan-JSON executor require a safety approval
  before execution.
- The real MoveIt graph now requires the link fixes: the laptop link is a plain
  static `10.42.0.1/24` (not NetworkManager `shared`), EEE is disabled, and the
  container's CycloneDDS config is mounted at `/config/` instead of under
  `/workspace` (a 0-byte placeholder bug).

### Verified (real hardware, 2026-11-18)

- `camera_forward` supervised approach: 66 candidates → 33 IK/FK → 3 plan-only
  OK → 3/3 safety approved; executed best plan, `execute_status=4`, final TCP
  (0.259, −0.001, 0.235) m vs planned (0.259, 0, 0.236) m.
- Kinematic limit: at `camera_forward` height the arm reaches only ~0.10 m in
  front of the camera; the 0.22 m optical-axis target is 0.124 m out of reach.
  The camera_forward view volume is not a grasp volume.
- Table alignment plan-only: 1 `PLAN_OK` + safety approved, but its candidate
  sits on the joint2 low bound, so it was not executed.

## [Unreleased] — 2026-11-18 table-down calibration and absolute-depth correction

### Added

- `evidence/calibration/20261118/table_down/` — a 15-pose table-down chessboard
  capture with the board flat on the table (20.0 mm squares): TSAI
  `T_link4_camera` = `[-0.03194, -0.00808, 0.05918] m`, RPY
  `[-1.478, 0.331, -90.187]°`; board scatter **2.7 mm / 0.42°**, held-out
  2.2 mm / 0.29°, leave-one-out 3.1 mm / 0.45° std, intrinsic RMS 0.052 px.
- Depth-camera check at the table pose: **340 mm** median versus a **~380 mm**
  RGB chessboard PnP distance.

### Changed

- Corrected the absolute-depth interpretation: the table-pose ~70 mm z offset is
  a **view-dependent RGB PnP depth/tilt bias at ~48° incidence** (plus the
  17.54 mm mecanum chassis), not an FK extrapolation. The chessboard squares are
  20.0 mm and the refined K/D changes z by only ~1 mm. Relative tag
  displacements (0.67 % lateral) stand; absolute table-top z needs the depth
  camera, a near-normal view, or a table-view depth correction.

## [Unreleased] — 2026-11-18 independent AprilTag displacement validation

### Added

- `tools/phase2/validate_hand_eye_tag.py` — detects an AprilTag in frames
  captured with `chessboard_capture.py --check-only`, composes the PnP pose with
  measured bus-servo FK and the validated `T_link4_camera`, and reports
  per-frame spread and residuals against an optional surveyed ground truth.
- `evidence/phase2/onsite_20261118/tag_displacement_validation/` — a flat 36h11
  sticker (ID 3, 24.9 mm black square) moved 100.0 mm laterally and 100.0 mm
  table-forward on the table with the arm held at joint4 = −1.55.

### Verified (real hardware, 2026-11-18)

- Lateral A→B: **100.67 mm** predicted vs **100.0 mm** ruler (0.67 %); table
  forward B→C: 104.87 mm vs 100.0 mm (4.87 %); angle between them **92.3°**.
  Detection 5/5 at each position, max reprojection 0.43 px.
- The absolute table-pose prediction is ~90 mm below the table plane. This is a
  constant FK/hand-eye extrapolation offset at joint4 = −1.55 (calibration used
  joint4 ≈ −0.71); it cancels in the relative displacements but absolute 3D use
  at table-down poses requires extending the calibration.

## [Unreleased] — 2026-11-18 Studio G5 read-only trajectory recorder and live TF verification

### Added

- **G5 read-only trajectory recorder.** New
  `ros2_gateway/armpi_debug_gateway/recorder.py` and pure-Python
  `trajectory_capture.py`. The recorder subscribes to the vendor command echo
  (`/controller_manager/joint_states`, truth `commanded`) and to
  `/controller_manager/servo_states` (bus-servo positions converted from pulses
  to radians, truth `measured`), detects the motion start from the last
  unchanged commanded sample, and writes a raw capture plus an optional G2
  `schema_version: 1` dataset. It creates no publisher, service client or
  motion endpoint; an out-of-range servo pulse aborts the capture.
- `tools/phase2/real_moveit_execute.py --trajectory-json` writes the inspected
  MoveIt plan in the recorder's planned-stream format.
- `armpi-debug-studio/tools/build_g5_dataset.py` attaches a plan JSON to a raw
  capture offline; `dataset_from_capture` preserves capture notes, topics,
  abort flags and counters in dataset metadata.
- `armpi-debug-studio/tools/verify_tf_snapshot.py` composes a gateway `/tf`
  snapshot and prints a root-to-child translation/quaternion for comparison
  with `tf2_echo`.
- **Environment-gated hardware Playwright.** `frontend/e2e/hardware.spec.ts`
  and `start-backend-hardware.sh` run only with `ARMPI_E2E_HARDWARE=1` and a
  private `ARMPI_GATEWAY_URL`/`ARMPI_GATEWAY_TOKEN_FILE`. The spec checks LIVE
  battery/camera, commanded joint state, G4's live `/tf`, and that a complete
  preview/execute attempt is rejected with `GATE_REJECTED`; it never runs in
  CI and never moves the arm.
- Gateway `/tf` now reports `max_dynamic_age_ms` and `max_static_age_ms`
  separately; latched static edges no longer make a live graph look stale.
- Evidence `evidence/studio_g5/20261118/`: a 5 s no-motion capture (242
  commanded / 241 measured samples at ~48 Hz) and a supervised
  `camera_forward` moving capture (0.892 s, 712 commanded / 711 measured
  samples), their imported G2 analyses, live `/tf` snapshot, `tf2_echo`
  comparison, hardware Playwright log and screenshots. Docs:
  `armpi-debug-studio/docs/g5-trajectory-recorder.md`.

### Fixed

- The preview-token payload no longer includes the live `from_rad`. Hardware
  command echo can move slightly between preview and execute; binding it
  produced `PREVIEW_PAYLOAD_MISMATCH` before the safety gate could evaluate the
  command. The token still binds command id, session, mapping hash, targets and
  durations.
- `servo_states_to_positions` accepts the vendor's scalar `id`/`position`
  fields (the first hardware run rejected them as non-integers).
- The Dashboard feedback-age hint no longer says "simulated in Mock" in
  hardware mode; it reports the actual command-echo/measured truth.

### Verified

- Backend `pytest` **76 passed**; gateway `pytest` **24 passed**; Playwright
  mock **10 passed**; hardware **3 passed**; `tsc -b && vite build` succeeds.
- Live `/tf`: 11 dynamic + 8 static edges, `status=ok`, freshest ~15–50 ms.
  Gateway `base_link→link5` composition `[0.16416, 0.00031, 0.23982] m` vs
  `tf2_echo` `[0.164, 0.000, 0.240] m` (below 0.4 mm). A stale laptop mock
  publisher was stopped before the comparison so the graph had no duplicate
  `camera_optical_calibrated` frame.
- A supervised `camera_forward` execute was recorded and imported: 0.892 s
  plan, 10 waypoints, 712 commanded / 711 measured samples at ~47.4 Hz,
  tracking max 0.008321 rad / RMSE 0.003962 rad and command-fidelity max
  0.008326 rad. The error is dominated by servo pulse quantization
  (`0.00419 rad` per tick) and a one-period start offset from the `0.005` arm
  threshold used in that run; it is not a full tracking-accuracy result.
- The recorder default motion threshold is now `0.002` rad, below one servo
  tick, so a single-tick move arms the capture at the last stable sample.

## [Unreleased] — 2026-11-18 Studio G4 read-only Visual Workbench

### Added

- **G4 Visual Workbench.** New `Visual Workbench` page and `/api/visual` API.
  The backend parses the project URDF/xacro (including the gripper include),
  builds the kinematic tree, and computes link frames from the current joint
  state. The response separates URDF/model, joint truth, camera metadata and
  calibration, and reports missing/cyclic/degraded TF inputs instead of
  inventing frames.
- Measured camera extrinsic from the validated TSAI hand-eye
  (`T_link4_camera`, 2026-11-17) is shown as `camera_optical_calibrated`;
  nominal URDF camera frames are explicitly marked. The camera body frame is
  `rgb_camera_link`, with a warning that PnP must use the calibrated optical
  frame.
- Metadata-only camera references: `POST/GET /api/visual/camera-references`
  store timestamp, resolution, encoding, truth, calibration version and TF
  status; image bytes are not stored.
- Optional read-only gateway `/tf`: subscribes to `/tf`/`/tf_static`
  (`tf2_msgs`), caches the latest edge per parent/child, and serves a
  status/edge/age snapshot. `/api/visual` composes it onto `base_link`, marks
  frames `live_tf`, and falls back to URDF FK for missing frames. This endpoint
  is no-ROS tested but **not yet verified on the robot**.
- `configs/visual_model.yaml` records URDF audit status and the camera
  extrinsic source/version.
- 8 new backend visual tests and 1 Playwright spec; backend is now **74 passed**
  (the total includes one serving cache-header test already in the worktree),
  Playwright **10 passed**; frontend build succeeds.

### Changed

- Studio README/status table, root README/README.zh-CN counts, Settings roadmap
  and E2E coverage docs updated. The Workbench is read-only: the `/tf` path is
  optional and not hardware-verified, there is no motion endpoint, and no
  safety-chain dependency; URDF limits remain provisional and mock camera
  frames remain simulated.

## [Unreleased] — 2026-11-18 Studio G3 offline Experiment Lab

### Added

- **G3 Experiment Lab (offline).** New `Experiment Lab` page and
  `/api/experiments` API. Immutable experiment definitions capture purpose,
  data origin, hardware/calibration/mapping versions, stop rules, planned
  pairs, seed and fixtures. Trials are append-only and tied to
  `experiment_id`/`pair_id`/`trial_id`, method order, success, continuous
  error, rotation error, failure code, measurement source, optional G2
  `trajectory_id` and artifact references.
- Failure vocabulary covers communication, timeout, joint limit, planning,
  trajectory execution, vision, calibration, grasp failure, safety veto and
  unknown. Duplicate pair/method and pair/order records are rejected.
- Paired analysis (`analysis_version g3-1`) mirrors the Phase 3
  `armpi_analysis.paired_trials` formulas: seeded bootstrap CI of the mean
  difference, exact sign test, exact McNemar, effect size, order effect,
  failure-code counts, incomplete-pair reporting, and warnings for simulated
  data / small samples / underpowered binary endpoints. CSV and metadata JSON
  exports are recomputable.
- Deterministic simulated demo: 16 pairs / 32 trials, clearly marked
  `simulated`; binary endpoint warning is shown (McNemar p=0.375).
- Contract and metric definitions:
  `armpi-debug-studio/docs/g3-experiment-lab.md`; evidence bundle
  `evidence/studio_g3/20261118/`.
- 9 new backend tests (`test_experiments.py`) and 1 Playwright spec; backend is
  now **65 passed**, Playwright **9 passed**; frontend build succeeds.

### Changed

- Studio README/status table, root README/README.zh-CN counts, Settings roadmap
  and E2E coverage docs updated. G3 stores references only: it does not read
  rosbag2, execute a trial, or touch hardware. No real experiment was recorded.

## [Unreleased] — 2026-11-18 Studio G2 offline Trajectory Inspector

### Added

- **G2 Trajectory Inspector (offline).** New `Trajectory Inspector` page and
  `/api/trajectories` API. A JSON dataset stores `planned`, `commanded` and
  `feedback` separately with explicit truth labels; the backend validates the
  contract, computes per-joint max/RMSE/mean/final error plus sampling quality
  (rate, max gap, gaps, dropout, jitter), and stores one append-only row keyed by
  the canonical dataset SHA-256. Duplicate imports return the existing record.
- Recomputable evidence: aligned CSV exports (`sample=feedback|commanded`),
  dataset JSON export, and `POST /recompute` comparing fresh analysis with the
  stored result.
- Client-side Replay clock with a global `REPLAY` source badge and an explicit
  "no hardware command" banner. There is no trajectory endpoint that dispatches
  motion; the Playwright test asserts play/pause appends no audit event.
- Contract and metric definitions:
  `armpi-debug-studio/docs/g2-trajectory-inspector.md`; evidence bundle
  `evidence/studio_g2/20261118/`.
- 8 new backend tests (`test_trajectory.py`) and 1 Playwright spec; backend is
  now **56 passed**, Playwright **8 passed**; frontend build succeeds.

### Changed

- `capabilities.replay` is now `true` in mock and hardware adapters. Hardware
  motion remains disabled and `feedback.truth=commanded` still produces
  "tracking unavailable" instead of a fake error.
- Studio README/status table, root README/README.zh-CN counts, Settings roadmap
  and E2E coverage docs updated. G2 data is offline-only; the bundled demo is
  simulated and no hardware trajectory was captured.

## [Unreleased] — 2026-11-18 onsite: power-cut and gripper baseline

### Added

- `evidence/phase2/onsite_20261118/`: onsite runbook steps 1–5 complete
  (post-reboot health, power-cut behavior, gripper sign/range, real MoveIt
  `camera_forward` execution, real AprilTag detection). Step 6 is blocked by a
  torque-control defect.
- `tools/phase2/real_moveit_execute.py`: guarded plan/execute client; the
  inspection and execution use the same trajectory and execution requires
  `--i-have-clear-workspace`.
- `tools/phase2/capture_real_frames.py` and `analyze_real_tag.py`: capture
  PPM + CameraInfo + TF in the CycloneDDS container and detect a physical tag
  offline.
- `tools/phase2/servo_hold.py`: verified position-hold recovery after a
  torque-off.

### Changed

- **Gripper model corrected from measurement.** Supervised no-object test:
  increasing `r_joint` closes; mechanical range is `-0.8378 … 0.0 rad`
  (servo register 502 open … 642 closed). Updated `gripper.urdf.xacro`,
  `joint_limits.yaml`, the SRDF `open`/`close` named states (open was
  `+1.2`, now `-0.80`; close stays `0.0`), and `initial_positions.yaml`.
- **Power-cut behavior verified:** no electronic e-stop and no brake; the arm
  falls immediately when main power is removed. Recovery is automatic in
  ~89 s and the vendor init re-homes `joint1` to `+0.8378 rad`.
- **Real `camera_forward` executed:** 101 waypoints at 0.05 scaling; the arm
  reached the target within 0.013 rad / 5 pulses, smooth and without collision.
  MoveIt first reported `CONTROL_FAILED` because its 1.25× duration allowance
  expired while the vendor controller overran; `allowed_execution_duration_scaling`
  was raised 1.2 → 1.6 and a no-op re-execution then returned `SUCCEEDED`.
- **Real AprilTag detected:** 5/5 frames, id 0, 0.322 m, ≤0.17 px reprojection,
  ≤1.1 mm base-translation spread; the live ROS localizer node was not run
  because Jazzy lacks CycloneDDS.
- **Arm joint-limit survey blocked:** `SetBusServoState` does not disable
  torque on the vendor stack (verified on joint1 and the empty gripper), while
  a `ServosPosition` position hold reliably re-engages the servos. The arm was
  recovered to torque-on; EEPROM limits were captured read-only. See
  `evidence/phase2/onsite_20261118/step6_blocked.md`.
- Hardware docs updated: `measured_joint_limits.md`,
  `hardware_inventory.md`, `phase2/JOINT_LIMITS.md`.

## [Unreleased] — 2026-11-17 Session 4: error budget, paired trials, host/DDS benchmark

### Added

- `armpi_analysis` ROS 2 Python package (dependency-free): `error_budget.py`
  (measured/unmeasured source schema, RSS/worst-case aggregation, dominance,
  seeded Monte-Carlo interval, Markdown/JSON report and an SVG contribution
  chart), `paired_trials.py` (seeded within-pair randomization, append-only
  JSONL recorder, bootstrap/sign/McNemar analysis) and `host_benchmark.py`
  (host metrics and read-only ROS topic rate/jitter/header-age measurement).
- `armpi_analysis/config/error_budget_baseline.json`: partial project budget
  with measured hand-eye/fiducial terms and explicit unmeasured required terms.
- `tools/phase4/run_benchmark.sh` and `tools/phase4/tests`: one-command
  read-only laptop + Pi host/ROS benchmark collection.
- `.github/workflows/python-ci.yml`: flake8 and pure-Python tests for the
  analysis package, measurement protocol and phase-4 tools.
- Docs: `docs/phase3/README.md`, `ERROR_BUDGET.md`, `PAIRED_TRIALS.md`,
  `HOST_BENCHMARK.md`; evidence bundle
  `evidence/phase3/session4_20261117/`.

### Changed

- Camera rate correction: the Aurora 930 RGB stream is **~12 Hz cross-host**;
  the earlier 8 Hz figure came from an in-container subscriber that undercounts
  large images. Hardware docs, README/README.zh-CN, ROADMAP and the Phase 1
  evidence index now state the measurement location.
- README/README.zh-CN/ROADMAP status tables now distinguish the implemented
  framework from the still-partial project budget.
- Phase 0 acceptance and evidence index now reference the Phase 3 framework and
  the read-only benchmark.

### Verified (offline/synthetic/read-only, 2026-11-17)

- `colcon build --symlink-install` for `arm_interfaces`,
  `armpi_grasp_planner`, `armpi_safety`, `armpi_fiducial_localizer` and
  `armpi_analysis`: **5 packages finished**.
- `colcon test` for the same packages: **94 tests, 0 errors, 0 failures**.
- Pure-Python CI command: **flake8 rc 0**; **49 tests passed**.
- Error budget demo: translation RSS 2.311 mm / worst 3.800 mm; rotation RSS
  0.447° / worst 0.600°; both gates pass.
- Partial project budget: translation RSS 3.699 mm / worst 4.980 mm (5 mm
  provisional gate); rotation RSS 0.682° / worst 0.960° (2° gate); status
  **partial**, 7 required sources unmeasured.
- Paired-trial simulation: 24 pairs, true 1.0 mm effect recovered as 1.025 mm
  (95% CI [−1.095, −0.950] mm; sign p=1.19e-07); success 75.0% → 95.8%
  (McNemar p=0.1250, underpowered).
- Read-only benchmark: `/joint_states` 20.00 Hz (jitter 0.16 ms local),
  `/controller_manager/joint_states` 48.8 Hz, camera ~12 Hz cross-host,
  ping 0.24 ms average; laptop 4–5% CPU, Pi 1.8–10.7%.

### Limits

- The error budget is **partial**; repeatability, backlash, grasp execution,
  gripper placement, object-fixture pose and angular terms are unmeasured.
- The 5 mm / 2° tolerance is a provisional protocol gate, not a physical grasp
  tolerance.
- Paired-trial results are **synthetic only**; no physical trial exists.
- The benchmark is one read-only snapshot with different RMW stacks
  (Jazzy Fast DDS ↔ Humble CycloneDDS); it is not a load sweep or a real-time
  guarantee.
- `get_node_names()` fails in the robot container (`empty node name returned by
  the RMW layer`) and camera/static-TF logs repeat `serdata.cpp` errors; topic
  data still flows. Root cause unresolved.
- Robot root disk is 84.7% used (6.6 GiB free); no cleanup was done (R14 is
  outside Session 4).

## [Unreleased] — 2026-11-17 Session 3: fiducial → grasp bridge, mock E2E, safety veto, measurement protocols

### Added

- `armpi_grasp_planner/target_bridge.py` + `grasp_target_bridge`: one-shot ROS 2
  node that subscribes to `target_pose`, transforms it to `base_link`, checks
  pose age and marker tilt, applies an optional marker-frame offset, and runs
  the joint-space grasp search with an optional safety veto.
- `armpi_grasp_planner/launch/mock_visual_grasp.launch.py`: mock-only chain
  (MoveIt mock, down-looking camera frame, synthetic AprilTag, localizer,
  safety supervisor) on an isolated DDS domain (77, localhost only).
- `arm_interfaces/srv/ValidateTrajectory.srv` and `armpi_safety`:
  pure-policy watchdog/veto (`policy.py`) and ROS supervisor
  (`safety_supervisor`) with `armpi_safety/validate_trajectory`,
  `armpi_safety/reset_stop`, `safety_status` and a latched `safety_stop`.
- `armpi_safety/config/safety_limits.yaml`: provisional joint/rate/workspace
  limits; FK workspace check via `end_effector_link`.
- `tools/phase2/run_mock_visual_grasp_e2e.sh`, `tools/phase2/safety_service_check.py`:
  reproducible mock E2E and safety service negative controls.
- `tools/phase2/measurement_protocol.py`: dry-run/simulate/analyze schedules and
  analyzers for repeatability, backlash, visual displacement and gripper
  envelope (x/y/z translation plus yaw).
- Docs: `docs/phase2/FIDUCIAL_GRASP_BRIDGE.md`, `SAFETY_LAYER.md`,
  `MEASUREMENT_PROTOCOL.md`; evidence bundle
  `evidence/phase2/session3_20261117/`.

### Changed

- `MoveItGraspEvaluator` can include the full planned trajectory in reports and
  call a safety service for every `PLAN_OK` candidate; the CLI success requires
  at least one safety-approved plan when `--safety-service` is set.
- `grasp_candidate_check` gained `--safety-service`,
  `--max-start-state-distance` and `--include-trajectory`; its `best` entry
  prefers a safety-approved plan.
- Phase 2 docs/README/ROADMAP/zh-CN README now separate the mock visual chain
  from a real grasp.

### Verified (mock/synthetic, 2026-11-17)

- `colcon test --packages-select arm_interfaces armpi_grasp_planner armpi_safety armpi_fiducial_localizer`:
  **58 tests, 0 failures**.
- `python3 -m pytest tools/phase2/tests -q`: **26 passed**.
- Mock E2E: synthetic marker → localizer → bridge → 80 candidates → 40 IK →
  40 FK → 26 collision-free → 3 `PLAN_OK`; safety approved 3/3; best `js_0249`,
  87 points, 8.51 s; `E2E PASS`.
- Safety service: valid approved; joint-limit, velocity and timestamp
  negative controls rejected with the expected codes.
- Measurement dry-run/simulate: all four protocols produce schedules and
  analysis reports; no hardware data.

### Limits

- The mock E2E is synthetic and plan-only; nothing was executed and no real
  grasp exists.
- Safety limits are provisional and the layer is a veto, not an e-stop or a
  controller path.
- Measurement results are simulated (`noise=0.3 mm`, `bias=1.0 mm`), not
  robot measurements.

## [Unreleased] — 2026-11-17 Session 2: calibration pose selection and synthetic hand-eye gate

### Added

- `tools/phase2/calibration_pose_selector.py`: offline selection of 20–30
  diverse, executable chessboard poses over `joint1..joint4`. It evaluates
  URDF FK with the validated `T_link4_camera`, filters projected board
  visibility/distance/incidence, and runs greedy farthest-point selection in
  a weighted camera-pose/image feature space. The report includes camera
  spread, view-angle spread, image coverage, rotation-axis rank and camera
  coplanarity.
- `tools/phase2/tests/`: pytest coverage for the synthetic hand-eye recovery,
  board projection, greedy selection, pose serialization and the real
  validated board/camera pair.
- `chessboard_capture.py --poses-file`: executes absolute five-joint targets
  from the selector’s `poses.json`, validates them against the provisional
  joint limits, and records `pose_source`/`poses_file` in `samples.json`.
- Evidence: `evidence/calibration/20261117/pose_selection/` (24-pose report,
  slim pose list, three-panel plot, README) and
  `docs/phase2/CALIBRATION_POSES.md`.

### Changed

- `tools/phase2/chessboard_hand_eye.py`: hand-eye math split into
  `solve_hand_eye_methods()` and `select_best_method()`; added
  `leave_one_out()` validation (every sample held out once); the URDF import
  is lazy so the math can be tested without a ROS environment. The refactor
  reproduces the committed 2026-11-17 calibration bit-for-bit.
- Phase 2 docs/status now distinguish the validated 14-pose result from the
  planned 24-pose capture.

### Verified (offline, 2026-11-17)

- Pose selector: 14,176 evaluated poses → 3,093 feasible → 24 selected, including
  the exact validated base pose. All checks pass: camera spread 0.096 m,
  view-angle spread 33.5°, image-centre std 92.8/50.6 px, rotation-axis rank
  ratio 0.067, planarity ratio 0.283.
- `python3 -m pytest tools/phase2/tests -q`: **13 passed**.
- The refactored solver on the committed dataset reproduces TSAI
  `T_link4_camera` exactly (max matrix difference 0.0), board scatter
  2.94 mm / 0.39°, held-out 3.29 mm / 0.43°, leave-one-out 3.41 mm / 0.44°
  std (max 6.52 mm / 0.81°), result PASS.
- The 24-pose set has **not** been executed on hardware; it is a plan for the
  next supervised capture.

## [Unreleased] — 2026-11-17 Session 1: camera frame contract, vendor baseline, real-background check

### Added

- `armpi_fiducial_localizer/frames.py` and
  `config/camera_optical_frame.yaml`: the measured
  `rgb_camera_link → camera_optical_calibrated` contract, derived from the
  validated hand-eye and the full-precision live body TF.
- `armpi_fiducial_localizer/camera_frame_publisher.py`: publishes the measured
  optical frame as a static TF; `fiducial_localizer.launch.py` starts it by
  default (`publish_optical_frame:=true`).
- `tools/phase2/capture_still.py`: no-motion RGB/depth/TF capture inside the
  robot container.
- `tools/phase2/real_background_fiducial_check.py`: negative control plus
  semi-synthetic marker accuracy on real frames.
- `docs/hardware/camera_frames.md` and `docs/hardware/vendor_calibration_baseline.md`.
- Evidence: `evidence/phase2/frames_20261117/`,
  `evidence/phase2/real_background_20261117/`,
  `evidence/phase2/vendor_calibration_20261117/`.

### Changed

- `FiducialLocalizer` now tags `target_pose` with `camera_frame`
  (`camera_optical_calibrated` by default), not the incoming `CameraInfo`
  body frame; `output_frame:=base_link` still works through TF.
- `FiducialDetector` refines the PnP pose with `solvePnPRefineLM` by default
  (`refine_pose=True`), removing the fronto-parallel IPPE ambiguity.
- `tools/humble_moveit/cyclonedds-laptop.xml` now binds the direct-Ethernet NIC
  `enx00e04ca373c7` (the robot config is on `eth0`).
- Hardware/phase docs corrected: 95% → 87% disk, wired link current, and the
  superseded body→optical numbers (89.34°, `[15.4, 10.0, -28.3] mm`) replaced
  by the full-precision result (89.798°, `[5.4, 10.1, -27.4] mm`).

### Verified (real hardware, 2026-11-17)

- Full-precision `link4 → rgb_camera_link` = `[-0.039995, -0.019122, 0.086003] m`;
  composing it with `T_link4_camera` reproduces the hand-eye to `1.4e-17`.
- 8 real RGB frames: **0** false detections with `DICT_APRILTAG_36h11`.
- Semi-synthetic 36h11 overlay on real frames: **15/15** detected, max
  **1.69 mm / 0.53°**, mean 1.12 mm / 0.30° (gate 5 mm / 2°).
- Vendor `transform.yaml` decoded as a table/white-area calibration; the
  shipped `hand2cam_tf_matrix` is **89.6 mm** from the validated
  link5-relative optical transform, so it is **not valid Baseline A** for the
  current camera mount.

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
