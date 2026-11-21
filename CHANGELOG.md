# Changelog

All notable changes to this project are documented here. Format based on
[Keep a Changelog](https://keepachangelog.com/en/1.1.0/).

## [Unreleased] — 2026-11-21 feature-freeze hardening (offline)

Offline only; no hardware motion and no new algorithms. This is the first
P0/P1 hardening batch after the feature-freeze audit.

### Added

- `tools/verify_config_consistency.py` and `tools/tests/`: read-only checks
  that the five arm limits agree across the xacro, MoveIt `joint_limits.yaml`,
  `safety_limits.yaml` and the Studio `joint_mapping.yaml`, and that the
  constant low-configuration correction agrees across the runners and
  `takeover.yaml`.
- `docs/phase2/MOTION_WRITE_PATHS.md`: the six arm/chassis write paths, their
  guards, limits and freeze rules.
- `THIRD_PARTY_NOTICES.md`: project MIT code, vendor-derived model assets used
  with permission, and the MoveIt/librealsense notices.
- `.github/workflows/studio-ci.yml`: backend tests, gateway tests and the mock
  Playwright suite on a stock runner.

### Fixed

- **README pair:** the quick-start test command now runs from the repository
  root with `PYTHONPATH`, covers the full offline tool suite, creates the
  Studio virtualenv and installs Chromium before Playwright. The Python
  prerequisite is now 3.11+ for the Studio backend.
- **English README:** the stale random-placement limitation ("predictions, not
  results") was replaced with the completed 24-pair result and its metric
  caveat.
- **Studio README:** current test counts (144 backend / 80 gateway / 16
  Playwright) are stated before the dated historical notes.
- **Package metadata:** the four `TODO: License declaration` packages now
  declare MIT (or `BSD-3-Clause` for the MoveIt config); the display package
  description is filled in.
- **Python CI:** now installs headless OpenCV and runs the full phase-2/3/4
  and `tools/tests` suites plus the config consistency check.

### Verification

- `tools/verify_config_consistency.py`: `CONFIG OK`, 5 joints and 6 correction
  sources agree.
- New consistency tests: 4 passed.
- Combined offline Python run: **248 passed / 7 skipped**.
- Studio backend: **144 passed**; gateway: **80 passed**.
- Mock Playwright: **10 passed**; mock pose/task: **3 passed**.
- `scripts/verify_offline.sh`: `OFFLINE CHECKS PASSED`.

## [Unreleased] — 2026-11-21 documentation unification

Documentation only; no new hardware claims. The README pair, ROADMAP,
acceptance report and this changelog were re-synchronized on the 2026-11-21
baseline (Studio G7/G8/G8.1 + the G8 fast path, and the completed
random-placement 24-pair trial).

### Fixed

- **Both READMEs:** test counts corrected to **144 backend / 80 gateway**
  (the contribution list and quick start still said 125/72; the English status
  row said 136/80). The Chinese README gained the missing
  random-placement and G8 fast-path key-result rows, and both status tables
  now record the camera feeds routed through the read-only gateway and the
  Chinese operator UI.
- **One error-budget baseline:** the partial 2026-11-19 row (RSS 3.995 mm,
  4/7 sources) was removed from both key-result tables; the table keeps the
  completed 2026-11-20 budget (RSS 13.325 mm, bound gates fail, empirical
  placement mean 3.080 mm) as the single status baseline.
- **README deliverables:** removed a duplicated "Random-placement 24-pair
  follow-up" bullet; the Chinese checklist now carries the same "fixed
  24-mark fan was superseded before data" note as the English one.
- **ROADMAP:** the Studio row no longer says "widened locate region
  0.15–0.55 m"; it records the actual locate window (`x 0.15–0.55 m`,
  `y ±0.15 m`) and the 30 mm minimum top extent that accepts a 34.6 mm visible
  top. §10 no longer says the new-flow session is "starting"; the trial
  completed all 24 pairs. §7's status date and the 2026-11-21 follow-up were
  aligned with the status table.
- **Acceptance report (`docs/phase0/PHASE0_ACCEPTANCE.md`):** §1 is now the
  2026-11-21 verdict — Studio G1–G8.1 + the G8 fast path at 144/80 tests, and
  the random-placement follow-up **Pass** (9/24 vs 24/24, McNemar
  p = 6.1×10⁻⁵). The old "24-site pre-registered, not run" row and the §6 gap
  table were replaced with the completed outcome. The Phase 0 mock checklist
  (§2) is unchanged.
- **Sibling contradiction (`docs/phase3/PAIRED_TRIALS.md`):** the site-diversity
  bullet no longer says "the hardware session has not run yet"; it records the
  fixed fan as superseded before data and the completed 2026-11-21
  random-placement outcome. The fixed-fan design stays as history.

### Verification

- Re-collected instead of copied: `backend/tests` → **144**, `ros2_gateway/tests`
  → **80**; Playwright suites unchanged (10 mock + 3 pose/task + 3 hardware).
- `git diff --check` clean; relative links in the changed documents resolve;
  the README TOC anchors were re-checked. No evidence file was modified.

## [Unreleased] — 2026-11-21 Studio 中文化 + 进度一致性

### Added

- **相机网关路由（Studio）：** `ARMPI_CAMERA_GATEWAY_URL` /
  `ARMPI_CAMERA_GATEWAY_TOKEN` 可选；设置后 `/camera/*` 与 `/aux-camera/*`
  走独立网关，运动/状态仍走 `ARMPI_GATEWAY_URL`。`tools/studio/start_studio.sh
  --takeover` 默认把相机路由到只读网关 8765（`CAMERA_GATEWAY_PORT=8765`），
  姿态网关 8766 保持 `--no-aux-camera --camera-topic /armpi_pose_camera_off`；
  主相机与辅助相机都从 8765 取图，UVC 辅助设备不再与运动网关争用。
  新增 `backend/tests/test_camera_routing.py`（4 项路由测试）。

### Fixed

- **G8 定位横向窗口与顶面门禁：** `task.locate.y_range_m` 从 0.09 m 放宽到
  0.15 m，`min_top_extent_m` 从 0.035 m 放宽到 0.030 m。2026-11-21 实际点云里的
  39 mm 木块先位于 `(0.426, -0.100, -0.057) m`（仅因横向超出 ±90 mm 被报
  `LOCATE_FAILED`），旋转移位后又测到 34.6 mm 顶面（被 35 mm 门禁误判为
  `partial_top_view`）。现在前者返回 `APPROACH_REQUIRED` 和横向底盘脉冲，后者
  直接接受；`LOCATE_FAILED` 的文案同时列出被拒簇的门禁原因和中心点（例如
  “横向超出定位窗口，中心 […] m”），不再只有 `no usable cube point`。新增定位
  窗口、顶面门禁与拒绝原因测试，后端 **144 passed**。

### Changed

- **Debug Studio 全中文界面：** 导航、页面标题、表头、状态标签、按钮、提示与解释
  全部改为中文；协议枚举（`Completed`、`commanded`、`LIVE` 等）通过
  `frontend/src/labels.ts` 映射为中文标签，原始代码保留在 tooltip/详情中，便于与日志、
  CSV 和 evidence 对照。
- **过期文案校正：** 总览/诊断/设置/关节控制不再声称“G1 只读、运动未实现”；
  改为 G1–G8.1 + G8 快路径的现状（硬件默认只读，运动需显式接管；限位已普查并验证；
  `/servo_states` 是命令缓存）。设置页进度与限制列出 G6/G7/G8/G8.1 与实测 13.46 s /
  2.27 mm 落点。
- **后端用户可见文案中文化：** 实验方法/失败代码标签、分析限制与警告、G2 示例名称与
  说明、G4 的 URDF/TF 警告、存储故障与硬件探针提示；协议值、错误码与 evidence 字段
  保持原样。
- 前端 `index.html` 标题与产品名改为“ArmPi 调试工作台”。
- 同步更新 Playwright e2e 断言（10 mock + 3 pose-scope 通过）与后端测试断言。

## [Unreleased] — 2026-11-21 Studio G8 fast task path

### Added

- **Fast G8 task run (Studio):** the task plan now emits **7 operator phases**
  instead of 13 steps and executes each arm phase as one dense S-curve
  (`backend/app/domain/profile.py`, the pure port of the hardware-verified
  `tools/phase2/motion_profile.py`): 4.0 rad/s / 20 rad/s² free moves,
  0.6/0.35 rad/s grasp descents and 1.4/0.8 rad/s release descents. The
  separate `lift_150` move is replaced by one grasp -> view move, and the
  measured 0.35/0.5/0.45 s gripper timings replace the slow path's 1.5/2.6/4.0 s.
- **Dense-profile execution in the gateway:** `POST /motion/dispatch` accepts
  `profile.points`; the guard re-validates every segment (pulse limits,
  per-segment velocity, discrete acceleration, measured start, endpoint
  equality) and publishes one `ServosPosition` per segment at the segment
  cadence. The busy/stop/bus-verification state machine is unchanged; the
  verification read still checks the endpoint only.
- **Merged chassis approach:** the G8.1 base pulses are part of the task run.
  `APPROACH_REQUIRED` is handled inside the run (pulse -> settle -> re-plan,
  max 3), then the arm phases execute; the standalone "底盘接近" button and
  its confirmation modal are removed. The terminal runner
  `tools/studio/g8_task_run.py` implements the same loop and iterates phase
  commands with `profile`/`fast` bodies.
- Fast-path contract in `configs/takeover.yaml` under `task.fast`
  (`vmax/amax/dt`, descent/final velocities, gripper durations, a 900 raw/s
  sanity ceiling on the commanded interpolation rate, `gripper_settle_s 0.6`,
  `verify_settle_s 0.35`); the exporter writes it as the gateway
  `safety.profile` block. The slow pose path keeps its 0.5 rad/s cap, 1.0 s
  floor and 100 raw/s gripper validation.
- Tests: `backend/tests/test_profile.py` (reference-profile parity),
  `ros2_gateway/tests/test_profile.py` (per-segment rejection, fast gripper,
  endpoint equality), and task/mock tests for the 7-phase plan and the profile
  preview/execute token. Backend **136 passed**, gateway **80 passed**, pose
  scope Playwright **3 passed** (the full task run completes in mock).
- **Hardware verification (2026-11-21):** the fast path ran on the real robot.
  The first attempt performed the in-run base pulse (13 mm) and the first three
  arm profiles, then `close_pre` was rejected before publication because the
  adapter did not forward `fast: true` (the gateway applied the 1.0 s slow
  floor). After the fix the second run passed: 7 phases, 5.466 s commanded /
  **13.46 s wall** (the old 13-step plan was 33.56 s), `close_pre` 309,
  `close_grip` 414, `held: true`, `release_open` 174, landing **2.27 mm
  planar**. Evidence:
  `evidence/studio_g8_fast/20261121/` (`run1.json`, `run2.json`,
  `after_putback_plan.json`).

### Fixed

- The hardware adapter now forwards `fast: true` on a single-command dispatch
  (gripper-only fast steps). Without it the gateway used the slow 1.0 s
  duration floor and rejected a 0.35 s `close_pre`; the fix is covered by
  `test_fast_flag_is_forwarded_to_the_gateway`.

### Changed

- `POST /api/motion/preview` and `/api/motion/execute` accept an optional
  `profile {vmax_rad_s, amax_rad_s2, dt_s}` and `fast` flag. A profile binds
  the endpoint and profile parameters in the preview token; the duration is
  recomputed from the live start at execute time so a few pulses of actual
  drift do not break the token.
- `docs/g8-task-takeover.md` and the Studio README describe the 7 phases, the
  per-segment gateway checks and the in-run base approach. The fast integration
  is **hardware-verified (2026-11-21)**; the 4.0/20 profile math is the
  2026-11-20 hardware-verified values and the Studio integration adds the
  per-command bus verification settles.

## [Unreleased] — 2026-11-21 Studio G8.1 chassis approach

### Added

- **G8.1 bounded chassis approach (Studio):** a cube that is visible but
  outside the 0.220-0.255 m direct-grasp band now produces an
  `APPROACH_REQUIRED` plan with a pulse vector instead of `FIT_REJECTED`; one
  operator confirmation runs up to three bounded open-loop base pulses
  (`POST /api/chassis/pulse` -> `MotorsSpeedControl` at 20 Hz + zeros), each
  followed by a fresh cloud and a new plan. The aim point is `(0.230, 0)`, the
  middle of the verified band on the tool-down axis, so the constant
  correction's `joint1 ~ 0` envelope still holds.
- Gateway `chassis` contract (duty <= 10, wheel <= 30, 0.2-2.0 s, 4.9 mm/s per
  duty), `POST /chassis/pulse`, `verification: open_loop`, a matched-subscriber
  wait, and `chassis_enabled`/`chassis` in `/health`; the exporter copies the
  block from `configs/takeover.yaml`.
- Locate region widened to `x 0.15-0.55 m` (the whole visible floor); the
  direct-grasp band and the approach gate the reach.
- `tools/studio/g8_approach.py` (plan-only or pulse-until-aligned) and the
  Task Takeover page's approach button + confirmation modal.
- Evidence `evidence/studio_g81/20261121/`: a cube at x 0.357 m -> two pulses
  (94 mm + ~42 mm) -> target radius 0.2351 m -> 13-step task, `close_grip` raw
  412, `held: true`, landing +0.5 mm x.

### Fixed

- The adapter's `/chassis/pulse` POST timeout is 20 s (the gateway handler can
  take ~5 s: discovery + pulse + zeros) and the missing `AdapterError` import
  in the hardware adapter.
- The min-range clip recovery no longer accepts two edges: a clipped boundary
  must also be perpendicular to the camera->edge direction
  (`recovery_radial_min: 0.85`), which rejects a cube edge that happens to sit
  in the same range band.

## [Unreleased] — 2026-11-21 Studio G8 task takeover

### Added

- **G8 task takeover (Studio):** one `POST /api/task/plan` captures the
  registered point cloud on demand, segments the floor cube, applies the
  validated constant correction and the deep-family fit, and returns a
  13-step task (`open_gripper`, `approach`, `pre_grasp`, `grasp`, `close_pre`,
  `close_grip`, `lift`, `view_verify`, `grip_verify`, put-back, release,
  `return_view`). Every pose step is validated at plan time and again by the
  ordinary preview from the actual state; the gripper steps are speed-capped
  with a 20 % margin (`speed_raw_per_s: 100`, durations 1.5/2.6/4.0 s).
- **Min-range clip recovery:** the Aurora reports no valid depth nearer than
  ~0.25 m at the view pose, so a nearby cube's top face loses its near part
  and the clipped centroid is biased 8-13 mm. When exactly one side of the
  top's minimum-area rectangle lies on the min-range sphere and the opposite
  edge and both side edges are intact, the centre is rebuilt from the far edge
  with the measured 39.4 mm cube size (`task.locate.cube_size_m`); the plan
  warns `TOP_CLIPPED_RECOVERED`. The whole-top gate itself is unchanged.
- Backend `POST /api/task/plan` and `POST /api/task/verify`, the Task Takeover
  page, `tools/studio/g8_task_run.py`, and `docs/g8-task-takeover.md`.
- Evidence `evidence/studio_g8/20261121/`: a real 13-step run (grasp actual
  `504/339/930/421/499`, `close_grip` stalled at raw 416 on the cube, grip
  verify `held: true`, put-back landing +3.6 mm x / +0.2 mm y / +0.4 mm z).

### Fixed

- The manual gripper preview in the pose UI now uses 5.5 s, so the full
  2.22 rad stroke stays under the 100 raw/s validation cap.
- `close_grip` was rejected on the first hardware attempt because the
  pre-close stalled 12 raw past its command and 2.0 s implied 0.51 rad/s.

## [Unreleased] — 2026-11-21 Studio G7 pose takeover

### Added

- **G7 pose takeover (Studio):** one whole five-joint pose or one `r_joint`
  gripper command per write, behind a separate gateway instance (8766) and the
  fixed `ARMPI_TAKEOVER_ACK` phrase. A 0.5 rad/s velocity cap, an FK gate over
  21 samples of the straight joint-space interpolation (workspace box,
  measured table plane + 10 mm, low-pose yaw), and five hardware-validated
  presets. The gripper uses a measured command range (170…700 raw) and timed
  completion.
- Backend `/api/motion/presets`, `pose` details in the preview, and the new
  `POSE_JOINT_SET`, `VELOCITY_EXCEEDED`, `GRIPPER_RANGE`, `WORKSPACE`,
  `TABLE_CLEARANCE`, `LOW_POSE_YAW`, `FK_UNAVAILABLE` codes.
- Gateway motion contract `mode=pose`, per-joint velocity validation, the
  `r_joint` contract and timed completion; `motion_mode` in `/health`.
- **Bus-servo completion:** after `duration + 0.5 s` (1.2 s for the gripper) the
  gateway reads the actual registers with `GetBusServoState` and completes an
  arm command only within ±12 pulses; the gripper records the settled register.
  The read is guarded (no read below 25 counts, because a negative register
  crashes the vendor node). A cache-only completion is `Unknown`.
- `tools/studio/verify_takeover.py` (read-only, 17 checks),
  `tools/studio/g7_first_motion.py` (staged real sequence) and
  `tools/studio/start_studio.sh --takeover`.
- Frontend **Pose Takeover** panel: preset library, five-joint editor with
  per-joint velocity, auto duration, FK TCP/table clearance, gripper
  open/close, confirm modal and hold request; the command status shows the
  bus-verified actual registers and deviations.
- Evidence: `evidence/studio_g7/20261121/` — real `view_table → grasp_low →
  view_table` + gripper sequence (low pose actual `502/339/928/336/499`, max 4
  pulses; gripper settled 184/645), 17/17 no-motion checks on the final code;
  docs `g7-pose-takeover.md` and `ros2_gateway/deploy/takeover.md`.

### Fixed

- **The vendor `/controller_manager/servo_states` is a command cache, not a
  measured position.** `ServoManager.set_position` writes the commanded pulse
  into it; a cache-based completion appeared ~0.12 s after dispatch for a 2.5 s
  interpolation. The gateway now bus-verifies every arm command, the backend
  labels the continuous stream `truth=commanded`, and a cache-only `completed`
  status is rejected as `Unknown`. G5's `feedback` stream is the same cache; its
  docs now say so.
- The pose editor rounds preset values to 6 decimals and clamps to the adopted
  limit; rounding `joint4 = -1.5917`/`joint3 = -1.80117` to 4 decimals crossed
  the limit and produced a false `OUT_OF_LIMIT`.
- `Settings` no longer claims `mapping_verified` is false (stale since the
  2026-11-19 survey).

## [Unreleased] — 2026-11-21 home-return between the two trials

### Changed

- **Trial flow:** after a held grip the robot now drives the base **back to
  the start while holding the cube** (the reverse-model return validated on
  2026-11-20: 245.3 mm reversed, 11.06 mm landing against the fitted target)
  and releases it at the start. The operator takes it from there and places
  it at the same spot for the second method. A miss returns empty and leaves
  the cube at the spot.
- `paired_trial_session.Session` gained the `_before_release` hook and
  `release_anchor`: the site flow keeps the pre-grasp anchor; the free-
  placement flow measures the **home release error** against the fitted TCP.
- `random_site_session.run` now runs a locate/approach per trial, with the
  operator prompt for place / re-place; `approaches.jsonl` records one
  approach/return per trial (two per completed pair); `site_report.py`
  reports "approach records" instead of "retries".
- The operator prompts are Chinese and require no measuring.
- **Pre-grasp gap (2026-11-21 operator feedback).** The random-site runner now
  reuses the final approach measurement (same arm pose, same instant) as the
  trial's pre-grasp measurement instead of re-segmenting, and records
  `pre_grasp_gap_s` and `pre_measure_reused` in the trial notes (and prints
  the gap). Measured post-hoc: one segmentation call 1.36 s and a register
  read about 1 s, so the duplicate measurement cost about 2.4 s of the pause
  between the base arrival and the grasp. This is a post-data tooling change
  and alters no measured outcome.

### Fixed

- The home landing: a cube absent from the start, or a landing farther than
  60 mm from the fitted release target, is recorded as `NO_GRIP` (the grip
  claim was false) instead of aborting the session; the landing is measured
  with a wide region first. The first new-flow attempt (before this fix)
  aborted on a missing home landing; its approach/error is in
  `approaches.jsonl` and no trial was recorded.
- A **clipped top face** at the start (the cube at the image border) is
  accepted when the height/size gates pass; the release error is a biased
  secondary value recorded as `landing_partial`, and a rejected landing now
  prints the segmenter's real reason (the p003 lesson). This fixed the
  `p011` abort after 10 new-flow trials (5 complete pairs) had been recorded;
  the resume starts at `p011`.

- A **locate miss** with the cube visible in the colour image but outside the
  segmenter's 3-D region (wide region `x 0.15-0.60`, `|y| <= 0.40`; front box
  `x 0.212-0.565`) now prompts the operator to move it to about 0.3-0.5 m,
  centred, and retries up to 2 before aborting. This fixed the `p024` miss
  after 16 new-flow trials (8 complete pairs); the resume starts at `p024`.
- The resumed session completed `p024` (baseline `MISS`, intervention held,
  16.04 mm home release error), then reached **18 trials = 9 complete
  pairs**. At `p005` the four-pulse closed-loop approach oscillated and the
  final wrist cluster merged with the gripper fingers (y extent 59.2 mm >
  50), so the measurement was rejected three times; the trial was not
  recorded and the operator stopped the session. The aborted trial's
  approach and its single 3.0 s reverse-home pulse (`done: false`, about
  104 mm short of home) are in `approaches.jsonl`; the resume starts at
  `p005`.
- The next resumed segment ran 9 more trials: `p005`, `p021`, `p007` and
  `p016` completed (8/8 held; home release errors 2.47-12.34 mm) and `p001`
  recorded only its baseline (`MISS`). The `p001` intervention aborted
  before any grasp with `bus servo read failed`, the known serial link death
  (`device reports readiness to read but returned no data`); relaunching
  only `ros_robot_controller` in the zsh environment restored the bus
  (`init_finish`, then `503,596,833,113,500`), and the arm was re-homed to
  the view pose with the gripper open. **27 trials = 13 complete pairs plus
  a partial `p001`**; the log is `session_20261121_1350.log`, the resume
  starts at `p001` (intervention).
- The release gripper open stalled at register 421: the tool's 0.45 s
  interpolation default never reached the open register (~183) and the cube
  stayed between the fingers, so `p015` / intervention was recorded as a
  false `NO_GRIP`. That record is preserved in
  `discarded_release_stall_trial.jsonl` (D2) and excluded; `p015` restarts.
  The release now commands `--release-open-duration 3.5` (the servo tracks
  at about 100 pulses/s), `fast_motion.gripper_wait` extends its timeout to
  cover the commanded duration, and the runner records
  `release_open_register` and warns the operator above 230. The session then
  completed `p001` (intervention held, 12.41 mm), reaching **28 valid trials
  = 14 complete pairs**.

### Data

- **D1:** the first old-flow trial (`p020 / intervention`, success, 4.38 mm,
  site put-back) and its approach records are preserved in
  `discarded_site_release_attempt.jsonl` and
  `discarded_site_release_approaches.jsonl` and excluded. `p020` restarts
  under the home-return flow.
- **D2:** the false `p015` / intervention `NO_GRIP` (the release gripper open
  stalled at 421 and the cube stayed in the fingers) is preserved in
  `discarded_release_stall_trial.jsonl` and excluded; `p015` re-ran.
- **Final random-placement result (2026-11-21):** 24 pairs / 48 valid
  records, no invalid record. Binary success (primary) **baseline 9/24 =
  37.5% vs intervention 24/24 = 100%**, McNemar exact **p = 6.1e-05** (15
  discordant, all intervention-only). Secondary home release error (9 pairs
  with both): mean difference (intervention - baseline) **+2.12 mm**, 95% CI
  [-9.92, +9.32], sign 1 win / 8 losses (p = 0.039); the anchor is the fitted
  TCP of the corrected target, so it is **not** a like-for-like accuracy
  comparison and is descriptive only. Baseline failures: `MISS` = 14,
  `NO_GRIP` = 1 (`p004`: the grip did not survive the return).
  `analysis.json` / `analysis.md` and `site_report.md` are generated.

## [Unreleased] — 2026-11-21 random-placement 24-pair trial (pre-registered)

### Added

- `tools/phase3/random_region.py`: the free-placement rule and the reference
  envelope (radius about 0.25-0.55 m, bearing about -20..+25 deg; a guide,
  not a gate), and the seeded pair/method plan with fixtures `r01`...`r24`.
- `tools/phase3/random_site_session.py`: the session runner. Per pair it
  prompts for a by-eye placement (no measuring), uses the locator chain
  wrist-wide -> chassis D435i -> `joint1` yaw sweep, reports the realized
  point (its region membership is descriptive), grasps directly if it is
  already in the 0.226-0.234 m band, otherwise drives the base toward it and
  then grasps. The front-camera capture/copy/measure path and the
  `--precheck` free near/far warm-up are new.
- `ext_cam_measure.py --depth-cluster`: the far-field depth top-face cluster
  used by the front locator (30-50 mm above the support plane, 5 mm x/y
  grid, restricted to the reference envelope's bounding box so a larger
  background cluster cannot win). The colour rectification stays the default;
  the precheck frame showed an over-exposed far top that the colour thresholds
  miss.
- `docs/phase3/RANDOM_SITE_DESIGN.md` and
  `evidence/phase3/onsite_20261121_random/`: the free-placement
  pre-registration, reference envelope, plan (seed 17) and hashes, written
  before data.
- `site_report.py` renders a random site as `random` and uses the realized
  initial point for the radius/bearing columns.
- Tests: `test_random_region.py` and `test_random_site_session.py`,
  including the region check, the locator fallback order and a 48-record dry
  run.

### Changed

- `paired_trial_session.ROBOT_TOOLS` installs `ext_cam_capture.py` for the
  front-camera locator.
- `site_diversity_session.SiteSession` now has `_locate_initial`,
  `_locate_after` and `_validate_initial` hooks; the fixed-fan runner keeps
  the wide/yaw behavior and the random runner adds the front camera and the
  free-placement report. It also prints whether the robot grasped directly
  (already in the band) or moved the base first.
- The fixed 24-mark fan design, layout and plan are **superseded before
  data**; the evidence directory has no trial records.

### Status

- **Pre-registered, not run.** No free-placement trial has been performed.
  The free near/far warm-up must pass before the 48-trial session.

## [Unreleased] — 2026-11-20 documentation progress reconciliation

### Changed

- The four status documents (README, README.zh-CN, ROADMAP and the acceptance
  report) and the Phase 2 index now include the last two fast-motion runs
  (in-view 4.77 s / 4.44 s, 0.92 mm landing; far return-home 4.76 s / 4.31 s,
  11.06 mm landing) and state the validated count as **four real-cube runs at
  4.0/20**; earlier text still said two runs.
- The Studio README now states the current **G6** milestone, **93 backend + 47
  gateway** tests, the passing 2026-11-20 reboot test and the verified joint
  limits; the gateway README documents the opt-in `--enable-motion` instance
  instead of claiming no write path exists.
- Phase 2 measurement status, the calibration-pose envelope, and the G4/G5/E7
  and gateway-deployment notes that still described provisional limits or an
  untested reboot are marked current or superseded. Broken relative links in
  the G6/E7 records were fixed.
- The 2026-11-20 changelog sections are ordered newest-first.

## [Unreleased] — 2026-11-20 in-view and far-with-return fast runs

### Added

- `fast_motion_e2e.py --return-home`: after a verified grip the driver
  reverses the commanded base approach while holding the cube, re-checks the
  grip, and only then releases. New helpers `net_base_travel` (drops pulses
  with no matched subscriber) and `return_home` (same open-loop reverse model
  as the 24-site session, chunked at the 3 s vendor cap); the landing is then
  measured against the fitted target instead of the cube's own start.
  Four new tests in `test_fast_motion_e2e.py`.

### Measured (2026-11-20, two supervised real-cube runs)

- **In view, no base move:** cube at (0.2369, 0.0003) m, joints
  (0.0216, -0.6618, -1.8012, -0.2534), close register 422, grasp 4.77 s,
  release 4.44 s, landing **0.92 mm** from the cube's start
  (`e2e_cube_run2.json`).
- **Far + return home:** cube at (0.4325, -0.1674) m (0.464 m, -21 deg); two
  closed-loop pulses (294.7 mm + 32.4 mm) brought it to (0.2277, -0.0007) m;
  close register 421; grasp 4.76 s; one 2.50 s reverse pulse undid the
  245.3 mm approach with 0 clusters in the cube box afterwards; release
  4.31 s; landing **11.06 mm** from the fitted target
  (`e2e_far_return_home.json`). The return is open-loop: 2026-11-19 measured a
  32.1 mm home error that needed one correction pulse to reach 9.4 mm, so the
  home accuracy is centimetre-level and the landing figure is not a home
  error.

## [Unreleased] — 2026-11-20 progress unification (documentation only)

### Changed

- **README, README.zh-CN, ROADMAP and the acceptance report now reflect the
  full 2026-11-20 progress.** Added the chassis-front D435i (calibration,
  depth service, collision box, dual-camera grasp), fast jerk-limited motion
  (11.67 s → 8.94 s), the completed error budget (translation 13.325 mm RSS,
  bound gates fail), the pre-registered 24-site follow-up, the clock-sync
  timer and the live joint2-limit reload to the status tables, key results,
  safety notes, evidence maps and next steps.
- **Status counts corrected:** the Studio backend is now **93 passed** and the
  gateway **47 passed** (previously documented as 92/38). Two backend
  assertions that still expected a provisional mapping were updated to the
  verified 2026-11-19 mapping; no runtime behaviour changed.
- The README deliverables checklist now marks the error budget complete (with
  its bound-gate failure), the D435i and fast motion complete, and the 24-site
  follow-up explicitly unchecked (pre-registered only).
- ROADMAP next steps no longer say to complete the error budget; they now start
  with running the pre-registered 24-site trial and then address the dominant
  `gripper_placement` term, the rotation/ruler runners and the yawed grasp.

## [Unreleased] — 2026-11-20 24-site paired trial (pre-registered)

### Added

- `tools/phase3/site_layout.py`: the pre-registered 24-site fan (radii 0.30,
  0.35, 0.40, 0.45 m x bearings -10, -4, +2, +8, +14, +20 deg), the envelope
  checks and the operator markdown table.
- `tools/phase3/site_diversity_session.py`: a `paired_trial_session.Session`
  subclass that adds, per pair, the wide view-pose measurement, a `joint1`
  yaw-search fallback, the closed-loop base approach to the 0.226-0.234 m
  band, the prior-24/24 envelope guard, and the reverse-model home return.
  The grasp, grip check, landing gates and recorder are unchanged.
- `docs/phase3/SITE_DIVERSITY_DESIGN.md`: the pre-registered questions,
  procedure, capability and paired outcomes, safety, envelope rationale,
  threats, pre-session checks and analysis; no pooling with the 2026-11-19
  single-fixture trial.
- `evidence/phase3/onsite_20261120_sites/`: the layout JSON, operator table,
  seeded plan (seed 17) and hashes, written before data.
- Tests: `test_site_layout.py` and `test_site_diversity_session.py`,
  including the plan-to-site assignment, the base-return model cancellation
  and a 48-record dry run.
- `tools/phase3/site_report.py`: joins the approach sidecar and the trial
  records into the capability/outcome table (retries, locate, approach, pulse
  count, final envelope, abort reason); `test_site_report.py` covers the
  last-attempt rule and the retry count.

### Changed

- `paired_trial_session.Session.trial` takes `prompt=True` and merges an
  optional `extra_notes` hook into the record notes; the defaults preserve the
  2026-11-19 runner behavior.

### Status

- **Pre-registered, not yet run.** No 24-site trial or robot motion has been
  performed for this experiment. The dry run writes 48 synthetic records to a
  temporary directory only.

## [Unreleased] — 2026-11-20 throughput v2 (arm envelope default, faster base)

### Changed

- **Arm:** `fast_motion.py`, `move_to_pose.py` and
  `low_config_grasp_validate.py` now default to the measured envelope
  **`vmax = 4.0 rad/s`, `amax = 20 rad/s^2`** for the free moves (was 3.0/12).
- **Release path:** the holding-cube descent runs at **1.4 rad/s** (was 0.6),
  the release final approach at **0.8** (was 0.35), the release gripper open
  interpolates in **0.45 s** (was 0.8) and the grip wait is **0.5 s** (was 0.7;
  the measured stall was at 0.42 s). New CLI overrides:
  `--release-descent-vmax`, `--release-final-vmax`,
  `--release-open-duration`.
- **Base:** `base_pulse_vector.py` accepts `--speed-mm-s` (scale
  `duty = 0.182 * v`), raises the vector duty cap from 12 % to **20 %** and the
  wheel cap from 15 % to 30 % (a 45-degree diagonal at duty 20 is 28.3), waits
  for a matched subscriber instead of `sleep(1.5)`, and sends 4 zero messages
  instead of 10.
- `tools/phase2/tests/test_base_pulse.py`: 5 new tests for the duty/wheel caps,
  the speed conversion and the nominal travel.

- **Auto-base:** `fast_motion_e2e.py` drives the base in up to three
  closed-loop duty-20 pulses while the cube is outside the 0.226-0.234 m band.
  The plan is ``C - (0.230, 0)``, so one pulse removes both the radius error
  and the lateral offset and the grasp stays in the `joint1 ~ 0` family.
  `--no-auto-base` restores the manual pulses. A new wide measurement region
  (`--x-range 0.15,0.60 --y-range 0.40 --max-xy 0.12`) lets the base phase see
  a cube that the default region rejects.

### Measured (2026-11-20, two supervised real-cube runs)

- Full grasp + put-back motion **11.67 s -> 8.94 s (-23 %)**: grasp
  5.447 -> 4.699-4.729 s, release 6.216 -> 4.207-4.259 s. The release descent
  alone fell 2.677 -> 1.241 s. Every step settled in 0.14-0.16 s at 4.0/20 and
  the cube was held through both runs (0 clusters in the cube box).
- Base duty 20: a 1.786 s pulse travelled **178.9 mm (100.2 mm/s)**; a 0.484 s
  strafe travelled 45.3 mm (93.5 mm/s). Direction error 1.6-2.5 deg. The
  0.182 duty/mm-s scale is ~9 % optimistic at duty 20, so the planner keeps
  4.9 mm/s per duty and undershoots.
- Auto-base placed the cube at `(0.2316, -0.0025)` m in one pulse from
  `(0.2228, -0.0469)`, and the grasp used `joint1 = 0.0065`; the yawed
  bearing-only run used `joint1 = -0.2878` and landed 6.72 mm from the start
  vs **2.22 mm** for the aligned run.
- Evidence: `evidence/phase2/onsite_20261120/throughput_v2/`.

### Open

- The aligned auto-base was validated from one initial cube placement; other
  bearings have not been run. Fallback for a drop: `--vmax 3.0 --amax 12
  --release-descent-vmax 0.6`.

## [Unreleased] — 2026-11-20 fast, measured arm motion

### Added

- `tools/phase2/motion_profile.py`: jerk-limited joint-space S-curve (trapezoid
  with half-cosine ramps), synchronised over all joints, with exact endpoints
  and a 25 ms minimum segment (the vendor clamps shorter segments to 20 ms).
- `tools/phase2/vendor_servo_map.py`: single source for the pulse<->radian
  mapping, the flipped joint3, the EEPROM/measured hard bounds, the adopted
  bounds and the measured gripper values.
- `tools/phase2/fast_motion.py`: dense-waypoint trajectory client with a
  **measured** bus-servo settle (3 counts over 0.12 s), raw gripper commands, a
  zero-motion per-segment bench, a speed sweep and a post-move hold trace.
- `docs/phase2/FAST_MOTION.md` and
  `evidence/phase2/onsite_20261120/fast_motion/`: design, measured speed
  envelope and the raw hardware reports.
- `tools/phase3/fast_motion_e2e.py`: one supervised fast grasp + put-back
  through the validated chain (wrist measurement, `C_view + e`, deep-family
  joint fit, grip verification, landing displacement).
- 13 profile / bounds unit tests in
  `tools/phase2/tests/test_motion_profile.py`.

### Changed

- `low_config_grasp_validate.py` and `move_to_pose.py` now default to the
  S-curve path with measured settles, a fast pre-grasp descent and a slow final
  0.15 rad; the gripper opens in parallel with the approach. `--slow` restores
  the previous single-point 6-9 s moves.
- `paired_trial_session.py` deploys the three new modules with the other robot
  tools.

### Measured

- Vendor trajectory path overhead: 0.27/0.10/0.07/0.01 ms per segment at
  20/40/80/200 points over 2 s (100 Hz waypoints are free).
- Real view<->approach path: return 9 s -> **0.92 s** at the default
  `vmax = 3.0 rad/s`, `amax = 12 rad/s^2`; settle 0.05 s at every speed up to
  4.0 rad/s / 20 rad/s^2, with 1-3 counts of post-stop tail jitter (no servo
  saturation in the tested range).
- Full no-cube grasp cycle **5.43 s** vs ~37 s (6.8x): approach 1.01,
  pre-grasp 0.39, final 0.56, close 0.57 + 0.89, return 0.92, all settled.
- Gripper: ~420-460 counts/s travel, 0.1 s command queueing delay, 13-19 count
  load error (register 535 for the 550 grip command).
- Real-cube end-to-end: the cube was 0.2824 / -0.0552 m from the base, so two
  closed-loop base pulses (duty 10, 1.0 s + 0.58 s) brought it to
  0.2305 / 0.0016 m; measurement (0.2305, 0.0016, -0.0484) m, joints
  (0.0294, -0.6618, -1.8012, -0.3041), grasp **5.45 s**, close register 414
  with **0 clusters** left in the cube box, release **6.22 s**, landing error
  **1.23 mm** planar (previous dual-camera run 3.65 mm).

### Open

- `--slow` was not re-run on hardware; it is a straight port of the old code.
- The holding-cube return ran at the 3.0 rad/s default, not at the tested
  4.0 rad/s ceiling.

## [Unreleased] — 2026-11-20 dual-camera supervised grasp

### Added

- `tools/phase2/ext_cam_capture.py`: container-side capture of one D435i
  colour frame, one depth frame and both `camera_info` messages, with a
  skew/age gate (the colour stream can deliver stale frames under load).
- `tools/phase2/ext_cam_measure.py`: laptop-side, pose-independent top-face
  measurement — depth plane → top plane → rectified colour → saturated blob —
  with gate checks, a JSON report and an overlay image.
- `tools/phase2/rs_extrinsics_probe.cpp` and
  `evidence/calibration/20261120/depth_to_color_extrinsics.json`: the D435i
  depth→colour extrinsic (14.8 mm baseline) used to map depth planes into the
  calibrated `ext_cam_optical` frame.
- `evidence/phase2/onsite_20261120/dual_grasp/`: the full session — before,
  view, after-grasp and release frames, front-camera measurements, wrist
  segments, grasp/release logs and a session report.

### Measured

- Front-camera top-face measurement at the view pose: (0.230924, −0.008485,
  −0.053333) m, **2.58 mm** (x/y) from the wrist measurement and repeatable to
  0.09–0.62 mm across an arm move; its z is 5.7–7.5 mm below the wrist's
  (the documented 116 vs 121 mm depth/board height ambiguity).
- **`e = (+13.7, +3.9, +8.9) mm` is a low-configuration execution correction,
  not a view-pose measurement bias.** The two cameras agree on the cube at the
  view pose while `e` is +13.7 mm in x, so the validated target
  `C_view + e` was kept; the front camera acted as a pre-grasp veto and as the
  post-grasp/post-release verifier.
- Dual-camera grasp **PASS**: wrist floor check 0 clusters, no cube-sized blob
  in the front view, and the put-back returned the cube **3.65 mm** planar
  (front estimate; wrist estimate 5.22 mm).

## [Unreleased] — 2026-11-20 phase-3 runners: rotation tolerance and ruler displacement

### Added

- `tools/phase3/rotation_tolerance_run.py`: commands the relative yaw between
  the gripper opening axis and the cube on `joint5` instead of rotating the
  object by hand; the tool point is invariant, so the validated deep family and
  constant correction stay unchanged. It compensates the cube's own placement
  yaw from the view measurement, records commanded and measured (bus-servo)
  `joint5`, repeats yaw in seeded blocks and uses the envelope session's
  cube-box grip classifier. `low_config_grasp_validate.py` gained `--yaw` for
  the four-joint approach/grasp/release moves; 6 unit tests.
- `tools/phase3/visual_displacement_run.py`: slides a 60 mm 36h11 tag through a
  seeded two-axis grid of known steel-ruler offsets at the fixed view pose; the
  arm does not move. It fits the ruler frame's unknown yaw as a nuisance
  parameter and reports raw and de-rotated displacement residuals; 5 unit tests.

### Status

- Both runners are implemented and dry-run/tested, but **no hardware data has
  been collected**. The `visual_displacement` protocol is therefore still
  unmeasured, and the 2° rotation tolerance remains provisional.

## [Unreleased] — 2026-11-20 off-board D435i: mount, calibration, depth and collision

### Added

- Chassis-front Intel RealSense D435i on the USB 3.0 bus (colour UVC
  1280×720; depth 640×480 Z16 through librealsense 2.55.1). The camera is rigid
  to `base_link`, not a vendor device.
- `tools/phase2/uvc_chessboard_intrinsics.py`, `front_camera_extrinsic.py` and
  `ext_cam_frames.py`: refined intrinsics from a flat plus propped-board sweep
  (**fx = fy 903.5 px**, cx/cy 644.9/363.7, **RMS 0.371 px**), the chassis
  extrinsic (**[0.136, 0.027, 0.038] m**, **31.6°** pitch, −1.1° yaw), a
  latched `camera_info` and the `base_link → ext_cam_optical` static TF.
  12 synthetic regression tests.
- `tools/phase2/ext_cam_node.py` and `armpi-ext-cam.service`: the single owner
  of `/dev/video4`, publishing `/ext_cam/color/image_raw` at 1280×720/15 fps
  with latched `camera_info` and TF. The gateway gained `RosImageCamera` and
  `--aux-camera-topic`, so the Studio preview follows the ROS topic instead of
  opening the device.
- `tools/phase2/ext_cam_collision.py`: a 90×25×26 mm planning-scene box at
  `base_link` (0.145, 0.027, 0.032) with the calibrated orientation. The five
  validated operating poses stay collision-free; a 0.3 m padded control
  collides against `link1`–`link3`, proving the box is in the checker.
- `tools/phase2/patches/librealsense-2.55.1-d400-hid-gate.patch` and
  `armpi-rs-depth.service`: depth through `realsense2_camera`, ordered after
  the vendor bringup, before the colour unit, with an IR wake-up and a
  SIGKILL-escalating stop.
- `docs/hardware/camera_mount.md` section covering the mount, frames,
  calibration, depth limits and services.

### Measured

- Depth **61.5 % valid**, median 0.312 m, **10.6 Hz** depth-only and 4–9 Hz
  with colour; IMU unavailable because the kernel has no
  `CONFIG_HID_SENSOR_HUB`.
- Reboot test: `NRestarts=0`, wake-up on attempt 1, node up 45 s after boot,
  depth and colour streaming; `base_link → ext_cam_optical` survives the reboot.
- Dual-camera grasp: **2.58 mm** x/y agreement with the wrist camera; the
  validated `C_view + e` target was kept because `e` is an execution offset,
  not a measurement bias; grip and put-back verified (3.65 mm front estimate).

### Changed

- The Studio aux feed moved from a gateway-owned UVC device to the
  **single-owner ROS topic** (`RosImageCamera`, `--aux-camera-topic`); the
  gateway no longer opens `/dev/video4`. This supersedes the UVC ownership
  described in the Studio auxiliary camera entry below.

### Open

- IMU cannot work on this kernel; depth starts at ~0.2 m, so the grasp band is
  colour-only; the collision box protects MoveIt plans only; the camera has a
  single owner and must stay that way.

## [Unreleased] — 2026-11-20 Studio auxiliary camera

### Added

- **The Studio Dashboard shows the off-arm RealSense D435i beside the main
  Aurora camera.** The read-only gateway captures the D435i color stream
  directly over UVC (`/dev/video4`, 640×480 YUYV, ~9 fps) without a RealSense
  ROS driver, and serves `/aux-camera/status` + `/aux-camera/snapshot`. The
  backend adds `aux_camera` to `/api/peripherals` and proxies
  `/api/peripherals/aux-camera/snapshot`; the Dashboard lays the two feeds side
  by side with the battery card below them on narrower windows. Mock mode
  renders a labelled SVG placeholder. Depth and IR are not captured. Both feeds
  verified live on the robot 2026-11-20 (aux `measured`, 640×480, ~9 fps).
- **Superseded later the same day:** the single-owner ROS driver
  (`ext_cam_node.py`, `armpi-ext-cam.service`) took `/dev/video4`; the gateway
  now follows `/ext_cam/color/image_raw` via `RosImageCamera` and
  `--aux-camera-topic` instead of opening the UVC device. See the D435i entry
  above.

## [Unreleased] — 2026-11-20 execution, placement and fixture measurements

### Added

- `tools/phase3/execution_fixture_run.py`: supervised runner for the three
  remaining error-budget sources. `fixture` detects each stable operator
  placement at the fixed view pose (removal or >3 mm displacement) and records
  its spread; `trials` runs pick–lift–put-back grasps and records the
  commanded-vs-bus-servo execution error at the grasp pose plus the segmented
  landing displacement. Both commands are resumable and write JSON/Markdown
  reports. 13 unit tests in `tools/phase3/tests/test_execution_fixture.py`.
- `error_budget_onsite_20261120.json` and
  `error_budget_fixture_20261120.json`: completed pipeline and
  fixture-inclusive budgets.
- `evidence/phase3/onsite_20261120/`: 20 fixture placements, 20 put-backs,
  raw/clean/discarded records, reports, budget reports and a session README.

### Measured

- **Grasp execution**, 20 grasps: 4.220 mm / 1.069° mean, **5.386 mm /
  1.495° max** (commanded vs bus-servo registers at the grasp pose).
- **Gripper placement**, 20 put-backs: **3.080 mm mean, RMS 4.082 mm,
  11.514 mm max**; mean xy bias (−0.99, +1.63) mm. One tail event shifted the
  cube 11.0 mm in −x while still holding and landing validly.
- **Object-fixture pose**, 20 tape placements: **5.057 mm radial RMS /
  14.815 mm radial max**; per-axis SD 5.112 / 0.884 / 0.362 mm. Three off-tape
  placements were excluded and kept as discarded evidence.
- **Completed bound budget**: pipeline translation **RSS 13.325 mm / worst
  24.014 mm** (fail vs 6 mm), rotation **RSS 1.858° (pass) / worst 3.322°
  (fail)**; fixture-inclusive **19.926 mm RSS**. The dominant translation term
  is `gripper_placement` (74.7 %), the dominant rotation term is
  `grasp_execution_rotation` (64.7 %). The empirical corrected placement mean
  (3.080 mm) is inside 6 mm; the tail is not.
- Deviations kept: initial x = 0.262 m placement refused (23.4 mm z
  residual); three off-tape fixture placements; two clipped landings (trial 15
  and trial 20) re-run after recentring; one 11.5 mm placement tail; four
  operator recentring events caused by the put-back drift.

## [Unreleased] — 2026-11-20 operations: clock sync, live limit reload, reboot test

### Added

- `tools/net/armpi-clock-sync.sh` and `install-clock-sync.sh`: a laptop-pushed
  systemd user timer every 60 s keeps the Pi ~2 s ahead of the workstation.
  The dedicated key is passphrase-free but restricted on the robot to the
  forced command `/usr/local/sbin/armpi-clock-set` (`get` or `set <epoch>`); it
  cannot open a shell. Documented in `docs/hardware/ros_environment.md`.

### Fixed

- The live stack still held joint2's pre-survey **1.2985 rad** bound after the
  2026-11-19 file edit. Rebuilt the description with `--symlink-install` and
  restarted `move_group`, `armpi_safety` and the Studio backend; joint2 = 1.2 is
  now rejected with `JOINT_LIMIT`, and the headers/current-state docs match
  1.1519 rad / 775 pulse. 17 planner + 23 safety tests passed.

### Verified

- Clock sync: forcing the Pi 301 s behind restored it to laptop + 2 s; the
  timer runs; the restricted key opens no shell.
- Reboot test (table clear): SSH answered in 25 s, the container and both
  camera units came up, the vendor Aurora launch finished ~2.5 min after boot,
  both camera snapshots returned 200, the calibrated D435i TF survived, and
  the clock sat 0.9 s ahead.

## [Unreleased] — 2026-11-20 documentation unification

### Changed (documentation only, no new hardware claims)

- **README, README.zh-CN, ROADMAP and the acceptance report now share one
  status.** The four documents were aligned to the 2026-11-19 hardware
  evidence: formal five-joint survey with `mapping_verified=true` for the
  operating envelope; 24-pair trial 12/24 → 24/24; repeatability 0.449 mm RMS,
  backlash 1.117 mm and a ±9 mm tolerance envelope (conservative 6 mm); the
  error budget still partial in grasp execution, gripper placement and fixture
  pose (measured later the same day; see the entry above).
- Removed the duplicate error-budget row in both READMEs and made the
  tolerance unambiguous: the **RSS 3.995 mm passes the measured 6 mm
  tolerance; the 7.114 mm worst case does not**; the 2° rotation gate remains
  provisional. The 2026-11-19 budget entry now matches the measured 6 mm.
- Replaced the stale next step ("replace the correction with a joint-offset
  calibration") in both READMEs and the roadmap. Joint offsets and `link4`
  models were already tested and rejected; the open test is a **yawed grasp at
  `j1 ≈ 0.3–0.6 rad` with the cube rotated to face the gripper**, because the
  two failed yawed attempts presented the cube's ~50 mm diagonal against a
  ~40 mm opening.
- ROADMAP and the acceptance report no longer say the limits are partially
  measured or that no paired trial exists; the roadmap no longer describes the
  5 mm gate as current. Their Milestone D/E, safety and gap tables now match
  the README.

## [Unreleased] — 2026-11-19 autonomous pick-lift-place of the floor cube

### Verified (real hardware, 2026-11-19)

- **Formal joint-limit survey on all five arm joints; `mapping_verified=true`.**
  One joint at a time with servo torque released by hand, in the vendor
  vertical pose (all joints at 0 rad), plus joint2 at the low operating pose:
  joint1 **58…957**, joint2 **186…873** upright / **327…795 hand at the low
  pose**, joint3 **100…923**, joint4 **130…882**, joint5 **122…869** pulse;
  repeat spreads 0…2 pulses. The result is that the "mechanical stops" are
  **self-collision-limited and pose-dependent** (joint2's high stop 795 vs
  873; joint3's low stop 939 vs 923), so the static limits are verified for
  the operating envelope, not as universal stops. The adopted high bound of
  joint2 was **tightened 810 → 775 pulse (1.2985 → 1.1519 rad)** in the URDF,
  MoveIt, safety and Studio configs; no operation uses that region. One
  crash occurred during the first joint5 attempt (wrist fell, negative
  register); the node alone was restarted and torque restored, and the second
  attempt succeeded. Evidence:
  [`evidence/phase2/onsite_20261119/joint_limits/`](evidence/phase2/onsite_20261119/joint_limits/README.md).
- **Gripper tolerance envelope measured: the physical tolerance is ±9 mm, not
  the provisional 5 mm.** 46 grasps, two per point, offsetting the commanded
  target along one base axis at a time: **`x`** (finger length) passes at
  every offset to **±9 mm**; **`z`** (height) passes at **±9 mm**; **`y`**
  (opening) passes over **[-6, +12] mm** with 1/2 at −9 and 0/2 at −12, ±15.
  The usable `y` interval is ±9 mm about a centre **3 mm to the +y side**,
  which is the constant correction's own `y` residual (true ≈ +7 mm, constant
  +3.9), not a gripper asymmetry. The 5 mm gate is replaced by a measured
  6 mm conservative tolerance; the budget's RSS (3.995 mm) fits and the worst
  case (7.114 mm) does not. Ten releases landed too tilted to measure and are
  recorded as `landing_unmeasured`; grip outcomes are unaffected. Evidence:
  [`evidence/phase3/onsite_20261119/envelope/`](evidence/phase3/onsite_20261119/envelope/).
  New runner `tools/phase3/gripper_envelope_run.py` (4 tests).
- **Yaw extension of the low-configuration envelope.** The constant correction
  had only been validated at `j1 ≈ 0`; a fixed-tag measurement at a yawed grasp
  pose (`j1 = 0.487 rad = 27.9°`, `joint4 = -0.339`) gives a required
  correction of **(9.14, 7.18, 8.75) mm** (three repeats, spread 1.54 / 0.13 /
  0.80 mm). The constant `(13.7, 3.9, 8.9)` is **5.62 mm** away, with the
  grip-critical `y` component **3.28 mm** (inside the ±4 mm margin); the fitted
  `joint_offsets4`, `link4_translation` and `link4_se3` models are **14.4, 17.6
  and 19.2 mm** away and flip the `y` sign. The correction does rotate slightly
  with yaw (direction 38° vs the constant's 16°), but far less than the fitted
  models require. Two yawed *grasps* also failed for a separate, recorded
  reason: a square cube at a 20–30° bearing presents its ~50 mm diagonal,
  beyond the ~40 mm opening, so future yawed grasps must rotate the cube to
  face the arm. Evidence:
  [`evidence/phase3/onsite_20261119/yaw_envelope/`](evidence/phase3/onsite_20261119/yaw_envelope/).
- **Error-budget motion terms measured: repeatability and backlash.** A 60 mm
  36h11 tag fixed on the floor was observed from the grasp pose
  `[0, −0.6618, −1.8012, −0.3917, 0]`; the tag centre was reconstructed from
  the measured bus-servo joints and the measured hand–eye. **30 visits**:
  radial **0.449 mm RMS**, **1.017 mm max**; per-axis std 0.278 / 0.211 /
  0.295 mm; the joints repeat to **0–1 pulse**; tag rotation max **0.867°**
  (mean 0.446°). **Backlash**, 15 forward + 15 reverse visits from
  `joint4 ± 0.1 rad`: hysteresis **1.117 mm** (vector −0.971, −0.064,
  −0.549 mm); `joint4` settles 10 pulses apart. All 45 captures detected the
  tag (PnP RMS 0.40 px mean, frame margin 45–55 px). The budget now reads
  translation **RSS 3.995 mm** (pass) / worst case **7.114 mm** (fail vs the
  measured 6 mm tolerance) and rotation **RSS 1.103° / worst 1.827°** (both
  pass); the hand–eye term is still 68 % of the translation RSS, grasp
  execution, gripper placement and fixture pose remain unmeasured. New tool
  `tools/phase3/measurement_run.py`; config
  `config/error_budget_onsite_20261119.json`. Evidence:
  [`evidence/phase3/onsite_20261119/`](evidence/phase3/onsite_20261119/README.md).
- **First physical paired trial: 24 pairs, 48 trials.** Pre-registered design
  and a committed 24-pair randomization plan (seed 7) compared the raw
  FK/hand–eye target (`baseline`) with the constant low-configuration
  correction `(+13.7, +3.9, +8.9) mm` (`intervention`). **Baseline 12/24
  (50.0 %) vs intervention 24/24 (100 %)**; all 12 discordant pairs favour the
  intervention, exact McNemar **p = 0.00049**. Among the 12 pairs where both
  methods gripped, the intervention places the cube **2.19 mm** closer to the
  pre-grasp measurement (95 % CI **[−3.96, −0.83] mm**, Cohen's `dz = −0.75`,
  sign test 11 wins / 1 loss, **p = 0.0063**). Placement error: **3.31 mm**
  (SD 1.48) vs **4.97 mm** (SD 2.87). Four protocol deviations were recorded
  and handled: a runner unit error (D1), a grip-classifier artefact from a
  background cluster (D2), a contaminated placement with a tape roll in the
  grasp region plus a negative-argument crash (D3), and a landing measurement
  that selected a 21.7 mm object (D4). Discarded records are kept in the
  evidence bundle. Evidence:
  [`evidence/phase3/onsite_20261119/`](evidence/phase3/onsite_20261119/README.md).
- New tools for the trial: `tools/phase2/grasp_joint_fit.py` (offline
  deep-family `joint1`/`joint4` fit, reproduces the recorded validation joints
  to 2 mrad without MoveIt), `tools/phase2/bus_pulses.py` (read-only register
  reader), `tools/phase2/move_to_pose.py` (container single-pose move) and
  `tools/phase3/paired_trial_session.py` (resumable session runner with
  `--dry-run`, target/fit guards, a cube-box grip classifier and placement
  gates). 14 session tests and 5 joint-fit tests are in the pure-Python CI.
- **Low-configuration model selection: joint offsets rejected, constant offset
  supported.** The roadmap's planned "joint-offset calibration over the low
  configuration" was tested against the 10-pose tag survey. A fixed `link4`
  translation (3 parameters), a fixed `link4` 6-DoF transform (6) and joint
  offsets `dq1..dq4` (4) all fit the tag data to **1.4–2.3 mm RMS**, so the
  survey alone cannot choose between them; `dq1` is unidentifiable (a flat
  direction from 0.02 to 0.13 rad) and the `link4` translation is dominated by
  a 44 mm lumped `z` term. The four passing 2026-11-19 grasps discriminate:
  evaluated at the runs' actual measured view and grasp joints (reconstructed
  from the bus-servo registers, exact on the survey's own samples), every
  configuration-dependent model predicts a correction **10.6–15.0 mm away**
  from the constant `(+13.7, +3.9, +8.9) mm` they used — the validation run
  landed **1.3 mm** from its measurement. The **constant base-frame offset is
  therefore the supported model**, bounded to tool-down grasps with
  `j1 ±0.06 rad` and the view pose at `j4 −1.58…−1.66`; a yawed grasp
  (`j1 ≈ 0.3–0.6 rad`, where the two model classes differ by 12–16 mm) remains
  the open extension test. The fixed-tag yaw survey already measured the
  correction at `j1 = 0.487 rad`; the two yawed grasp attempts failed on
  cube-vs-finger geometry (the cube presents its ~50 mm diagonal to a ~40 mm
  opening), not on the correction. Evidence:
  [`evidence/calibration/20261119/low_config_error/model_selection.md`](evidence/calibration/20261119/low_config_error/model_selection.md).
- New tools: `tools/phase2/low_config_model_fit.py` (numpy-only profile fit,
  LOO, AIC and implied correction) and `tools/phase2/low_config_correction.py`
  (model correction for a given view/grasp pair), with five synthetic tests in
  the pure-Python CI list.
- **Out-of-view search, one base approach, grasp and return.** The cube was
  outside the frame at the tilted view; a **+16° `joint1` yaw at the raised
  pose** found it at **0.469 m** (bearing +38.8°), 0.22 m beyond the 0.2447 m
  reach boundary. Three closed-loop diagonal base pulses moved the cube to
  (0.2284, −0.0010) m; the validated deep family gripped it and a 150 mm lift
  held it (0 clusters above the floor). The base was reversed in one 6.3 s
  pulse; a fixed floor landmark measured the return error (32.1 mm) and one
  0.75 s correction pulse brought it to **9.4 mm**. The cube was released
  1.1 mm from its pre-grasp position in the robot frame, at ≈(0.2115,
  0.0278) m in the original base frame. Evidence:
  [`evidence/phase2/onsite_20261119/search_and_return/`](evidence/phase2/onsite_20261119/search_and_return/).
- One attempt, no operator: vision-measured a 39.4 mm wooden cube (top face
  41.0 × 37.7 mm, bounds 38.7 × 40.0 mm, centre (0.2265, 0.0009) m), grasped
  it with `joint5 = 0`, lifted the TCP from −0.0375 to +0.1097 m
  (**≈149 mm** above the floor), lowered back and released. All 10 arm
  trajectories and 3 gripper goals returned `status=4`; safety approved each
  arm plan.
- **`joint5` must be 0 at the table-down view.** With the half-open gripper at
  `joint5 = 1.383` its fingers cross the camera→cube ray: the segmented top
  face shrinks to 35.0 × 37.7 mm and the per-slice centroid moves by up to
  5 mm. `joint5` is a pure roll about the TCP (the TCP is invariant), so it can
  be set freely; 0 also aligns the finger axis to 2.3° of the cube face.
- The same-orientation IK for the lift fails (`/compute_ik` code −31); the
  150 mm lift needs a joint-space target (j2 0.15, j3 −1.40, j4 −1.20 at
  x 0.238). The floor reach boundary still caps the grasp TCP 5.9 mm short of
  the cube centre (along the finger length, not across the opening).
- Absolute base-frame estimates drift ~10 mm between captures because the real
  arm pitch differs from FK by ~2.5° (sag/backlash): the same-transform
  re-segmentation after placing the cube back moved the fitted floor plane
  10 mm. Cube-relative numbers (height above the floor, size, yaw) are stable.
- Evidence: [`evidence/phase2/onsite_20261119/grasp_150mm/`](evidence/phase2/onsite_20261119/grasp_150mm/).
- Second pass after operator feedback (same day): start at the **adopted-low
  forward pose** (`joint1 0, joint2 −0.6618, joint3 −1.8012, joint4 −0.6704,
  joint5 0`, measured 499/343/929/339/499) and keep `joint5 = 0` for the whole
  attempt (no wrist roll, so the fingers never cross the lens). The one-step
  150 mm lift (`+149.9 mm` TCP, 8.3 s, one plan/execute) replaced the three
  motions of the first pass; the cube was placed back within **6.6 mm and
  ≈8°**, verified in the same view frame (floor plane identical to 0.1 mm).
  The depth camera is **blind at the low pose** (72 mm), so the target still
  comes from the table-down view. Evidence:
  [`evidence/phase2/onsite_20261119/grasp_150mm_attempt2/`](evidence/phase2/onsite_20261119/grasp_150mm_attempt2/).
- Third pass: adopted is the start/finish pose, and the object-centring gate was
  **measured to be incompatible** with the hardware — the depth camera's near
  cutoff is 0.176 m and the link4 mount only centres the cube at ≤0.10 m
  (blind), while the FK/hand-eye reads the same cube 20–50 mm apart at those
  poses. The reach planner's flat `joint3 ≈ −1.6` solution closed the pads above
  the cube (miss, cube pushed away), so the retry used the **proven deep family**
  (`joint3 = −1.8012`, `joint4 = −0.410`): TCP aligned to 0.5 mm / 0.0 mm in
  x/y, 3.3 mm below the top. **The round FAILED**: the operator confirms the
  cube was never held. The in-run "grip" evidence — a 7.6 px centroid shift at
  a 15 mm lift and wood at the bottom of the 150 mm image — was a projection
  artefact: the camera rotates during the diagnostic, and the FK/hand-eye is
  50–90 px off in this joint region, so the wood was the cube still on the
  floor. A held cube sits ~0.10 m from the camera (bottom edge and inside the
  depth cutoff), so the grip check must instead put the **floor spot** in view.
  Evidence:
  [`evidence/phase2/onsite_20261119/grasp_150mm_attempt3/`](evidence/phase2/onsite_20261119/grasp_150mm_attempt3/).
- Fourth pass (only "in view", adjust the arm) also failed and measured the
  cause directly: with `joint5 = 0` the TCP is a **fixed image pixel
  (364.8, 344.2)** (verified at four arm poses), and at the low configuration
  the cube sits **7–10 mm (50–90 px)** away from it while the same model is
  0.2 px accurate at the table-down view. A joint1 probe matched the model
  Jacobian (340 px/rad measured vs 337), so the error is the absolute
  FK/hand-eye at the low configuration, not the servo response; the gripper
  has only ±4.3 mm of margin. Evidence:
  [`evidence/phase2/onsite_20261119/grasp_150mm_attempt4/`](evidence/phase2/onsite_20261119/grasp_150mm_attempt4/).
- Fifth pass **PASSED** with a one-point low-configuration correction measured
  by a 60 mm AprilTag (36h11 ID 0) on the floor: the tag reads
  (0.2373, 0.0202, −0.0788) at the validated view (on the floor) and
  (0.2293, 0.0049, −0.0846) at the grasp configuration, so
  **e = (−8.0, −15.3, −5.8) mm**. Grasping at `C_view + e` held the cube (the
  view-pose check found no floor cluster), a one-step lift reached
  TCP +0.0993 (object ≈148 mm), and the cube was released and the arm returned
  to adopted. The tag size matters: an early 73 mm estimate put the tag 59 mm
  below the floor. Evidence:
  [`evidence/phase2/onsite_20261119/grasp_150mm_attempt5/`](evidence/phase2/onsite_20261119/grasp_150mm_attempt5/).

## [Unreleased] — 2026-11-18 depth table plane, supervised approach, link/DDS fixes

### Security

- **The robot is now inbound-locked to the direct Ethernet cable.** A
  default-deny `inet lockdown` table accepts only `eth0` (the point-to-point
  link to the laptop); `wlan0` is outbound-only except DHCP replies. SSH
  password authentication is off, `wayvnc` and the vendor `remote.service`
  (unauthenticated TCP 9026) are disabled, and `armpi-gateway` now binds
  `10.42.0.218:8765`. Tailscale was already absent. The nftables service is
  enabled at boot, with a drop-in so `systemctl stop nftables` no longer
  flushes docker's NAT table. Studio and the Humble MoveIt container were
  repointed to the wired link; `tools/humble_moveit/cyclonedds-laptop.xml`
  now binds `enx00e04ca373c7`. Install/rollback/verification:
  `docs/hardware/network_access.md`.

### Fixed

- **The gripper open stop was wrong; the range is 2.22 rad, not 0.8378.** The
  2026-11-18 measurement recorded `r_joint = -0.8378` (raw 500, bus-servo
  register 502) as fully open, but the servo was binding there: `open` only
  produced a partial finger opening. With the linkage freed by hand (torque
  released on servo ID 1 only, `enable_torque=[1, 1]`), a staircase sweep
  found the real linkage stop at register **167** (raw ~150) and the closed
  stop at register **640** (raw 700). Adopted open is raw **170** /
  register 188 / `r_joint = -2.22`, i.e. 21 counts inside the stop; the
  command↔register ratio is ≈0.86 with ~34 counts of backlash. Updated the
  `r_joint` limit (URDF `-2.22`, `joint_limits.yaml`, SRDF `open`, simulation
  initial position), widened the five mimic joints from ±1.57 to ±2.22, fixed
  the grasp planner's stale `GRIPPER_OPEN_POSITION = 1.2` (outside every
  range) to `-2.20`, and corrected the hardware docs. New instrument:
  `tools/phase2/gripper_stop_survey.py`. Evidence:
  `evidence/phase2/onsite_20261118/gripper_stops_20261118/`. Verified after
  the 20:05 MoveIt rebuild: plan-only `open` reaches `r_joint = -2.210130`.
  Also found while verifying: a killed
  `real_moveit.launch.py` can leave its `move_group` orphaned (a second
  instance from 19:14 was still serving `/plan_kinematic_path` with the old
  limits; it was terminated), and **`armpi_safety` rejects gripper-only
  trajectories** (`code=WORKSPACE`, "FK failed: joint1"), so
  `real_moveit_execute.py --execute` cannot run a MoveIt gripper move until
  the safety node skips the arm FK checks for `r_joint`-only trajectories.
- **A `docker exec` shell on the robot needs the RMW too.** Setting
  `CYCLONEDDS_URI` alone is not enough: without
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` the shell runs Fast DDS, discovers
  the laptop's `move_group` nodes, and then times out on every service call
  (`/plan_kinematic_path`, `/armpi_safety/validate_trajectory`). Also
  documented: the vendor `ServosPosition` path is a serial interpolation
  command, so publishing at 20 Hz re-baselines each move and creeps.

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
- `evidence/phase2/onsite_20261118/first_grasp/` — **the first real markerless
  grasp** of the 39.4 mm floor cube: cloud segmentation (`39.2 × 39.8 mm`
  cluster, `1.9/−4.1/−3.5 mm` vs the RGB), the reach limit (`TCP` at the cube
  top only at `x = 0.233 m` with `joint2/joint3` on the adopted bounds and
  `--joint-limit-margin 0.0`), the "look-then-grasp" view pose, the autonomous
  placement miss, the DDS data-plane incident and the wired fallback, the
  operator-placed retry that **held and lifted** the cube, and the release jam
  that needed a gripper servo torque release.
- `tools/phase2/object_segment_rgbd.py` — markerless RGB-D object segmentation:
  RANSAC support plane, 5 mm voxel-grid connected components, size/height/region
  gates, top-slab fit (centroid, SVD tilt, minimum-area rectangle). Pure numpy,
  container-side, subscribes only. A partial top view biases the centroid; the
  extent is reported so a full-view gate can reject it.
- `tools/phase2/tests/test_object_segment.py` — 8 tests for the clustering,
  minimum-area rectangle, slab fit and the gate rejections.
- `tools/phase2/real_moveit_execute.py --joints` — plan/execute to an explicit
  five-joint target, used for the metrology view pose (the SRDF has no named
  state for it).
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
- `docs/hardware/patches/controller_mecanum_hardened.patch` and
  `docs/hardware/vendor_baseline.md` item 5 — the deployed chassis node is a
  hardened fork of the vendor `mecanum.py`: 0.5 s `/cmd_vel` watchdog at
  20 Hz, zero duty on timeout and shutdown, and the measured m/s → duty map
  (`duty = v × 182`, cap 60) replacing the vendor wheel-rps conversion
  (vendor `9c54a5d5…` → hardened `6881ae4d…`).
- `tools/phase2/rgbd_snapshot.py` — same-stamp RGB + registered cloud + raw
  depth/IR + joints snapshot for the Humble container (numpy only; no cv2).
- `tools/phase2/rgbd_board_check.py` and `rgbd_tag_check.py` — offline
  same-frame RGB-D plane comparisons for a chessboard and a matte AprilTag.
- `evidence/phase2/onsite_20261118/rgbd_same_surface_20261118.md` and its
  reports — closes the board-thickness leftover.

### Changed

- **`base_link` collision geometry now uses the real base mesh.** The 2025-07
  vendor model had a 1 m × 1 m × 25 mm floor plate centred on `base_link`
  (z = −12.5…+12.5 mm), i.e. **65–90 mm above the measured floor**; it blocked
  every grasp whose fingers or wrist crossed z = 0. Replaced with the newer
  vendor model's `base_link.STL` (0.289 × 0.177 × 0.076 m). The measured floor
  is still published separately by `tools/phase2/table_collision.py`.
- **DDS moved from the office WiFi to the wired link.** The WiFi path
  (`wlp4s0 10.51.128.246 ↔ wlan0 10.51.128.226`) reached 100 % packet loss
  while both nodes stayed up and DDS discovery survived, so no robot data
  reached the laptop. Both `cyclonedds.xml` files now bind
  `enx00e04ca373c7` (10.42.0.1) and `eth0` (10.42.0.218); the WiFi versions are
  kept as `*.wifi.bak`.
- **Vendor stack restart re-initialises the servos.** Restarting
  `start_node.service` moved the arm from the folded grasp pose to the vendor
  init pose `700/500/500/500/500` on its own. The restart must be planned at a
  safe pose; this is now recorded in the first-grasp evidence.
- **Chassis motion is calibrated end to end.** After the operator adjusted the
  wheel actuators, the full sign table was re-verified on the floor (forward
  `[+,−,−,+]`, left `[+,+,+,+]`, CCW yaw `[+,−,+,−]`, reverse = negation) and
  the two-point scale reconfirmed (10 % → 55 mm/s, 20 % → 110 mm/s,
  `duty = v × 182`). The hardened `mecanum.py` now maps m/s to duty (cap 60)
  instead of wheel rps; `/cmd_vel ±0.05 m/s` publishes `±9.1` duty and moves
  the base ≈55 mm forward/back. Record:
  `evidence/phase2/onsite_20261118/chassis_motor_channels_20261118.md`.
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
