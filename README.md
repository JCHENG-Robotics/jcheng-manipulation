# JCHENG Manipulation: A ROS 2-Based Robotic Manipulation System with Quantitative Reliability Evaluation

**English** | [简体中文](README.zh-CN.md)

JCHENG Manipulation is a personal robotics project around the **ArmPi Ultra** 5-DoF arm: corrected model
and TF, constrained motion planning, calibration, real visual grasping, and a
quantitative error budget.

The vendor stack is treated as a **read-only baseline**. This repository contains
project-specific code, models, experiments and evidence; it does **not** copy
the 16 GB vendor tree.

> Historical note: the repository started as an 8-week ROS 2/Panda learning
> project. That record is preserved at
> [`docs/archive/README_8WEEK.md`](docs/archive/README_8WEEK.md) and
> [`docs/weekly-log/`](docs/weekly-log/). The old roadmap is at
> [`docs/archive/ROADMAP_8WEEK.md`](docs/archive/ROADMAP_8WEEK.md).

## Status (2026-11-17)

Status convention: **Implemented / Validated / In Progress / Planned**. Vendor
functionality is marked separately. `Validated` means there is a command, log or
data file in `evidence/`; it does **not** mean hardware validation.

| Area | Status | Evidence / limit |
|---|---|---|
| Panda learning demo | Implemented, simulation-validated | `panda-baseline-v1` tag; `docs/archive/README_8WEEK.md`; weekly logs |
| ArmPi URDF / TF | Implemented; model + live TF verified | `xacro` + `check_urdf`; live `base_link → rgb_camera_link` / `depth_camera_link`; **limits provisional** |
| ArmPi MoveIt 2 | Implemented, mock-validated; real `reset_servo` verified | KDL IK; joint-space plan `error_code=1`; 6D-constrained Pilz `PTP` (mock); Humble Docker MoveIt with one supervised real motion |
| ArmPi constrained grasp | Implemented, mock-validated | 5-DoF grasp search: 80 joint-space candidates → 40 IK OK → 26 collision-free → 3 Pilz `PTP` plans; plan-only |
| Fiducial target localization | Implemented, synthetic + real-background semi-synthetic validated | AprilTag 36h11; 5/5 synthetic poses (max 1.82 mm / 1.49°); 15/15 semi-synthetic on real frames (max 1.69 mm / 0.53°); 0 false detections on 8 real frames |
| Camera frame contract | Implemented, validated | `camera_optical_calibrated` = measured body→optical adapter (89.798°, 5.4/10.1/−27.4 mm); model `camera_optical_frame` stays mock-only |
| Phase 0 acceptance | **Complete (mock scope)** | [`docs/phase0/PHASE0_ACCEPTANCE.md`](docs/phase0/PHASE0_ACCEPTANCE.md) |
| Real ArmPi hardware | **Baseline captured, partially validated** | live Pi/ROS/servo/TF/camera baseline 2026-11-17; joint limits and IMU open |
| Camera / calibration | **Camera + validated hand-eye + selected pose set** | Aurora 930 RGB/depth live; RGB ~12 Hz cross-host (in-container subscribers undercount to ~8 Hz); 14-pose chessboard hand-eye with measured bus-servo joints: 2.94 mm / 0.39° board scatter, held-out 3.29 mm / 0.43°, leave-one-out 3.41 mm / 0.44° std; independent tag check 100.67 mm vs 100.0 mm lateral (0.67 %) and 104.87 mm vs 100.0 mm table-forward (4.87 %); table-down absolute poses remain outside the valid FK range; measured optical frame published; 24 diverse poses selected offline from 14,176 evaluated poses (not executed) |
| Visual grasping | In progress — mock chain validated | perception + grasp search + validated camera calibration and frame; fiducial → grasp bridge and safety veto pass a synthetic mock E2E; no real grasp yet |
| Safety layer | Implemented, mock-validated | state watchdog, joint/velocity/acceleration/start-state/duration/workspace checks; `ValidateTrajectory` veto service; provisional limits, no e-stop |
| Measurement protocols | Implemented, dry-run/simulate validated | repeatability/backlash/visual-displacement/gripper-envelope schedules and analyzers; no hardware data |
| Error budget framework | Implemented, demo-validated; project budget **partial** | [`docs/phase3/ERROR_BUDGET.md`](docs/phase3/ERROR_BUDGET.md); measured hand-eye + fiducial terms, RSS 3.699 mm / 0.682°, required repeatability/backlash/execution/gripper/fixture terms unmeasured |
| Paired trial framework | Implemented, simulated-validated | [`docs/phase3/PAIRED_TRIALS.md`](docs/phase3/PAIRED_TRIALS.md); 24-pair randomized plan, seeded bootstrap/sign/McNemar analysis; no physical trial |
| Host/DDS benchmark | Implemented, read-only hardware snapshot | [`docs/phase3/HOST_BENCHMARK.md`](docs/phase3/HOST_BENCHMARK.md); laptop + Pi host metrics, 20 Hz state, ~12 Hz camera cross-host, 0.24 ms ping |
| ArmPi Debug Studio | G1-Mock + read-only hardware + G2/G3/G4/G5 | 76 backend + 24 gateway tests pass; 10 mock + 3 hardware Playwright; E7 gateway hardware-verified and persistent (systemd); G2 trajectory analysis, G3 experiment records, G4 visual workbench with live `/tf` verified, G5 read-only trajectory recorder with a supervised moving capture; hardware motion disabled |
| M2.2 controller profiles | Implemented, simulation-validated | default / 0.1 / 0.2 trajectories succeeded in Gazebo (2026-11-15) |

The detailed audit (sources, conflicts, decisions) is in
[`docs/audit/2026-11-15/`](docs/audit/2026-11-15/).

## Repository layout

```text
armpi-debug-studio/        # G1 console (FastAPI + React) + read-only gateway; no hardware motion
docs/
├── armpi/                 # PRD + G0 vendor audit
├── audit/2026-11-15/      # source inventory, reconciliation, status, merge plan
├── hardware/              # hardware baseline (live 2026-11-17; joint limits/IMU open)
├── phase0/                # Phase 0 acceptance, 5-DoF constraints, historical audits
├── phase2/                # grasp candidate search; calibration/grasping prep
├── phase3/                # error budget, paired trials, host/DDS benchmark
├── archive/               # old README/ROADMAP, stale URDF, Phase II branch note
└── weekly-log/            # 8-week learning record
evidence/
├── m2_2_v2/               # M2.2 bags (gitignored), CSVs, reports, snapshots
├── armpi_moveit/          # model check, MoveIt smoke and Phase 0 logs
├── armpi_grasp/           # grasp candidate search evidence (mock, plan-only)
├── fiducial/              # synthetic fiducial detection + ROS TF evidence
├── phase1/                # live Pi/ROS/servo/TF/camera baseline (2026-11-17)
├── phase2/                # frames, joint limits, bridge E2E, safety, measurements
└── phase3/                # error budget, paired trial and host/DDS benchmark evidence
ros2_ws/src/
├── arm_basics/            # learning nodes (legacy)
├── arm_description/       # 2-DOF simple_arm + M2.2 controller profiles
├── arm_interfaces/        # custom action + trajectory validation service
├── arm_moveit_demo/       # Panda pick-and-place demo
├── armpi_fiducial_localizer/  # AprilTag/ArUco target localization
├── armpi_grasp_planner/   # 5-DoF candidate generation + bridge + mock E2E launch
├── armpi_safety/          # state watchdog + trajectory veto
├── armpi_analysis/        # error budget, paired trials, host/DDS benchmark
├── armpi_ultra_description/   # 5-DoF ArmPi xacro + gripper + camera frames
└── armpi_ultra_moveit_config/ # 5-DoF MoveIt config (mock + real-robot launch)
tools/
├── humble_moveit/         # Humble Docker MoveIt/RViz for the real robot
├── phase1/                # live hardware baseline helpers
├── phase2/                # calibration, fiducial, bridge E2E and measurement tools
└── phase4/                # read-only host/DDS benchmark collector
```

## Build

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --symlink-install
```

The Panda demo also needs the MoveIt/Panda underlay described in the archived
8-week README (a separate `~/ws_moveit` workspace).

## Run

### ArmPi model and MoveIt (mock hardware)

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

# model check
xacro ros2_ws/src/armpi_ultra_description/urdf/armpi_ultra.xacro > /tmp/armpi.urdf
check_urdf /tmp/armpi.urdf

# model-only RViz (no MoveIt); use_rviz:=false for a headless check
ros2 launch armpi_ultra_description display.launch.py

# MoveIt demo: mock_components, no real hardware
# add use_rviz:=false for a headless run
ros2 launch armpi_ultra_moveit_config demo.launch.py \
  ros2_control_hardware_type:=mock_components \
  use_sim_time:=false use_gazebo:=false use_real:=false

# constrained 6D goal on gripper_link (plan only, no execution)
python3 tools/phase0/constrained_plan_check.py \
  --pipeline pilz_industrial_motion_planner --planner PTP
```

Expected: `move_group` loads, KDL IK initializes, `arm_controller` and
`gripper_controller` are active. The joint-space smoke test is in
`evidence/armpi_moveit/MOVEIT_SMOKE_20261115.md`; the constrained-goal check
returns `error_code=1 (SUCCESS)` with a non-zero trajectory point count. See
[`docs/phase0/PHASE0_ACCEPTANCE.md`](docs/phase0/PHASE0_ACCEPTANCE.md).

> **Do not treat this as hardware validation.** `mock_components` is a fake
> system; `joint_states` semantics, limits, camera and gripper are unmeasured.

### 5-DoF grasp candidate search (mock, plan-only)

With the headless MoveIt mock above still running:

```bash
# positive control: an FK-exact Cartesian candidate
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-joints 0.3 0.9 -1.8 -1.7 0.4 --max-plan 3 --json-out /tmp/grasp_cartesian.json

# practical search: joint-space sampler around a target point
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-point 0.1382 0.0225 0.0896 --sampler joint \
  --max-plan 3 --json-out /tmp/grasp_joint.json

# negative control: unreachable point (expects exit 1)
ros2 run armpi_grasp_planner grasp_candidate_check \
  --target-point 0.60 0.0 0.30 --sampler joint --json-out /tmp/grasp_unreachable.json
```

The pipeline is sanity → full-pose IK → FK verification → collision validity →
plan-only Pilz `PTP`. It never executes. On 2026-11-16 the joint sampler produced
80 candidates → 40 IK OK → 26 collision-free → 3 plans. See
[`docs/phase2/GRASP_CANDIDATE_SEARCH.md`](docs/phase2/GRASP_CANDIDATE_SEARCH.md)
and [`evidence/armpi_grasp/20261116/`](evidence/armpi_grasp/20261116/).

### Fiducial target localization (real camera + synthetic checks)

```bash
# synthetic accuracy check: 5 known poses, JSON + overlays
ros2 run armpi_fiducial_localizer fiducial_synthetic_check \
  --output-dir evidence/fiducial/20261116

# real camera: localizer + measured optical frame publisher
ros2 launch armpi_fiducial_localizer fiducial_localizer.launch.py \
  image_topic:=/depth_cam/rgb/image_raw \
  camera_info_topic:=/depth_cam/rgb/camera_info \
  marker_size_m:=0.06 output_frame:=base_link

# synthetic image stream (no camera) for a quick node check
ros2 run armpi_fiducial_localizer synthetic_marker_publisher \
  --tvec 0 0 0.4 --rvec 3.141592653589793 0 0 --marker-size 0.06
```

The AprilTag 36h11 detector publishes `target_pose` in
`camera_optical_calibrated` (the measured optical frame) by default, or in
`base_link` after TF. It uses `solvePnPRefineLM` to remove the fronto-parallel
IPPE ambiguity. Results: 5/5 synthetic poses, max 1.82 mm / 1.49°; 15/15
semi-synthetic poses on real frames, max 1.69 mm / 0.53°; 0 false detections on
8 real frames. See
[`docs/phase2/FIDUCIAL_TARGET_LOCALIZATION.md`](docs/phase2/FIDUCIAL_TARGET_LOCALIZATION.md),
[`docs/hardware/camera_frames.md`](docs/hardware/camera_frames.md),
[`evidence/fiducial/20261116/`](evidence/fiducial/20261116/) and
[`evidence/phase2/real_background_20261117/`](evidence/phase2/real_background_20261117/).

### Fiducial → grasp mock E2E (plan-only, safety-vetoed)

```bash
# One command: synthetic marker -> localizer -> bridge -> grasp search ->
# safety veto. It starts/stops the mock chain and uses isolated domain 77.
tools/phase2/run_mock_visual_grasp_e2e.sh evidence/phase2/session3_20261117
```

Manual form (two terminals, same isolated environment):

```bash
ros2 launch armpi_grasp_planner mock_visual_grasp.launch.py use_rviz:=false
export ROS_DOMAIN_ID=77 ROS_LOCALHOST_ONLY=1
ros2 run armpi_grasp_planner grasp_target_bridge \
  --safety-service /armpi_safety/validate_trajectory \
  --include-trajectory --json-out /tmp/bridge.json
```

The bridge accepts `target_pose` in `camera_optical_calibrated`, transforms it
to `base_link`, rejects stale/tilted markers, and runs the joint-space grasp
search. On 2026-11-17 it produced 3/3 plan-only Pilz `PTP` plans, all approved
by the safety service; it never executes. See
[`docs/phase2/FIDUCIAL_GRASP_BRIDGE.md`](docs/phase2/FIDUCIAL_GRASP_BRIDGE.md).

### Safety supervisor (veto only)

```bash
ros2 launch armpi_safety safety_supervisor.launch.py
ros2 service list | grep armpi_safety
```

It watches `/joint_states`, publishes `safety_status` / `safety_stop`, and
exposes `armpi_safety/validate_trajectory` + `armpi_safety/reset_stop`. It does
not forward or execute commands. Limits are provisional; see
[`docs/phase2/SAFETY_LAYER.md`](docs/phase2/SAFETY_LAYER.md).

### Phase 3 measurement protocols (no motion)

```bash
python3 tools/phase2/measurement_protocol.py repeatability \
  --mode dry-run --trials 30 --json-out /tmp/repeatability.json
python3 tools/phase2/measurement_protocol.py backlash \
  --mode simulate --json-out /tmp/backlash_sim.json
python3 tools/phase2/measurement_protocol.py visual_displacement \
  --mode simulate --json-out /tmp/visual_sim.json
python3 tools/phase2/measurement_protocol.py gripper_envelope \
  --mode simulate --json-out /tmp/gripper_sim.json
```

`dry-run` writes a schedule and does not import ROS; `simulate` exercises the
analyzers with synthetic noise/bias; `analyze` reads a later hardware file.
Joint-limit measurement remains the separate physical
`tools/phase2/joint_limit_survey.py` (read-only mode is safe; torque-off survey
needs support). See
[`docs/phase2/MEASUREMENT_PROTOCOL.md`](docs/phase2/MEASUREMENT_PROTOCOL.md)
and [`docs/phase2/JOINT_LIMITS.md`](docs/phase2/JOINT_LIMITS.md).

### Error budget, paired trials and host benchmark (offline/read-only)

```bash
# Error budget: checked-in partial project budget + synthetic demo
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.error_budget \
  --input ros2_ws/src/armpi_analysis/config/error_budget_baseline.json \
  --markdown-out /tmp/error_budget.md
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.error_budget --demo

# Paired trials: randomized plan, synthetic simulation, analysis
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.paired_trials \
  plan --pairs 24 --seed 7 --json-out /tmp/paired_plan.json
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.paired_trials \
  simulate --pairs 24 --seed 7 --jsonl-out /tmp/paired.jsonl
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.paired_trials \
  analyze --input /tmp/paired.jsonl --markdown-out /tmp/paired.md

# Read-only host/DDS benchmark (laptop + robot; no motion)
tools/phase4/run_benchmark.sh evidence/phase3/session4_20261117/with_camera
```

The error budget is **partial**: repeatability, backlash, grasp execution,
gripper placement and fixture pose are still unmeasured. The paired-trial
analysis is validated on synthetic data only. The benchmark is read-only and
does not command the robot. See
[`docs/phase3/`](docs/phase3/README.md) and
[`evidence/phase3/session4_20261117/`](evidence/phase3/session4_20261117/).

### M2.2 controller profiles (Gazebo, 2-DOF `simple_arm`)

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch arm_description gazebo.launch.py controller_profile:=gain_0p2
```

Profiles: `default`, `gain_0p1`, `gain_0p2`. The 2026-11-15 regression sent
`joint1=0.2, joint2=0.2` to `/arm_controller/follow_joint_trajectory`; all three
returned `error_code=0`. See
[`evidence/m2_2_v2/rework1_runtime/REWORK1_RUNTIME_20261115.md`](evidence/m2_2_v2/rework1_runtime/REWORK1_RUNTIME_20261115.md).

### ArmPi Debug Studio (Mock, read-only hardware, G5 recorder)

```bash
cd armpi-debug-studio/backend
python3 -m venv .venv && . .venv/bin/activate
pip install -e '.[dev]'
pytest
uvicorn app.main:create_app --factory --host 127.0.0.1 --port 8000

# another terminal
cd armpi-debug-studio/frontend
npm install && npm run dev
npm run test:e2e          # builds + 10 Playwright mock tests

# hardware mode, explicit and private; read-only, never in CI
ARMPI_E2E_HARDWARE=1 \
ARMPI_GATEWAY_URL=http://10.42.0.218:8765 \
ARMPI_GATEWAY_TOKEN_FILE=/tmp/armpi_gateway_token \
npm run test:e2e:hardware
```

The Mock UI must show `SIMULATION`. With `ARMPI_MODE=hardware` and the
read-only gateway running on the robot, the UI shows `LIVE` battery and camera
data; joint positions are a command echo. Hardware motion remains disabled by
design. The gateway was hardware-verified on 2026-11-17 and now runs as a host
systemd service; its `/tf` path was verified on 2026-11-18 (11 dynamic + 8
static edges, `base_link→link5` within 0.4 mm of `tf2_echo`). See
[`armpi-debug-studio/docs/e7-gateway-verification.md`](armpi-debug-studio/docs/e7-gateway-verification.md),
[`armpi-debug-studio/ros2_gateway/deploy/README.md`](armpi-debug-studio/ros2_gateway/deploy/README.md)
and [`armpi-debug-studio/README.md`](armpi-debug-studio/README.md). Joint
limits remain provisional; the read-only survey and the torque-off tool are in
[`docs/phase2/JOINT_LIMITS.md`](docs/phase2/JOINT_LIMITS.md).

The **G5 read-only recorder** captures the vendor command echo and
`/controller_manager/servo_states` measured positions and writes a G2 dataset.
A 5 s no-motion capture and a supervised `camera_forward` moving capture
(0.892 s, 712 commanded / 711 measured samples) both imported successfully;
the moving dataset reports tracking max 0.008321 rad / RMSE 0.003962 rad, which
is dominated by servo pulse quantization and a one-period start offset. See
[`armpi-debug-studio/docs/g5-trajectory-recorder.md`](armpi-debug-studio/docs/g5-trajectory-recorder.md).

## Model caveats (important)

- **Joint limits are provisional.** `joint3` upper bound is `0.0` (vendor
  software mapping), not `+2.09`. `r_joint` upper bound is `1.257`, the
  intersection of URDF and vendor software range. Mechanical limits and all
  `effort="1000"` values are unmeasured; every arm limit carries a
  `PROVISIONAL` comment.
- **Camera frames split model vs hardware.** `camera_connet_link` is the vendor
  mount; the model `camera_link` / `camera_optical_frame` are nominal mock
  frames (identity/REP-103). On the real robot `rgb_camera_link` is a body
  frame and the measured optical frame is `camera_optical_calibrated`, derived
  from the validated hand-eye and the full-precision live TF. The old
  model-only frame is not published on hardware; see
  [`docs/hardware/camera_frames.md`](docs/hardware/camera_frames.md).
- **IK is KDL, full-pose by default.** TRAC-IK is not installed.
  `position_only_ik: false` is the checked-in default, so a 6D goal is not
  silently reduced to position. Use `config/kinematics_reachability.yaml` only
  for “can the arm reach this point?” checks. See
  [`docs/phase0/ARMPI_5DOF_CONSTRAINTS.md`](docs/phase0/ARMPI_5DOF_CONSTRAINTS.md).
- **Constrained goals use Pilz `PTP`.** OMPL's goal sampler fails on the 6D goal
  with KDL even though `/compute_ik` solves it; `LIN` fails because a
  fixed-orientation straight line leaves the 5-DoF reachable set. Details are in
  the Phase 0 acceptance document.
- **Grasp search is reachable-set based.** At a fixed tool point the reachable
  orientations form a thin 5-DoF manifold, so most Cartesian orientation
  neighbours fail IK. Use the joint-space sampler (`--sampler joint`) for a
  target point; it reports the residual `target_offset_m` for each candidate.
  The TCP offset is nominal and all plans are plan-only.
- **Fiducial localization is synthetic + real-background validated, not a
  physical-target test.** The AprilTag detector uses a printed edge length
  (`marker_size_m`, default 0.06 m) and the real camera `CameraInfo`. The
  measured intrinsics and the `link4 → camera` optical frame are validated;
  the real-background test composites the marker, so it does not measure
  lighting/printing/mount errors or a physical target.
- **Stale URDF removed.** The vendor pre-generated `armpi_ultra.urdf` (joint1 and
  r_joint `fixed`) is archived at `docs/archive/armpi_ultra.urdf.stale`; use
  xacro only.
- **Hardware is live but motion is limited.** The Pi/ROS/servo/camera stack runs
  and one supervised `reset_servo` motion plus plan-only checks were verified.
  Joint limits and the IMU remain open; there is no autonomous motion and no
  real grasp.

## Evidence and reproducibility

- `evidence/m2_2_v2/` — four MCAP bags, analysis CSVs/PNGs, source snapshot,
  Gate 2 reports. `*.mcap` is gitignored; keep a local/remote backup.
- `evidence/armpi_moveit/` — expanded URDF, `check_urdf`, MoveIt logs and the
  Phase 0 constrained-plan evidence.
- `evidence/armpi_grasp/` — grasp candidate search JSON and headless MoveIt log
  (mock, plan-only).
- `evidence/fiducial/` — synthetic marker accuracy JSON/overlays and the ROS
  localizer/TF integration logs.
- `evidence/calibration/20261117/` — chessboard hand-eye dataset, per-method
  validation and the offline 24-pose selection.
- `evidence/phase2/real_background_20261117/` — real-background negative
  control and semi-synthetic pose accuracy.
- `evidence/phase2/frames_20261117/` — full-precision body→optical frame
  contract.
- `evidence/phase2/vendor_calibration_20261117/` — decoded vendor calibration
  and Baseline A status.
- `evidence/phase2/session3_20261117/` — synthetic marker → grasp → safety E2E,
  safety service negative controls, and measurement dry-run/simulate reports.
- `evidence/phase3/session4_20261117/` — partial error budget, paired-trial
  simulation, and read-only laptop/robot host/DDS benchmark.
- `evidence/README.md` and `evidence/m2_2_v2/MANIFEST.md` explain what is tracked.

Never present Mock/Gazebo results as hardware validation.

## Documentation

- [Technical roadmap](ROADMAP.md) — public technical roadmap with current status.
- [Phase 0 acceptance](docs/phase0/PHASE0_ACCEPTANCE.md) — checklist, evidence
  and remaining gaps.
- [5-DoF constraints](docs/phase0/ARMPI_5DOF_CONSTRAINTS.md) — task-space limits
  and grasp policy.
- [Phase 2 grasp search](docs/phase2/GRASP_CANDIDATE_SEARCH.md) — candidate
  generation, screening pipeline, mock results and limitations.
- [Phase 2 fiducial localization](docs/phase2/FIDUCIAL_TARGET_LOCALIZATION.md) —
  AprilTag detection, pose convention, real-background accuracy and frame
  contract.
- [Phase 2 fiducial → grasp bridge](docs/phase2/FIDUCIAL_GRASP_BRIDGE.md) —
  `target_pose` → TF → grasp search → safety veto, mock E2E result.
- [Phase 2 safety layer](docs/phase2/SAFETY_LAYER.md) — watchdog, limits,
  workspace and trajectory-veto service.
- [Phase 3 measurement protocols](docs/phase2/MEASUREMENT_PROTOCOL.md) —
  repeatability, backlash, visual displacement and gripper envelope.
- [Phase 3 error budget](docs/phase3/ERROR_BUDGET.md) — source schema, RSS/worst
  case, partial project budget and the missing measurements.
- [Phase 3 paired trials](docs/phase3/PAIRED_TRIALS.md) — matched-pair recorder,
  randomization and exact paired analysis.
- [Phase 3 host/DDS benchmark](docs/phase3/HOST_BENCHMARK.md) — read-only
  laptop/Pi metrics, topic rates/jitter/age and the camera-rate correction.
- [Camera frame contract](docs/hardware/camera_frames.md) — body vs optical
  frames and `camera_optical_calibrated`.
- [Vendor calibration baseline](docs/hardware/vendor_calibration_baseline.md) —
  decoded vendor matrices and Baseline A status.
- [Phase 2 joint limits](docs/phase2/JOINT_LIMITS.md) — read-only bus-servo
  evidence, the torque-off measurement procedure and the CycloneDDS gotcha.
- [Audit 2026-11-15](docs/audit/2026-11-15/README.md) — sources, conflicts,
  status, merge plan, open decisions.
- [Debug Studio PRD](docs/armpi/ARMPI_DEBUG_STUDIO_PRD_V1.md)
- [G0 vendor audit](docs/armpi/g0/README.md)
- [Hardware baseline](docs/hardware/README.md)
- [Phase 0 audits](docs/phase0/)
- [8-week learning log](docs/weekly-log/)

`MASTER_PLAN_V3_PRIVATE.md` is intentionally **not** in this public repository.

## License

See [LICENSE](LICENSE).
