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

## Status (2026-11-16)

Status convention: **Implemented / Validated / In Progress / Planned**. Vendor
functionality is marked separately. `Validated` means there is a command, log or
data file in `evidence/`; it does **not** mean hardware validation.

| Area | Status | Evidence / limit |
|---|---|---|
| Panda learning demo | Implemented, simulation-validated | `panda-baseline-v1` tag; `docs/archive/README_8WEEK.md`; weekly logs |
| ArmPi URDF / TF | Implemented, model-checked | `xacro` + `check_urdf`; single root; camera frames; **limits provisional** |
| ArmPi MoveIt 2 | Implemented, mock-validated | KDL IK; joint-space plan `error_code=1`; 6D-constrained Pilz `PTP` plan success (mock) |
| ArmPi constrained grasp | Implemented, mock-validated | 5-DoF grasp search: 80 joint-space candidates → 40 IK OK → 26 collision-free → 3 Pilz `PTP` plans; plan-only |
| Phase 0 acceptance | **Complete (mock scope)** | [`docs/phase0/PHASE0_ACCEPTANCE.md`](docs/phase0/PHASE0_ACCEPTANCE.md) |
| Real ArmPi hardware | **Not validated** | no serial device, no robot connection, Docker workspace inaccessible |
| Camera / calibration | Planned | `link4 → camera` frames added; extrinsic and intrinsics **not measured** |
| Visual grasping | Planned | no fiducial/calibration/grasp pipeline |
| Error Budget v1 | Planned | M2.2 is a 2-DOF Gazebo controller study, not the project error budget |
| ArmPi Debug Studio | G1-**Mock** only | 41 backend tests pass; frontend builds; E7 gateway not implemented |
| M2.2 controller profiles | Implemented, simulation-validated | default / 0.1 / 0.2 trajectories succeeded in Gazebo (2026-11-15) |

The detailed audit (sources, conflicts, decisions) is in
[`docs/audit/2026-11-15/`](docs/audit/2026-11-15/).

## Repository layout

```text
armpi-debug-studio/        # G1-Mock console (FastAPI + React); no hardware motion
docs/
├── armpi/                 # PRD + G0 vendor audit
├── audit/2026-11-15/      # source inventory, reconciliation, status, merge plan
├── hardware/              # hardware baseline templates ([U] until measured)
├── phase0/                # Phase 0 acceptance, 5-DoF constraints, historical audits
├── phase2/                # grasp candidate search; calibration/grasping prep
├── archive/               # old README/ROADMAP, stale URDF, Phase II branch note
└── weekly-log/            # 8-week learning record
evidence/
├── m2_2_v2/               # M2.2 bags (gitignored), CSVs, reports, snapshots
├── armpi_moveit/          # model check, MoveIt smoke and Phase 0 logs
└── armpi_grasp/           # grasp candidate search evidence (mock, plan-only)
ros2_ws/src/
├── arm_basics/            # learning nodes (legacy)
├── arm_description/       # 2-DOF simple_arm + M2.2 controller profiles
├── arm_interfaces/        # custom action
├── arm_moveit_demo/       # Panda pick-and-place demo
├── armpi_grasp_planner/   # 5-DoF candidate generation + MoveIt screening
├── armpi_ultra_description/   # 5-DoF ArmPi xacro + gripper + camera frames
└── armpi_ultra_moveit_config/ # 5-DoF MoveIt config (mock_components)
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

### ArmPi Debug Studio (Mock only)

```bash
cd armpi-debug-studio/backend
python3 -m venv .venv && . .venv/bin/activate
pip install -e '.[dev]'
pytest
uvicorn app.main:create_app --factory --host 127.0.0.1 --port 8000

# another terminal
cd armpi-debug-studio/frontend
npm install && npm run dev
```

The UI must show `SIMULATION`. Hardware mode is read-only; motion is disabled by
design. See [`armpi-debug-studio/README.md`](armpi-debug-studio/README.md).

## Model caveats (important)

- **Joint limits are provisional.** `joint3` upper bound is `0.0` (vendor
  software mapping), not `+2.09`. `r_joint` upper bound is `1.257`, the
  intersection of URDF and vendor software range. Mechanical limits and all
  `effort="1000"` values are unmeasured; every arm limit carries a
  `PROVISIONAL` comment.
- **Camera frames are nominal.** `camera_connet_link` is the vendor mount;
  `camera_link` and `camera_optical_frame` were added with an identity/REP-103
  rotation. The real `link4 → camera` extrinsic is unknown.
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
- **Stale URDF removed.** The vendor pre-generated `armpi_ultra.urdf` (joint1 and
  r_joint `fixed`) is archived at `docs/archive/armpi_ultra.urdf.stale`; use
  xacro only.
- **No hardware.** No `/dev/rrc`, no camera, no base, no real motion.

## Evidence and reproducibility

- `evidence/m2_2_v2/` — four MCAP bags, analysis CSVs/PNGs, source snapshot,
  Gate 2 reports. `*.mcap` is gitignored; keep a local/remote backup.
- `evidence/armpi_moveit/` — expanded URDF, `check_urdf`, MoveIt logs and the
  Phase 0 constrained-plan evidence.
- `evidence/armpi_grasp/` — grasp candidate search JSON and headless MoveIt log
  (mock, plan-only).
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
