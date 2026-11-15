# ArmPi Ultra 5-DoF Manipulation — ROS 2 / MoveIt 2

A personal robotics project around the **ArmPi Ultra** 5-DoF arm: corrected model
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

## Status (2026-11-15)

Status convention: **Implemented / Validated / In Progress / Planned**. Vendor
functionality is marked separately. `Validated` means there is a command, log or
data file in `evidence/`; it does **not** mean hardware validation.

| Area | Status | Evidence / limit |
|---|---|---|
| Panda learning demo | Implemented, simulation-validated | `panda-baseline-v1` tag; `docs/archive/README_8WEEK.md`; weekly logs |
| ArmPi URDF / TF | Implemented, model-checked | `xacro` + `check_urdf`; single root; camera frames; **limits provisional** |
| ArmPi MoveIt 2 | Implemented, mock-validated | KDL IK loads; joint-space plan `error_code=1`; mock_components |
| ArmPi constrained grasp | Planned | `position_only_ik` is reachability-only; no grasp planner yet |
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
├── phase0/                # JER-39 / JER-59 read-only audits
├── archive/               # old README/ROADMAP, stale URDF, Phase II branch note
└── weekly-log/            # 8-week learning record
evidence/
├── m2_2_v2/               # M2.2 bags (gitignored), CSVs, reports, snapshots
└── armpi_moveit/          # model check + MoveIt smoke logs
ros2_ws/src/
├── arm_basics/            # learning nodes (legacy)
├── arm_description/       # 2-DOF simple_arm + M2.2 controller profiles
├── arm_interfaces/        # custom action
├── arm_moveit_demo/       # Panda pick-and-place demo
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

# MoveIt demo: mock_components, no real hardware
ros2 launch armpi_ultra_moveit_config demo.launch.py \
  ros2_control_hardware_type:=mock_components \
  use_sim_time:=false use_gazebo:=false use_real:=false
```

Expected: `move_group` loads, KDL IK initializes, `arm_controller` and
`gripper_controller` are active. A joint-space `/move_action` goal is verified in
`evidence/armpi_moveit/MOVEIT_SMOKE_20261115.md`.

> **Do not treat this as hardware validation.** `mock_components` is a fake
> system; `joint_states` semantics, limits, camera and gripper are unmeasured.

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
  intersection of URDF and vendor software range. Mechanical limits are not
  measured.
- **Camera frames are nominal.** `camera_connet_link` is the vendor mount;
  `camera_link` and `camera_optical_frame` were added with an identity/REP-103
  rotation. The real `link4 → camera` extrinsic is unknown.
- **IK is KDL, position-only.** TRAC-IK is not installed. `position_only_ik` may
  be used for reachability, never as the final grasp policy; a 5-DoF grasp needs
  a constrained orientation.
- **Stale URDF removed.** The vendor pre-generated `armpi_ultra.urdf` (joint1 and
  r_joint `fixed`) is archived at `docs/archive/armpi_ultra.urdf.stale`; use
  xacro only.
- **No hardware.** No `/dev/rrc`, no camera, no base, no real motion.

## Evidence and reproducibility

- `evidence/m2_2_v2/` — four MCAP bags, analysis CSVs/PNGs, source snapshot,
  Gate 2 reports. `*.mcap` is gitignored; keep a local/remote backup.
- `evidence/armpi_moveit/` — expanded URDF, `check_urdf`, MoveIt log and action
  result.
- `evidence/README.md` and `evidence/m2_2_v2/MANIFEST.md` explain what is tracked.

Never present Mock/Gazebo results as hardware validation.

## Documentation

- [Technical roadmap](ROADMAP.md) — imported from `ROADMAP_PUBLIC.md`.
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
