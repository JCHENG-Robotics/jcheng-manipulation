# MERGE_EXECUTION_REPORT — 2026-11-15

> Executed after the Board replied "都按你的推荐去做吧".
> **No `git add`, `git commit`, `git push`, `git reset`, `git clean`, or
> destructive sync was performed.** All Git history remains unchanged; the
> working tree is dirty for Board review.

## 1. Scope executed

| Decision | Action |
|---|---|
| D-01 A | M2.2 rework1 adopted; local validated variant preserved under `evidence/m2_2_v2/pre_rework_snapshot/` and `/home/jerichocheng/backups/2026-11-15/` |
| D-02 A | Studio source imported under `armpi-debug-studio/`; dependencies/data excluded |
| D-03 A | Studio relabelled **G1-Mock**, not PRD G1 |
| D-04 A | Minimal ArmPi model repair: joint3/r_joint limits, camera frames, provisional markers, stale URDF archived, gripper accel limits |
| D-05 A | Phase II branch archived in `docs/archive/phase2-week01.md`; not merged |
| D-06 B | Pending: Board exports container workspace (not executed) |
| D-07 A | PRD/G0/phase0/audit imported; MASTER_PLAN kept outside repo (private copy in `~/文档/private/armpi-plan/`) |
| D-08 A | MCAP/DB not tracked; CSV/PNG/manifests/reports tracked; mcap gitignored |
| D-09 A | S05 partial copy, S10 WSL2 residue and S04 0-file skeleton quarantined under `~/backups/2026-11-15/quarantine/` after the parallel session went idle |
| D-10 B | Parallel session `armpi_ultra_ws_clean` left isolated; not merged/audited |
| D-11 A | No Git remote operation; Board can relay via M70q |
| D-12 A | Repo rename deferred |

## 2. Snapshot / rollback points

| Item | Path |
|---|---|
| Full repo tar (pre-merge) | `/home/jerichocheng/backups/2026-11-15/repo-before-merge.tar.gz` |
| Local M2.2 patch | `/home/jerichocheng/backups/2026-11-15/local-m2.2-tracked.patch` |
| Local M2.2 files | `/home/jerichocheng/backups/2026-11-15/local-m2.2-variant/` |
| M70q targeted source | `/home/jerichocheng/backups/2026-11-15/m2_2-v2-targeted/` |
| Studio source snapshot | `/home/jerichocheng/backups/2026-11-15/armpi-debug-studio-src/` |
| Quarantine | `/home/jerichocheng/backups/2026-11-15/quarantine/` |
| Post-merge full tar (local) | `/home/jerichocheng/backups/2026-11-15/repo-after-merge.tar.gz` (37M, SHA `f68327c4…`) |
| Post-merge text/evidence tar (M70q) | `<user>@<M70Q_TAILSCALE_IP>:~/backups/merged-text-2026-11-15.tar.gz` (954K, SHA `6a3a7da1…`; excludes `.git`, meshes, demo video, deps, MCAP) |

> The 37M full tar is local-only (the slow SSH link truncated a first upload).
> The 954K M70q copy preserves all text, configs, docs, evidence logs/CSV/PNG
> and the Studio source; vendor meshes/16G assets remain on R9000P/USB.

## 3. Imported / changed

### Imported

- `armpi-debug-studio/` — 57 files, 560K; 28 Python + 16 TS/TSX.
- `docs/phase0/` — JER-39, JER-59.
- `docs/armpi/` — PRD v1.0 + G0 (8 files).
- `docs/audit/2026-11-15/` — 6 reports.
- `docs/hardware/` — 7 baseline templates + README.
- `ros2_ws/src/armpi_ultra_description/` — xacro + gripper + meshes + launch + rviz.
- `ros2_ws/src/armpi_ultra_moveit_config/` — MoveIt config (mock_components).
- `evidence/m2_2_v2/` — 4 bags (gitignored), analysis, snapshots, Gate 2 reports, runtime regression.
- `evidence/armpi_moveit/` — model, check_urdf, MoveIt logs.
- `docs/archive/` — old README/ROADMAP, stale URDF, joint_limits.yaml.bak, phase2 note.

### Changed

| File | Change |
|---|---|
| `ros2_ws/src/armpi_ultra_description/urdf/armpi_ultra.xacro` | joint3 upper `0.0`; `camera_link` + `camera_optical_frame` added; comments |
| `.../urdf/gripper.urdf.xacro` | `r_joint` upper `1.257`; comment |
| `.../urdf/armpi_ultra.urdf` | moved to `docs/archive/armpi_ultra.urdf.stale` |
| `.../config/joint_limits.yaml` | explicit position limits, gripper acceleration limits |
| `.../config/initial_positions.yaml` | vendor startup arm pose; r_joint 0.0 placeholder |
| `.../config/kinematics.yaml` | KDL fallback; position-only caveat |
| `.../config/armpi_ultra.srdf` | gripper open state `1.2` (within new limit) |
| `ros2_ws/src/arm_description/*` | rework1 files (hashes below) |
| `README.md` | rewritten for the ArmPi project |
| `ROADMAP.md` | replaced with `ROADMAP_PUBLIC.md`; old archived |
| `.gitignore` | Python/Node/deps/data/tsbuildinfo rules |
| `CHANGELOG.md` | new |

## 4. Verification results

### 4.1 Build

| Command | Result |
|---|---|
| `colcon build --packages-select armpi_ultra_description armpi_ultra_moveit_config arm_description --symlink-install` | 3 packages finished |

### 4.2 M2.2

| Check | Result |
|---|---|
| `colcon test --packages-select arm_description` | 1 CTest, 8 pytest cases, 0 failures |
| `colcon test-result` | 9 tests, 0 errors, 0 failures |
| Gazebo `default` trajectory | SUCCEEDED, `error_code=0` |
| Gazebo `gain_0p1` trajectory | SUCCEEDED, `error_code=0` |
| Gazebo `gain_0p2` trajectory | SUCCEEDED, `error_code=0` |

Evidence: `evidence/m2_2_v2/rework1_runtime/REWORK1_RUNTIME_20261115.md`.

### 4.3 ArmPi model / MoveIt

| Check | Result |
|---|---|
| `xacro` | 17 links, 16 joints |
| `check_urdf` | success; single root `base_link` |
| camera frames | `camera_connet_link → camera_link → camera_optical_frame` |
| KDL plugin | loaded successfully |
| joint-space `/move_action` | `error_code.val=1`, planning_time 0.0328 s |
| gripper acceleration error | not reproduced |

Evidence: `evidence/armpi_moveit/MOVEIT_SMOKE_20261115.md`.

### 4.4 Studio

| Check | Result |
|---|---|
| backend `pytest` | 41 passed |
| frontend `npm run build` | `tsc -b && vite build` success |

## 5. M2.2 file hashes (adopted rework1)

```text
8952926b… CMakeLists.txt
b66ed73c… package.xml
61cb8415… launch/gazebo.launch.py
a4f89f14… urdf/simple_arm.xacro
708408e9… config/arm_controllers_gain_0p1.yaml
5ceea231… config/arm_controllers_gain_0p2.yaml
0a09592e… arm_description/controller_config.py
4a6114c9… test/test_controller_config.py
```

The pre-rework source snapshot (the exact source of the 2026-11-10 bags) is at
`evidence/m2_2_v2/pre_rework_snapshot/` with the original hashes
`ee85a1e4…` / `b3be05fa…` / `5ceea231…`.

## 6. Not done / pending

1. **No Git writes** — Board must review `git status` and commit. Suggested
   split:
   - `feat(armpi): import 5-DoF model and MoveIt config`
   - `feat(studio): import G1-Mock console`
   - `feat(m2.2): adopt Gate 2 rework1 and runtime regression`
   - `docs: add audit, hardware templates, roadmap and changelog`
2. **D-06** — container/robot workspace export still pending.
3. **Position-constrained grasp plan** — not attempted; KDL position-only is
   reachability-only. TRAC-IK/pick_ik not installed.
4. **Playwright** — no test file; browser acceptance still unreproducible.
5. **E7 ROS 2 gateway** — not implemented.
6. **Hardware** — no serial device, no camera, no base, no real motion.
7. **S17** — `Projects_M70q/armpi_ultra_ws_clean` finished at 11:34 (session
   idle): 28M/266 files, 12 packages, `docs/REPORT.md`/`AUDIT.md`/
   `VERIFICATION.md`/`DEPLOYMENT.md`/`ROLLBACK.md`. Its self-reported checks are
   x86_64/Jazzy only; no ARM64 build, no hardware, no deployment. **Not merged**
   into this repository; treat it as a separate vendor-slimming deliverable.
8. **S04** — the 0-file `Projects_M70q/ros2-industrial-arm-learning` skeleton
   was quarantined after the parallel session went idle.
9. **Phase II branch** — archived only; no merge.
10. **Cosmetic issues** — `clock_bridge` uses `ignition.msgs.Clock`; teardown
    logs `Gate 2 launch failure` on returncode 0; `sensors_3d.yaml` octomap
    plugin missing; KDL root-link inertia warning.

## 7. Rollback

- Restore full tree: `tar -xzf /home/jerichocheng/backups/2026-11-15/repo-before-merge.tar.gz`
  into a temporary directory, compare, then copy back only if needed.
- Restore S05/S10 from `~/backups/2026-11-15/quarantine/`.
- M2.2 pre-rework source and patch are in the snapshot/backup directories.
- No Git ref/history was changed, so `git status`/`git diff` show the exact
  delta for Board review.
