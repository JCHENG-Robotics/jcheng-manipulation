# Evidence Index

This directory stores **simulation/Mock** evidence. None of it is hardware
validation. Raw MCAP bags and Studio SQLite data are gitignored; keep the local
and M70q copies as backups.

## `m2_2_v2/` — 2-DOF `simple_arm` controller study

| Path | Content |
|---|---|
| `bags/` | four MCAP bags (gitignored) |
| `analysis/`, `analysis_1p5s/`, `analysis_settling/` | CSV + PNG from the 2026-11-10 runs |
| `config_snapshot/` | exact source snapshot used for the bags |
| `pre_rework_snapshot/` | preserved pre-rework source variant (11-10) |
| `gate2/` | Gate 2 implementation reports (self-reported) |
| `rework1_runtime/` | 2026-11-15 Gazebo regression for default/0.1/0.2 |
| `SHA256SUMS.txt` | original checksum list (paths relative to the old workspace) |
| `MANIFEST.md` | regenerated manifest for this repository |

## `armpi_moveit/` — model + MoveIt smoke test

- `armpi_merged.urdf`, `check_urdf.txt`
- `armpi_plan2.log`, `armpi_plan2_result.log` (joint-space plan, `error_code=1`)
- `MOVEIT_SMOKE_20261115.md`

## `armpi_grasp/` — 5-DoF grasp candidate search (mock, plan-only)

- `20261116/cartesian_exact_fk.json` — exact-FK Cartesian control
- `20261116/joint_sampler.json` — joint-space sampler run (`PLAN_OK` entries)
- `20261116/unreachable.json` — zero-candidate negative control
- `20261116/moveit_headless_20261116.log` — headless MoveIt log
- `20261116/README.md` — commands, results and limitations

## `fiducial/` — synthetic fiducial localization (no real camera)

- `20261116/synthetic_pose_accuracy.json` — five known poses and errors
- `20261116/synthetic_pose_XX_overlay.png`, `synthetic_contact_sheet.png`
- `20261116/ros_camera_frame_check.json`, `ros_base_link_check.json`
- ROS localizer, publisher and static-TF logs
- `20261116/README.md` — commands, results and limitations

## Rules

1. Do not edit raw logs/bags; add a new dated report instead.
2. `mcap` files are ignored by Git; do not `git add -f` without a size/LFS
   decision.
3. Mock/Gazebo results must never be labelled hardware-validated.
