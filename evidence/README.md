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

## Rules

1. Do not edit raw logs/bags; add a new dated report instead.
2. `mcap` files are ignored by Git; do not `git add -f` without a size/LFS
   decision.
3. Mock/Gazebo results must never be labelled hardware-validated.
