# Acceptance report — jcheng-manipulation

> Status: **pass** as of 2026-11-23 for the frozen `v1.0.0` baseline.
> Operator: JerichoCheng. Reviewer: pending.
> Scope: the measured 5-DoF manipulation baseline on one ArmPi Ultra — model,
> calibration, planning, real grasping, paired trials, error budget, Studio,
> fast motion, full G8 and operations.
> The release gates below pass. The error-budget bounds fail and are recorded as
> limitations; they are experiment results, not release gates.
> **This is not a safety certification.** The platform has no e-stop, brake,
> independent encoder or self-collision model.

## 1. Gates

| Gate | Command / run | Level | Result | Evidence |
|---|---|---|---|---|
| Config consistency | `tools/verify_config_consistency.py` | L0 | pass (`CONFIG OK`) | `RELEASE.md` |
| Model, URDF and TF | colcon build + static checks | L0 | pass (17 links / 16 joints) | `docs/hardware/` |
| Backend tests | Studio backend suite | L1 | pass (148) | `RELEASE.md` |
| Gateway tests | Studio gateway suite | L1 | pass (93) | `RELEASE.md` |
| Analysis tests | `armpi_analysis` | L1 | pass (34) | `RELEASE.md` |
| Safety/planner/fiducial | suite | L1 | pass (59) | `RELEASE.md` |
| Tools | phase2/3/4 + shared/parity/config | L1 | pass (242 passed / 7 skipped) | `RELEASE.md` |
| Studio mock E2E | Playwright | L3 | pass (10 mock + 3 token-mode dashboard) | `RELEASE.md` |
| Reproducible ROS build | digest-pinned `ros-colcon` run `45098657645` | L0 | pass (7 packages; 118 tests, 0 failures, 1 skip) | `evidence/ros_colcon_20261123/README.md` |
| Studio CI | run `45100626118` | L0–L3 | pass | `RELEASE.md` |
| Full G8 | supervised hardware run | L5 | pass (8 phases / 11 bus-verified commands) | `evidence/g8_acceptance_20261123/README.md` |
| Post-change smoke | supervised hardware run | L5 | pass (7 phases / 10 commands) | `evidence/g8_smoke_20261123/README.md` |
| Fast G8 | supervised hardware run | L5 | pass (13.46 s wall, 2.27 mm landing) | `evidence/studio_g8_fast/20261121/README.md` |
| Safety stop round trip | latch/hold/409/clear on hardware | L5 | pass | `evidence/studio_validation_20261123/` |
| Paired fixed-site trial | pre-registered 24 pairs | L5 | pass (12/24 → 24/24, p = 0.00049) | `docs/phase3/PAIRED_TRIALS.md` |
| Paired random-placement trial | pre-registered 24 pairs | L5 | pass (9/24 → 24/24, p = 6.1×10⁻⁵) | `docs/phase3/RANDOM_SITE_DESIGN.md` |
| Error budget (result, not a gate) | measured budget | L5 | complete; **bound gates fail** (see §4) | `docs/phase3/ERROR_BUDGET.md` |

## 2. Verified claims

| Claim | Level | Evidence |
|---|---|---|
| Hand-eye calibration: 2.94 mm / 0.39° over 14 poses; hold-out 3.29 mm / 0.43° | L5 | `docs/phase2/CALIBRATION.md` |
| The five joints were surveyed and `mapping_verified=true` for the operating envelope | L5 | `docs/phase2/JOINT_LIMITS.md` |
| Fixed-site paired trial raised grasp success from 12/24 to 24/24 | L5 | `docs/phase3/PAIRED_TRIALS.md` |
| Random-placement paired trial repeated the effect: 9/24 → 24/24 | L5 | `docs/phase3/RANDOM_SITE_DESIGN.md` |
| Full G8: 8 phases / 11 bus-verified commands; put-back at 0.236 m inside the 0.220–0.255 m band; `/safety_stop=false` | L5 | `evidence/g8_acceptance_20261123/` |
| Post-change smoke: 7 phases / 10 commands; one 15.6 mm base pulse; corrected put-back 0.2323 m | L5 | `evidence/g8_smoke_20261123/` |
| Fast motion: grasp + put-back 11.67 s → 8.94 s; G8 fast landing 2.27 mm | L5 | `evidence/studio_g8_fast/20261121/` |
| Reproducible build: 7 public packages, 118 tests, 0 failures, 1 skip | L0 | `evidence/ros_colcon_20261123/` |
| `/safety_stop` latch/hold/409/clear and the real `armpi_safety` node work | L5 | `evidence/studio_validation_20261123/` |
| Dual-camera agreement: 2.58 mm x/y; grip and put-back PASS | L5 | `docs/phase3/` |
| The base is open-loop; no odometry or navigation is claimed | L0 | `docs/phase0/PHASE0_ACCEPTANCE.md` |

## 3. Not verified

| Item | Reason | Risk |
|---|---|---|
| 6 mm translation error-budget bound | Measured RSS 13.325 mm; the bound fails | Cannot claim 6 mm accuracy |
| 2° rotation bound | Worst 3.322° exceeds the provisional bound | Rotation error is not bounded |
| Fast path in both directions | Validated in one direction only | Reverse fast motion is unproven |
| Yawed-grasp correction | The cube must first be rotated to face the gripper; test open | Unknown behavior for yawed grasps |
| Mobile-base navigation | No encoders/IMU/odometry; G8.1 is an assisted open-loop step | Base motion invalidates the table calibration |
| Hardware/pose Playwright for the release | Not rerun; release rerun covered mock and token-mode dashboard | UI regressions on hardware paths are not excluded |
| Ecosystem M6 items (real MoveIt plan-only, S14 base, full G8 re-run) | Not part of the frozen baseline; tracked in `jcheng-system` | The new runtime path is not end-to-end re-validated |
| S7/S9 (stale-state injection, runtime publisher detection) | Ecosystem work, not baseline | Runtime guard residuals |
| Thermal or long-run stability | Not measured | Long sessions are not characterised |

## 4. Limitations

- **The 6 mm error-budget bound is not met.** Translation RSS is 13.325 mm
  (worst 24.014 mm); rotation RSS is 1.858° (worst 3.322°). The empirical
  corrected placement mean of 3.080 mm is a different statistic and must not be
  read as passing the bound.
- **Fast motion is one-directional.** The 4.0/20 profile is bus-verified but
  was validated in one direction.
- **The base is open-loop.** Any base motion invalidates the table calibration.
- **No hardware safety chain.** No e-stop, brake, independent encoder, torque
  sensing or self-collision model; the robot falls when power is cut.
- **TCP-only geometric guards.** Workspace and table checks use the tool point;
  link/gripper collision is outside the model.
- **Single platform and site.** One robot, one table, a small number of
  supervised sessions; not a population-level reliability claim.
- **Not autonomously safe.** The v1.0.0 result is supervised operator tooling.

## 5. Sign-off

| Role | Name | Date |
|---|---|---|
| Operator | JerichoCheng | 2026-11-23 |
| Reviewer | pending | — |

## 6. Machine-readable report

The repository keeps per-run machine-readable evidence (for example
`evidence/g8_acceptance_20261123/g8_run.json` and the `acceptance.json` files
under `evidence/*_20261123/`). They are per-run evidence records, not
`acceptance-report.v1` documents; a conforming report would need the
`jcheng-interfaces` schema fields (`operator`, `reviewer`, `lockfile`,
`scenarios`, `metrics`, `result`, `limitations`).
