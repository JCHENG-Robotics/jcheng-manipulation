# Phase 3 — Error Budget, Paired Trials and Host Benchmark

> Status: **Implemented and offline/synthetic-validated (2026-11-17, Session 4);
> updated 2026-11-19.** There is still no paired hardware trial. The error
> budget is explicitly partial, the paired-trial analysis is validated on
> synthetic data, and the host/DDS benchmark was run read-only on the live
> laptop and robot. Later real unpaired grasps (3/5) provide execution evidence
> but not the repeatability/backlash/gripper terms the budget requires.

Session 4 of the remote work plan covers R11 (error-budget framework), R12
(paired-trial recorder), R13 (distributed host/DDS benchmark) and R15
(CI/documentation closure). The code lives in the dependency-free
`armpi_analysis` package and `tools/phase4/`.

| Document | Purpose |
|---|---|
| [ERROR_BUDGET.md](ERROR_BUDGET.md) | JSON source schema, RSS/worst-case aggregation, Monte Carlo interval, partial baseline result and limitations |
| [PAIRED_TRIALS.md](PAIRED_TRIALS.md) | Matched-pair record schema, seeded randomization, bootstrap/sign/McNemar analysis, synthetic validation |
| [HOST_BENCHMARK.md](HOST_BENCHMARK.md) | Read-only host metrics, DDS topic rate/jitter/age, laptop-vs-Pi results, camera-FPS correction |
| [../phase2/MEASUREMENT_PROTOCOL.md](../phase2/MEASUREMENT_PROTOCOL.md) | Physical schedules that will feed the budget and trials |

## State

| Item | State | Note |
|---|---|---|
| Error-budget framework | **Implemented, demo-validated** | `armpi_analysis/error_budget.py`; measured/unmeasured sources stay separate; RSS, worst case, dominance, Monte Carlo interval, Markdown/JSON report and SVG chart |
| Partial project budget | **Reported, partial** | Measured hand-eye + fiducial terms; repeatability, backlash, execution, gripper and fixture pose remain required and unmeasured |
| Paired-trial recorder | **Implemented, simulated-validated** | append-only JSONL; seeded within-pair randomization; exact paired tests |
| Host/DDS benchmark | **Implemented, hardware-measured (read-only)** | laptop + Pi host, robot-container and cross-host ROS topics, ping; no command sent |
| Pure-Python CI | **Added** | flake8 + pytest for the analysis package, measurement protocol and phase-4 tools |
| Real paired grasp trials | **Not started** | 3/5 real unpaired grasps exist (2026-11-19), but no paired hardware trial or physical tolerance experiment |

Evidence: [`evidence/phase3/session4_20261117/`](../../evidence/phase3/session4_20261117/).
The unpaired real-grasp evidence is in
[`evidence/phase2/onsite_20261119/`](../../evidence/phase2/onsite_20261119/);
it is not a paired trial.

## What Session 4 does not claim

- The error budget is **not complete** and its tolerance is provisional.
- The paired-trial analysis is validated on synthetic data, not on grasps.
- The benchmark is a single read-only snapshot; it is not a load sweep or a
  real-time guarantee.
- Session 4 itself performed no robot motion, grasp, calibration capture or
  vendor demo; later sessions did, but they are not paired trials.
