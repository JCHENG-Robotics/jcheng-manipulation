# Phase 3 — Error Budget, Paired Trials and Host Benchmark

> Status: **Implemented; updated 2026-11-20.** The 2026-11-19 on-site session
> completed the first physical paired trial (24 pairs: baseline 12/24 vs
> intervention 24/24, McNemar p = 0.00049) and measured the error-budget motion
> and tolerance terms: repeatability 0.449 mm RMS / 1.017 mm max, backlash
> 1.117 mm, and a 46-grasp tolerance envelope of ±9 mm (conservative 6 mm).
> The budget remains partial in grasp execution, gripper placement and fixture
> pose, and the 2° rotation gate is still provisional.

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
| Partial project budget | **Reported, partial** | Measured hand-eye + fiducial + repeatability + backlash + tolerance terms; translation RSS 3.995 mm passes the 6 mm tolerance, worst 7.114 mm fails; rotation 1.103° / 1.827° passes the provisional 2° gate; execution, gripper placement and fixture pose remain required and unmeasured |
| Paired-trial recorder | **Implemented, synthetic + hardware validated** | append-only JSONL; seeded within-pair randomization; exact paired tests; first hardware use 2026-11-19 |
| Host/DDS benchmark | **Implemented, hardware-measured (read-only)** | laptop + Pi host, robot-container and cross-host ROS topics, ping; no command sent |
| Pure-Python CI | **Added** | flake8 + pytest for the analysis package, measurement protocol and phase-4 tools |
| Real paired grasp trials | **Complete (2026-11-19)** | 24 pairs / 48 trials: baseline 12/24 vs intervention 24/24, McNemar p = 0.00049; paired placement improvement 2.19 mm (p = 0.0063); four deviations recorded |
| Gripper tolerance envelope | **Measured (2026-11-19)** | 46 grasps; opening axis [-6, +12] mm (±9 mm about +3 mm), `x`/`z` pass at ±9 mm; conservative budget tolerance 6 mm |

Evidence: [`evidence/phase3/session4_20261117/`](../../evidence/phase3/session4_20261117/)
and the on-site session [`evidence/phase3/onsite_20261119/`](../../evidence/phase3/onsite_20261119/README.md).
The earlier unpaired real-grasp evidence is in
[`evidence/phase2/onsite_20261119/`](../../evidence/phase2/onsite_20261119/).

## What Session 4 does not claim

- The error budget is **not complete**; the translation tolerance is now the
  measured 6 mm envelope, but the 2° rotation gate remains provisional.
- The paired-trial analysis was validated on synthetic data in Session 4; the
  2026-11-19 physical trial is its first hardware use.
- The benchmark is a single read-only snapshot; it is not a load sweep or a
  real-time guarantee.
- Session 4 itself performed no robot motion, grasp, calibration capture or
  vendor demo; the 2026-11-19 on-site session did, including the paired trial.
