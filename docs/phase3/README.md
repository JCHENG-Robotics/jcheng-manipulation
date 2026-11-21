# Phase 3 — Error Budget, Paired Trials and Host Benchmark

> Status: **Implemented; updated 2026-11-21.** The 2026-11-19 on-site session
> completed the first physical paired trial (24 pairs: baseline 12/24 vs
> intervention 24/24, McNemar p = 0.00049) and measured the error-budget motion
> and tolerance terms: repeatability 0.449 mm RMS / 1.017 mm max, backlash
> 1.117 mm, and a 46-grasp tolerance envelope of ±9 mm (conservative 6 mm).
> The 2026-11-20 session measured the last three sources: grasp execution
> 5.386 mm / 1.495° max (20 grasps), gripper placement 11.514 mm max /
> 3.080 mm mean (20 put-backs) and object-fixture pose 14.815 mm radial max
> (20 tape placements). The component budget is **complete** but **fails** the
> 6 mm / 2° bound gates (translation RSS 13.325 mm, rotation RSS 1.858°); the
> empirical placement mean (3.080 mm) is inside 6 mm. The 2° rotation gate is
> still provisional. The 2026-11-21 random-placement pre-registration
> supersedes the fixed 24-mark fan before data; the random session has not
> been run.

Session 4 of the remote work plan covers R11 (error-budget framework), R12
(paired-trial recorder), R13 (distributed host/DDS benchmark) and R15
(CI/documentation closure). The code lives in the dependency-free
`armpi_analysis` package and `tools/phase4/`.

| Document | Purpose |
|---|---|
| [ERROR_BUDGET.md](ERROR_BUDGET.md) | JSON source schema, RSS/worst-case aggregation, Monte Carlo interval, completed 2026-11-20 budget and limitations |
| [PAIRED_TRIALS.md](PAIRED_TRIALS.md) | Matched-pair record schema, seeded randomization, bootstrap/sign/McNemar analysis, synthetic validation |
| [SITE_DIVERSITY_DESIGN.md](SITE_DIVERSITY_DESIGN.md) | Superseded before data (2026-11-21): fixed 24-mark fan, one floor site per pair, locate + closed-loop base approach |
| [RANDOM_SITE_DESIGN.md](RANDOM_SITE_DESIGN.md) | Pre-registered 2026-11-21 random-placement 24-pair design: operator-random points in the sensor-covered region, wrist/yaw/front-camera locator chain, prior-envelope guard |
| [HOST_BENCHMARK.md](HOST_BENCHMARK.md) | Read-only host metrics, DDS topic rate/jitter/age, laptop-vs-Pi results, camera-FPS correction |
| [../phase2/MEASUREMENT_PROTOCOL.md](../phase2/MEASUREMENT_PROTOCOL.md) | Physical schedules that will feed the budget and trials |

## State

| Item | State | Note |
|---|---|---|
| Error-budget framework | **Implemented, demo-validated** | `armpi_analysis/error_budget.py`; measured/unmeasured sources stay separate; RSS, worst case, dominance, Monte Carlo interval, Markdown/JSON report and SVG chart |
| Project budget | **Complete, bound gates fail** | All sources measured: hand-eye 3.29 mm, fiducial 1.69 mm, repeatability 1.017 mm, backlash 1.117 mm, execution 5.386 mm / 1.495°, placement 11.514 mm, fixture 14.815 mm. Pipeline translation RSS 13.325 mm (worst 24.014) vs 6 mm; rotation RSS 1.858° (worst 3.322°) vs the provisional 2°; empirical placement mean 3.080 mm is inside 6 mm. Fixture-inclusive RSS 19.926 mm |
| Paired-trial recorder | **Implemented, synthetic + hardware validated** | append-only JSONL; seeded within-pair randomization; exact paired tests; first hardware use 2026-11-19 |
| Host/DDS benchmark | **Implemented, hardware-measured (read-only)** | laptop + Pi host, robot-container and cross-host ROS topics, ping; no command sent |
| Pure-Python CI | **Added** | flake8 + pytest for the analysis package, measurement protocol and phase-4 tools |
| Real paired grasp trials | **Complete (2026-11-19)** | 24 pairs / 48 trials: baseline 12/24 vs intervention 24/24, McNemar p = 0.00049; paired placement improvement 2.19 mm (p = 0.0063); four deviations recorded |
| 24-site fixed-fan follow-up | **Superseded before data (2026-11-21)** | 24 pre-marked sites, layout/plan (seed 17) and runner were prepared but never run; replaced by the random-placement design |
| Random-placement 24-pair trial | **Pre-registered (2026-11-21); in progress (8/24 pairs)** | 24 pairs, operator places the cube by eye (no measuring); the r 0.25-0.55 m / bearing -20..+25 deg envelope is a guide; direct grasp when already in the 0.226-0.234 m band, otherwise wrist -> front camera -> yaw sweep locate + base approach; after a held grip the robot carries the cube back to the start and releases it there between the two trials. 16 new-flow trials (8 complete pairs) recorded; a clipped start top view is accepted as `landing_partial`; a locate miss with the cube outside the segmenter's 3-D region now prompts an operator move-and-retry. One old-flow attempt discarded (D1) |
| Gripper tolerance envelope | **Measured (2026-11-19)** | 46 grasps; opening axis [-6, +12] mm (±9 mm about +3 mm), `x`/`z` pass at ±9 mm; conservative budget tolerance 6 mm |
| Execution, placement, fixture | **Measured (2026-11-20)** | 20/20 put-backs; execution 4.220 mm / 1.069° mean (5.386 mm / 1.495° max); placement 3.080 mm mean, 11.514 mm max; fixture 5.057 mm RMS, 14.815 mm radial max; three off-tape and two clipped-landing deviations kept as discarded evidence |
| Rotation tolerance / visual displacement | **Runners added (2026-11-20), not run** | `rotation_tolerance_run.py` sweeps relative cube yaw on `joint5`; `visual_displacement_run.py` uses a seeded steel-ruler grid with a nuisance ruler yaw. Both have unit tests and dry runs, no hardware data yet |

Evidence: [`evidence/phase3/session4_20261117/`](../../evidence/phase3/session4_20261117/),
the on-site session [`evidence/phase3/onsite_20261119/`](../../evidence/phase3/onsite_20261119/README.md),
the 2026-11-20 completion [`evidence/phase3/onsite_20261120/`](../../evidence/phase3/onsite_20261120/README.md),
the superseded fixed-fan layout/plan
[`evidence/phase3/onsite_20261120_sites/`](../../evidence/phase3/onsite_20261120_sites/README.md)
and the random-placement pre-registration
[`evidence/phase3/onsite_20261121_random/`](../../evidence/phase3/onsite_20261121_random/README.md).
The earlier unpaired real-grasp evidence is in
[`evidence/phase2/onsite_20261119/`](../../evidence/phase2/onsite_20261119/).

## What Session 4 does not claim

- The error budget is **complete but not passing** at the bound level: every
  source is a measured maximum, the pipeline translation RSS is 13.325 mm
  against the 6 mm tolerance, and the rotation worst case (3.322°) exceeds the
  provisional 2° gate. The empirical placement mean (3.080 mm) is inside
  6 mm. The open gaps are the tail event, the unmeasured cube yaw and the
  provisional 2° rotation gate.
- The paired-trial analysis was validated on synthetic data in Session 4; the
  2026-11-19 physical trial is its first hardware use.
- The benchmark is a single read-only snapshot; it is not a load sweep or a
  real-time guarantee.
- Session 4 itself performed no robot motion, grasp, calibration capture or
  vendor demo; the 2026-11-19 on-site session did, including the paired trial.
