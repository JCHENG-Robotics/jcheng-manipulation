# Paired Trials

> Status: **Implemented and simulated-validated (2026-11-17, Session 4).**
> No physical paired grasp trial exists. The recorder and analysis are ready
> for the first on-site experiment.
>
> The Studio **G3 Experiment Lab** (2026-11-18) exposes the same pair/method
> schema and statistical formulas through immutable experiment definitions,
> append-only SQLite trials, failure-code/artifact traceability, CSV/JSON
> export and an Experiment Lab page. See
> [`armpi-debug-studio/docs/g3-experiment-lab.md`](../../armpi-debug-studio/docs/g3-experiment-lab.md).
> It has no hardware trial data either.

## Design

The experiment unit is a matched pair: the same object fixture is attempted
once with the baseline method and once with the intervention. This controls
object placement and fixture geometry, which are the dominant nuisance factors
for a low-cost arm.

Randomization happens at two levels:

1. the order of the two methods inside each pair;
2. the order in which pairs are executed.

Both are seeded and reproducible. The plan is a separate JSON file; recording
is append-only JSON Lines and refuses a pair/method that is already present or
does not match the plan.

## Record schema

| Field | Type | Rule |
|---|---|---|
| `pair_id` | string | non-empty; one baseline and one intervention per pair |
| `trial_order` | integer | 1 or 2 within a pair; unique per pair |
| `method` | string | `baseline` or `intervention` |
| `object_pose_fixture` | string | non-empty fixture identifier; must match the plan |
| `continuous_error_mm` | number or null | finite, non-negative; required for a successful grasp, may be null for a failure |
| `grasp_success` | boolean | binary outcome |
| `failure_code` | string or null | null on success; non-empty code on failure (`MISS`, `SLIP`, ...) |
| `recorded_at` | string | optional ISO-8601 timestamp |
| `notes` | string | optional free text |

The recorder adds `schema_version: 1`. Unknown extra fields are preserved.

## Commands

```bash
PKG=ros2_ws/src/armpi_analysis
export PYTHONPATH="${PKG}"

# 1. Randomized plan: 24 pairs, one fixture name per pair is also accepted
python3 -m armpi_analysis.paired_trials plan \
  --pairs 24 --seed 7 --fixtures grid_A3_center \
  --json-out evidence/phase3/session4_20261117/paired_trials_plan.json

# 2. Record one trial (append-only; plan is optional but recommended)
python3 -m armpi_analysis.paired_trials record \
  --jsonl-out /tmp/trials.jsonl \
  --plan evidence/phase3/session4_20261117/paired_trials_plan.json \
  --pair-id p001 --method baseline --error-mm 2.4 --success

# 3. Validate the file
python3 -m armpi_analysis.paired_trials validate --input /tmp/trials.jsonl

# 4. Analyze complete pairs
python3 -m armpi_analysis.paired_trials analyze --input /tmp/trials.jsonl \
  --json-out /tmp/analysis.json --markdown-out /tmp/analysis.md
```

After `colcon build`, use `ros2 run armpi_analysis paired_trials ...`.

## Analysis

With `d_i = error_intervention,i - error_baseline,i`:

- **Continuous outcome**: mean and SD of `d`, a seeded bootstrap 95% interval
  for the mean, Cohen's `dz`, and an exact two-sided sign test. A negative mean
  difference means the intervention reduced error.
- **Binary outcome**: per-method success rate, the discordant counts
  (`baseline_only`, `intervention_only`) and an exact McNemar test.
- **Order effect**: the paired difference split by which method ran first.
  This is descriptive; it is not a randomized crossover of time.
- **Data quality**: invalid records, incomplete pairs, duplicates and failure
  codes are listed rather than silently dropped.

Incomplete pairs are excluded from the paired analysis and listed in the
report. A record with a successful grasp but no continuous error is rejected.

## Synthetic validation

The checked-in simulation uses 24 pairs, a true 1.0 mm improvement, 0.5 mm
pair noise, baseline success 0.6 and intervention success gain 0.2:

```text
paired difference: -1.025 mm   (intervention - baseline)
improvement:        1.025 mm
95% bootstrap CI:  [-1.095, -0.950] mm
sign test:          24 wins / 0 losses / 0 ties, p = 1.19e-07
success:            75.0% -> 95.8%; McNemar p = 0.1250
```

The continuous path recovers the known effect. The success path is deliberately
underpowered at 24 pairs: 1 baseline-only vs 6 intervention-only discordant
pairs gives p = 0.125, which is not significant. A real trial must be sized
for the binary outcome if success rate is a primary endpoint.

Evidence:
[`evidence/phase3/session4_20261117/paired_trials_analysis.md`](../../evidence/phase3/session4_20261117/paired_trials_analysis.md)
and
[`paired_trials_analysis.json`](../../evidence/phase3/session4_20261117/paired_trials_analysis.json).

## What the first on-site trial still needs

- A verified gripper sign and open/close range.
- A physically repeatable fixture with measured placement.
- A measured continuous error for each grasp (fixture/caliper or vision
  ground truth), not only success/failure.
- A sample size justified from the pilot variance for the continuous endpoint
  and from the expected discordance rate for the binary endpoint.
- The safety layer active and the joint limits measured before repeated
  motion.

## Limitations

- The simulation draws independent noise; real pairs may be correlated.
- The exact sign test ignores effect size; report it with the interval.
- McNemar tests paired success only and does not explain a failure.
- The order-effect estimate is observational.
- A fixture is assumed geometrically identical across its pairs.
