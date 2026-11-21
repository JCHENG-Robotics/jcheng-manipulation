# Paired Trial Analysis

- Generated: `2026-11-21T06:58:57.988283+00:00`
- Seed: 7; bootstrap draws: 10000
- Complete pairs: 24; input records: 48; invalid: 0

## Method summary

| Method | n | Error mean (mm) | Error SD (mm) | Success rate |
|---|---:|---:|---:|---:|
| baseline | 24 | 9.075 | 17.669 | 37.5% |
| intervention | 24 | 13.295 | 2.668 | 100.0% |

## Paired continuous error

- Pairs with both errors: 9
- Mean paired difference (intervention - baseline): 2.116 mm
- Mean improvement (baseline - intervention): -2.116 mm
- 95% CI of the difference: [-9.920, 9.316] mm
- Effect size (dz): 0.121
- Sign test: 1 wins / 8 losses / 0 ties; p = 0.0391

## Paired success

- Baseline: 37.5%
- Intervention: 100.0%
- Discordant: baseline-only 0; intervention-only 15
- McNemar exact p: 6.10e-05

## Randomization and order

- Baseline first: 12; intervention first: 12
- Order difference (intervention-first minus baseline-first): 8.687 mm
- observational; not a randomized crossover of time

## Failure codes

| Method | Codes |
|---|---|
| baseline | MISS=14, NO_GRIP=1 |
| intervention | - |

## Limitations

- Analysis is only as good as the recorded fixture and measurement.
- The exact sign test ignores effect size; report it with the mean difference and interval.
- McNemar tests paired success only; it does not explain a failure.
- Order-effect estimates are observational, not a randomized crossover of time.
- A fixture is assumed geometrically identical across its pairs.
