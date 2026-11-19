# Paired Trial Analysis

- Generated: `2026-11-19T09:32:38.608337+00:00`
- Seed: 7; bootstrap draws: 10000
- Complete pairs: 24; input records: 48; invalid: 0

## Method summary

| Method | n | Error mean (mm) | Error SD (mm) | Success rate |
|---|---:|---:|---:|---:|
| baseline | 24 | 4.965 | 2.873 | 50.0% |
| intervention | 24 | 3.314 | 1.478 | 100.0% |

## Paired continuous error

- Pairs with both errors: 12
- Mean paired difference (intervention - baseline): -2.188 mm
- Mean improvement (baseline - intervention): 2.188 mm
- 95% CI of the difference: [-3.955, -0.830] mm
- Effect size (dz): -0.746
- Sign test: 11 wins / 1 losses / 0 ties; p = 0.0063

## Paired success

- Baseline: 50.0%
- Intervention: 100.0%
- Discordant: baseline-only 0; intervention-only 12
- McNemar exact p: 0.0005

## Randomization and order

- Baseline first: 9; intervention first: 15
- Order difference (intervention-first minus baseline-first): 0.427 mm
- observational; not a randomized crossover of time

## Failure codes

| Method | Codes |
|---|---|
| baseline | MISS=12 |
| intervention | - |

## Limitations

- Analysis is only as good as the recorded fixture and measurement.
- The exact sign test ignores effect size; report it with the mean difference and interval.
- McNemar tests paired success only; it does not explain a failure.
- Order-effect estimates are observational, not a randomized crossover of time.
- A fixture is assumed geometrically identical across its pairs.
