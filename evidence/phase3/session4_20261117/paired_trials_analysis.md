# Paired Trial Analysis

- Generated: `2026-11-17T12:11:52.987345+00:00`
- Seed: 7; bootstrap draws: 10000
- Complete pairs: 24; input records: 48; invalid: 0

## Method summary

| Method | n | Error mean (mm) | Error SD (mm) | Success rate |
|---|---:|---:|---:|---:|
| baseline | 24 | 2.144 | 0.585 | 75.0% |
| intervention | 24 | 1.120 | 0.667 | 95.8% |

## Paired continuous error

- Pairs with both errors: 24
- Mean paired difference (intervention - baseline): -1.025 mm
- Mean improvement (baseline - intervention): 1.025 mm
- 95% CI of the difference: [-1.095, -0.950] mm
- Effect size (dz): -5.496
- Sign test: 24 wins / 0 losses / 0 ties; p = 1.19e-07

## Paired success

- Baseline: 75.0%
- Intervention: 95.8%
- Discordant: baseline-only 1; intervention-only 6
- McNemar exact p: 0.1250

## Randomization and order

- Baseline first: 9; intervention first: 15
- Order difference (intervention-first minus baseline-first): 0.029 mm
- observational; not a randomized crossover of time

## Failure codes

| Method | Codes |
|---|---|
| baseline | MISS=6 |
| intervention | MISS=1 |

## Limitations

- Analysis is only as good as the recorded fixture and measurement.
- The exact sign test ignores effect size; report it with the mean difference and interval.
- McNemar tests paired success only; it does not explain a failure.
- Order-effect estimates are observational, not a randomized crossover of time.
- A fixture is assumed geometrically identical across its pairs.
