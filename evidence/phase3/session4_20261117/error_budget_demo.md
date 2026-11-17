# Error Budget — synthetic_error_budget_demo

- Generated: `2026-11-17T12:11:52.776157+00:00`
- Task: synthetic_validation
- Status: **COMPLETE**
- Samples: 10000 (seed 7)
- Tolerance basis: synthetic only

## translation_mm

| Source | Category | Kind | Value (mm) | Basis | Contribution | Evidence |
|---|---|---|---:|---|---:|---|
| `calibration_translation` | calibration | random | 2.000 | std | 74.9% | synthetic |
| `perception_translation` | perception | random | 1.000 | std | 18.7% | synthetic |
| `repeatability_translation` | repeatability | random | 0.500 | std | 4.7% | synthetic |
| `guard_translation` | other | bounded | 0.300 | max | 1.7% | synthetic |

- RSS: **2.311 mm**
- Worst case: **3.800 mm**
- Tolerance: 4.000 mm; RSS pass **True**; worst-case pass **True**; margin 1.689 mm
- 95% interval: [0.503, 4.674] mm (Monte Carlo, 10000 draws, seed 7).
- Dominant source: `calibration_translation`

## rotation_deg

| Source | Category | Kind | Value (deg) | Basis | Contribution | Evidence |
|---|---|---|---:|---|---:|---|
| `calibration_rotation` | calibration | random | 0.400 | std | 80.0% | synthetic |
| `perception_rotation` | perception | random | 0.200 | std | 20.0% | synthetic |

- RSS: **0.447 deg**
- Worst case: **0.600 deg**
- Tolerance: 1.000 deg; RSS pass **True**; worst-case pass **True**; margin 0.553 deg
- 95% interval: [0.063, 0.923] deg (Monte Carlo, 10000 draws, seed 7).
- Dominant source: `calibration_rotation`

## Limitations

- Synthetic input; not a hardware error budget.
- RSS assumes independent sources; correlated sources would be over-counted.
- Worst case is the linear sum of measured magnitudes and is conservative.
- Unmeasured sources are excluded from the totals; a partial budget is not a validated task error.
- A Monte Carlo interval is only reported when every random/bounded source has a distribution.
- The tolerance is a provisional gate unless a physical tolerance experiment has been recorded.
