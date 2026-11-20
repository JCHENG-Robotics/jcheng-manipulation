# Error Budget

> Status: **Framework implemented and demo-validated; project budget partial
> (2026-11-17, Session 4; updated 2026-11-19).** The tolerance is no longer
> provisional: a 46-grasp tolerance grid measured the opening-axis envelope at
> **[-6, +12] mm** (centre +3 mm, ±9 mm about the true cube centre) with `x`
> and `z` passing at ±9 mm and no failure. The conservative worst-side value,
> **6 mm**, replaces the 5 mm gate; the 2° rotation gate is still provisional. Calibration, perception,
> **repeatability and backlash** are now measured. The 2026-11-19 on-site
> session added the two motion terms with a fixed AprilTag at the grasp pose:
> repeatability **1.017 mm max** (0.449 mm radial RMS, 30 visits) and backlash
> **1.117 mm** (15 + 15 visits, joint4 ±0.1 rad). With them the translation
> budget is **RSS 3.995 mm** (pass) and **worst case 7.114 mm** (fail) against
> the measured 6 mm tolerance; rotation is **RSS 1.103°**, worst case **1.827°**
> (both pass the provisional 2° gate). A physical paired trial (24 pairs) additionally measured the
> **end-to-end placement error**: 3.31 mm mean (SD 1.48) for the corrected
> method and 4.97 mm (SD 2.87) for the uncorrected one. That is a task-level
> outcome, not a decomposition: **grasp execution, gripper placement and
> fixture pose remain required and unmeasured as separate sources.**

## Purpose

The error budget is the Phase 3 bridge between the measurement protocols and
the reliability study. It keeps every source of task error in one auditable
JSON file, combines the measured sources, exposes what is still missing, and
reports whether the current estimate fits a provisional task tolerance.

It is deliberately a **framework**, not a claimed result. A source without a
measurement stays in the file with `"status": "unmeasured"` and is excluded
from the totals. The report is marked `partial` and lists every missing
required source.

## Source schema

```json
{
  "schema_version": 1,
  "name": "fiducial_grasp_partial_20261117",
  "task": "fiducial_grasp",
  "tolerance": {"translation_mm": 5.0, "rotation_deg": 2.0},
  "tolerance_basis": "provisional protocol gates",
  "sources": [
    {
      "id": "hand_eye_translation",
      "category": "calibration",
      "metric": "translation_mm",
      "value": 3.29,
      "kind": "bounded",
      "basis": "held_out_max",
      "status": "measured",
      "distribution": null,
      "required": true,
      "evidence": "evidence/calibration/20261117/",
      "notes": "held-out final base error; treated as a bound"
    }
  ]
}
```

| Field | Meaning |
|---|---|
| `metric` | `translation_mm` or `rotation_deg`; metrics are aggregated separately |
| `category` | `calibration`, `perception`, `repeatability`, `backlash`, `execution`, `gripper`, `model`, `other` |
| `kind` | `random` (1-sigma-like), `bounded` (within a magnitude), `systematic` (fixed bias) |
| `basis` | How the number was obtained (`std`, `ci95`, `held_out_max`, `semi_synthetic_max`, ...) |
| `status` | `measured` or `unmeasured`; an unmeasured source cannot carry a value |
| `distribution` | Optional `normal` or `uniform`; required to compute a Monte Carlo interval |
| `required` | Whether the budget is complete without this source |

For a `normal` source, `sigma` may be given explicitly or inferred from
`basis: std` (`sigma = value`) or `basis: ci95` (`sigma = value / 1.96`). A
`systematic` source cannot have a distribution.

## Aggregation

For each metric:

```text
rss         = sqrt(sum(value_i^2))       # independent sources
worst_case  = sum(value_i)               # linear, conservative
```

The report also gives each source's share of the squared total, the RSS if
that source were removed, and the dominant source. It is available as JSON,
Markdown and a dependency-free SVG contribution chart (`--svg-out`). The
chart's bar length is the share of the squared total; the dominant source is
highlighted, and missing required sources are printed below each metric.

A 95% interval is computed only when every measured random/bounded source has
a distribution:

- `normal`: draw `N(0, sigma)`;
- `uniform`: draw `U(-value, +value)`;
- `systematic`: fixed value.

The interval is a seeded Monte Carlo percentile interval of the aggregate RSS.
It is not a confidence interval for a physical error, and it is deliberately
not printed when the required distributions are absent.

### Completion rule

A metric is `complete` when it has at least one measured source and no required
unmeasured source. The whole budget is `complete` only when every metric is
complete. A partial budget's totals are lower bounds for the missing
categories, not a validated task error.

## Current partial budget

Input:
[`ros2_ws/src/armpi_analysis/config/error_budget_baseline.json`](../../ros2_ws/src/armpi_analysis/config/error_budget_baseline.json).
Evidence:
[`evidence/phase3/session4_20261117/error_budget_baseline_report.md`](../../evidence/phase3/session4_20261117/error_budget_baseline_report.md)
and
[`error_budget_baseline_report.svg`](../../evidence/phase3/session4_20261117/error_budget_baseline_report.svg).

### Gripper tolerance envelope (2026-11-19)

46 grasps, two per offset point, one base axis at a time:

| Axis | Offsets | Passing |
|---|---|---|
| `x` (finger length) | 0, ±3, ±6, ±9 | all |
| `y` (opening) | 0, ±3, ±6, ±9, ±12, ±15 | −6…+12 (1/2 at −9; 0/2 at ±15 and −12) |
| `z` (height) | 0, ±3, ±6, ±9 | all |

The `y` interval `[-6, +12]` is ±9 mm about a centre 3 mm to the +y side,
which is the constant correction's own `y` residual, not a gripper asymmetry.
The physical tolerance is therefore **±9 mm** in the opening axis and ≥9 mm in
`x`/`z`; the budget uses 6 mm as the conservative worst side.

### 2026-11-17 baseline (calibration + perception only)

| Metric | RSS | Worst case | Tolerance | Status |
|---|---:|---:|---:|---|
| Translation | **3.699 mm** | **4.980 mm** | 5.000 mm (provisional) | partial, both gates pass |
| Rotation | **0.682°** | **0.960°** | 2.000° (provisional) | partial, both gates pass |

### 2026-11-19 on-site update (adds repeatability, backlash and the tolerance)

Measured sources:

| Source | Value | Basis | Contribution |
|---|---:|---|---:|
| `hand_eye_translation` | 3.29 mm | held-out final base max | 67.8% |
| `fiducial_translation` | 1.69 mm | semi-synthetic max on real frames | 17.9% |
| `arm_repeatability` | 1.017 mm | hardware max, 30 visits | 6.5% |
| `arm_backlash` | 1.117 mm | hardware hysteresis, 15 + 15 | 7.8% |
| `hand_eye_rotation` | 0.43° | held-out final base max | 15.2% |
| `fiducial_rotation` | 0.53° | semi-synthetic max on real frames | 23.1% |
| `arm_repeatability_rotation` | 0.867° | hardware max, 30 visits | 61.7% |

The updated report is
[`evidence/phase3/onsite_20261119/error_budget.md`](../../evidence/phase3/onsite_20261119/error_budget.md),
generated from
[`config/error_budget_onsite_20261119.json`](../../ros2_ws/src/armpi_analysis/config/error_budget_onsite_20261119.json).

| Metric | RSS | Worst case | Tolerance | Status |
|---|---:|---:|---:|---|
| Translation | **3.995 mm** | **7.114 mm** | **6.000 mm (measured)** | RSS passes, worst case fails |
| Rotation | **1.103°** | **1.827°** | 2.000° (provisional) | both pass |

The tolerance basis changed from the 5 mm protocol gate to the measured
gripper envelope. With the conservative 6 mm, the same conclusions hold: the
RSS fits, the worst case does not. Report:
[`evidence/phase3/onsite_20261119/error_budget_envelope.md`](../../evidence/phase3/onsite_20261119/error_budget_envelope.md).

Required and still unmeasured:

- grasp execution (translation and rotation);
- gripper placement (translation);
- object-fixture pose (translation).

The repeatability and backlash maxima come from one session at the grasp pose
and are treated as bounds, like the calibration terms. The dominant translation
term is still the hand–eye calibration (3.29 mm, 68 % of the RSS); the dominant
rotation term is now the arm's own repeatability (0.867°, 62 %).

The budget has no 95 % interval because the measured values are maxima, not
per-source distributions. With four bound-like terms the 7.114 mm worst case
exceeds the measured 6 mm tolerance while the RSS (3.995 mm) still passes;
the two statistics bracket the same terms and neither is a confidence bound.
The report stays **partial** because three required sources are unmeasured.

The translation tolerance is now the measured 6 mm gripper-envelope value;
the 2° rotation tolerance is still the existing protocol gate, not a measured
physical grasp tolerance. It must be replaced by a direct rotation experiment
before it can support a reliability claim.

## Commands

```bash
# Aggregate the checked-in partial baseline
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.error_budget \
  --input ros2_ws/src/armpi_analysis/config/error_budget_baseline.json \
  --json-out evidence/phase3/session4_20261117/error_budget_baseline_report.json \
  --markdown-out evidence/phase3/session4_20261117/error_budget_baseline_report.md \
  --svg-out evidence/phase3/session4_20261117/error_budget_baseline_report.svg

# Synthetic demo with known inputs and distributions
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.error_budget \
  --demo --json-out /tmp/error_budget_demo.json

# Emit an empty template
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.error_budget \
  --template --json-out /tmp/error_budget_template.json
```

After `colcon build`, the same entry points are available as
`ros2 run armpi_analysis error_budget ...`.

## Synthetic validation

`--demo` uses four translation sources and two rotation sources with known
values and distributions:

```text
translation RSS 2.311 mm, worst case 3.800 mm, 95% interval [0.503, 4.674] mm
rotation    RSS 0.447 deg, worst case 0.600 deg, 95% interval [0.063, 0.923] deg
```

The demo is complete by construction and both gates pass. Its interval is wide
because each source is sampled as a zero-mean contribution; that is expected
for an RSS of several independent terms.

## Limitations

- RSS assumes independent sources. Calibration, perception and the fixture may
  share errors; the report does not model correlation.
- The measured values mix held-out maxima and semi-synthetic maxima. They are
  bounds, not a common statistical basis.
- A partial budget must not be presented as a measured task error.
- The envelope boundary between 6 and 12 mm in `y` is bracketed, not resolved
  (two trials per point); the 6 mm budget value is deliberately conservative.
- The measurement session's PnP is tag-relative; a tag rotation bias would
  enter the reconstructed positions as a common shift and cancel in the spread,
  so the repeatability/backlash values are relative, not absolute.
- The tolerance is provisional and must come from the gripper-envelope
  experiment.
- The Monte Carlo interval is a sensitivity tool, not a physical confidence
  interval.
