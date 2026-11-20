# Error Budget

> Status: **Framework implemented and demo-validated; project budget complete
> (2026-11-17, Session 4; updated 2026-11-19 and 2026-11-20).** The tolerance
> is no longer provisional: a 46-grasp tolerance grid measured the opening-axis
> envelope at **[-6, +12] mm** (centre +3 mm, ±9 mm about the true cube centre)
> with `x` and `z` passing at ±9 mm. The conservative worst-side value,
> **6 mm**, replaces the 5 mm gate; the 2° rotation gate is still provisional.
> Calibration, perception, repeatability, backlash, **grasp execution,
> gripper placement and object-fixture pose** are now all measured. The
> 2026-11-20 on-site session added the last three sources: execution
> **5.386 mm / 1.495° max** (20 grasps, commanded vs bus-servo registers),
> placement **11.514 mm max / 3.080 mm mean** (20 put-backs) and fixture
> **14.815 mm radial max / 5.057 mm RMS** (20 tape placements).
>
> The component budget is **complete but fails the bound gates**: translation
> **RSS 13.325 mm** (worst case 24.014 mm) vs the 6 mm tolerance, rotation
> **RSS 1.858°** (worst 3.322°) vs the 2° gate. The dominant translation term
> is `gripper_placement` (74.7 % of the RSS); the dominant rotation term is
> `grasp_execution_rotation` (64.7 %). The empirical task-level placement is
> still **3.080 mm mean** (RMS 4.082 mm) over 20 corrected put-backs, inside
> 6 mm; the tail (11.514 mm) is what pushes the bound budget over. The
> fixture-inclusive aggregate (19.926 mm RSS) is reported separately because
> every trial re-measures the cube, so the fixture term is a nuisance in this
> pipeline.

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

## Budget history

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

### 2026-11-20 completion of the last three sources

The 2026-11-20 on-site session measured the three sources that were still
missing. The cube stayed on a radial yellow tape strip; the operator lifted
and re-placed it 20 times for the fixture term, then the arm performed 20
pick–lift–put-backs with the validated correction.

| Source | Value | Basis | Contribution |
|---|---:|---|---:|
| `grasp_execution` | 5.386 mm | hardware max, 20 grasps | 16.3 % |
| `grasp_execution_rotation` | 1.495° | hardware max, 20 grasps | 64.7 % |
| `gripper_placement` | 11.514 mm | hardware max, 20 put-backs | 74.7 % |
| `object_fixture_pose` | 14.815 mm | hardware radial max, 20 placements | fixture config only |

Execution is the commanded joint vector at the grasp pose versus the bus-servo
registers read back at that same pose, mapped through the same KDL model; the
pulse-difference construction cancels a constant servo offset. Placement is
the segmented landing point minus the pre-grasp measurement. The fixture term
is the spread of 20 operator placements on the tape (three off-tape
placements were excluded and kept as discarded evidence).

The **pipeline** aggregate omits `object_fixture_pose` because every trial
re-measures the cube at the view pose; the **fixture-inclusive** aggregate is
a separate configuration. Both are complete:

| Aggregate | Translation RSS | Translation worst | Rotation RSS | Rotation worst |
|---|---:|---:|---:|---:|
| Pipeline | **13.325 mm** (fail) | **24.014 mm** (fail) | **1.858°** (pass) | **3.322°** (fail) |
| Fixture-inclusive | **19.926 mm** (fail) | **38.829 mm** (fail) | 1.858° (pass) | 3.322° (fail) |

Evidence:
[`evidence/phase3/onsite_20261120/`](../../evidence/phase3/onsite_20261120/README.md),
configs
[`error_budget_onsite_20261120.json`](../../ros2_ws/src/armpi_analysis/config/error_budget_onsite_20261120.json)
and
[`error_budget_fixture_20261120.json`](../../ros2_ws/src/armpi_analysis/config/error_budget_fixture_20261120.json).

The dominant translation term is now `gripper_placement` (74.7 %), followed by
`grasp_execution` (16.3 %); the dominant rotation term is
`grasp_execution_rotation` (64.7 %). The hand–eye calibration is no longer the
dominant translation term (6.1 %).

The empirical task-level placement is **3.080 mm mean** (RMS 4.082 mm, max
11.514 mm) over the 20 corrected put-backs, inside the 6 mm tolerance. The
tail event is one put-back that shifted the cube 11.0 mm in −x and 3.4 mm in
−y while still holding and landing validly. The component RSS (13.3 mm) and
the empirical mean (3.1 mm) answer different questions: the components are
maxima treated as independent bounds, the mean is the measured corrected
outcome, and the execution offset is already absorbed by the constant
correction. Neither statistic is a confidence bound.

The translation tolerance is now the measured 6 mm gripper-envelope value and
the completed bound budget fails it (13.325 mm RSS); the empirical mean
(3.080 mm) passes. The 2° rotation tolerance is still the existing protocol
gate, not a measured physical grasp tolerance; the completed bound budget
passes it on RSS (1.858°) but fails it on the linear worst case (3.322°). A
direct rotation-tolerance experiment is still required before either rotation
statement can support a reliability claim.

## Commands

```bash
# Aggregate the checked-in partial baseline
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.error_budget \
  --input ros2_ws/src/armpi_analysis/config/error_budget_baseline.json \
  --json-out evidence/phase3/session4_20261117/error_budget_baseline_report.json \
  --markdown-out evidence/phase3/session4_20261117/error_budget_baseline_report.md \
  --svg-out evidence/phase3/session4_20261117/error_budget_baseline_report.svg

# 2026-11-20 completed budgets (pipeline and fixture-inclusive)
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.error_budget \
  --input ros2_ws/src/armpi_analysis/config/error_budget_onsite_20261120.json \
  --json-out evidence/phase3/onsite_20261120/error_budget.json \
  --markdown-out evidence/phase3/onsite_20261120/error_budget.md \
  --svg-out evidence/phase3/onsite_20261120/error_budget.svg

PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.error_budget \
  --input ros2_ws/src/armpi_analysis/config/error_budget_fixture_20261120.json \
  --json-out evidence/phase3/onsite_20261120/error_budget_fixture.json \
  --markdown-out evidence/phase3/onsite_20261120/error_budget_fixture.md \
  --svg-out evidence/phase3/onsite_20261120/error_budget_fixture.svg

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

- RSS assumes independent sources. Calibration, perception, execution,
  placement and the fixture may share errors; the report does not model
  correlation. The execution offset is systematic and the constant correction
  was fitted with it present, so the component RSS is not additive to the
  corrected task outcome.
- Placement is a task-level landing displacement, not a pure release term; it
  overlaps execution and the residual model correction.
- The measured values mix held-out maxima, semi-synthetic maxima and hardware
  maxima. They are bounds, not a common statistical basis.
- The object-fixture term is a nuisance in this pipeline because every trial
  re-measures the cube; the fixture-inclusive aggregate is reported separately
  and is not the pipeline's task error.
- A completed bound budget is not a measured task error either; the empirical
  task-level mean and the bound RSS answer different questions.
- The envelope boundary between 6 and 12 mm in `y` is bracketed, not resolved
  (two trials per point); the 6 mm budget value is deliberately conservative.
- The measurement session's PnP is tag-relative; a tag rotation bias would
  enter the reconstructed positions as a common shift and cancel in the spread,
  so the repeatability/backlash values are relative, not absolute.
- The cube's yaw is not measured. In the 2026-11-20 trials the release bias
  changed after each human re-placement (+4.3 to +6.7 mm in y), so the
  placement term includes an unmodelled orientation effect.
- The rotation tolerance is still a provisional gate, not a measured physical
  tolerance.
- The Monte Carlo interval is a sensitivity tool, not a physical confidence
  interval.
