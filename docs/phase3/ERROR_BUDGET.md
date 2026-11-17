# Error Budget

> Status: **Framework implemented and demo-validated; project budget partial
> (2026-11-17, Session 4).** The measured terms are calibration and perception
> only. Repeatability, backlash, grasp execution, gripper placement and fixture
> pose are still required but unmeasured.

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

| Metric | RSS | Worst case | Tolerance | Status |
|---|---:|---:|---:|---|
| Translation | **3.699 mm** | **4.980 mm** | 5.000 mm (provisional) | partial, both gates pass |
| Rotation | **0.682°** | **0.960°** | 2.000° (provisional) | partial, both gates pass |

Measured sources:

| Source | Value | Basis | Contribution |
|---|---:|---|---:|
| `hand_eye_translation` | 3.29 mm | held-out final base max | 79.1% |
| `fiducial_translation` | 1.69 mm | semi-synthetic max on real frames | 20.9% |
| `hand_eye_rotation` | 0.43° | held-out final base max | 39.7% |
| `fiducial_rotation` | 0.53° | semi-synthetic max on real frames | 60.3% |

Required and still unmeasured:

- arm repeatability (translation and rotation);
- backlash (translation);
- grasp execution (translation and rotation);
- gripper placement (translation);
- object-fixture pose (translation).

The budget has no 95% interval because the measured values are maxima, not
per-source distributions. The 4.980 mm worst case is only 0.020 mm below the
provisional 5 mm gate, so any unmeasured term will move the budget. This is the
main reason the report is marked **partial**, not passing.

The 5 mm / 2° tolerance is the existing protocol gate, not a measured physical
grasp tolerance. It must be replaced by the gripper-envelope experiment before
it can support a reliability claim.

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
- The tolerance is provisional and must come from the gripper-envelope
  experiment.
- The Monte Carlo interval is a sensitivity tool, not a physical confidence
  interval.
