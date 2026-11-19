# Paired hardware trial — 2026-11-19 on-site session

> Status: **PASS. 24 complete pairs, 48 trials, no invalid records.**
> Pre-registered design: [`docs/phase3/PAIRED_TRIAL_DESIGN.md`](../../../docs/phase3/PAIRED_TRIAL_DESIGN.md).
> Randomization plan committed before data collection (`4871e70`).
>
> **Primary (binary):** baseline **12/24 = 50.0 %**, intervention **24/24 =
> 100 %**; all 12 discordant pairs favour the intervention (exact McNemar
> **p = 0.00049**).
> **Secondary (continuous, 12 complete pairs):** the intervention places the
> cube **2.19 mm** closer to the pre-grasp measurement on average
> (95 % CI [−3.96, −0.83] mm, Cohen's `dz = −0.75`, sign test 11 wins / 1 loss,
> **p = 0.0063**).

## 1. What was compared

| Method | Target sent to the joint fit |
|---|---|
| `baseline` | `C_view` — the cube top-face centroid measured at the view pose, no correction |
| `intervention` | `C_view + (+13.7, +3.9, +8.9) mm` — the grasp-validated constant low-configuration correction |

Both methods use the same measured joints, the same deep grasp family
(`joint2 = −0.6618`, `joint3 = −1.8012`, `joint5 = 0`, `joint1`/`joint4` fitted
to the target), the same approach, closing and release sequence. The order of
the two methods inside a pair, and the pair order, come from the committed plan.

## 2. Results

| Method | n | Success | Error mean (mm) | SD (mm) | Median (mm) | IQR (mm) | Min–max (mm) |
|---|---:|---:|---:|---:|---:|---|---|
| baseline | 24 | **12 (50.0 %)** | 4.965 | 2.873 | 4.06 | 3.28–5.24 | 2.52–13.15 |
| intervention | 24 | **24 (100 %)** | **3.314** | **1.478** | **2.92** | 2.26–4.69 | 0.70–6.33 |

Error is the xy distance between the release landing and the pre-grasp
measurement, i.e. the placement residual of the full pick-and-place. A failed
trial has no landing and no continuous value.

| Test | Result |
|---|---|
| Paired success (exact McNemar) | baseline-only 0, intervention-only 12, **p = 0.00049** |
| Paired error (n = 12, exact sign test) | 11 intervention wins, 1 loss, 0 ties, **p = 0.0063** |
| Mean paired improvement | **2.188 mm**, 95 % bootstrap CI **[−3.955, −0.830]** |
| Effect size | Cohen's `dz = −0.746` |
| Order effect (observational) | baseline-first 9 pairs, intervention-first 15; order difference 0.43 mm |
| Failure codes | baseline `MISS` × 12; intervention none |

Systematic landing bias (mean landing − pre-grasp measurement):

| Method | bias x (mm) | bias y (mm) | SD x (mm) | SD y (mm) |
|---|---:|---:|---:|---:|
| baseline | −3.73 | −2.84 | 2.45 | 2.06 |
| intervention | **−2.03** | **−1.02** | 2.18 | 1.78 |

The intervention roughly halves the systematic bias but does not remove it: a
residual ≈ −2 mm in `x` remains, consistent with the ~3 mm `y` disagreement
between the constant-correction survey and the model fits
([`model_selection.md`](../../calibration/20261119/low_config_error/model_selection.md)).

## 3. Why the baseline still grips half the time

The correction is a model error, so the physical TCP is at `TCP_model − e`:
with the baseline target the true TCP lands **13.7 mm closer to the base and
8.9 mm lower** than the measured top-face point. The pads close **behind and
inside** the cube rather than centred on it, so whether the cube is caught
depends on the edge geometry — it was caught in 12 of 24 trials and pushed away
in the other 12. The intervention commands the target that puts the true TCP on
the measured point and grips every time.

Model-space TCP height above the measured top face (the fit's `z` residual):

| Method | mean (mm) | range (mm) |
|---|---:|---|
| baseline | 6.95 | 4.98–11.26 |
| intervention | 1.90 | −0.81–6.15 |

The intervention's model TCP is commanded closer to the cube top because the
`+8.9 mm` component is the measured model error in that direction.

## 4. Deviations (all recorded in the pre-registered design note)

| Code | What happened | Handling |
|---|---|---|
| D1 | The correction was added in millimetres to a target in metres; the first pair ran a 13.9 m target | pair discarded, target/fit guards added, plan restarted |
| D2 | The grip classifier counted a background cluster at (0.499, −0.249) m; a real grip was recorded as a miss | cube-box classifier; the trial was released manually and retained with 2.9 mm |
| D3 | A tape roll in the grasp region: p019/baseline measured the cube 59.3 mm above the floor, p023/baseline merged the cube with a neighbour; a negative `--grasp` value then crashed argparse | placement height/extent gates; p019 baseline discarded and re-run flat; `--grasp=` form |
| D4 | p003/intervention's landing selected a 21.7 mm object, not the cube | landing gate; the record was withdrawn and the single trial re-run (2.30 mm) |

Discarded evidence is kept as `discarded_unit_error_attempt.jsonl`,
`discarded_grip_classifier_attempt.jsonl`,
`discarded_protocol_violation_attempt.jsonl` and
`trials_with_p003_artifact.jsonl`.

## 5. Limits

- **One session, one cube (39.4 mm), one fixture, one battery state
  (7.03 V).** This is a feasibility-scale paired trial, not a population
  estimate.
- **Placement is controlled by a tape mark, not a jig.** The pre-trial
  measurement removes the placement from the target, so the outcome is not
  sensitive to a few millimetres of operator placement, but the pair's "same
  placement" is only as good as the mark.
- **The continuous comparison is conditional on the baseline succeeding**
  (12 of 24 pairs). It answers "when both methods grip, which places better",
  not "what is the error of a failed grasp".
- **The intervention error includes release dynamics.** 3.31 mm is a
  pick-and-place placement error, not a calibration term; it is within the
  provisional 5 mm gate but must not be read as a calibration accuracy.
- **The order effect is observational.** The plan randomizes order, but this
  is not a randomized crossover of time.

## 6. Files

| File | Content |
|---|---|
| `paired_trials_plan.json` | 24-pair plan, seed 7, fixture `floor_square_A` |
| `trials.jsonl` | 48 validated records, append-only, with per-trial joints/residuals/landing |
| `analysis.json` / `analysis.md` | the framework's full report |
| `trials_table.md` | every trial in one table |
| `container_reports/` | per-trial segmentation and action JSONs from the two containers |
| `discarded_*.jsonl`, `trials_with_p003_artifact.jsonl` | the four deviations' raw records |

## 7. Reproduction

```bash
# session (interactive; resumable; skips completed entries)
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 tools/phase3/paired_trial_session.py

# analysis
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.paired_trials \
  analyze --input evidence/phase3/onsite_20261119/trials.jsonl \
  --json-out evidence/phase3/onsite_20261119/analysis.json \
  --markdown-out evidence/phase3/onsite_20261119/analysis.md
```
