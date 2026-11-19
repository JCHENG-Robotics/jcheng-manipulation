# Low-configuration model selection — 2026-11-19

> Status: **PASS (negative result).** The planned "joint-offset calibration
> over the low configuration" was attempted and **not supported**. Three
> configuration-dependent error models all fit the 10-pose tag survey to
> 1.4–2.3 mm RMS, but every one of them predicts a grasp correction
> **10.6–15.0 mm away** from the correction that the 2026-11-19 grasps actually
> used and passed with. The supported model remains a **constant base-frame
> offset**, bounded to the tested envelope.

## 1. Question

The 2026-11-19 survey measured the low-configuration error as one constant
vector, `(+11.6, +4.0, +8.5) mm` against the table-down hand-eye, with a
0.4–1.0 mm per-axis spread. A constant is a measurement, not a model: it says
nothing about configurations that were not surveyed, and it cannot say whether
the error would rotate with a yawed grasp or change with the view pose. The
roadmap's Milestone E named a **joint-offset calibration over the low
configuration** as the proper fix. This note tests that hypothesis against
three physical models.

## 2. Models

All models are applied to the `link4` pose. The camera is mounted on `link4`
and the TCP hangs below it, so any `link4` error moves both by the same rigid
transform; this is what makes tag observations informative about the TCP.

| Model | Parameters | Interpretation |
|---|---|---|
| `none` | 0 | current URDF + hand-eye, no correction |
| `link4_translation` | 3 | a fixed translation `dt` in the `link4` frame: `d(q) = R_link4(q) @ dt` |
| `link4_se3` | 6 | a fixed translation and small rotation in the `link4` frame |
| `joint_offsets4` | 4 | angle offsets `dq1..dq4` on the measured joints: `d(q) = FK(q + dq) - FK(q)` |

The tag is a fixed point in `base_link`. The common tag point is a free
nuisance parameter, so each model is fitted by minimising the spread of its
corrected predictions (the profile optimum over the nuisance). A model that
explains the data makes all ten poses agree on one point.

Data: `dataset_low_tag_20261119.json`, 2 reference poses + 8 low-family poses,
hand-eye `calibration_table_down_20261118.json`.

## 3. Fit quality

| Model | Params | Residual RMS (mm) | Residual max (mm) | LOO RMS (mm) | LOO max (mm) |
|---|---:|---:|---:|---:|---:|
| `none` | 0 | 6.096 | 12.379 | 6.774 | 13.755 |
| `link4_translation` | 3 | 2.280 | 3.565 | 2.740 | 4.183 |
| `link4_se3` | 6 | 1.438 | 2.598 | 1.833 | 3.249 |
| `joint_offsets4` | 4 | 1.663 | 2.766 | 2.278 | 4.062 |

Every correction model beats `none`; the most flexible one (`link4_se3`) fits
best. On tag consistency alone, the survey **cannot choose** between the
models: the whole difference between them is smaller than the 2.4 mm residual
of a single constant.

Identifiability is unequal:

- `link4_translation` is uniquely identified — 12 restarts from random
  parameters all converge to `(-8.1, +0.4, +44.0) mm`.
- `joint_offsets4` has a **flat direction in `dq1`**: costs identical to
  1e-12 over `dq1 = 0.02 … 0.13 rad`, so `joint1`'s offset is not observable
  from a fixed tag. `dq2 = -0.053`, `dq3 = +0.046`, `dq4 = +0.073` rad are
  pinned.
- The fitted `link4` translation is dominated by `z` (`+44 mm`), which is a
  lumped parameter, not a claim about the URDF geometry.

## 4. Discriminating test: the grasp-validated correction

The tag survey alone is ambiguous, but the grasps are not. Four supervised runs
(2026-11-19) applied the constant `(+13.7, +3.9, +8.9) mm` at their own view
and grasp configurations and **gripped**: the low-configuration validation
(release **1.3 mm** from the pre-grasp measurement), the quick replay, the
far-cube grasp (release 6.8 mm) and the search-and-return grasp (release
1.1 mm). A correct model must predict the correction at *those* configurations.

The runs' view and grasp joints are recovered from the bus-servo registers
(500 = zero, 0.0041888 rad/pulse; `joint3` is inverted). This reconstruction
reproduces the survey's `R_ref1` measured joints exactly. The table below
reports `e_model = d(view) - d(grasp)`, the vector the pipeline adds to the
view measurement, against the applied `(+13.7, +3.9, +8.9) mm`.

| Model | Run | Model correction (mm) | Difference from applied (mm) | Norm |
|---|---|---|---:|---:|
| `link4_translation` | low-config validation | (+5.48, −0.14, +3.59) | (−8.22, −4.04, −5.31) | **10.58** |
| `link4_translation` | quick replay | (+5.48, −0.14, +3.59) | (−8.22, −4.04, −5.31) | **10.58** |
| `link4_translation` | far cube | (+3.69, −0.19, +2.57) | (−10.01, −4.09, −6.33) | **12.53** |
| `link4_translation` | search and return | (+4.51, −0.01, +3.03) | (−9.19, −3.91, −5.87) | **11.58** |
| `link4_se3` | low-config validation | (+1.70, −0.03, +3.03) | (−12.00, −3.93, −5.87) | **13.92** |
| `link4_se3` | far cube | (+1.09, −0.10, +2.11) | (−12.61, −4.00, −6.79) | **14.87** |
| `joint_offsets4` | low-config validation | (+1.39, −3.54, +5.51) | (−12.31, −7.44, −3.39) | **14.78** |
| `joint_offsets4` | far cube | (+1.64, −4.10, +5.14) | (−12.06, −8.00, −3.76) | **14.96** |

Every model misses the x component by 8–12.6 mm. The validation run's landing
measured the x error directly at **1.3 mm**, so the applied x correction was
right to about a millimetre. All three configuration-dependent models are
therefore rejected at the configurations the robot actually runs at.

The reason is visible in the fit: the 44 mm lumped `link4` translation makes the
correction sensitive to the link4 orientation. The survey's reference pair sits
at measured `j4 = −1.583` with `j2 = 0.260`, `j3 = −1.202`; the runs' view pose
measured `j4 = −1.663`, `j2 = 0.230`, `j3 = −1.223`. Over that small pose
difference the fitted model expects the correction to fall from 13.7 to
≈ 5.5 mm — 3.6 mm of the change from `j4` alone (45 mm/rad) and the rest from
`j2`/`j3` — and the grasps show it does not.

## 5. Conclusion

- **Joint offsets are not supported.** They fit the tag survey no better than a
  constant, `dq1` is unidentifiable, and their TCP prediction is 12–15 mm from
  the validated correction.
- **A fixed `link4`-frame offset is not supported either.** It is the closest
  physical model and uniquely identified, but its extrapolation across 0.08 rad
  of `j4` is contradicted by four passing grasps.
- **The supported model is a constant base-frame offset**, valid over the tested
  envelope: tool-down grasps with `j1` within ±0.06 rad of 0 and the view pose
  at `j4 = −1.58 … −1.66` rad. The correction is reproducible to ~1–2 mm there.
- This is a **bounded empirical model**, not a physical calibration. It does not
  claim what happens for a yawed grasp (`j1 ≈ 0.5 rad`) or a different view
  pitch. Both would be needed before the envelope can be widened.

The roadmap item "joint-offset calibration over the low configuration" is
therefore closed with a negative result: measuring it was worth doing, and it
removed a plausible but wrong hypothesis instead of adding an unjustified model.

## 6. What would change the conclusion

A **yawed grasp** is the clean discriminator between a base-frame constant and
a `link4`-frame offset. Using the fitted `link4_translation` model at the
runs' view pose and a yawed grasp from the same family, the model predicts
`(+8.5, −6.3, +4.6) mm` at `j1 = 0.3 rad`, `(+11.1, −11.6, +4.6) mm` at
`0.6 rad` and `(+15.2, −15.8, +4.6) mm` at `0.9 rad`, against the constant
`(+13.7, +3.9, +8.9) mm` — differences of 12, 16 and 20 mm. The grip margin is
about ±4 mm, so one such grasp with the landing measured in the same frame is
decisive: it either widens the envelope or bounds the yaw dependence.

## 7. Reproduction

```bash
# on the laptop, Jazzy workspace sourced
python3 tools/phase2/low_config_model_fit.py \
  --samples evidence/calibration/20261119/low_config_error/dataset_low_tag_20261119.json \
  --calibration evidence/calibration/20261118/table_down/calibration_table_down_20261118.json \
  --json-out evidence/calibration/20261119/low_config_error/report_low_config_model_fit.json \
  --markdown-out evidence/calibration/20261119/low_config_error/report_low_config_model_fit.md

# correction implied by a fitted model at two configurations (note: use `=` for
# negative joint lists, argparse treats a leading `-` as an option)
python3 tools/phase2/low_config_correction.py \
  --report evidence/calibration/20261119/low_config_error/report_low_config_model_fit.json \
  --model link4_translation \
  --view-joints=0,0.2574,-1.18773,-1.70,-0.01676 \
  --grasp-joints=-0.015,-0.6618,-1.8012,-0.34,0

# tests (included in the pure-Python CI)
python3 -m pytest -q tools/phase2/tests/test_low_config_model_fit.py
```

## 8. Files

| File | Content |
|---|---|
| `report_low_config_model_fit.json` | every model: parameters, cost, AIC, per-pose residuals, LOO, implied correction |
| `report_low_config_model_fit.md` | the comparison table alone |
| `../../../../tools/phase2/low_config_model_fit.py` | fit tool (numpy only) |
| `../../../../tools/phase2/low_config_correction.py` | model correction at a given view/grasp pair |
| `../../../../tools/phase2/tests/test_low_config_model_fit.py` | synthetic recovery, sign and identifiability tests |
