# Low-configuration model error survey — 2026-11-19

> Status: **survey PASS and grasp-validated the same day.** A 60 mm 36h11 tag
> fixed on the floor was observed from 2 reference poses in the validated
> table-down volume and 8 poses of the low (grasp) family. The reference pair
> agrees to **1.8 mm / 1.55°** and the tag PnP reprojection is **0.12–0.49 px**
> with 35–108 px of frame margin at every pose. The model error over the whole
> low family is a **constant vector**: the per-pose spread is **0.40–0.99 mm**
> and the residual after one constant correction is at most **2.4 mm**. So a
> single local correction is sufficient — no configuration-dependent model is
> needed inside this volume.
>
> The correction was then used on a real cube: **grip, 150 mm lift and release
> 1.3 mm from the measurement**
> ([validation](../../../phase2/onsite_20261119/low_config_grasp_validation/README.md)),
> which also settles the sign conflict against the pass-5 single-tag value.
>
> The correction value depends on which hand-eye is treated as the model:
> `(+11.6, +4.0, +8.5) mm` for the table-down fit (the one the 2026-11-19
> grasps used) and `(+5.2, +1.8, +5.8) mm` for the camera-forward fit that the
> stack publishes as `camera_optical_calibrated`.

## 1. Why this survey replaced a plain re-calibration

The chessboard fits are already internally consistent (2.94 mm camera-forward,
2.7 mm table-down). The grasp error was not a bad fit: it was the model being
**extrapolated** from the calibration volume to the low configuration, where
the board cannot be framed (the camera sits 0.11–0.14 m above the floor, so
the 220 × 160 mm board does not fit in the view — a 0.20 m camera height is
needed, which forces `j2 ≈ 0`, 0.6 rad away from the grasp pose).

Instead the error was measured **relative to a fixed target** seen from a
validated reference pose and from the grasp family. Because the same physical
target is observed through the same camera in both, the view-dependent PnP
bias largely cancels and the residual is the model error between the two
configurations — which is exactly the quantity a grasp correction needs.

## 2. Setup

| Item | Value |
|---|---|
| Target | AprilTag 36h11 **ID 0**, 60 mm black square, flat on the floor |
| Target placement (predicted, camera-forward model) | `(0.289, 0.014)` m |
| Reference poses | `R_ref1` `[0, 0.2574, −1.18773, −1.5917, −0.01676]`, `R_ref2` with `j1 = +0.08` |
| Low family | `G_grasp` `[0, −0.6618, −1.8012, −0.3917, 0]`, `G_mid`, `j2 +0.05/+0.10`, `j3 +0.05`, `j4 +0.10`, `j1 ±0.06` |
| FK joints | measured bus-servo positions (median of 3 reads, start and end) |
| Approach | every pose from a pre-pose (`j1 −0.08`, `j2 +0.05`, `j3 +0.05`) |
| Capture | 4 s trajectories, 0.5 s settle, 3 warm-up frames, 3 frames/pose |
| Detection | `findChessboard`-style `solvePnP(IPPE)` + `solvePnPRefineLM` on the 4 corners |
| Battery | 6573 mV (earlier grasp sessions: 7645–7812 mV) |

`G_j4_m10` (`j4 = −0.4917`) was dropped from the set: a single tag cannot be
framed by both the reference pair and that pose (the feasible placement window
collapsed to 4 mm). `j4` is covered by `G_grasp`, `G_j4_p10` and `G_mid`.

## 3. Result

Reference truth (mean of `R_ref1`, `R_ref2`), and the error field:

| Pose | rms (px) | margin (px) | Δ from reference, table-down model (mm) |
|---|---:|---:|---|
| R_ref1 | 0.43 | 36 | 0.00 (reference) |
| R_ref2 | 0.37 | 35 | 0.00 (reference) |
| G_grasp | 0.32 | 50 | +12.77, +4.94, +9.00 |
| G_mid | 0.49 | 55 | +9.34, +4.32, +7.97 |
| G_j2_up05 | 0.16 | 100 | +12.13, +4.48, +9.07 |
| G_j2_up10 | 0.21 | 106 | +11.53, +3.60, +8.25 |
| G_j3_up05 | 0.12 | 102 | +11.67, +4.18, +8.24 |
| G_j4_p10 | 0.44 | 108 | +12.41, +3.98, +8.41 |
| G_j1_p06 | 0.30 | 82 | +12.10, +3.44, +8.93 |
| G_j1_m06 | 0.26 | 88 | +11.20, +3.04, +8.21 |

| Metric | Table-down model | Camera-forward model |
|---|---|---|
| Reference pair agreement (max pairwise) | **1.81 mm** | **0.95 mm** |
| Reference rotation spread | 1.55° | 1.41° |
| Mean error over the low family | **(+11.64, +4.00, +8.51) mm** | **(+5.22, +1.81, +5.80) mm** |
| Mean \|error\| | **14.96 mm** | **8.01 mm** |
| Per-pose error std | **0.99 / 0.57 / 0.40 mm** | **0.81 / 0.67 / 0.45 mm** |
| Residual after one constant correction | **max 2.39 mm** | **max 2.02 mm** |
| Servo shift during a capture | 0–1 pulse | 0–1 pulse |

The direction of the error is stable: `+x` (away from the base), `+y`
(robot's left) and `+z` (up) — i.e. at the low configuration the model places
the observed point **behind and below** where it physically is, and the grasp
target must be pushed by the same vector.

## 4. Joint1 approach test

The same grasp pose was captured after approaching `joint1` from below
(`pre = target − 0.08`) and from above (`pre = target + 0.08`):

| Quantity | From below | From above | Δ |
|---|---:|---:|---:|
| commanded `joint1` | 0.0 | 0.0 | — |
| bus register, servo 6 | **495** | **506** | **11 pulses (4.6°)** |
| measured `joint1` (rad) | −0.0230 | +0.0251 | 0.048 |
| camera→tag `tvec.x` (mm) | −10.3 | +1.4 | **11.7 mm** |
| predicted tag centre, table model | (0.28693, 0.01575, −0.08711) | (0.28682, 0.01463, −0.08746) | **1.1 mm** |

The servo settles 11 pulses apart depending on the approach direction, and the
image moves by the same 4.6° — so the **bus register faithfully tracks the
physical joint** and the measured-joint FK absorbs the dead band. This is why
the survey's residual can be ~1 mm even though the servo never reaches its
commanded value exactly. It also means the *approach direction must be
reproduced* at grasp time; the survey used `j1 −0.08`, the same direction as a
descent from the view pose.

## 5. Conflict with the 2026-11-19 pass-5 correction

Pass 5 published `e = (−8.0, −15.3, −5.8) mm` from **one** tag observation and
used it for a successful grasp. That vector has the opposite sign in x and y
to this survey, so it was checked against its own evidence images:

| Check | Result |
|---|---|
| Pass-5 grasp-config pose recomputed from `tag_graspcfg.jpg` | `(0.2282, 0.0063, −0.0854)` with the table-down transform — reproduces the published `(0.2293, 0.0049, −0.0846)` to **1.4 mm**, so the transform and pose pairing is right |
| Tag framing in `tag_graspcfg.jpg` | **margin 8 px**, 209 px side (52 % of the image height), reprojection **1.0 px** |
| Tag framing in `tag_view.jpg` | **margin 7 px**, 96 px side |
| Best pose fitting the published reference prediction `(0.23725, 0.02017, −0.07879)` | x/z match to ~1 mm at `j2 0.18, j3 −1.21, j4 −1.61`, but **y is −0.063 vs +0.020 — an 83 mm inconsistency no `j1 = 0` pose in the family can fix** |

Both pass-5 tags sat within 8 px of the frame edge, where a planar tag's
out-of-plane pose is weakly constrained (the same view earlier showed a fitted
tag plane tilted 8.4° against a 3.7° table). A several-degree tilt error at
0.13 m is a 10 mm position error, which is the size of the y discrepancy. The
survey's tags were framed at 35–108 px margin with 0.12–0.49 px reprojection,
so **the survey supersedes the single-point correction** — and the survey's
sign was confirmed by a grasp the same day: applying
`(+13.7, +3.9, +8.9) mm` to a cube measured from the tilted view produced a
grip, a 150 mm lift and a clean release 1.3 mm from the measurement. See
[`../../../phase2/onsite_20261119/low_config_grasp_validation/`](../../../phase2/onsite_20261119/low_config_grasp_validation/README.md).
At the grasp configuration the model therefore **overestimates** x by ~14 mm
relative to the tilted view; the pass-5 vector pointed the other way.

## 6. Limitations

- The absolute accuracy of the correction still rests on the hand-eye fit: the
  two fits differ by ~2.5°, which at 0.14–0.30 m is 6–13 mm. That difference is
  exactly the difference between the two correction columns, so **the model
  that is used for the grasp must be the one the correction was measured
  against**.
- The reference sits at `j4 = −1.5917` (the EEPROM limit) at the edge of the
  validated table-down volume, where the tag's own absolute z is biased low
  (the predicted tag centre is `−0.087 … −0.096 m`, 9–18 mm below the table).
  The `x/y` components are far less sensitive to this than `z`, but `e_z`
  carries it: treat `e_z` as ±5 mm.
- `j4` coverage is `−0.29 … −0.545` rad around the grasp value. The adopted
  pose (`j4 = −0.6704`) cannot see a tag that the reference pair also sees.
- One tag placement, one battery state (6573 mV). Servo compliance differs with
  voltage; the y-dominant error is geometric, but the low battery is recorded
  here as a caveat.
- `j1` dead band is 11 pulses (4.6°). The correction is valid for the
  `j1 −0.08` approach used here.

## 7. Files

| File | Content |
|---|---|
| `poses.json` | the 10 poses, tag identity, target placement window |
| `dataset_low_tag_20261119.json` | 10 samples: commanded + measured joints, servo start/end pulses, camera K/D, tag PnP |
| `report_low_config_error_table_down_model.json` | full report against the table-down fit |
| `report_low_config_error_camera_forward_model.json` | full report against the camera-forward fit |
| `report_low_config_model_fit.json` | four-model fit: parameters, cost, AIC, per-pose residuals, LOO, implied correction |
| `model_selection.md` | model-comparison report and the grasp-based rejection |
| `../joint1_approach_test/` | the two approach-direction samples |
| private images | `low_tag_survey_20261119` inside the robot container (not committed) |

## 8. Model selection (added 2026-11-19)

The survey's constant vector was tested against three configuration-dependent
models (fixed `link4` translation, fixed `link4` 6-DoF, joint offsets). All fit
the ten tag poses to 1.4–2.3 mm RMS, but every one of them predicts a grasp
correction **10.6–15.0 mm** away from the `(+13.7, +3.9, +8.9) mm` constant
that four 2026-11-19 runs used and passed with (joints reconstructed from their
bus-servo registers). The joint-offset hypothesis is therefore **rejected**, and
the constant base-frame offset remains the supported model inside the tested
envelope. Full analysis:
[`model_selection.md`](model_selection.md); artefacts:
[`report_low_config_model_fit.json`](report_low_config_model_fit.json).

## 9. Reproduction

```bash
# inside the robot container, zsh environment (CycloneDDS)
python3 /tmp/low_config_tag_survey.py \
  --output-dir /tmp/low_tag_survey_20261119 \
  --poses-file /tmp/low_tag_poses.json --repeats 3 --servo-reads 3

# on the laptop, Jazzy workspace sourced
python3 tools/phase2/low_config_error_report.py \
  --samples evidence/calibration/20261119/low_config_error/dataset_low_tag_20261119.json \
  --calibration evidence/calibration/20261118/table_down/calibration_table_down_20261118.json \
  --output evidence/calibration/20261119/low_config_error/report_low_config_error_table_down_model.json
```

Note: the sampling script must be started with the **zsh** environment
(`source ~/.zshrc`, i.e. CycloneDDS + `CYCLONEDDS_URI`). A node restarted from a
plain bash environment runs Fast DDS and the servo-state service then fails
with `failed to create array for field 'cmd'`.
