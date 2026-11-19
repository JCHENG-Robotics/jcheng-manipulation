# Yaw extension of the low-configuration envelope — 2026-11-19

> Status: **measured; the constant correction holds to `j1 ≈ 0.5 rad` and the
> fitted configuration-dependent models are rejected again.**

## 1. Question

The constant correction was calibrated at `j1 ≈ 0`. Does the required
correction rotate with base yaw, as a fixed `link4`-frame offset would, or does
it stay a base-frame constant?

## 2. Method (tag, no grasp)

A 60 mm 36h11 tag was placed flat at `(0.272, 0.106) m`. The arm captured it
with `low_config_tag_survey.py --check-only` from two poses:

| Pose | Measured joints | Tag PnP |
|---|---|---|
| view | `(0.0105, 0.2388, −1.2189, −1.6629, −0.0168)` | 0.22 px RMS, 95 px margin, 0.319 m |
| yawed grasp | `(0.4859, −0.6744, −1.7970, −0.3393, 0)` | 0.42 px RMS, 24 px margin, 0.146 m |

The tag is a fixed point, so `e = m_grasp − m_view` is the correction the yawed
pose needs. Three repeats (alternating view and grasp captures) quantify the
noise. No gripper is involved, which removes the object-orientation confound
described in section 4.

## 3. Result

`e = (9.14, 7.18, 8.75) mm`, spread over three repeats (1.54, 0.13, 0.80) mm.

| Model | Predicted `e` (mm) | Δ (mm) | Norm |
|---|---|---|---:|
| **constant** | (13.7, 3.9, 8.9) | (+4.56, −3.28, +0.15) | **5.62** |
| `joint_offsets4` | (9.65, −6.89, 5.70) | (+0.51, −14.07, −3.05) | 14.41 |
| `link4_translation` | (8.85, −9.73, 4.05) | (−0.29, −16.91, −4.70) | 17.55 |
| `link4_se3` | (1.55, −9.62, 3.40) | (−7.59, −16.80, −5.35) | 19.20 |

The measured direction (38°) lies between the constant (16°) and a full
`link4` yaw rotation (44°): the correction rotates **partially** with yaw. Its
magnitude shrinks (11.8 mm vs the constant's 14.2 mm). The fitted models
overshoot in `y` — the gripper-opening direction — by 14–17 mm and flip its
sign. At 28° the constant's residual is 5.6 mm, of which the grip-critical `y`
part is 3.3 mm, inside the ±4 mm margin.

## 4. The failed yawed grasps (recorded, not evidence against the correction)

Two grasps at `j1 ≈ 0.45` were attempted first, with the constant and with the
`link4` model's correction. Both missed and pushed the cube sideways. The
cause was geometric, not the correction: the square 39.4 mm cube was
axis-aligned, so at a 20–30° bearing the pads face its **diagonal**
(39.4·(cos + sin) ≈ 50 mm), beyond the ~40 mm opening. The raw runs are
`yaw_constant_run.json` and `yaw_model_run.json`. Any future yawed grasp must
rotate the cube so a face looks at the arm.

## 5. Files

| File | Content |
|---|---|
| `yawtag_view4.json`, `rep_view2/3.json` | tag captures at the view pose |
| `yawtag_grasp4.json`, `rep_grasp2/3.json` | tag captures at the yawed grasp pose |
| `analysis.json` | measured `e`, repeats, spread and the four model predictions |
| `yaw_constant_run.json`, `yaw_model_run.json` | the two geometric-failure grasps |
