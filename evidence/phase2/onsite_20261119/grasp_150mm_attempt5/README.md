# Fifth grasp — low-configuration model-error correction (2026-11-19)

> Status: **PASS.** The cube was grasped, lifted in one motion to ≈148 mm above
> the floor, lowered back and released; the arm returned to the **adopted**
> pose. The grip was confirmed before the lift by the view-pose check that the
> floor cube disappears, and visually at the lift.

This attempt followed the failures of passes 3–4 (§3/§4 of
[`../grasp_150mm_attempt3/`](../grasp_150mm_attempt3/) and
[`../grasp_150mm_attempt4/`](../grasp_150mm_attempt4/)), whose measured cause
was a **~8–15 mm FK/hand-eye error at the low grasp configuration**.

## 1. Low-configuration error measured with an AprilTag

The operator placed a printed **AprilTag 36h11 ID 0, 60 mm black square** flat
on the floor (the ruler-measured size; an earlier 100 mm-scale-bar estimate of
73 mm was wrong and put the tag 59 mm below the floor).

| Pose | Tag centre (model, base_link) | Note |
|---|---|---|
| table-down view (model validated) | **(0.23725, 0.02017, −0.07879)** | z lands on the floor (−0.079) — model accurate |
| adopted (`j1=0, j2=−0.6618, j3=−1.8012, j4=−0.6704`) | (0.23051, 0.00779, −0.08689) | 8 mm below the floor |
| grasp configuration (`j = 0.030, −0.6618, −1.8012, −0.495`) | (0.22926, 0.00489, −0.08461) | −8.5 mm below the floor |

The model error at the grasp configuration is therefore

```text
e = (−8.0, −15.3, −5.8) mm      (x, y, z)
```

The **y component dominates**, which is exactly the lateral offset that made the
pads close beside a 39.4 mm cube in passes 3–4 (only ±4.3 mm of margin).
The tag PnP reprojection error was 0.12 px (view) and 0.93 px (grasp).

## 2. Corrected grasp

The cube was measured again at the validated view pose after the operator
removed the calibration sheet:

| Quantity | Value |
|---|---|
| Cube centre | **(0.2249, 0.0156)**, top z **−0.0345**, extent 39.2 × 39.8 mm, height 40.2 mm |
| Corrected target `C + e` | **(0.2157, −0.0001, −0.0403)** |
| Fitted joints (deep family) | **`−0.004, −0.6618, −1.8012, −0.522, 0`** |
| Model TCP | (0.2157, −0.0003, −0.0431) — dx 0.0, dy −0.2, dz −2.8 mm |
| Close | `r_joint −2.22 → −0.63` |

## 3. Verification and result

| Step | Result |
|---|---|
| Grip check | back at the view pose the segmentation found **0 floor clusters** — the cube is no longer on the floor (**held**) |
| One-step lift | `0.000, 0.300, −1.540, −1.270, 0` → TCP **(0.2145, 0.0008, 0.0993)** |
| Object bottom | ≈ **148 mm above the floor** (−0.0795 m), target 150 |
| Lift image | cube visible between the fingers (`a5_lift.jpg`) |
| Lower / release | grasp joints `status=4`; `r_joint −2.22` `SUCCEEDED` |
| Return | adopted, `status=4`; registers **498 / 343 / 928 / 341 / 501**, gripper −2.22 |
| Final cube | on the floor in front of the adopted camera (`a5_final.jpg`) |

## 4. What this establishes

1. The low-configuration FK/hand-eye error is **systematic and measurable**:
   −8 / −15 / −6 mm at the grasp configuration, while the same model is
   sub-millimetre at the table-down view.
2. A single AprilTag observation in the working configuration is enough to
   compensate it for a grasp: `target = C_view + e`. The corrected grasp
   succeeded on the first try.
3. The earlier "grip confirmed" errors came from judging the held-vs-floor
   question from a camera that sits 0.10 m from the held object; the reliable
   check is **the floor spot at the view pose** (it worked here).

## 5. Limits / follow-ups

- One tag point at ~15 mm from the cube: the error is assumed constant over
  that distance. A proper fix is a **joint-offset calibration** over the low
  configuration (the hand-eye transform is a single constant and cannot absorb
  a configuration-dependent error); the tag can also be used at several poses
  for a local hand-eye fit.
- The tag size must be right: a wrong size put the tag 59 mm below the floor
  and would have corrupted the correction.
- The cube was released where the corrected grasp put it; it ended ~10 mm from
  its pre-attempt position.

## 6. Files

| File | Content |
|---|---|
| `tag_view.jpg`, `calib_new.jpg`, `tag_graspcfg.jpg` | tag at the view, adopted and grasp configurations |
| `seg_a5b.json` | clean cube measurement used for the grasp |
| `seg_a5_verify.json` | grip verification (0 floor clusters) |
| `a5_lift.jpg` | cube held at the 148 mm lift |
| `a5_final.jpg` | after release, from the adopted pose |
| `tag_probe_host.py` | host-side tag PnP + link4 composition |
| `fk_link4.py` | link4 FK from the measured pulses (container) |
| `fit5.py`, `lift5b.py` | corrected grasp fit and one-step lift search |
