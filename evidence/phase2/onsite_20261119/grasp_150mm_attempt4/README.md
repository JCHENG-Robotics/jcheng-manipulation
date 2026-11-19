# Fourth grasp — "in view is enough" (2026-11-19)

> Status: **FAILED.** The cube stayed on the floor; the descending open
> gripper pushed it from the measured position (0.2192, 0.0049) to
> (0.2344, 0.0264). The post-grasp verification at the validated view pose
> (the floor cube must disappear) detected the cube still on the floor, so the
> failure was caught before the lift.
>
> The operator stopped the trial-and-error here and chose a **local low-pose
> hand-eye re-calibration** before further grasps.

## What was tried

Operator requirement for this round: the cube only has to be **in view**;
adjust the arm to grasp (no centring requirement).

| Step | Value |
|---|---|
| View-pose measurement (clean) | centre **(0.2192, 0.0049)**, top −0.0378, bounds 38.5 × 40.5 mm, extent 38.8 × 39.9 mm, height 40.3 mm |
| Grasp fit (proven deep family) | `j = 0.030, −0.6618, −1.8012, −0.495, 0` → TCP (0.2192, 0.0049, −0.0425), dx 0.0, dy 0.0, dz −4.7 mm |
| Close | `−2.22 → −0.63` |
| Verification | back to the view pose: the floor cube is **still there** at (0.2344, 0.0264) — 26 mm from the grasp point |

## The measured cause

At the grasp pose the cube's RGB centroid is at u = 334–386 while the **TCP is
a fixed pixel (364.8, 344.2)** in this camera (the camera→TCP transform is
constant when `joint5 = 0`; verified identical at the view, pre-grasp, grasp
and adopted joints). The cube therefore sits **~7–10 mm laterally away from
the gripper axis** at the low configuration.

One joint1 probe confirmed the image Jacobian is right: +0.14 rad moved the
cube's centroid by 47.6 px (340 px/rad measured vs 337 px/rad model). The
error is not the Jacobian — it is the **absolute FK/hand-eye error at the low
joint configuration** (≈10 mm laterally, 50–90 px), while the same model is
0.2 px accurate at the table-down view.

Closing a gripper whose pads open to ±24 mm around a 39.4 mm cube leaves only
±4.3 mm of margin, so a 10 mm error makes the pads close beside the cube and
the descending open finger shoves it away.

## Why the same recipe worked in passes 1–2

Passes 1–2 used the same deep joint family and the same view-pose measurement
and succeeded; the cube was then at (0.2265, 0.0009) and (0.2305, 0.0085).
The difference is not understood from the data available here — the model's
low-pose error is configuration-dependent and needs the calibration below.

## Next step (operator choice)

Re-do the **hand-eye calibration over the low joint configurations**
(`joint2 ≈ −0.56…−0.66`, `joint3 ≈ −1.5…−1.8`, `joint4 ≈ −0.4…−0.6`) with the
chessboard at 0.20–0.25 m, then measure and grasp entirely inside that
calibrated region. The table-down calibration does not extrapolate to these
configurations.

## Files

| File | Content |
|---|---|
| `seg_a4_view.json` | clean view-pose measurement used for the grasp |
| `seg_a4_verify.json` | post-grasp verification: the cube is still on the floor |
| `a4_grip.jpg` | grasp pose after closing (cube off to the left of the TCP) |
| `a4_pre.jpg`, `a4_pre2.jpg`, `a4_pre3.jpg` | pre-grasp images for the joint1 probe |
| `a4_verify.jpg` | the view pose after the attempt |
| `tcp_pixel.py` | the constant TCP pixel (364.8, 344.2) at several poses |
| `deep_fit4.py` | deep-family fit to the measured cube |
