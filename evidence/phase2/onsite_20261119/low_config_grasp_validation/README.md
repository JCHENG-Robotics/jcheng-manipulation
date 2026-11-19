# Low-configuration grasp validation — 2026-11-19

> Status: **PASS.** The correction measured by the tag survey
> `(+13.7, +3.9, +8.9) mm` was applied to a cube measured from the tilted
> table-down view. The cube was gripped, lifted 150 mm, lowered and released,
> landing **1.3 mm** from where it had been measured. This resolves the sign
> conflict with the pass-5 single-tag correction `(−8.0, −15.3, −5.8) mm`:
> at the grasp configuration the model **overestimates** x by ~14 mm relative
> to the tilted view, it does not underestimate it.

## 1. What was validated

The survey in
[`../../calibration/20261119/low_config_error/`](../../calibration/20261119/low_config_error/README.md)
measured the model error between a tilted table-down view and the grasp family
as `(+13.7, +3.9, +8.9) mm` (tag-referenced, free PnP) or `(+13.9, +3.5) mm`
in x, y with the tag anchored to the floor plane. Both agree; the resolution
below uses the first, because it is the same convention the sweep used.

One open question remained: the 2026-11-19 pass-5 grasp had succeeded with the
**opposite** sign in x and y. A grasp was the only way to settle it.

## 2. Procedure

| Step | Detail |
|---|---|
| Measurement pose | `[0, 0.2574, −1.18773, −1.70, −0.01676]` — tilted table-down, camera 0.22 m up. `joint4` reaches raw 100, i.e. 20 counts past the EEPROM limit of 120 |
| Object | light wooden cube, top face **38.4 × 40.7 mm**, height 39.7 mm |
| Measurement | registered point cloud → floor plane → top slab; centroid **(0.22622, −0.00595, −0.05056)** in model coordinates |
| Correction | `(+13.7, +3.9, +8.9) mm` |
| Grasp target | **(0.23992, −0.00205, −0.04166)** |
| Joint fit | `[−0.015, −0.6618, −1.8012, −0.340, 0]`, TCP **(0.23967, −0.00232, −0.03726)**, residual `(−0.25, −0.27, +4.4) mm` |
| Close | `r_joint −1.6` then `−0.63` |
| Grip check | return to the measurement pose, fresh snapshot: **0 cloud points** in the cube box |
| Lift | `[0, 0.300, −1.540, −1.270]`, cube still between the fingers, floor empty |
| Release | at the grasp pose, then back to the measurement pose |

Servo registers: at the grasp `496 / 343 / 928 / 412`, at the measurement pose
`496 / 556 / 791 / 104`.

## 3. Result

| Check | Result |
|---|---|
| Cube leaves the floor | **yes** — the cube box had 4362 points before the grasp and **0** after |
| Handoff to the lift | **held** — the 150 mm lift at `[0, 0.300, −1.540, −1.270]` kept the cube between the fingers, floor empty |
| Release | cube landed at **(0.2275, −0.0046)**, **1.3 mm** from the pre-grasp measurement |
| Joint-limit margin | all joints within the adopted range (`joint4` commanded −0.34) |

## 4. Independent datum from the same session

The depth plane fitted through the model at the tilted measurement pose puts
the floor at **z = −0.0918 m** at the cube's location, while the calibrated
floor is **−0.0784 m** (depth plane + 215 mm ruler check, 2026-11-18). So the
model carries a **~13 mm z error at that pose** — the same order as the
correction itself, and the reason the measurement and the grasp must be
reconciled by a measured correction rather than assumed consistent.

## 5. Files

| File | Content |
|---|---|
| `grasp_run.json` | every step's servo registers for the grasp run |
| `release_run.json` | the release run |
| `grip_check_view.png` | cube held between the fingers at the measurement pose |
| `lift150.png` | cube held at the 150 mm lift, floor empty below |
| `after_release.png` | cube back on the floor |

## 6. Limits

- One grasp, one cube position, one battery state (6.57 V). The grip margin for
  a 39–40 mm cube is about ±4 mm, so a passing grasp bounds the residual error
  to roughly that, not better.
- The correction used the model at the tilted pose, which is **outside** the
  validated table-down volume (`joint4 −1.70` vs the sampled
  `−1.634 … −1.474`) and carries the ~13 mm z error above. The x/y correction
  is therefore the load-bearing quantity; z was taken from the tag-referenced
  `e_z = +8.9 mm` and the fit then sat 4.4 mm high, which did not prevent the
  grip.
- The pass-5 correction `(−8.0, −15.3, −5.8) mm` is superseded. Its tag
  observations sat within 7–8 px of the frame edge (reprojection 1.0 px, tag
  side 209 px), where a planar tag's out-of-plane pose is weak; this run's cube
  measurement used the full 40 mm top face at 327 px from the frame centre.
