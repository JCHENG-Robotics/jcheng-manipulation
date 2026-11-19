# Quick replay grasp — 2026-11-19

> Status: **PASS.** The grasp validated the same day in
> [`../low_config_grasp_validation/`](../low_config_grasp_validation/README.md)
> was repeated **without a new measurement**: same grasp joints, same verify
> pose, same closing sequence. The cube was gripped, lifted 150 mm, put back
> and released. This is a repeatability datum for the recipe, not a new
> calibration result.

## 1. What was done

| Step | Value |
|---|---|
| Start pose | park `[0, 0.300, −1.14773, −1.55404, 0]` (registers 496/572/779/122/500) |
| Grasp joints | `[−0.015, −0.6618, −1.8012, −0.340]` — reused from the validation run |
| Sequence | open `−2.22` → approach with `joint4 = 0` → descend → close `−1.6` → close `−0.63` → return to verify pose |
| Verify pose | `[0, 0.2574, −1.18773, −1.70, −0.01676]` (tilted table-down, camera 0.22 m up) |
| Grip check | live RGB-D segmentation at the verify pose: **0 clusters**, 0 points above the floor |
| Lift | `[0, 0.300, −1.540, −1.270]`, `error_code 0` |
| Put back | lower to the grasp joints, open `−2.22`, return to the verify pose |
| End pose | park `[0, 0.300, −1.14773, −1.55404, 0]` |

No pre-grasp measurement was taken: the cube was assumed to still be where the
previous run released it, `(0.2275, −0.0046)`. The grip margin for a 39–40 mm
cube is about ±4 mm, so this only works while the cube stays within a few mm of
that spot.

## 2. Result

| Check | Result |
|---|---|
| Servo registers at the grasp | **496 / 339 / 929 / 421** (servo 6/5/4/3) — identical to the validation run |
| Servo registers at the verify pose | **496 / 555 / 792 / 103** — identical to the validation run |
| Cube leaves the floor | **yes** — 0 clusters above the floor at the verify pose |
| Handoff to the lift | **held** — cube visible between the pads at 150 mm, floor empty |
| Release | cube back on the floor at **(0.2180, −0.0033)**, top slab 38.3 × 41.2 mm, height 42.4 mm |
| Release displacement | **9.5 mm** in x from the previous run's landing point (0.2275, −0.0046); the pre-grasp position was not re-measured, so this is not a placement-accuracy claim |

`joint4` at the verify pose reaches raw **103**, i.e. 17 counts past the servo
EEPROM limit of 120 — the same over-travel as the validation run (101).

## 3. Limits

- No fresh measurement: the run relies entirely on the cube not having moved
  since the previous release. A miss would have been caught by the grip check,
  not prevented.
- The approach started from the park pose rather than the verify pose; the
  `joint1` dead band still settled at register 496, the same value as the
  validation run, so the direction effect was reproduced.
- One grasp, one cube position, one battery state; no battery voltage was read
  this session.

## 4. Files

| File | Content |
|---|---|
| `grasp_run.json` | servo registers at every step of the grasp |
| `release_run.json` | the put-back run |
| `grip_check_seg.json` | segmentation at the verify pose: **0 clusters** |
| `after_release_seg.json` | segmentation after the put-back: one 38.3 × 41.2 mm cluster |
| `grip_check_view.png` | cube held between the pads at the verify pose |
| `lift150.png` | cube held at the 150 mm lift, floor empty |
| `after_release.png` | cube back on the floor, right side up |
