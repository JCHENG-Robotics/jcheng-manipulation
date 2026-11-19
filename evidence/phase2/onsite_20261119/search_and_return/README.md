# Out-of-view search, one base approach, grasp and return — 2026-11-19

> Status: **PASS.** The cube was outside the camera field of view at the
> tilted table-down pose. A **+16° `joint1` yaw** brought it into the frame
> at **0.469 m** from the base, i.e. ~0.22 m beyond the 0.2447 m reach
> boundary. The base was driven **once** toward it in three closed-loop
> pulses (re-segmenting between pulses), landing the cube at
> **(0.2284, −0.0010) m** — inside the validated 0.226–0.234 m measurement
> band. The validated low-configuration grasp then gripped the cube, a
> 150 mm lift held it (0 clusters above the floor), and the base returned
> home. A fixed floor landmark measured the return error (32.1 mm after the
> first reverse pulse, **9.4 mm after one correction pulse**). The cube was
> released **1.1 mm** from its pre-grasp position in the robot frame, at
> **(0.2115, 0.0278) m** in the original base frame.

## 1. Search — the cube was not in view

The arm started at the tilted table-down view
`[0, 0.2574, −1.18773, −1.70, −0.01676]` (measured 497/565/782/122/500).
The first frame ([`00_start_view_no_cube.png`](00_start_view_no_cube.png))
shows only the marble floor and the fixed floor marker on the right.

A **`joint1` yaw sweep at the raised view pose** was used (the raised pose is
required: `joint1` must not be yawed at the low grasp pose). At 6 s per move:

| Step | `joint1` | Frame | Result |
|---|---:|---|---|
| 1 | 0.00 rad (0°) | [`01_search_yaw00.png`](01_search_yaw00.png) | no cube |
| 2 | +0.14 rad (+8°) | [`02_search_yaw08.png`](02_search_yaw08.png) | no cube |
| 3 | **+0.28 rad (+16°)** | [`03_search_yaw16_found.png`](03_search_yaw16_found.png) | **cube at the upper-left** |

The sweep shows the "look around" action: three small yaw steps, one frame
per step. The negative side was not needed; the search stopped at the first
sighting.

Live RGB-D segmentation at the +16° pose
([`seg_search_y16.json`](seg_search_y16.json), measured pulses
561/561/782/103/496):

| Quantity | Value |
|---|---|
| Top face | **39.8 × 37.5 mm**, height 38.3 mm, 3474 points |
| Centroid C | **(0.36556, 0.29418, −0.05521) m** |
| Distance from the base | **0.4692 m**, bearing **+38.8°** |
| Reach boundary at that z | 0.2447 m (deep family) |

**Distance verdict: not graspable.** The cube is ~0.22 m beyond the reach
boundary and 39° off the forward axis, so the arm cannot reach it from the
starting base position.

## 2. One base approach (closed-loop)

The base was pulsed diagonally with the operator-confirmed mecanum signs
(forward `[+,−,−,+]`, left `[+,+,+,+]`; duty 10 %), re-segmenting the cube
from the same view between pulses:

| Pulse | Direction (vx, vy) | Duration | Cube base position | Travel |
|---|---|---:|---|---:|
| start | — | — | (0.36556, 0.29418) | — |
| 1 | (0.414, 0.910) | 2.8 s | (0.30449, 0.17260) | 136 mm |
| 2 | (0.381, 0.925) | 3.0 s | (0.24286, 0.03141) | 154 mm |
| 3 | (0.291, 0.957) | 0.75 s | (0.23272, −0.00005) | 33 mm |

Net displacement **0.3255 m** (forward 0.1372, left 0.2952). The last
position was rejected only by the top-view gate at the yawed pose
(`partial_top_view`, min extent 34.25 mm < 35 mm), not by the cluster gate:
the cluster itself was 36.1 × 40.1 mm.

The arm then returned to the validated **`joint1 = 0`** view and measured
again:

| Quantity | Value |
|---|---|
| Measured pulses | 502 / 561 / 782 / 103 / 496 |
| Top face | 41.8 × 36.2 mm |
| Centroid C | **(0.22838, −0.00102, −0.04899) m** ([`04_measure_view.png`](04_measure_view.png)) |

This is inside the validated 0.226–0.234 m band; no further base motion was
needed.

## 3. Grasp and lift

The day's validated recipe was applied unchanged:

| Step | Value |
|---|---|
| Correction | `(+13.7, +3.9, +8.9) mm` |
| Target T | (0.24208, 0.00288, −0.04009) m |
| Deep-family fit | `[0.0140, −0.6618, −1.8012, −0.3220, 0]` |
| TCP | (0.2420, 0.0027, −0.0364) m — residual (−0.1, −0.2, +3.7) mm |
| Registers at grasp | 502 / 339 / 929 / 427 (servo 6/5/4/3) |
| Registers at verify | 502 / 556 / 792 / 102 |
| Grip check | **0 clusters**, 0 points above the floor ([`grip_check_seg.json`](grip_check_seg.json)) |
| Lift | `[0.014, 0.300, −1.540, −1.270, 0]`, `error_code 0` ([`06_lift150.png`](06_lift150.png)) |

## 4. Return and place

The base was driven back along the exact reverse direction in one 6.3 s
pulse, (−0.421, −0.906). With the cube held, a **fixed cyan/black floor
marker** ([`07_after_return_holding.png`](07_after_return_holding.png)) was
used as a world landmark: its base-frame position was compared with the
frame captured before the approach.

| Check | Base-frame landmark | Base offset from home |
|---|---|---|
| Before the approach | (0.3541, −0.1108, −0.0940) | 0 |
| After the 6.3 s reverse pulse | (0.3699, −0.1388, −0.0932) | **32.1 mm** (15.8 mm back, 28.0 mm left) |
| After one 0.75 s correction pulse (0.491, −0.871) | (0.3518, −0.1017, −0.0919) | **9.4 mm** (2.3 mm x, 9.1 mm y) |

The landmark position is computed from the RGB ray through the marker, the
RANSAC floor plane and the measured bus-servo FK + hand-eye chain, so the arm
pose difference between the two frames is compensated. The translation-only
interpretation is an approximation: a small base yaw would fold into the
same numbers.

The cube was then lowered to the grasp joints, the gripper opened to −2.22
and the arm returned to the view:

| Check | Value |
|---|---|
| Registers at release | 502 / 342 / 928 / 418 |
| Landing (release frame) | **(0.22730, −0.00024, −0.04878) m** |
| Robot-frame residual vs pre-grasp | **(−1.1, +0.8) mm** |
| Landing in the original base frame | **≈(0.2115, 0.0278) m** |
| Top face after release | 39.7 × 37.3 mm, upright ([`08_after_release.png`](08_after_release.png)) |

The robot-frame residual is the gripper placement repeatability. The
**home-frame** position adds the measured base return error at the moment of
release; the cube is therefore ~2.8 cm left of the robot's original forward
line, not because of the grip but because the base correction was applied
*after* the release.

## 5. Limits

- **Search coverage:** only the positive yaw side was executed. The shell
  loop's negative-side moves failed on `argparse` (a joint vector starting
  with `-` was parsed as an option), so steps 4–5 in `/tmp` are duplicates of
  step 3. The cube was already found, so the demo was not repeated; a
  production search should sweep both sides.
- **One base approach, no odometry:** the outbound motion is closed-loop
  (cube re-segmented between pulses), the return is open-loop and was
  verified *after the fact* with the floor marker. The 9.4 mm residual is the
  base's end position, not a calibrated odometry claim.
- **Correction ordering:** the return correction pulse was applied after the
  cube had been released. A next run should do the landmark check and
  correction while still holding the cube, then release, which would place
  the cube within the 9.4 mm base residual instead of the 32.1 mm pre-release
  residual.
- The back-drive measurement is translation-only; base yaw is not measured.
- One run, one battery state (7.03 V at the start and at the end).
- The `z` of the floor at the tilted view is biased ~13–16 mm below the
  calibrated plane (known open-loop limitation); the x/y corrections are the
  load-bearing quantities.

## 6. Files

| File | Content |
|---|---|
| `00_start_view_no_cube.png` | the initial view: no cube |
| `01_search_yaw00.png`, `02_search_yaw08.png`, `03_search_yaw16_found.png` | the yaw search, cube found at +16° |
| `seg_search_y16.json` | cube measurement at the search pose |
| `04_measure_view.png`, `seg_measure.json` | the post-approach measurement at `joint1 = 0` |
| `grasp_run.json` | servo registers for every grasp step |
| `05_grip_check_held.png`, `grip_check_seg.json` | grip check: cube off the floor, 0 clusters |
| `06_lift150.png` | cube held at the lift pose, floor empty |
| `07_after_return_holding.png` | the returned view, cube held, marker in frame |
| `release_run.json` | servo registers for the release |
| `08_after_release.png`, `after_release_seg.json` | cube back on the floor, upright |
| `run_log.json` | all pulse commands, cube track, fits and landmark checks |
