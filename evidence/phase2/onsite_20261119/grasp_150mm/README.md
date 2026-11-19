# Fixed-position grasp, 150 mm lift and return — 2026-11-19

> Status: **PASS with caveats.** One autonomous attempt, no operator
> intervention: a 39.3 mm wooden cube on the marble floor was located by a
> markerless RGB-D segmentation, grasped between the two fingers, lifted to
> **≈149 mm** above the floor, lowered back to the same grasp pose and
> released. The cube ended within a few millimetres of its original spot
> (same-transform re-measurement: 7.5 mm; the pose model itself drifts ~10 mm
> between captures, see §7).
>
> This is the first run in this repo where **pick → lift → place → release**
> all succeeded without the operator. The 2026-11-18 first grasp needed the
> operator to slide the cube between the fingers and jammed on release.

Operator request: *"恢复初始位置，尝试一次视野内抓取，要求通过视觉判断尺寸并精准抓取，
然后抬起来距离地面 150 mm 之后再放回原位。"*

## 1. Setup

| Item | Value |
|---|---|
| Robot | ArmPi Ultra / Pi 5, vendor stack in container `ArmPiUltra` |
| Laptop stack | Humble container `armpi-humble`: `move_group`, `armpi_safety` |
| Object | wooden cube, **39.4 mm** edge, no marker |
| Support surface | marble floor; plane **z = −0.0836 m** in `base_link` at the view pose (211,820 inliers, 0.6 mm RMS, 0.48° to +z) |
| Camera | Aurora 930, RGB 640×400, cloud `/depth_cam/depth_registered/points` |
| Hand-eye | table-view TSAI, `evidence/calibration/20261118/table_down/` |
| Battery | 7814 mV (70.7 %) at start and at the end |
| Clock | robot 1.1 s ahead of the laptop (required for MoveIt execute) |

`--joints` targets in this document are `joint1..joint5` in radians; all arm
motions went through `real_moveit_execute.py` (Pilz PTP, velocity scaling
0.05) and the `/armpi_safety/validate_trajectory` veto. The gripper went
through `/gripper_controller/follow_joint_trajectory` (the safety node still
rejects gripper-only trajectories).

## 2. Sequence and measured results

| # | Action | Command / target | Result (measured) |
|---:|---|---|---|
| 1 | Restore `init_pose` | `init_pose` | 115 wp, 11.38 s, `status=4`; pulses **501/672/886/120/498** (joint4 clamped to the adopted −1.5917) |
| 2 | Move to the table-down view | `0.504, 0.685, −1.652, −1.559, 1.383` | 107 wp, 10.51 s, `status=4`; pulses 621/664/893/129/827 |
| 3 | Segment (view 1) | — | cube found, but the **top face was clipped to 35.0 × 37.7 mm** (real 39.4) and the centroid was ambiguous by ≈4 mm — the half-open gripper (`r_joint = −0.8378`) sat in the camera line of sight |
| 4 | Rotate the wrist to horizontal | `0.504, 0.685, −1.652, −1.559, 0.0` | 106 wp, 10.47 s, `status=4`; pulses 619/662/892/130/499 |
| 5 | Segment (view 2, clean) | — | top face **41.0 × 37.7 mm**, bounds **38.7 × 40.0 mm**, centroid **(0.2265, 0.0009)**, top z **−0.0418**, height above floor **40.2 mm**, tilt 1.7°, yaw 2.3° mod 90 |
| 6 | Grasp search | target (0.2265, 0.0010, −0.0418), `--joint-limit-margin 0.0` | 44 candidates, 22 IK, 5 plans; best TCP **(0.2320, 0.0008, −0.0396)**, pitch 15.5°, **offset 5.9 mm** (floor reach boundary) |
| 7 | Execute the grasp with `joint5 = 0` | `−0.0012, −0.6618, −1.8012, −0.3917, 0.0` | 105 wp, 10.32 s, safety OK, `status=4`; pulses **501/344/928/405/497** |
| 8 | Close the gripper | `r_joint` −2.22 → −1.0 → **−0.63** | all `status=4`; grip gap ≈ 38.8 mm vs the 39.3 mm measured width (≈0.5 mm interference) |
| 9 | Test lift +37 mm | `−0.001, −0.462, −1.601, −0.792, −0.092` | 58 wp, 5.68 s, `status=4`; the cube rose with the gripper (image position fixed relative to the camera) |
| 10 | Lift to 150 mm | `−0.0012, 0.1382, −1.4012, −1.1917, −0.0917` | 70 wp, 6.89 s, `status=4` |
| 11 | Height correction | `0.0, 0.154, −1.388, −1.208, −0.0922` | 14 wp, 1.25 s, `status=4`; TCP z = **+0.1097 m** |
| 12 | Lower back | `−0.0012, −0.6618, −1.8012, −0.3917, 0.0` | 82 wp, 8.05 s, `status=4` |
| 13 | Release | `r_joint −2.22` | `status=4`; fingers open |
| 14 | Retreat to the view | `0.504, 0.685, −1.652, −1.559, 0.0` | 106 wp, 10.35 s, `status=4`; pulses **622/662/894/128/499** |
| 15 | Re-segment the result | same joints as step 5 | cube flat on the floor (top − floor = 43 mm), top face 39.8 × 37.4 mm, **yaw unchanged (88.0° tool-frame)** |

### Vision-derived size and the grasp numbers

- Vision size: **39.3 mm** (top-face min-area rectangle 41.0 × 37.7 mm and
  cluster bounds 38.7 × 40.0 mm average to 39.3; nominal cube 39.4).
- The independent RGB top-face check was **not usable** at this pose: the
  colour mask includes the visible front side face and back-projects to
  42–47 mm. The cloud-referenced numbers above are the trusted ones.
- TCP-to-cube centre error across the opening axis: **0.2 mm** in y.
- The 5.9 mm TCP shortfall is **along the finger length** (x): the fingers are
  ~32 mm long in the approach direction and the cube is 39 mm wide in x, so
  the shortfall reduces the grip depth, not the straddle.

### Lift height

| Quantity | Value |
|---|---|
| Grasp TCP z | −0.0375 m (measured FK at the grasp state) |
| Lift TCP z | **+0.1097 m** (measured FK) |
| TCP travel | **+147.2 mm** |
| Cube bottom at grasp | ≈ −0.0820 m (top − 40.2 mm) |
| Cube bottom at lift | ≈ +0.0652 m |
| Height above the floor plane | **≈149 mm** (target 150 mm; ±3 mm from the ±2 mm plane and ±2 mm FK) |

## 3. The decisive finding: the wrist roll must be 0 at this view

`joint5` is a **pure roll about the TCP axis** — FK gives the same TCP
(0.232, 0.0008, −0.0396) for every `joint5` in the range. But the finger axis
does rotate:

| `joint5` (rad) | finger-opening azimuth | vs cube face normal (2.3°/92.3°) |
|---:|---:|---|
| +1.408 (planned grasp solution) | 170.3° | 12° |
| +1.108 | 152.6° | 30° |
| +0.508 | 118.2° | 26° |
| **0.000** | **90.0°** | **2.3°** |
| −1.592 | −1.3° | 3.6° |

At the table-down view the half-open fingers sat across the camera→cube ray.
With `joint5 = 1.383` the segmented top face was **35.0 × 37.7 mm** and the
per-slice centroid moved up to 5 mm; with `joint5 = 0` it was
**41.0 × 37.7 mm** and three different estimators agreed to 0.1–1.7 mm. The
planned grasp solution (`joint5 = 1.408`) would also have twisted the cube in
the fingers on the way to the lift, so the grasp was executed with
`joint5 = 0` instead (same TCP, 2.3° roll error).

## 4. What failed and what worked

| Item | Result |
|---|---|
| Same-orientation IK for the lift | **fails** (`/compute_ik` code −31, all seeds) — the 5-DoF wrist cannot hold the floor-grasp orientation while raising |
| Joint-space search at the lift point | **works** — 5 plans at TCP z ≈ +0.107, pitch 39°, `joint5 = −0.092` |
| Deeper grasp | **not reached** — at the adopted low limits the TCP cannot go below z −0.0396 at x 0.232, i.e. ~2 mm *above* the cube top; the pads straddle only the upper ~31 mm of the 39.4 mm cube |
| Release | **succeeded** with the full −2.22 rad opening (the 2026-11-18 run jammed at −0.83) |
| Place-back accuracy | 7.5 mm by same-transform comparison; the pose model changed by ~10 mm between captures (see §7) |

## 5. Open-loop limits

- No force/contact feedback: the grip is a blind `r_joint` command; release is
  confirmed by camera only.
- The 150 mm height is derived from FK + the RANSAC floor plane, not a
  physical ruler.
- One object, one pose, one attempt; slip margin not measured.

## 6. Follow-ups (operator feedback, 2026-11-19)

1. **Start low, not folded.** The session started at `init_pose` (folded) and
   then at a table-down view with `joint5 = 1.383`. The next attempt should
   start directly at the **adopted-low pose** (arm at the adopted low limits)
   with the gripper **horizontal (`joint5 = 0`)** so it never blocks the lens.
2. **Grip deeper.** Lower the grasp TCP by a few millimetres (the physical
   low limits allow ~8 mm more than the adopted ones; alternatively remove the
   reach shortfall). Keep the pad bottom clear of the floor.
3. **Lift in one step.** The first run used 37 mm test + 107 mm + 6 mm. Plan
   the full 150 mm lift as a single joint-space target.
4. Closer the loop on release (servo register or a second view).

## 7. Measurement caveat — pose model drift

The same-transform re-segmentation after placing the cube back gave a floor
plane **10 mm higher** (z@0.24: −0.0737 vs −0.0836) and a **3.05°** tilt to +z
(vs 0.48°). The commanded/measured servo pulses differed by only 1–3 counts,
so this is the real arm pitch being a couple of degrees off the FK model
(sag/backlash), not a joint-state error. At 0.24 m a 2.5° pitch error is
~10 mm, which is the same order as the measured 7.5 mm XY shift. The
**cube-relative** numbers (height above the fitted floor, size, yaw) are
stable; the **absolute** base-frame position has ~10 mm of pose uncertainty
per capture.

## 8. Files

| File | Content |
|---|---|
| `seg_view1.json` | clip case: `joint5 = 1.383`, top face 35.0 × 37.7 mm |
| `seg_view3.json` | clean case: `joint5 = 0`, top face 41.0 × 37.7 mm, centre (0.2265, 0.0009) |
| `seg_after_samepose.json` | post-place cloud re-segmented with the pre-grasp joints (same transform) |
| `grasp_cube.json` | 44-candidate grasp search, 5 plans, best `js_0010` |
| `lift30.json`, `lift150.json`, `lift155.json` | lift searches at −7.5 mm, +107 mm and +114 mm targets |
| `view1.jpg` | view with the half-open fingers in the line of sight |
| `view3.jpg` | clean view used for the measurement |
| `grasp_pose.jpg`, `grip2.jpg` | at the grasp pose, before and after closing |
| `lift150_final.jpg` | cube held at the top of the lift |
| `after_place.jpg` | cube released on the floor after retreat |
| `analyze_cube.py` | robust XY min-area-rectangle analysis of a cloud |
| `rgb_top_face.py` | RGB top-face check (documented failure mode) |
| `lift_ik.py` | `/compute_fk` + `/compute_ik` probe used to show the IK failure |
| `search_local.py` | local joint-space search for the final 6 mm |
