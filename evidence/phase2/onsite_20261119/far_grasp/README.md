# Far-cube grasp with a closed-loop base approach — 2026-11-19

> Status: **PASS.** The cube had been placed by the operator far inside the
> park-view field of view at model **x ≈ 0.431 m**, about 0.20 m beyond the
> arm's reach. The base was driven forward in pulses, re-measuring the cube
> from the park view between pulses, until it sat at **x ≈ 0.234 m**. A clean
> measurement at the validated tilted pose, the survey correction
> `(+13.7, +3.9, +8.9) mm` and the deep grasp family then gripped the cube,
> held it through a 150 mm lift and released it **6.8 mm** from where it had
> been measured.

## 1. Locating the cube and moving the base

The arm was parked, the cube was segmented from the park view
`[0, 0.300, −1.14773, −1.55404, 0]` (`/tmp/hand_eye.json` = the table-down
hand-eye). No `mecanum_chassis_node` was running (`cmd_vel_subscribers = 0`),
so a bounded driver published `MotorsSpeedControl` directly on
`/ros_robot_controller/set_motor_speeds` at 20 Hz with the operator-confirmed
forward signs `[+, −, −, +]` for IDs 1–4, duty 10 %, and an explicit zero.

| Pulse | Cube x (model) | Travel | Note |
|---|---:|---:|---|
| start | 0.4312 | — | visible, `x_out_of_region` |
| forward 3.0 s | 0.2886 | −142.6 mm | ~48 mm/s average |
| forward 1.4 s | 0.2212 | −67.4 mm | top face clipped, tool rejected it |
| back 0.6 s | 0.2424 | +21.2 mm | ~35 mm/s, overshoot |
| forward 0.28 s | 0.2343 | −8.1 mm | clean top face |
| back 0.15 s | 0.2343 | 0.0 mm | **inside the dead band** |

Net base travel **≈ 0.197 m forward**, with the cube's measured y drifting
`−0.0179 → −0.0104` (+7.5 mm), i.e. the base also drifted/yawed slightly.

The dead band is real but short: a 0.15 s pulse moved nothing while 0.28 s
moved 8 mm, so closed-loop re-measurement between pulses is the practical way
to place the base within the 0.22–0.25 m measure≡reach band.

## 2. Measurement and fit

| Item | Value |
|---|---|
| Measurement pose | `[0, 0.2574, −1.18773, −1.70, −0.01676]` (same as the validated run) |
| Top face | **38.7 × 39.6 mm**, height **39.6 mm**, tilt 1.3°, yaw 3.0° |
| Centroid C | **(0.23428, −0.01042, −0.05069)** |
| Correction | `(+13.7, +3.9, +8.9) mm` (the survey's tilt-pose value) |
| Target | **(0.24798, −0.00652, −0.04179)** |
| Fit | `j1 = −0.0263` (azimuth), `j2 = −0.6618`, `j3 = −1.8012` (adopted bounds), `j4 = −0.294` |
| TCP | **(0.2455, −0.0044, −0.0351)** |
| Residual | **(−2.5, +2.1, +6.4) mm** |

The x residual is not a measurement error: with `j2`/`j3` on their adopted
bounds the family reaches `x ≈ 0.2447` at this z, so the target at 0.248 is
**2.5 mm beyond the reach boundary**. The z residual (+6.4 mm, vs +4.4 mm in
the validated run) puts the TCP a few mm above the cube top, which the pads
span.

An earlier measurement at `x = 0.2212` was rejected by the segmenter
(`partial_top_view`): the top-face min-area extent was **34.5 × 40.8 mm**
because the near edge fell off the bottom of the image
([`measure_view_clipped.png`](measure_view_clipped.png)). This is the
measure≡reach band in practice: at ~0.221 m the top face is already clipped,
and the clipped centroid biases the aim by ~3 mm.

## 3. Result

| Step | Value |
|---|---|
| Grasp joints | `[−0.0263, −0.6618, −1.8012, −0.294]` (j5 = 0) |
| Registers at grasp | 498 / 339 / 929 / **433** (servo 6/5/4/3) |
| Grip check | segmentation at the verify pose: **0 clusters** above the floor |
| Lift | `[−0.020, 0.150, −1.530, −1.020]` — TCP `(0.2453, −0.0034, 0.1013)`, a vertical +136 mm model rise (≈150 mm physical); `error_code 0`, cube held, floor empty |
| Put back | lower to the grasp joints, open `−2.22`, return to the verify pose |
| Release landing | **(0.23505, −0.00364)** = **+0.8 mm x / +6.8 mm y** from the pre-grasp measurement |
| Final cube | top face 38.1 × 39.5 mm, height 39.6 mm, upright |

The y displacement on release (6.8 mm) is the placement residual, not a grip
failure: the cube was gripped, lifted and released in the same run.

## 4. Limits

- One run, one battery state (7.03 V at the start of the base approach).
- The base has no odometry; the placement was closed-loop by vision, and the
  ~7.5 mm y drift over 0.197 m is an open-loop artefact, not a calibration.
- The grasp residual is 3.3 mm radially (x −2.5, y +2.1) against a ±4.3 mm
  grip margin, with the x part forced by the reach boundary. A base position
  ~5 mm nearer would have removed most of it.
- The lift is "150 mm" in the same sense as the validated run: the model z at
  the low configuration carries a ~−13 mm bias, so the model rise is
  +136 mm while the physical rise is ~148–150 mm.
- `joint4` reaches raw **103** at the measurement pose, 17 counts past the
  servo EEPROM limit of 120 (same over-travel as before).

## 5. Files

| File | Content |
|---|---|
| `far_meas4_seg.json` | the clean measurement (38.7 × 39.6 mm top face) |
| `far_verify_seg.json` | grip check: **0 clusters** |
| `far_after_release_seg.json` | cube back on the floor at (0.23505, −0.00364) |
| `grasp_run.json` | servo registers for the grasp run |
| `release_run.json` | the put-back run |
| `measure_view_clipped.png` | the rejected 0.221 m view (near edge clipped) |
| `far_grip_check.png` | cube held at the verify pose |
| `far_lift150.png` | cube held at the ~150 mm lift, floor empty |
| `far_after_release.png` | cube back on the floor, upright |
