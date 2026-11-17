# Chessboard camera / hand-eye calibration — 2026-11-17

> Status: **VALIDATED (second run, measured bus-servo positions)**.
> 14 usable poses; board-pose scatter **2.94 mm / 0.39°** (max 5.44 mm /
> 0.74°); held-out final base **3.29 mm / 0.43°**; 2-fold cross-validation
> **2.5–3.2 mm / 0.31–0.43°**. The 5 mm / 2° gate passes. The first run
> (cached commands only) is kept below as the 13.4 mm / 2.06° baseline.
> Raw RGB images are private and are **not** in this repository.

## Validated result (servo feedback)

The solver selected **TSAI**; PARK/HORAUD/DANIILIDIS agree within ~1.7 mm and
~0.6°.

```text
T_link4_camera (camera optical frame in link4):
translation = [-0.0348, -0.0091, 0.0586] m
RPY         = [0.951, -0.058, -89.789] deg
```

| Metric | Value |
|---|---|
| Board pose in `base_link`, translation std | **2.94 mm** (max 5.44 mm) |
| Board pose in `base_link`, rotation std | **0.39°** (max 0.74°) |
| Held-out `final_base` translation / rotation | **3.29 mm / 0.43°** |
| Cross-validation held-out (even / odd) | 2.51–2.54 / 3.16–3.19 mm; 0.30–0.31 / 0.42–0.43° |
| Chessboard intrinsic refinement | fx 418.31 / fy 418.34 vs driver 418.05 / 419.10; RMS **0.060 px** |
| Corner RMS | 0.09–0.19 px |
| Servo shift during a capture | 0–1 pulse (≤ 0.24°) |
| Static TF vs hand-eye (informational) | 33.7 mm / 89.34° |

`inv(static_tf_link4_rgb) @ T_link4_camera` is an **89.34° rotation about the
-z axis** with translation `[15.4, 10.0, -28.3] mm`; this is the expected
body-frame → optical-frame 90° roll plus an optical-centre offset. The static
TF is **not** an optical extrinsic and is not a pass gate.

> **Update (2026-11-17, Session 1):** the three-decimal snapshot used above had
> x = -0.050 m. The full-precision live TF is
> `[-0.039995, -0.019122, 0.086003] m`; the measured adapter is **89.798°**
> with translation **`[5.4, 10.1, -27.4] mm`** and is published as
> `camera_optical_calibrated`. The hand-eye fit is unchanged. See
> [`evidence/phase2/frames_20261117/`](../../phase2/frames_20261117/).

Frame note: the live `depth_cam_color_optical_frame` is identity to
`rgb_camera_link` in TF, so it is a body frame, not the OpenCV/PnP optical
frame. Use `T_link4_camera` directly in perception code, or the published
`camera_optical_calibrated` frame in TF.

## Why the second run succeeded

The controller's `/joint_states` and `/controller_manager/servo_states` are
cached commands. The hardware read
`/ros_robot_controller/bus_servo/get_state`
(`ros_robot_controller_msgs/srv/GetBusServoState`) returns the servo's own
position register. At the same commanded pose the measured positions differed
by up to **2.16°** (RMS 1.13°), depending on the pose; at 0.32 m that is ~11 mm,
matching the first run's 13.4 mm scatter. Feeding `measured_joints` into FK
reduced the board scatter to 2.9 mm without changing the camera, board or
solver.

Servo mapping from the vendor `servo_controller.yaml`:

```text
joint1..joint5 = servo IDs 6, 5, 4, 3, 2
radians = (pulse - 500) * 240/360 * 2*pi / 1000
joint3 is inverted: radians = (500 - pulse) * 0.00418879
```

## Intermediate run (same poses, cached commands)

Before the servo read was added, the same capture base and 15-pose set with
cached `joint_states` gave 14 usable poses and a board scatter of
**9.9 mm / 1.38°** (max 19.3 mm / 2.56°); intrinsic refinement fx 417.57,
RMS 0.055 px (`calibration_center_20261117.json`,
`dataset_center_20261117.json`). Replacing only the joint source with the servo
read reduced the scatter to 2.9 mm. This isolates the improvement to the
measured joints, not the pose set or detector.

## Validated-run procedure

| Item | Value |
|---|---|
| Board | OpenCV chessboard, 11×8 inner corners, 20 mm squares |
| Camera | Aurora 930, 640×400 `bgr8`, frame `rgb_camera_link` |
| Capture base | `[0.0, 0.59888, -1.25664, -0.71209, 0.0]` (`camera_forward` + joint2 0.18 rad, board centred) |
| Approach | `target − 0.08 rad` on joint1, then target |
| Capture | 4 s single-point trajectories, 0.5 s settle, 3 warm-up frames, 3 frames per pose (median) |
| Detector | `findChessboardCorners` + `cornerSubPix` + `solvePnP(IPPE)` + `solvePnPRefineLM` |
| Servo read | 3 reads before and after the frames, per-servo median |
| Poses | 15 targets; 14 detected (index 11 not detected) |

Per-pose details are in `dataset_servo_20261117.json`: commanded joints,
`measured_joints` (start/end), raw pulses, board PnP/corners and the 3-frame
spread.

## Independent checks

- **Held-out final base**: the `final_base` sample is excluded from the fit and
  predicted at 3.29 mm / 0.43°.
- **2-fold cross-validation** (offline, `validation_servo_20261117.json`): fit
  on 7 poses, evaluate the other 7. Held-out board scatter 2.51–3.19 mm /
  0.30–0.43° for TSAI/PARK/HORAUD.
- **Intrinsics**: the chessboard refinement (fx 418.31 vs driver 418.05) now
  agrees with the driver, unlike the first run (446.3). This is an independent
  check that the camera and dataset are clean.
- **Frame convention**: the 89.34° roll about -z (three-decimal snapshot) and
  the full-precision 89.798° adapter both match the expected 90° body/optical
  difference. The full-precision value is the published
  `camera_optical_calibrated` transform.

## First run (baseline, cached commands)

The first run used cached `joint_states` as physical joint angles: 8 usable
poses, PARK, board scatter **13.4 mm / 2.06°**, static-TF difference
78.2 mm / 101.2°, intrinsic refinement fx 446.3 vs driver 418.1 (inconclusive).
It is preserved in `calibration_20261117.json` and
`dataset_20261117.json`.

Repeatability findings from that run:

| Test | Result |
|---|---|
| Two stationary frames, no motion | 0.03 mm / 0.02° |
| Same base command before/after joint1 +0.08, direct approach | 24.1 mm |
| Same test with joint1-only pre-pose (`target − 0.08`, then target) | 0.8 mm |
| All-joint pre-pose (`−0.10` on joints 1–4) | camera out of board view; not used |

The second run confirms the joint1-only pre-pose and the stationary-frame
stability; the remaining ~3 mm is the open-loop servo tracking error that the
bus-servo read captures.

## Files

| File | Content |
|---|---|
| `calibration_servo_20261117.json` | validated solver report: all methods, selected TSAI, board stats, held-out base, intrinsics, joint source |
| `dataset_servo_20261117.json` | 16 samples: 15 targets (14 detected) + final base, with commands, measured joints, pulses, K/D, board corners/PnP (no images) |
| `validation_servo_20261117.json` | held-out base, 2-fold CV, static-TF relation, method agreement |
| `capture_servo_20261117.log` | capture console log: base approach, per-pose pre-pose/target, board RMS/spread/servo shift, return to base |
| `calibration_center_20261117.json` | intermediate same-pose run with cached commands (9.9 mm / 1.38°) |
| `dataset_center_20261117.json` | intermediate dataset (16 poses, no images) |
| `calibration_20261117.json` | first-run solver report (cached commands) |
| `dataset_20261117.json` | first-run dataset (13 poses, no images) |
| `static_tf_link4_rgb.txt` | raw `tf2_echo link4 rgb_camera_link` |
| private raw images | `~/jcheng/calibration/20261117/chessboard_calib_servo_20261117/` and `.../chessboard_calib_20261117/` (do not commit) |

## Limitations

- The arm is open-loop; the bus-servo position is the servo's own register, not
  an independent external metrology. No joint encoders.
- One fixed board pose. Validated by held-out poses and a held-out base, but not
  by an external target at a surveyed base-frame position.
- Vendor URDF link geometry; small model errors are absorbed into the hand-eye
  transform and may grow outside the ~0.25–0.40 m calibration volume.
- Valid only for the current camera mount.
- No real grasp has been run. Use the transform with a safety margin.

## Next steps

1. Independent target validation: fiducial or marker at a measured base-frame
   position, compare the calibrated prediction.
2. Wire the transform into the fiducial localizer / grasp planner and run a
   supervised, no-object approach.
3. Add an external measurement if better than ~3 mm is required; more
   chessboard poses will not remove the open-loop arm error.
