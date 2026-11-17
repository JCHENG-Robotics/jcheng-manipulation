# Chessboard Camera / Hand-Eye Calibration

> Status: **first live baseline (2026-11-17); not validated**. The camera
> streams RGB/depth and the chessboard pose is stable, but the arm's open-loop
> repeatability limits the hand-eye fit. Raw images stay private; the numerical
> evidence is in
> [`evidence/calibration/20261117/`](../../evidence/calibration/20261117/).

## Goal and frames

The camera is mounted on `link4`. The calibrated transform is

```text
T_link4_camera : camera optical frame -> link4
```

The image/PnP pose is in the camera **optical** frame. The live static TF
`link4 → rgb_camera_link` is the vendor **body** frame; it is not necessarily
the image optical frame. A first fit found a ~101° roll between the two, so do
not use the static TF alone as the measured optical extrinsic.

## Target and procedure

| Item | Value |
|---|---|
| Board | OpenCV chessboard, 11×8 inner corners, 20 mm squares |
| Camera | Aurora 930 RGB, 640×400, `bgr8`, `rgb_camera_link` |
| Robot pose | link4 FK from the project URDF (matches live TF to <0.1 mm) |
| Capture | small joint offsets (±0.06–0.08 rad), 4 s trajectories, fresh frame after the action result |
| Detector | `findChessboardCorners` + `cornerSubPix` + `solvePnP` |

Tools:

- `tools/phase2/chessboard_capture.py` — runs inside the robot container
  (OpenCV + vendor `FollowJointTrajectory` action). Saves images, joints,
  camera K/D and board PnP to a private directory.
- `tools/phase2/chessboard_hand_eye.py` — runs on the laptop with the Jazzy
  workspace sourced. Computes link4 FK, checks board-pose consistency, runs all
  five `cv2.calibrateHandEye` methods, compares with the static TF and reports
  a chessboard intrinsic refinement.

## First result (8 usable poses)

| Metric | Value |
|---|---|
| Selected method | PARK |
| `T_link4_camera` translation | `[0.0053, -0.0090, 0.0326] m` |
| `T_link4_camera` RPY | `[-3.823, -0.527, -101.586]°` |
| Board pose in `base_link`, translation std | **13.4 mm** |
| Board pose in `base_link`, rotation std | **2.06°** |
| Static TF vs hand-eye difference | 78.2 mm / 101.2° |
| Corner RMS | 0.06–0.34 px |
| Chessboard intrinsic refinement | fx 446.3 vs driver 418.1; RMS 0.079 px (inconclusive) |

The hand-eye fit improves board consistency versus the vendor static TF
(40 mm / 4.94°), but 13.4 mm / 2.06° is not a validated calibration. Use the
driver K/D for now; the refined focal length is not trustworthy with 8
close-range, low-diversity views.

## Open-loop repeatability is the limit

The vendor `joint_states` are cached commands, not measured feedback. Tests on
2026-11-17:

- Two stationary frames: **0.03 mm / 0.02°** — the board and camera are stable
  when the arm does not move.
- Same base command before/after a joint1 +0.08 move, direct approach:
  **24.1 mm** translation (another base check: 14.3 mm / 2.48°).
- Same test with a **joint1-only pre-pose** (`target − 0.08 rad`, then target):
  **0.8 mm** translation; the board stays visible.

Use the joint1-only pre-pose for future captures. An all-joint pre-pose
(`−0.10` on joints 1–4) moved the camera out of the board view and is not
usable with this geometry.

## Vendor baseline

The private vendor calibration app (`app/calibration.py`) uses a **25 mm
AprilTag 36h11, IDs 1/2/3**, a fixed white area (0.167 × 0.108 m) and
`/kinematics/get_current_pose`; it writes `transform.yaml`. It is a white-area
extrinsic baseline, not a multi-pose hand-eye calibration. It was **not run**
in this session because the target placed on the table is a chessboard, not the
vendor tag. Running it requires the vendor tag and a separate comparison.

## Limitations

- Not validated: board std 13.4 mm / 2.06° is above the 5 mm / 2° gate.
- The arm is open-loop; repeatability/backlash is a first-order error source.
- The board was on a stand and may have shifted between runs; it must be
  taped/clamped and checked before/after.
- The 101° body/optical difference is a hypothesis from the fit; it needs an
  independent check (e.g., a known target at a known base position).
- The full joint1-pre dataset was interrupted by a robot WiFi drop; the first
  dataset is the one reported here.
- No real grasp or fiducial target at a known base position has been used to
  validate the calibrated transform.

## Next

1. Reconnect the robot over direct Ethernet, charge the battery, secure the
   board.
2. Rerun the full capture with
   `--pre-offset 0.08 --pre-joint1-only` and larger rotation diversity.
3. Add an independent validation: place a fiducial at a measured base-frame
   position and compare the calibrated prediction.
4. Only then use the transform for visual grasping; keep the vendor static TF
   clearly labelled as the body-frame nominal.
