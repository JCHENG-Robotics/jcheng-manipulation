# Chessboard Camera / Hand-Eye Calibration

> Status: **Validated on 2026-11-17 (second run)**. Using the physical
> bus-servo positions in FK, 14 chessboard poses give a board-pose scatter of
> **2.94 mm / 0.39°** (max 5.44 mm / 0.74°), the held-out final base is
> **3.29 mm / 0.43°**, and 2-fold cross-validation gives **2.5–3.2 mm /
> 0.31–0.43°**. The 5 mm / 2° board gate passes. The first run (cached
> `joint_states` only) reached 13.4 mm / 2.06° and is kept as the baseline in
> [`evidence/calibration/20261117/`](../../evidence/calibration/20261117/).

## Goal and frames

The camera is mounted on `link4`. The calibrated transform is

```text
T_link4_camera : camera optical frame -> link4
```

The image/PnP pose is in the OpenCV **optical** frame (x right, y down, z
forward). The live `depth_cam_color_optical_frame` and
`depth_cam_depth_optical_frame` are **identity to `rgb_camera_link`** in the
driver TF; they are body frames, not the OpenCV optical frame despite their
names. The live static TF `link4 → rgb_camera_link` is the vendor **body**
frame. The measured body→optical adapter is
`inv(T_link4_rgb_body) @ T_link4_camera` with the full-precision live TF:
**89.798° about -z**, translation `[5.4, 10.1, -27.4] mm`. It is published as
`camera_optical_calibrated`; see
[`docs/hardware/camera_frames.md`](../hardware/camera_frames.md). Use
`T_link4_camera` directly in perception code, or use the published frame in
TF. Do not replace the URDF static TF with the optical transform; add a
separate optical frame if the URDF needs one.

> The earlier 89.34° / `[15.4, 10.0, -28.3] mm` numbers came from a
> three-decimal `tf2_echo` snapshot with x = -0.050 m. The full-precision live
> TF has x = -0.040 m. The hand-eye fit did not use the snapshot, so the
> calibration result is unchanged; only the body→optical adapter changed.

## Target and procedure

| Item | Value |
|---|---|
| Board | OpenCV chessboard, 11×8 inner corners, 20 mm squares |
| Camera | Aurora 930 RGB, 640×400, `bgr8`, `rgb_camera_link` |
| Capture base | `[0.0, 0.59888, -1.25664, -0.71209, 0.0]` (joint1–joint5), i.e. `camera_forward` with joint2 +0.18 rad to centre the board |
| Robot pose | link4 FK from the project URDF, using **measured bus-servo positions** |
| Approach | pre-pose `target − 0.08 rad` on joint1 only, then target (takes up the largest backlash) |
| Capture | 4 s single-point trajectories; 0.5 s settle, 3 warm-up frames, then 3 frames per pose combined by median |
| Detector | `findChessboardCorners` + `cornerSubPix` + `solvePnP(IPPE)` + `solvePnPRefineLM` |
| Servo feedback | `/ros_robot_controller/bus_servo/get_state`, 3 reads per end, median |
| Poses | 15 targets: base, joint1 ±0.12, joint2 ±0.06, joint3 ±0.06, joint4 ±0.12 and mixed-sign combinations; 14 detected |

Servo mapping and conversion (vendor `servo_controller.yaml`):

```text
joint1..joint5 = servo IDs 6, 5, 4, 3, 2
radians = (pulse - 500) * 240/360 * 2*pi / 1000     # joint3 is inverted:
radians = (500 - pulse) * 0.00418879                # pulse 1000 -> -2.094 rad
```

The controller's `/joint_states` and `/controller_manager/servo_states` are
cached commands. The bus-servo read is a hardware position read; at the same
commanded pose it differed from the command by up to **2.16°** (RMS 1.13°),
which is exactly the scatter that limited the first calibration.

Tools:

- `tools/phase2/chessboard_capture.py` — runs inside the robot container.
  Moves the arm, reads the bus servos, saves images, commands, measured joints,
  camera K/D and board PnP to a private directory.
- `tools/phase2/chessboard_hand_eye.py` — runs on the laptop with the Jazzy
  workspace sourced. Computes link4 FK, prefers `measured_joints`, checks
  board-pose consistency, runs all five `cv2.calibrateHandEye` methods,
  compares with the static TF as information, refines the chessboard
  intrinsics and checks a held-out `final_base` sample.

Reproduction (inside the robot container):

```bash
sudo docker cp tools/phase2/chessboard_capture.py ArmPiUltra:/tmp/
sudo docker exec -d -u ubuntu -w /home/ubuntu ArmPiUltra /bin/zsh -c \
  'source ~/.zshrc >/dev/null 2>&1; exec python3 /tmp/chessboard_capture.py \
     --output-dir /tmp/chessboard_calib_servo_20261117 \
     --base-pose 0.0,0.59888,-1.25664,-0.71209 \
     --delta 0.12 --pre-offset 0.08 --pre-joint1-only \
     --final-base --servo-reads 3 > /tmp/calib_servo_run.log 2>&1'
```

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 tools/phase2/chessboard_hand_eye.py \
  --samples ~/jcheng/calibration/20261117/chessboard_calib_servo_20261117/samples.json \
  --output evidence/calibration/20261117/calibration_servo_20261117.json
```

## Validated result (14 poses)

```text
T_link4_camera
translation = [-0.0348, -0.0091, 0.0586] m
RPY         = [0.951, -0.058, -89.789] deg
```

| Metric | Value | Gate |
|---|---|---|
| Board pose in `base_link`, translation std | **2.94 mm** | ≤ 5 mm |
| Board pose in `base_link`, rotation std | **0.39°** | ≤ 2° |
| Board max translation error | 5.44 mm | — |
| Board max rotation error | 0.74° | — |
| Held-out `final_base` translation / rotation | **3.29 mm / 0.43°** | ≤ 5 mm / 2° |
| Chessboard intrinsic refinement | fx 418.31 / fy 418.34 vs driver 418.05 / 419.10; RMS **0.060 px** | ≤ 1 px |
| Corner RMS | 0.09–0.19 px | — |
| Servo shift during a capture | 0–1 pulse (≤ 0.24°) | — |

Method agreement (`board_translation_std_m` / `board_rotation_std_deg`):

| Method | translation (m) | RPY (deg) | board std |
|---|---|---|---|
| **TSAI (selected)** | `[-0.0348, -0.0091, 0.0586]` | `[0.95, -0.06, -89.79]` | 2.94 mm / 0.39° |
| PARK | `[-0.0343, -0.0091, 0.0589]` | `[0.40, -0.11, -89.93]` | 2.97 mm / 0.39° |
| HORAUD | `[-0.0343, -0.0091, 0.0589]` | `[0.40, -0.11, -89.93]` | 2.97 mm / 0.39° |
| DANIILIDIS | `[-0.0360, -0.0083, 0.0600]` | `[0.96, -0.01, -89.94]` | 3.00 mm / 0.39° |
| ANDREFF | `[0.0292, -0.0187, -0.0008]` | `[0.49, -0.11, -89.87]` | 6.31 mm / 0.39° (outlier) |

2-fold cross-validation (fit on 7 poses, evaluate the other 7):

| Split | TSAI held-out | PARK held-out | HORAUD held-out |
|---|---|---|---|
| even | 2.54 mm / 0.31° | 2.51 mm / 0.30° | 2.51 mm / 0.30° |
| odd | 3.16 mm / 0.42° | 3.19 mm / 0.43° | 3.19 mm / 0.43° |

The static-TF comparison is **informational**, not a pass gate: 33.7 mm /
89.34° against the three-decimal snapshot, dominated by the body-vs-optical
frame convention. With the full-precision live TF the measured body→optical
adapter is **89.798° / `[5.4, 10.1, -27.4] mm`** (see
[`docs/hardware/camera_frames.md`](../hardware/camera_frames.md)). The board
scatter, held-out base and cross-validation are the meaningful checks.

## Why the first run failed

The first dataset used the controller's cached `joint_states` as if they were
physical joint angles. The bus-servo read shows the physical positions differ
from those commands by up to 2.16° (RMS 1.13°), depending on the pose. At a
0.32 m board distance, 2° is about 11 mm, which matches the 13.4 mm board
scatter of the first run. A controlled intermediate run with the same centered
base and 15-pose set but cached commands gave 9.9 mm / 1.38°; replacing only
the joint source with measured servo positions reduced the scatter to 2.9 mm.
The camera, board, pose set and detector were unchanged.

The first run's other findings still hold: the board and camera are stable when
the arm does not move (0.02–0.03 mm over repeated frames), and the joint1-only
pre-pose is the validated approach protocol. The earlier claim that
`servo_states` is not feedback was correct for
`/controller_manager/servo_states`; the bus-servo service is the separate
hardware read.

## Limitations

- The arm is open-loop; the bus-servo position is the servo's own register, not
  an independent external metrology. There are no joint encoders.
- One fixed board pose was used. The result is validated by held-out poses and
  a held-out final base, but not by an external target at a surveyed
  base-frame position.
- Link geometry comes from the vendor URDF. Small link/zero errors are absorbed
  into the hand-eye transform and may grow outside the calibration volume
  (roughly 0.25–0.40 m in front of the camera).
- The transform is valid only for the current camera mount. Remounting the
  camera invalidates it.
- Raw RGB images stay private and are not committed.
- This is a calibration result, not an absolute metrology claim. Use it for
  visual grasping with a safety margin; no real grasp has been run yet.

## Next

1. Add an independent validation: place a fiducial at a measured base-frame
   position and compare the calibrated prediction.
2. `T_link4_camera` is now wired into the localizer through the
   `camera_optical_calibrated` frame; next run a supervised, no-object
   approach through the grasp planner.
3. If better than ~3 mm is required, add an external measurement (e.g. a dial
   indicator or a second fixed camera) rather than more chessboard poses; the
   remaining error is the open-loop arm, not the detector.
