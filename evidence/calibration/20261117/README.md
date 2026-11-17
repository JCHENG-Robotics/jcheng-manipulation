# Chessboard camera / hand-eye calibration — first baseline (2026-11-17)

> Status: **first live baseline; NOT validated**. 8 usable poses out of 13.
> A joint1-only pre-pose protocol reduced same-command repeatability from about
> 24 mm to 0.8 mm, but the full pre-pose dataset was interrupted when the robot
> dropped off WiFi. Raw RGB images are private and are **not** in this
> repository.

## Setup

| Item | Value |
|---|---|
| Board | OpenCV chessboard, **11×8 inner corners, 20 mm squares** |
| Board scale check | 24.8 px/square ≈ 0.337 m at fx 418; depth median ≈ 0.35 m |
| Camera | Aurora 930, 640×400 `bgr8`, frame `rgb_camera_link` |
| Camera intrinsics | driver K/D from `/depth_cam/rgb/camera_info` |
| Robot pose | link4 FK from the project URDF; host FK matched live TF to <0.1 mm |
| Static TF reference | `link4 → rgb_camera_link` = `[-0.050, -0.019, 0.087] m`, RPY `[0.163, 0.583, -0.449]°` (vendor body frame) |

## Data

- `dataset_20261117.json`: 13 target poses; 8 usable (indices
  `0,1,2,3,5,7,9,12`); 5 had the board outside the frame (indices
  `4,6,8,10,11`).
- Procedure: ±0.08 rad on joint1/joint4, ±0.06 rad on joint2/joint3 around the
  `camera_forward` pose; 4 s single-point trajectories via
  `/arm_controller/follow_joint_trajectory`; fresh frame after the action
  result; 11×8 corner detection + `solvePnP`.
- Corner RMS: 0.06–0.34 px.

## Hand-eye result

The solver selected **PARK**:

```text
T_link4_camera (camera in link4):
translation = [0.0053, -0.0090, 0.0326] m
RPY         = [-3.823, -0.527, -101.586] deg
```

Validation:

| Metric | Value |
|---|---|
| Board pose in `base_link`, translation std | **13.4 mm** (max 18.6 mm) |
| Board pose in `base_link`, rotation std | **2.06°** (max 2.64°) |
| Difference from vendor static TF | 78.2 mm / 101.2° |
| Chessboard intrinsic refinement | fx 446.3 / fy 446.9 vs driver 418.1 / 419.1; RMS 0.079 px |

Interpretation and limits:

- The **101° difference is not a simple calibration error**: the live static TF
  is the vendor **body** frame, while the image/PnP pose is in the camera
  **optical** frame. The hand-eye fit improves board consistency versus the
  static TF (40 mm / 4.94°), which is consistent with a real optical-frame roll
  of roughly 90–100° that the vendor static TF does not represent.
- The 13.4 mm / 2.06° board residual is still too large for a validated
  calibration. The dominant cause is the arm's open-loop repeatability/backlash,
  not the board detection (corner RMS <0.35 px).
- The chessboard intrinsic refinement is **inconclusive**: 8 close-range views
  with limited orientation diversity trade focal length against distance. Use
  the driver K/D for perception until a better intrinsic dataset exists.
- The solver prints `RESULT FAIL` because the 5 mm/2° board gate and the
  static-TF gate are not met. The static rotation gate is not physically
  meaningful (body vs optical); the board std is the real limitation.

## Repeatability / backlash finding

| Test | Result |
|---|---|
| Two stationary frames, no motion | **0.03 mm / 0.02°** (board and camera stable) |
| Same base command before/after joint1 +0.08, no pre-pose | **24.1 mm** translation (base tvec `[-0.0854,-0.1260,0.3417]` → final `[-0.0613,-0.1256,0.3433]`) |
| First dataset: base check vs sample 0 | 14.3 mm / 2.48° |
| Same base before/after joint1 +0.08 with **joint1-only pre-pose** (`target − 0.08` then target) | **0.8 mm** translation; board stayed visible |
| All-joint pre-pose (`−0.10` on joints 1–4) | board pushed out of frame; not usable with this geometry |

The joint1-only pre-pose is the recommended protocol: it takes up the largest
backlash without moving the camera out of the board view.

## Interruption and robot state

- The full joint1-pre dataset was started, but the SSH command hit its 280 s
  timeout. The robot then dropped off WiFi (`ARP INCOMPLETE`, no route), `eth0`
  was down, and the laptop's USB Ethernet NIC was no longer present.
- The first dataset and solver report are safe on the laptop. The interrupted
  pre-pose data may remain in the robot container at
  `/tmp/chessboard_calib_j1pre_20261117/` until retrieved or lost on reboot.
- Reconnect power/charger and the direct Ethernet cable
  (`10.42.0.218` ↔ `10.42.0.1`) before the next long capture. WiFi alone was
  not stable for a multi-minute motion sequence.

## Files

| File | Content |
|---|---|
| `calibration_20261117.json` | solver report: all five methods, selected PARK, static comparison, intrinsics, validation |
| `dataset_20261117.json` | 13 poses with joints, camera K/D, board corners/PnP (no images) |
| `static_tf_link4_rgb.txt` | raw `tf2_echo link4 rgb_camera_link` |
| private raw images | `~/jcheng/calibration/20261117/chessboard_calib_20261117/` (operator/office visible; do not commit) |

## Next steps

1. Power/charge the robot and reconnect the direct Ethernet cable; verify SSH
   over `10.42.0.218` before moving.
2. Secure the chessboard (tape/clamp) and verify stationary frames before and
   after the sequence.
3. Retrieve the interrupted pre-pose dataset if present; otherwise rerun the
   full sequence with `--pre-offset 0.08 --pre-joint1-only` over Ethernet
   (`--repeats` defaults to 3 and records per-pose translation spread).
4. Improve rotation diversity (move the camera back or center the board) and
   measure per-joint repeatability; rerun the solver.
5. Do not use the calibrated transform for real grasping until the board std is
   <5 mm / <2° and the open-loop repeatability is characterized.
