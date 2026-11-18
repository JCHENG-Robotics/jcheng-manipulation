# Step 5 — real AprilTag detection through the calibrated optical frame

Date: 2026-11-18 09:48–09:52 (robot local time, UTC+8). Arm static at
`camera_forward`; no motion during capture.

## Setup

| Item | Value |
|---|---|
| Marker | printed AprilTag 36h11, **id 0**, 0.06 m black square |
| Placement | hand-held ~0.32 m in front of the camera, facing it |
| Camera | Aurora 930 RGB 640×400 `bgr8`, `rgb_camera_link` |
| Pose | `camera_forward` (`joint1≈0.008`, `joint2≈0.415`, `joint3≈−1.244`, `joint4≈−0.716`, `joint5≈0.008`) |
| Capture | 5 frames, 0.3 s apart, via `tools/phase2/capture_real_frames.py` in the Humble/CycloneDDS container |
| Detection | `FiducialDetector` (36h11, 0.06 m, IPPE + `solvePnPRefineLM`), captured intrinsics |
| Frame chain | `T_base_marker = T_base_rgb_body @ T_rgb_optical @ T_optical_marker` |
| `T_base_rgb_body` | live TF captured with each frame |
| `T_rgb_optical` | `armpi_fiducial_localizer/config/camera_optical_frame.yaml` (measured 89.798°, `[5.4, 10.1, −27.4] mm`) |

## Per-frame results

| Frame | Reprojection (px) | Distance (m) | Tilt (°) | Base position (m) |
|---|---:|---:|---:|---|
| 00 | 0.17 | 0.322 | 3.0 | (0.486, −0.028, 0.297) |
| 01 | 0.07 | 0.322 | 1.0 | (0.486, −0.029, 0.295) |
| 02 | 0.10 | 0.322 | 2.1 | (0.486, −0.029, 0.296) |
| 03 | 0.05 | 0.322 | 2.1 | (0.486, −0.029, 0.296) |
| 04 | 0.07 | 0.322 | 2.5 | (0.486, −0.029, 0.296) |

All five frames detected **id 0** with no false IDs.

## Stability summary

| Metric | Value |
|---|---|
| Detection rate | **5 / 5** |
| Base translation mean | **(0.4859, −0.0286, 0.2958) m** |
| Base translation std | (0.22, 0.07, 0.37) mm |
| Base translation range | (0.66, 0.21, 1.11) mm |
| Base rotation spread | **2.32°** (hand-held tag) |
| Camera distance | 0.322 m |
| Camera-frame tilt | 1.0–3.0° (mean 2.14°) |
| Max reprojection error | **0.17 px** |

The marker sits ~0.292 m forward of the camera and ~16 mm above it in
`base_link`; the pose is physically plausible for the hand-held placement.

## Pass criteria

| Criterion | Result |
|---|---|
| Stable detection | PASS (5/5, sub-pixel reprojection) |
| Plausible pose | PASS (0.32 m in front, small tilt) |
| No jumps | PASS (≤1.1 mm translation range; 2.3° rotation spread is consistent with hand motion) |

## Caveats

- The tag was **hand-held**, so the 2.3° rotation spread is not a detector
  repeatability figure. A fixed tag would be needed to separate detector noise
  from operator motion.
- There is **no surveyed ground-truth marker position**; this checks detection
  stability and plausibility, not absolute accuracy.
- The live ROS `fiducial_localizer` node was **not** run: the laptop's Jazzy
  install has no `rmw_cyclonedds_cpp`, so it cannot subscribe to the robot's
  CycloneDDS camera topics. The same detector and the same measured frame
  contract were composed offline from captured frames + live TF. The live
  `target_pose` topic/TF path remains covered only by the mock E2E test.
- The 0.06 m size refers to the black marker square, not the white quiet zone.

## Files

| File | Content |
|---|---|
| `frame_00..04.ppm` | Raw captured RGB frames |
| `frame_00..04.json` | Intrinsics, timestamps, `base_link -> rgb_camera_link`, joint state |
| `frame_00..04.png` | Viewable copies |
| `frame_00..04_overlay.png` | Detected marker axes |
| `summary.json` | Per-frame poses and stability statistics |
| `../../../../tools/phase2/capture_real_frames.py` | Capture tool |
| `../../../../tools/phase2/analyze_real_tag.py` | Detection/composition tool |
