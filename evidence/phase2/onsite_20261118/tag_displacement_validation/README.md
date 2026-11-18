# Independent AprilTag relative-displacement validation — 2026-11-18

> Status: **PASS for the lateral scale; partial for the table-forward
> direction.** A flat 36h11 sticker (ID 3, 24.9 mm black square) was moved
> 100.0 mm twice on the table, with the robot stationary. The calibrated
> hand-eye predicted **100.67 mm** for the lateral move (+0.67 mm, 0.67 %) and
> **104.87 mm** for the table-forward move (+4.87 mm, 4.87 %); the two
> predicted directions are **92.3°** apart. The absolute table-pose prediction
> is ~90 mm below the table, so the transform is **not** usable for absolute
> 3D at this arm pose; the relative test cancels that constant offset.

## Setup

| Item | Value |
|---|---|
| Tag | AprilTag `DICT_APRILTAG_36h11`, **ID 3**, black square **24.9 mm**, flat printed sticker |
| Camera | Aurora 930 RGB, 640×400 `bgr8`, driver K/D |
| Arm pose | `[joint1..joint5] = [0.0, 0.2974, −1.14773, −1.55404, −0.01676]` (table-down pose) |
| FK joints | **measured bus-servo positions** (`/ros_robot_controller/bus_servo/get_state`) |
| Hand-eye | `T_link4_camera` from `evidence/calibration/20261117/calibration_servo_20261117.json` (TSAI) |
| Capture | 5 RGB frames per position, robot stationary; `chessboard_capture.py --check-only` |
| Detector | `FiducialDetector` (36h11, 24.9 mm, IPPE + `solvePnPRefineLM`) |
| Ruler | steel L-square on the table; the tag edge was kept against the ruler |
| Motion | A → B: 100.0 mm lateral; B → C: 100.0 mm table-forward; Tag kept flat, no rotation |

The table pose is **outside the chessboard calibration pose range** (calibration
used joint4 ≈ −0.71 ± 0.12; this pose is joint4 = −1.55). It was chosen because
the table is a flat, rigid plane and the tag can be slid with a steel ruler.

## Procedure

1. The tag was placed flat on the table with one black-square edge against the
   steel square.
2. Position A was recorded at the start mark; the arm did not move during the
   whole sequence.
3. The tag was slid **100.0 mm** along the horizontal ruler arm to B.
4. The tag was slid **100.0 mm** along the vertical ruler arm to C.
5. Each position was captured as 5 frames; the per-position mean is used.

## Results

Per-position `base_link` mean (m) and 5-frame std (mm):

| Position | x | y | z | std (x, y, z) |
|---|---:|---:|---:|---|
| A | 0.34644 | 0.00493 | −0.09363 | 0.24, 0.01, 0.39 |
| B | 0.33521 | −0.09511 | −0.09256 | 0.50, 0.07, 0.23 |
| C | 0.43962 | −0.10262 | −0.09891 | 0.72, 0.08, 0.70 |

Displacements (predicted in `base_link`, measured by the ruler):

| Segment | Predicted vector (mm) | Predicted norm | Measured | Error |
|---|---|---:|---:|---:|
| A → B (lateral, camera-right) | `[−11.23, −100.04, +1.07]` | **100.67 mm** | 100.0 mm | **+0.67 mm (0.67 %)** |
| B → C (table-forward) | `[+104.40, −7.51, −6.35]` | **104.87 mm** | 100.0 mm | **+4.87 mm (4.87 %)** |
| Angle A→B vs B→C | — | **92.32°** | 90° | +2.32° |

Detection:

| Position | Detections | Max reprojection | Rotation spread |
|---|---:|---:|---:|
| A | 5 / 5 | 0.22 px | 0.46° |
| B | 5 / 5 | 0.16 px | 0.40° |
| C | 5 / 5 | 0.43 px | 0.23° |

## Interpretation

- **Lateral scale is validated.** The camera-right / image-plane displacement
  is 100.67 mm for a ruler-measured 100.0 mm, a 0.67 % error over a 100 mm
  baseline. This is the direction where the PnP is strongest.
- **The table-forward displacement is ~5 % high.** B→C moves the tag farther
  from the camera (0.360 → 0.421 m) and toward the image edge; at the ~48°
  camera/tag incidence the PnP depth/forward axis is the weak direction. The
  5 % error is a view-geometry/PnP limitation at this extrapolated pose, not a
  hand-eye lateral-scale error.
- **Orthogonality is good.** 92.3° for two ruler directions that are 90° apart;
  the 2.3° residual is consistent with ruler/tag alignment and the B→C depth
  error.
- **The absolute table-pose RGB depth is biased.** The tag is on the table
  (physical table ≈ −0.024 m: mecanum chassis 17.54 mm + base mesh bottom
  6.46 mm), but the predicted z is ≈ −0.09 m. A table-down chessboard
  calibration gives 2.7 mm / 0.42° relative scatter but places the board at
  z ≈ −0.070 m; a depth-camera check gives **340 mm** versus a **~380 mm** RGB
  PnP distance. The offset is a **view-dependent RGB PnP depth/tilt bias at the
  ~48° table view** (the 20.0 mm board size and refined K/D were checked), not
  an FK extrapolation. It cancels in relative displacements (the arm never
  moved), but absolute table-top z from RGB only is not usable. Use the depth
  camera for the table plane/z, use a near-normal view, or add a table-view
  depth correction.

## Pass criteria

| Criterion | Result |
|---|---|
| Lateral 100 mm displacement within 1 % | **PASS — 0.67 %** |
| Two ruler directions orthogonal within 5° | **PASS — 92.3° (2.3° error)** |
| Table-forward 100 mm displacement within 2 % | **FAIL — 4.87 %** (view-angle/PnP limit) |
| Absolute table-pose RGB depth near the table plane | **FAIL — ~70 mm low** (48° PnP depth bias; depth camera 340 vs RGB ~380 mm) |
| Detection stability | **PASS — 5/5 each, reprojection ≤0.43 px** |

## Limitations

- No external tracking system; the ruler/tag alignment and the 24.9 mm black
  square measurement are the ground truth (~0.5 mm each).
- One arm pose, one tag orientation, one lighting setup.
- The table pose is outside the camera_forward calibration pose range. Its
  absolute RGB depth was previously described as ~40–50 mm biased at the ~48°
  view, but that number came from a wrong chassis estimate; the depth-plane /
  215 mm ruler correction puts the table at z ≈ −0.078 m and the tag's absolute
  z is 6–13 mm low at the table view. A same-frame matte-tag test measured a
  ~4.6 mm same-surface RGB-vs-depth gap (see
  [`../rgbd_same_surface_20261118.md`](../rgbd_same_surface_20261118.md)).
  This is a view-geometry/PnP and RGB-D registration limit, not a rejection of
  the calibrated `T_link4_camera` for relative/lateral use.
- Raw RGB frames stay private (they show the operator); only the numerical
  JSON is committed.

## Files

| File | Content |
|---|---|
| `validate_A.json`, `validate_B.json`, `validate_C.json` | per-frame detections, measured joints, `base_link` poses and summaries |
| `displacement_summary.json` | A/B/C means, predicted displacements, measured values, errors, angle, detection stats |
| private frames | `~/jcheng/calibration/20261118/tag_validation/sticker_{A,B,C}/` (do not commit) |
| tool | `tools/phase2/validate_hand_eye_tag.py` |

## Reproduction

Capture (inside the robot container, arm already at the table pose):

```bash
ros2 launch bringup vision.launch.py   # if the camera is not running
python3 /tmp/chessboard_capture.py \
  --output-dir /tmp/tag_sticker_A_20261118 \
  --check-only --count 5 --repeats 1 --warmup 2 --settle 0.3
```

Analyze (laptop, Jazzy workspace sourced):

```bash
python3 tools/phase2/validate_hand_eye_tag.py \
  --samples ~/jcheng/calibration/20261118/tag_validation/sticker_A/samples.json \
  --calibration evidence/calibration/20261117/calibration_servo_20261117.json \
  --marker-size 0.0249 --marker-id 3 \
  --output evidence/phase2/onsite_20261118/tag_displacement_validation/validate_A.json
```
