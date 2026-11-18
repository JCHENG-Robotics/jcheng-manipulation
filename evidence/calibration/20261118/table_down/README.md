# Table-down chessboard calibration and absolute-depth check — 2026-11-18

> Status: **relative calibration PASS; absolute-depth interpretation superseded
> by a later depth-plane + ruler check.** The table-down fit has a
> **2.7 mm / 0.42°** board scatter, a 2.2 mm / 0.29° held-out final base and a
> 3.1 mm / 0.45° leave-one-out std. The camera_forward eye-to-hand transform
> remains the primary validated transform. The original claim that the RGB PnP
> is ~46 mm below the table was based on a chassis estimate; the follow-up
> depth plane and a 215 mm ruler check put the table at **z ≈ −0.078 m**, with
> the RGB board top 6.1 mm above it (board thickness). See
> [the follow-up](#follow-up-2026-11-18-depth-plane-and-ruler-check).

## Setup

| Item | Value |
|---|---|
| Board | OpenCV chessboard, 11×8 inner corners, **20.0 mm squares (measured with a caliper)** |
| Placement | flat on the table, under the table-down camera view |
| Arm pose | `[joint1..joint5] = [0.0, 0.2974, −1.14773, −1.55404, −0.01676]` |
| Robot base | mecanum-wheel chassis; **base bottom 17.54 mm above the table**; base/table are the same plane |
| FK joints | measured bus-servo positions |
| Capture | 15 targets around the table pose + final base; 4 s trajectories, joint1-only pre-pose, 3 frames/pose |
| Detection | 16/16 samples, corner RMS 0.058–0.069 px |
| Intrinsics | driver K/D; 16-view refinement fx 417.37 / fy 417.89, RMS 0.052 px |

## Relative result (PASS)

Selected TSAI:

```text
T_link4_camera (table-view fit)
translation = [-0.03194, -0.00808, 0.05918] m
RPY         = [-1.478, 0.331, -90.187] deg
```

| Metric | Value |
|---|---|
| Board pose in `base_link`, translation std | **2.7 mm** |
| Board pose in `base_link`, rotation std | **0.42°** |
| Board max translation / rotation error | 6.1 mm / 0.88° |
| Held-out final base | 2.2 mm / 0.29° |
| Leave-one-out std | 3.1 mm / 0.45°; max 7.2 mm / 0.94° |
| Intrinsic refinement RMS | 0.052 px |

The table-view transform differs from the camera_forward TSAI result by
`[2.9, 1.0, 0.6] mm` and ~2.5° of rotation. Both fits are internally
consistent; the difference is within the pose/view-dependent calibration
uncertainty. The table-view fit does **not** replace the camera_forward fit.

## Absolute-depth finding (do not use RGB-only at this view)

> **Superseded 2026-11-18 (later session).** The −0.024 m "physical table"
> below is a chassis-geometry estimate. A depth point-cloud plane plus a
> direct 215 mm ruler measurement put the table at **z ≈ −0.078 m** and the
> RGB board top 6.1 mm above it (board thickness). Read the
> [follow-up](#follow-up-2026-11-18-depth-plane-and-ruler-check) before using
> the numbers in this section.

| Quantity | Value |
|---|---|
| Physical table plane in `base_link` | ≈ −0.024 m (base bottom 17.54 mm above the table + base mesh bottom −6.46 mm) |
| Board mean z from the table-view fit | **−0.0704 m** (≈46 mm below the table) |
| Tag z from the same view (24.9 mm sticker) | ≈ −0.093 m |
| Depth-camera median at the table pose | **340 mm** |
| RGB chessboard PnP distance | ≈380 mm |

Checks that rule out the simple explanations:

- **Board size**: one square measured **20.0 mm**, matching the calibration.
- **Intrinsics/distortion**: re-running the board PnP with the table-view
  refined K/D changes z by only ~1.2 mm (driver −0.0703 → refined −0.0691 at
  the same pose).
- **Base/table height**: the mecanum chassis adds 17.54 mm; it explains only
  part of the ~46 mm offset.
- **Depth camera**: the independent depth median is 340 mm, while the RGB PnP
  is ~380 mm — the RGB view overestimates depth by roughly 40 mm.

Conclusion: at ~48° incidence the RGB planar PnP has a **systematic
depth/tilt bias** at this table view. The board consistency across small pose
offsets is excellent (2.7 mm), so the bias is nearly constant and the hand-eye
fit absorbs part of it; it does not affect the lateral/relative displacement
validation (0.67 % over 100 mm). For absolute table-top positioning, use the
**depth camera** for the table plane/z, use a near-normal camera view, or add a
table-view depth-correction calibrated against a known plane. Do not feed the
table-view RGB-only absolute z into a grasp target.

## Follow-up (2026-11-18): depth plane and ruler check

The absolute-depth conclusion above was re-tested with an independent depth
plane and a direct ruler measurement. Details:
[`evidence/phase2/onsite_20261118/depth_plane_20261118.md`](../../../phase2/onsite_20261118/depth_plane_20261118.md).

| Check | Result |
|---|---|
| `/depth_cam/depth_registered/points` RANSAC + SVD (table-down pose) | 147,626 inliers, **0.734 mm RMS**, normal **3.66°** from `base_link` +z |
| Table z from the depth plane | **−0.0787 m** under the camera, −0.0810 m at `(0.34, 0)` |
| Gripper fingertip height above the table (ruler) | **215 mm** |
| TCP FK z (measured bus-servo joints) | 0.1366 m → table z **−0.0784 m** |
| Depth plane vs ruler | **1.3 mm** |
| RGB board plane (16 samples) | −0.07054 ± 0.00165 m |
| Board top − depth plane | **+6.1 mm** (board thickness) |

Findings:

- The 340 mm "depth median" above was a raw scene median (it includes closer
  objects), not a table-plane distance. The table plane's inlier median is
  0.3676 m and the plane under the camera is ~0.354 m.
- The RGB PnP and the depth camera agree on the absolute table height within
  the 6.1 mm board thickness. There is no 40–50 mm RGB depth bias at this
  view.
- The table is at **z ≈ −0.078 m**, not −0.024 m. The chassis estimate is
  ~54 mm off; do not use it as ground truth.
- The AprilTag absolute z is separately biased low (6–13 mm at A/B/C,
  tag-plane tilt 8.4° vs table 3.7°); the lateral 0.67 % scale result stands.
- The **table-view TSAI** remains the correct hand-eye for this view. Using the
  camera_forward TSAI at the table pose shifts absolute z by ~10 mm.

## Files

| File | Content |
|---|---|
| `calibration_table_down_20261118.json` | solver report: TSAI T, board stats, held-out and LOO |
| `dataset_table_down_20261118.json` | 16 samples with measured joints, K/D, board corners/PnP (no images) |
| `depth_check_20261118.txt` | depth-camera command/output and interpretation |
| private images | `~/jcheng/calibration/20261118/table_down/images/` (do not commit) |

## Reproduction

Capture (camera running, board flat on the table):

```bash
python3 /tmp/chessboard_capture.py \
  --output-dir /tmp/chessboard_table_20261118 \
  --base-pose 0.0,0.30,-1.15,-1.55,-0.013 \
  --delta 0.08 --pre-offset 0.08 --pre-joint1-only --duration 4.0 --final-base
```

Solve:

```bash
python3 tools/phase2/chessboard_hand_eye.py \
  --samples evidence/calibration/20261118/table_down/dataset_table_down_20261118.json \
  --output evidence/calibration/20261118/table_down/calibration_table_down_20261118.json
```

Depth check (inside the robot container):

```bash
python3 /tmp/camera_stream_check.py --seconds 3
```
