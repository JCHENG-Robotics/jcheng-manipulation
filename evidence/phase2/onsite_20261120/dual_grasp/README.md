# Dual-camera supervised grasp — 2026-11-20

> Status: **PASS (grip + put-back).** The chassis-mounted D435i independently
> measured the cube in `base_link` and agreed with the wrist camera's
> view-pose measurement to **2.6 mm** in x/y; it served as a pre-grasp veto and
> as the post-grasp/post-release verifier. The grasp itself used the validated
> wrist target (`C_view + e`), because the dual-camera measurement showed that
> the constant correction `e` is **not** a view-pose measurement bias and
> cannot be replaced by a second camera. The cube was gripped, the wrist
> segmenter found **0 clusters** on the floor, the front view showed no
> cube-sized blob, and the put-back returned the cube with a
> **3.65 mm** planar displacement measured by the pose-independent front
> camera (wrist estimate 5.22 mm).

## 1. What was attempted

One supervised pick–lift–verify–put-back on the 39.4 mm wooden cube with both
cameras active:

| Camera | Mount | Role in this attempt |
|---|---|---|
| Deptrum Aurora 930 | `link4` (wrist) | Validated markerless RGB-D measurement at the view pose; supplies the grasp target with the constant correction `e = (+13.7, +3.9, +8.9) mm`; verifies the floor is empty after the grip |
| Intel RealSense D435i | chassis front, rigid to `base_link` | Independent base-frame top-face measurement (own depth plane + rectified colour); pre-grasp veto; verifies the cube leaves the table and returns |

The front camera is not in the arm loop. It measures the object from a
pose-independent viewpoint; the wrist camera keeps the validated
low-configuration target. This is the honest division of labour established by
the measurements below.

## 2. Measurement method

Front camera (`tools/phase2/ext_cam_capture.py` → `ext_cam_measure.py`):

1. capture colour 1280x720 `bgr8` and depth 640x480 `Z16` (millimetres);
2. RANSAC the support plane in the depth optical frame, map it to `base_link`
   with the SDK depth→colour extrinsic (14.8 mm baseline) and the calibrated
   `T_base_front`;
3. lift the plane by the measured cube edge (39.4 mm) to the top-face plane;
4. rectify the undistorted colour image onto that plane at 0.5 mm/px
   (120 x 90 mm window) — a true homography only after undistortion;
5. segment the saturated, top-face-bright blob and take its centroid.

The rectified blob's long side (43–46 mm) includes the front/side-face
projection at the near edge; the short side (40.5–42.4 mm) is the top-face side
against the 39.4 mm cube. The centroid is the reported point.

Wrist camera: the validated markerless RGB-D segmenter
(`tools/phase2/object_segment_rgbd.py`) with the measured bus-servo registers
and the table-down hand-eye. `/joint_states` is stale on this stack; all wrist
measurements use the registers.

## 3. Measurements

First row: the arm's starting pose (not the validated view pose). Then the
validated view pose `[0, 0.2574, -1.18773, -1.70, -0.01676]`.

| State | Registers (j1..j5) | Wrist top face (m) | Front top face (m) | Front − wrist (mm) |
|---|---|---|---:|---:|---|
| before (start pose) | 497, 596, 830, 111, 500 | (0.231567, −0.006975, −0.045726) | (0.231539, −0.007913, −0.053240) | (+0.03, −0.94, −7.51) |
| view | 497, 561, 791, 101, 496 | (0.228505, −0.007597, −0.047638) | (0.230924, −0.008485, −0.053333) | (+2.42, −0.89, −5.70) |
| after grasp | 497, 556, 792, 101, 496 | none (0 clusters) | no cube-sized blob | — |
| after release | 497, 558, 791, 103, 496 | (0.231858, −0.003589, −0.045954) | (0.231258, −0.004278, −0.053300) | (−0.60, +0.69, −7.35) |

Front-camera repeatability at the two pre-grasp states: x/y/z spread
(0.62, 0.57, 0.09) mm across the arm move. The wrist measurement moved
(3.06, 0.62, 1.91) mm between those states with the cube static. The front
depth plane repeated to 0.2 mm and 0.75–0.79 mm RMS with 85–90% inliers at a
3 mm threshold.

## 4. The finding that shaped the attempt

At the view pose the two cameras agree on the cube's x/y to **2.58 mm**, but
the grasp-validated intervention target is
`C_view + e = (0.24220, −0.00370, −0.03874) m`, i.e. **13.7 mm** farther in x.
If the front camera's physical point had been used as the target directly the
arm would have been commanded 11.5 mm short of the validated target — close to
the ±9 mm gripper-tolerance boundary and outside the 24/24 proven
intervention.

Conclusion: `e` is a **low-configuration execution correction** (the arm's
actual pads at the grasp pose sit short of the nominal fit), not a wrist
measurement bias. A second camera can confirm the object, but it cannot
replace `e` without a new grasp-validation experiment at the front-camera
target. This attempt therefore used:

```text
target = C_view + e                      (validated, 24/24)
joints = grasp_joint_fit(target)          # (−0.0227, −0.6618, −1.8012, −0.3200)
veto   = |C_front.xy − C_view.xy| ≤ 8 mm  # 2.58 mm -> pass
```

The absolute `z` of the front camera is 5.7–7.5 mm below the wrist's. The
front depth plane and the board-based extrinsic already disagreed by 5 mm in
the 2026-11-20 calibration (116 mm depth vs 121 mm board), so this is the known
height ambiguity, not a new failure. Its `x/y` is the useful product.

## 5. Grasp and verification

| Step | Result |
|---|---|
| Approach (j4 -> 0) | registers 497, 338, 929, 495, 500 |
| Descend to grasp | 497, 338, 929, 426, 500 |
| Close (−1.6, then −0.63) | 496, 339, 929, 426, 500 |
| Return to view | 497, 556, 792, 101, 496 |
| Wrist floor check | **0 clusters above the floor** |
| Front view check | 76 small saturated components (max 329 px) vs the ~70 000 px cube; no cube-sized blob; the auto anchor landed at x = 0.602 m and failed the x gate |
| Verdict | **held** (operator-visible; no knock-off) |

The cube was lowered back with the same grip joints, opened, and the arm
returned to the view pose.

Put-back displacement, both cameras:

| Camera | Before (m) | After (m) | Displacement (mm) |
|---|---|---|---:|
| front (pose independent) | (0.231539, −0.007913) | (0.231258, −0.004278) | (−0.28, +3.64), planar **3.65** |
| wrist (view vs release) | (0.228505, −0.007597) | (0.231858, −0.003589) | (+3.35, +4.01), planar **5.22** |

Both agree on the +y ~3.6–4.0 mm component; the x difference is the wrist's
pose-dependent measurement (~3 mm). The top-face yaw changed 0.035 rad -> 0.0
rad (2°). The pose-independent front estimate (3.65 mm) is inside the
2026-11-20 placement mean (3.080 mm) plus its tail.

## 6. Files

| Path | Contents |
|---|---|
| `before/`, `view/`, `after_grasp/`, `release/` | colour and 16-bit depth PNGs, capture metadata, front-camera `measure.json` + overlay, wrist `wrist_segment.json` |
| `grasp_run.json`, `release_run.json` | full action-server logs with bus-servo readbacks |
| `../../../calibration/20261120/depth_to_color_extrinsics.json` | SDK depth→colour extrinsic used to map depth points into the calibrated colour frame |
| `../../../calibration/20261120/front_camera_extrinsic.json` | calibrated `base_link -> ext_cam_optical` |
| `../../../calibration/20261120/uvc_color_intrinsics_refined.json` | refined 1280x720 colour intrinsics |

## 7. Limitations and next steps

- The front camera's absolute height inherits the 5 mm depth-vs-board
  ambiguity; only its x/y is used here.
- The colour stream occasionally delivers 10–12 s stale frames under load; the
  capture tool's `--max-skew`/`--max-age` gates rejected two such attempts and
  the successful captures were re-taken (0.07–0.35 s skew).
- The front camera's far-field depth (0.25–0.45 m) was not exercised in this
  attempt; it is the documented route for base approach and out-of-reach
  objects.
- The obvious next experiment is a pre-registered front-camera-target grasp
  (using `C_front + e` and, separately, `C_front`) to measure whether the
  pose-independent measurement improves placement. This attempt deliberately
  kept the validated target.

Evidence for the calibration and camera deployment:
[`../../../calibration/20261120/`](../../../calibration/20261120/README.md) and
[`../../../../docs/hardware/camera_mount.md`](../../../../docs/hardware/camera_mount.md).
