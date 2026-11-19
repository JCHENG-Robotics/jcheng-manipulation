# Fiducial Target Localization

> Status: **Implemented; synthetic and real-background validated; physical-tag
> scale check (2026-11-16/18)**. The real Aurora 930 streams RGB/depth and the
> camera optical extrinsic is validated (2.94 mm / 0.39° board scatter). The
> detector refines the PnP pose with `solvePnPRefineLM`. A **physical** AprilTag
> at a surveyed position was measured on 2026-11-18: 100.67 mm vs 100.0 mm
> lateral (0.67 %), 104.87 mm vs 100.0 mm table-forward (4.87 %). The real
> floor-cube grasps on 2026-11-19 were **markerless**, so this detector was not
> the grasp input.
>
> Plan reference: `MASTER_PLAN_V3_PRIVATE.md` §7 and §8 (fiducial detection →
> target pose → TF → constrained grasp).

## 1. Purpose

The grasp candidate search takes a target point in `base_link`. This package
provides the missing perception input:

```text
image + CameraInfo
→ AprilTag/ArUco 36h11 detection
→ 6D marker pose in the camera optical frame
→ optional TF transform to base_link
→ target_pose (PoseStamped)
→ grasp_candidate_check --target-point ...
```

It does not move the robot and does not perform calibration. The marker pose is
the marker centre, not a grasp point offset on an object.

## 2. Package

```text
ros2_ws/src/armpi_fiducial_localizer/
├── armpi_fiducial_localizer/
│   ├── detector.py             # OpenCV detection + solvePnP + LM refine
│   ├── frames.py               # frame contract + pose-frame resolution
│   ├── camera_frame_publisher.py  # measured body→optical static TF
│   ├── synthetic.py            # synthetic marker renderer for tests
│   ├── node.py                 # ROS 2 localizer node
│   ├── synthetic_publisher.py  # image + CameraInfo publisher
│   └── cli.py                  # synthetic accuracy check
├── config/camera_optical_frame.yaml  # measured frame contract
├── launch/fiducial_localizer.launch.py
└── test/                       # detector, frame contract, lint
```

`detector.py`, `frames.py` and `synthetic.py` are pure OpenCV/NumPy/yaml. The
node depends on `cv_bridge`, `tf2_ros` and `tf2_geometry_msgs`.

## 3. Marker convention

The default dictionary is `DICT_APRILTAG_36h11` (AprilTag 36h11). The detector
uses the square marker's four corners, `solvePnP(SOLVEPNP_IPPE_SQUARE)` and,
when available, `solvePnPRefineLM`.

The marker frame is:

| Axis | Direction |
|---|---|
| x | marker right |
| y | marker up |
| z | out of the printed face toward the viewer |

The returned pose is:

```text
p_camera = R_camera_marker * p_marker + t_camera_marker
```

The camera optical frame follows REP-103: x right, y down, z forward. A marker
facing the camera therefore has its marker z axis roughly opposite the camera
z axis. This is why the synthetic renderer's default `rvec` is 180° about x;
rendering with `rvec` near zero shows the mirrored back face and is not
detected reliably.

`marker_size_m` must match the printed edge length of the black marker square,
not the white quiet zone. The default is **0.06 m**.

## 4. ROS node

### Topics

| Direction | Topic | Type |
|---|---|---|
| subscribe | `image_raw` | `sensor_msgs/Image` |
| subscribe | `camera_info` | `sensor_msgs/CameraInfo` |
| publish | `target_pose` | `geometry_msgs/PoseStamped` |

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `image_topic` | `image_raw` | input image |
| `camera_info_topic` | `camera_info` | input intrinsics |
| `marker_dictionary` | `DICT_APRILTAG_36h11` | ArUco dictionary name |
| `marker_size_m` | `0.06` | printed marker edge length |
| `marker_id` | `-1` | `-1` = any; otherwise filter one ID |
| `camera_frame` | `camera_optical_calibrated` | frame that owns the PnP pose |
| `output_frame` | `''` | empty = camera frame; e.g. `base_link` |
| `max_reprojection_error_px` | `3.0` | reject poor corner fits |
| `tf_timeout_s` | `0.1` | TF lookup timeout |
| `publish_on_tf_failure` | `true` | publish in camera frame if TF fails |

The node selects the detection with the lowest reprojection error after the
optional marker-ID filter. It does not average or filter across frames.

### Frames

The RGB `CameraInfo` is tagged `rgb_camera_link`, which is a **body** frame.
PnP returns the **optical** frame. The node therefore tags `target_pose` with
`camera_optical_calibrated` (or the `camera_frame` override), never with the
incoming body frame. The measured body→optical transform is published as a
static child of `rgb_camera_link` by `camera_frame_publisher`, which the launch
file starts by default. See
[`docs/hardware/camera_frames.md`](../hardware/camera_frames.md).

### Run on the real camera

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 launch armpi_fiducial_localizer fiducial_localizer.launch.py \
  image_topic:=/depth_cam/rgb/image_raw \
  camera_info_topic:=/depth_cam/rgb/camera_info \
  marker_size_m:=0.06 marker_id:=0 output_frame:=base_link
```

The tool `tools/phase2/capture_still.py` captures real frames (no motion) for
offline tests:

```bash
# inside the robot container
python3 /tmp/capture_still.py --output-dir /tmp/real_capture_20261117 \
  --count 8 --interval 0.4 --depth --tf link4:rgb_camera_link
```

For a synthetic-only check, start the synthetic publisher with the localizer
(e.g. `--tvec 0 0 0.4 --rvec 3.141592653589793 0 0 --marker-size 0.06`).

## 5. Validation

### Synthetic accuracy (2026-11-16)

The check renders five markers at known camera-frame poses (0.30–0.45 m,
tilts up to 0.25 rad), runs detection and pose estimation, and reports
translation, rotation and reprojection errors.

| Metric | Value |
|---|---|
| Poses rendered / detected | 5 / 5 |
| Max translation error | **1.82 mm** |
| Mean translation error | 0.67 mm |
| Max rotation error | **1.49°** |
| Mean rotation error | 0.64° |
| Max corner reprojection error | 0.32 px |

Re-running the same five poses with the current detector (which enables
`solvePnPRefineLM`) gives max **1.63 mm / 0.96°**, mean 0.61 mm / 0.56°; the
2026-11-16 JSON remains the pre-refinement record. The real-background check
below is the current end-to-end detector result.

### Real-background semi-synthetic (2026-11-17)

Eight real RGB frames were captured at the current pose (no motion) with the
Aurora 930 and the driver intrinsics. A 36h11 marker with a white quiet zone
was composited at 15 known camera-frame poses (3 backgrounds × 5 poses;
0.23–0.30 m, in-plane rotations up to ±30°, tilts up to 12°).

| Metric | Value |
|---|---|
| Unmodified real frames | 8 frames, **0** false detections |
| Semi-synthetic poses detected | **15 / 15** |
| Max translation error | **1.69 mm** |
| Mean translation error | 1.12 mm |
| Max rotation error | **0.53°** |
| Mean rotation error | 0.30° |
| Gate (5 mm / 2°) | PASS |

The detector refinement matters: without `solvePnPRefineLM`, the
fronto-parallel case had a 1.82° rotation error from the IPPE ambiguity; with
refinement it is 0.10°.

Evidence:
[`evidence/phase2/real_background_20261117/`](../../evidence/phase2/real_background_20261117/)
(JSON + contact sheet). Raw frames stay private in
`~/jcheng/phase2/real_capture_20261117/`.

### Frame contract (2026-11-17)

The measured body→optical adapter is 89.798° about -z,
`[5.4, 10.1, -27.4] mm`; composing it with the full-precision live
`link4 → rgb_camera_link` reproduces `T_link4_camera` to `1.4e-17`. Evidence:
[`evidence/phase2/frames_20261117/`](../../evidence/phase2/frames_20261117/).

### ROS integration check (2026-11-16)

The localizer was exercised through ROS with a synthetic image, a static
`base_link → camera_optical_frame` transform (0.5 m) and the pose check tool
[`tools/phase2/fiducial_ros_check.py`](../../tools/phase2/fiducial_ros_check.py).

| Output frame | Expected position | Position error | Angle error | Result |
|---|---|---|---|---|
| `camera_optical_frame` | (0, 0, 0.4) m | 0.44 mm | 0.03° | PASS |
| `base_link` | (0, 0, 0.9) m | 0.44 mm | 0.03° | PASS |

That check predates the `camera_optical_calibrated` frame; it validates the
TF wiring, not the real optical extrinsic. Evidence:
[`evidence/fiducial/20261116/`](../../evidence/fiducial/20261116/).

## 6. OpenCV 4.6 compatibility note

OpenCV 4.6 exposes the newer `DetectorParameters` type but constructing it
directly and setting `cornerRefinementMethod` segfaults. The code therefore
prefers the legacy `DetectorParameters_create()` / `detectMarkers()` API when
it exists, and falls back to the newer API on later OpenCV versions. This is a
practical portability fix, not a claim about OpenCV quality.

## 7. Limitations

- The real-background check is **semi-synthetic**: the marker is composited,
  not physically present. It validates detection and pose math. A physical
  AprilTag at a surveyed position supplied an independent scale check on
  2026-11-18; the 2026-11-19 real grasps were markerless and did not use this
  detector.
- The marker interior is warped with a planar homography; lens distortion is
  applied to the corners but not inside the marker.
- Marker size must be measured on the printed target; a size error scales the
  translation error directly.
- Single-marker, single-frame detection. No temporal filtering, multi-view
  fusion, occlusion or reflective-surface handling.
- The published pose is the marker centre. A grasp point offset on the object
  is not modelled.
- The node does not verify that the detected marker is the intended object.

## 8. Next steps

1. Print a 36h11 marker with a measured edge length and place it at a surveyed
   `base_link` position; compare the localizer output with the surveyed pose.
2. Feed `target_pose` (with a grasp-point offset) into
   `armpi_grasp_planner --target-point --sampler joint`.
3. Add temporal filtering and outlier rejection if the real detection is noisy.
4. Re-run the frame-contract check after any camera remount; the hand-eye and
   `camera_optical_frame.yaml` must be updated together.
