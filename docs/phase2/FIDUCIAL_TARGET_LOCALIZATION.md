# Fiducial Target Localization

> Status: **Implemented, synthetic-validated.** No real camera was used.
> The camera intrinsic and the `link4 → camera` extrinsic are still unmeasured.
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
│   ├── detector.py             # OpenCV detection + solvePnP, no ROS
│   ├── synthetic.py            # synthetic marker renderer for tests
│   ├── node.py                 # ROS 2 localizer node
│   ├── synthetic_publisher.py  # image + CameraInfo publisher
│   └── cli.py                  # synthetic accuracy check
├── launch/fiducial_localizer.launch.py
└── test/                       # 11 tests: detector, synthetic, lint
```

`detector.py` and `synthetic.py` are pure OpenCV/NumPy. The node depends on
`cv_bridge`, `tf2_ros` and `tf2_geometry_msgs`.

## 3. Marker convention

The default dictionary is `DICT_APRILTAG_36h11` (AprilTag 36h11). The detector
uses the square marker's four corners and `solvePnP(SOLVEPNP_IPPE_SQUARE)`.

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
| `output_frame` | `''` | empty = camera frame; e.g. `base_link` |
| `max_reprojection_error_px` | `3.0` | reject poor corner fits |
| `tf_timeout_s` | `0.1` | TF lookup timeout |
| `publish_on_tf_failure` | `true` | publish in camera frame if TF fails |

The node selects the detection with the lowest reprojection error after the
optional marker-ID filter. It does not average or filter across frames.

### Run

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

ros2 launch armpi_fiducial_localizer fiducial_localizer.launch.py \
  image_topic:=/image_raw camera_info_topic:=/camera_info \
  marker_size_m:=0.06 marker_id:=0 output_frame:=base_link
```

The real camera is not connected. The node can be exercised with the synthetic
publisher:

```bash
# terminal 1: localizer
ros2 run armpi_fiducial_localizer fiducial_localizer --ros-args \
  -p image_topic:=/image_raw -p camera_info_topic:=/camera_info \
  -p marker_size_m:=0.06 -p marker_id:=0 -p output_frame:=base_link

# terminal 2: synthetic image at 0.4 m in the camera frame
ros2 run armpi_fiducial_localizer synthetic_marker_publisher \
  --tvec 0 0 0.4 --rvec 3.141592653589793 0 0 --marker-size 0.06
```

## 5. Synthetic accuracy check

```bash
ros2 run armpi_fiducial_localizer fiducial_synthetic_check \
  --output-dir evidence/fiducial/20261116
```

The check renders five markers at known camera-frame poses (0.30–0.45 m,
tilts up to 0.25 rad), runs detection and pose estimation, and reports
translation, rotation and reprojection errors. It writes JSON and overlay
images.

### Results (2026-11-16)

| Metric | Value |
|---|---|
| Poses rendered / detected | 5 / 5 |
| Max translation error | **1.82 mm** |
| Mean translation error | 0.67 mm |
| Max rotation error | **1.49°** |
| Mean rotation error | 0.64° |
| Max corner reprojection error | 0.32 px |

This validates the geometry and the detector wiring. It does **not** measure a
real camera, lens distortion, lighting, motion blur or the camera mount.

## 6. ROS integration check

The localizer was exercised through ROS with a synthetic image, a static
`base_link → camera_optical_frame` transform (0.5 m) and the pose check tool
[`tools/phase2/fiducial_ros_check.py`](../../tools/phase2/fiducial_ros_check.py).

| Output frame | Expected position | Position error | Angle error | Result |
|---|---|---:|---:|---|
| `camera_optical_frame` | (0, 0, 0.4) m | 0.44 mm | 0.03° | PASS |
| `base_link` | (0, 0, 0.9) m | 0.44 mm | 0.03° | PASS |

The base-frame result uses the static transform `(0, 0, 0.5)`, identity
rotation. All evidence is in
[`evidence/fiducial/20261116/`](../../evidence/fiducial/20261116/).

## 7. OpenCV 4.6 compatibility note

OpenCV 4.6 exposes the newer `DetectorParameters` type but constructing it
directly and setting `cornerRefinementMethod` segfaults. The code therefore
prefers the legacy `DetectorParameters_create()` / `detectMarkers()` API when
it exists, and falls back to the newer API on later OpenCV versions. This is a
practical portability fix, not a claim about OpenCV quality.

## 8. Limitations

- No real camera, lens, lighting or motion; no hardware validation.
- Camera intrinsics are synthetic in the test; the real `camera_info` and the
  `link4 → camera` extrinsic are unmeasured.
- Marker size must be measured on the printed target; a size error scales the
  translation error directly.
- Single-marker, single-frame detection. No temporal filtering, multi-view
  fusion, occlusion or reflective-surface handling.
- The published pose is the marker centre. A grasp point offset on the object
  is not modelled.
- The node does not verify that the detected marker is the intended object.

## 9. Next steps

1. Mount the camera and measure `camera_info` plus the real `link4 → camera`
   transform (Phase 1/2 hardware baseline).
2. Print a 36h11 marker with a measured edge length and run the node on the real
   camera; compare the published pose with a physical fixture.
3. Add temporal filtering and outlier rejection if the real detection is noisy.
4. Feed `target_pose` (with a grasp-point offset) into
   `armpi_grasp_planner --target-point --sampler joint`.
5. Re-run calibration with the same marker family; the calibration validation
   poses should use the measured camera/arm geometry.
