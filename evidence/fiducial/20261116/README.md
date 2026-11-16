# Fiducial Target Localization — 2026-11-16

> Scope: **synthetic only.** No real camera, lens, lighting or hardware was
> used. Synthetic accuracy is not real camera metrology.

## Environment

| Item | Value |
|---|---|
| ROS | Jazzy, x86_64 |
| OpenCV | 4.6.0 (`cv2.aruco`, AprilTag 36h11 dictionary) |
| Synthetic camera | 640×480, `fx=fy=700`, principal point at image centre |
| Marker | `DICT_APRILTAG_36h11`, ID 0, 0.06 m printed edge, 300 px source |
| ROS integration TF | static `base_link → camera_optical_frame`, translation (0, 0, 0.5), identity rotation |

## Commands

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash

# 1. synthetic accuracy: five known poses
ros2 run armpi_fiducial_localizer fiducial_synthetic_check \
  --output-dir evidence/fiducial/20261116

# 2. node + TF: synthetic image at 0.4 m in the camera frame
ros2 run tf2_ros static_transform_publisher --x 0 --y 0 --z 0.5 \
  --frame-id base_link --child-frame-id camera_optical_frame

ros2 run armpi_fiducial_localizer fiducial_localizer --ros-args \
  -p image_topic:=/image_raw -p camera_info_topic:=/camera_info \
  -p marker_size_m:=0.06 -p marker_id:=0 -p output_frame:=base_link

ros2 run armpi_fiducial_localizer synthetic_marker_publisher \
  --tvec 0 0 0.4 --rvec 3.141592653589793 0 0 --marker-size 0.06

python3 tools/phase2/fiducial_ros_check.py \
  --expected-position 0 0 0.9 --expected-quaternion 1 0 0 0 \
  --json-out evidence/fiducial/20261116/ros_base_link_check.json

# 3. node without TF: camera-frame output
ros2 run armpi_fiducial_localizer fiducial_localizer --ros-args \
  -p image_topic:=/image_raw -p camera_info_topic:=/camera_info \
  -p marker_size_m:=0.06 -p marker_id:=0

python3 tools/phase2/fiducial_ros_check.py \
  --expected-position 0 0 0.4 --expected-quaternion 1 0 0 0 \
  --json-out evidence/fiducial/20261116/ros_camera_frame_check.json
```

## Results

### Synthetic accuracy

| Metric | Value |
|---|---:|
| Poses rendered / detected | 5 / 5 |
| Max translation error | 1.82 mm |
| Mean translation error | 0.67 mm |
| Max rotation error | 1.49° |
| Mean rotation error | 0.64° |
| Max corner reprojection error | 0.32 px |

Per-pose values are in `synthetic_pose_accuracy.json`; overlays are in
`synthetic_pose_XX_overlay.png` and `synthetic_contact_sheet.png`.

### ROS integration

| Output frame | Expected position | Position error | Angle error | Result |
|---|---|---:|---:|---|
| `camera_optical_frame` | (0, 0, 0.4) m | 0.44 mm | 0.03° | PASS |
| `base_link` | (0, 0, 0.9) m | 0.44 mm | 0.03° | PASS |

The base-frame case uses the static 0.5 m transform above, not the real
`link4 → camera` extrinsic.

## Files

| File | Content |
|---|---|
| `synthetic_pose_accuracy.json` | Five-pose ground truth, detections and errors |
| `synthetic_pose_XX_overlay.png` | Detection overlay for each pose |
| `synthetic_contact_sheet.png` | Combined overlay contact sheet |
| `ros_camera_frame_check.json` | Camera-frame ROS pose comparison |
| `ros_base_link_check.json` | Base-frame ROS pose comparison after TF |
| `ros_camera_localizer_20261116.log` | Localizer log, camera-frame run |
| `ros_camera_publisher_20261116.log` | Synthetic publisher log, camera-frame run |
| `ros_camera_pose_check_20261116.txt` | Pose check console output, camera-frame run |
| `ros_localizer_20261116.log` | Localizer log, base-frame run |
| `ros_synthetic_publisher_20261116.log` | Synthetic publisher log, base-frame run |
| `ros_static_tf_20261116.log` | Static transform publisher log |
| `ros_pose_check_20261116.txt` | Pose check console output, base-frame run |

## OpenCV 4.6 note

OpenCV 4.6 exposes the newer `DetectorParameters` type, but constructing it
directly and setting `cornerRefinementMethod` segfaults. The package prefers
the legacy `DetectorParameters_create()` / `detectMarkers()` API when it exists.

## Limitations

- No real camera, intrinsics, `link4 → camera` extrinsic, lighting or lens
  distortion.
- `marker_size_m` must match the printed marker; a size error scales the
  translation error.
- Single marker and single frame; no temporal filtering, multi-view fusion or
  occlusion handling.
- The pose is the marker centre, not an object grasp-point offset.
- The node does not verify that the marker belongs to the intended object.
