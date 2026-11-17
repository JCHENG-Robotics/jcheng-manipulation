# Vendor Calibration Baseline — static analysis

> Status: **static analysis only (2026-11-17)**. No robot motion, no vendor
> calibration run. The shipped vendor matrices are decoded from source and
> compared with the validated project hand-eye. Conclusion: **the shipped
> `hand2cam_tf_matrix` is not valid for the current camera mount**; it must not
> be used as Baseline A without re-running the vendor procedure.

Evidence:
[`evidence/phase2/vendor_calibration_20261117/vendor_calibration_analysis.json`](../../evidence/phase2/vendor_calibration_20261117/vendor_calibration_analysis.json)

## Inputs

| File | SHA256 (first 16) | Content |
|---|---|---|
| `armpi_ultra_ws_clean/src/app/config/transform.yaml` | `2a3c90b621940087` | Table/white-area calibration: `extristric`, `plane`, `white_area_pose_cam`, `white_area_pose_world`, `corners` |
| `armpi_ultra_ws_clean/src/peripherals/config/camera_info.yaml` | `6ebe1c1617c423bd` | Only `hand2cam_tf_matrix`; **not** camera intrinsics |
| `…/hand2cam_tf_matrix_software/calibration/handeye_calibration.py` | vendor | Tsai-Lenz `get_cam2hand` implementation |
| `ros2_ws/src/app/app/{tag_stackup.py,calibration.py,object_sorting.py}` | vendor | Downstream use of both matrices |

The exact hashes are in the JSON evidence.

## What `transform.yaml` actually is

It is **not** a hand-eye calibration. It is the vendor app's table/white-area
calibration:

| Key | Shape | Meaning |
|---|---|---|
| `corners` | 5×3 | White-area corners in its own frame; rectangle 0.108 m × 0.167 m, origin at the centre |
| `white_area_pose_cam` | 4×4 | `T_cam_white`: white-area frame in the camera optical frame |
| `white_area_pose_world` | 4×4 | White area in the vendor robot/world frame at the calibration pose; rotation identity, translation `[0.227, 0.001, 0.033] m` |
| `extristric` | 4×3 | `[t; R]` world→camera projection matrix used with `cv2.projectPoints`; `t` is the white-area origin in camera, `R` is the camera orientation in a white-area frame rotated 180° about x |
| `plane` | 4 | White-area plane in camera coordinates; normal is opposite to the white-area +z axis |

Internal consistency checks:

- `extristric` agrees with `white_area_pose_cam` rotated 180° about x within
  **0.17° / 0.02 mm** (the two values were saved by separate runs).
- The plane reconstructed from `white_area_pose_cam` has normal
  `[-0.0221, 0.5360, 0.8439]` and offset `-0.2008 m`; the vendor `plane`
  is `[-0.0105, -0.5189, -0.8547, 0.1952]`. The normal is opposite
  (dot `-0.9993`); after flipping it, the offset differs by **5.6 mm**.
- `white_area_pose_world` cannot be compared with the current live TF because
  the vendor calibration pose is not recorded.

This file is useful for understanding the vendor app's table projection. It is
**not** the camera extrinsic for our `link4 → camera` chain and must not be
substituted for the validated hand-eye.

## What `camera_info.yaml` actually is

`camera_info.yaml` contains only `hand2cam_tf_matrix`. It does **not** contain
intrinsics. The live intrinsics come from the Aurora 930 driver and are in
`/depth_cam/rgb/camera_info` (see [camera_mount.md](camera_mount.md)).

The vendor matrix is

```text
H = [[ 0.00566,  0.11882,  0.99290, -0.11792],
     [-0.99884, -0.04681,  0.01129,  0.00354],
     [ 0.04782, -0.99181,  0.11841,  0.03568],
     [ 0,        0,        0,        1      ]]
|t| = 0.12325 m, rotation angle = 117.48°
```

The vendor source and downstream usage define its convention:

- `tag_stackup.py` computes `pose_end = H @ T_cam_tag`, then
  `pose_world = T_base_end @ pose_end`. In that stack `H` is used as
  **end-effector ← camera** (`T_end_cam`).
- `calibration.py` left-multiplies the driver body TF before use, so the file
  is coupled to the driver's `depth_cam_color_frame`/`depth_camera_link`
  naming, not to the OpenCV optical frame directly.

## Comparison with the validated project result

The validated result is `T_link4_camera` (camera optical in `link4`) from the
14-pose chessboard hand-eye; see [camera_mount.md](camera_mount.md). Convert it
to the vendor end-effector frame with the live `link4 → link5` TF before
comparing:

| Comparison | Translation difference | Rotation difference |
|---|---:|---:|
| Vendor `H` vs validated `T_link4_camera` | 87.1 mm | 84.2° |
| Vendor `H` vs validated `T_link5_camera` | 89.6 mm | 83.5° |
| Vendor `H` vs `T_link5_camera @ Rx(-90°)` | 89.6 mm | **7.1°** |

The last row is the informative one: after applying the expected body/optical
−90° x-roll convention, the **rotation agrees to 7.1°**, but the **translation
is 89.6 mm off**. The vendor matrix has a `|t|` of 123 mm from the end
effector; the current camera is 36 mm from `link5` and 69 mm from `link4`.
This is consistent with a calibration for a **different camera location or
mount**, not the current Aurora 930 on `link4`.

Therefore:

- The shipped `hand2cam_tf_matrix` is **stale/different** and cannot be used
  as the current camera extrinsic.
- It also cannot serve as Baseline A for the current hardware. A matrix
  comparison would conflate a frame-convention difference (7°) with a mount
  change (90 mm).

## Baseline A evaluation protocol (to run on hardware)

Do not compare matrices as the acceptance metric. Evaluate what the vendor
pipeline predicts:

1. **Record the vendor state**: hash `transform.yaml`, `camera_info.yaml`,
   the vendor `hand2cam_tf_matrix`, and the robot pose at calibration.
2. **Re-run the vendor hand-eye** with its own GUI and marker (`tag36h11`,
   25 mm) on the current camera mount. Save the raw hand/camera samples and
   the resulting matrix.
3. **Independent target**: place an AprilTag at a surveyed `base_link`
   position (or a fixed target measured with a ruler/caliper) and compare:
   - vendor `H` prediction (after the vendor's frame conversion),
   - project `T_link4_camera` prediction,
   - surveyed position.
4. **Metrics**: 3D position error (mm), orientation error (deg), and
   repeatability over at least 10 poses. Report the board-pose scatter if the
   vendor tool exposes it.
5. **Gate for Baseline A**: the vendor prediction must beat a pre-registered
   error bound (suggest 10 mm / 3° for the vendor stack) on the independent
   target; matrix agreement is informational only.
6. **Frames**: convert both methods into the same frame (`base_link` or the
   end-effector) before comparing. State the camera frame used by each method
   (vendor body frame vs OpenCV optical frame).

Until step 2 is done, the project's validated chessboard hand-eye remains the
only valid optical extrinsic.

## Reproduction

```bash
python3 - <<'PY'
import yaml, numpy as np
tv = yaml.safe_load(open('armpi_ultra_ws_clean/src/app/config/transform.yaml'))
print(np.array(tv['extristric']))
print(np.array(tv['white_area_pose_cam']))
PY
```

The full numeric comparison, hashes and the decoded vendor convention are in
[`vendor_calibration_analysis.json`](../../evidence/phase2/vendor_calibration_20261117/vendor_calibration_analysis.json).

## Limitations

- Static source/config analysis only; the vendor calibration GUI was not run.
- The vendor calibration pose is unknown, so `white_area_pose_world` cannot be
  replayed against the current TF.
- The 7.1° residual after the −90° convention is not explained; it may be a
  different end frame, a different camera tilt, or a stale calibration. The
  independent-target protocol is needed to decide.
