# Vendor Calibration Static Analysis — 2026-11-17

Read-only analysis of the shipped vendor calibration files. No robot motion,
no vendor calibration run.

## Inputs

| File | SHA256 (first 16) | Role |
|---|---|---|
| `armpi_ultra_ws_clean/src/app/config/transform.yaml` | `2a3c90b621940087` | Table/white-area calibration, **not** hand-eye |
| `armpi_ultra_ws_clean/src/peripherals/config/camera_info.yaml` | `6ebe1c1617c423bd` | `hand2cam_tf_matrix` only; not intrinsics |
| `…/hand2cam_tf_matrix_software/calibration/handeye_calibration.py` | vendor | Tsai-Lenz `get_cam2hand` |
| `ros2_ws/src/app/app/{tag_stackup.py,calibration.py,object_sorting.py}` | vendor | Downstream use |

## Findings

- `transform.yaml` is a table/white-area calibration:
  - `white_area_pose_cam` = `T_cam_white`;
  - `white_area_pose_world` = white area in the vendor world frame at an
    unknown pose;
  - `extristric` = `[t; R]` world→camera projection matrix; it agrees with
    `white_area_pose_cam` rotated 180° about x within 0.17° / 0.02 mm;
  - `plane` is the white-area plane in camera, normal opposite to the
    white-area +z axis, offset within 5.6 mm.
- `camera_info.yaml` is a hand-eye matrix only. Vendor code uses it as
  end-effector ← camera (`H @ T_cam_tag`, then `T_base_end @ ...`).
- The shipped matrix is **not valid for the current camera mount**:
  - vs validated `T_link4_camera`: 87.1 mm / 84.2°;
  - vs validated `T_link5_camera`: 89.6 mm / 83.5°;
  - vs `T_link5_camera @ Rx(-90°)`: 89.6 mm / **7.1°**.
  The 7.1° residual after the body/optical convention is small, but the
  89.6 mm translation means a different camera location/mount.
  `|t|` is 123 mm from the end effector; the current camera is 36 mm from
  `link5`.

## Baseline A status

**Not valid** for the current hardware. The vendor hand-eye must be re-run on
the current mount and evaluated with an independent surveyed target, not by
matrix difference. The protocol is in
[`docs/hardware/vendor_calibration_baseline.md`](../../../docs/hardware/vendor_calibration_baseline.md).

## Files

- `vendor_calibration_analysis.json` — hashes, decoded fields, internal
  consistency checks, link4/link5 comparisons and the conclusion.

## Limitations

- Static source/config analysis only; the vendor GUI was not run.
- The vendor calibration pose is unknown, so `white_area_pose_world` cannot be
  replayed against the current TF.
- The 7.1° residual is not explained; it may be a different end frame, a
  different camera tilt, or a stale calibration.
