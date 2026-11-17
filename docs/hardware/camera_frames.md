# Camera Frames — contract for the real robot

> Status: **measured contract (2026-11-17)**. The RGB driver frame
> `rgb_camera_link` is a **body** frame; OpenCV PnP returns the **optical**
> frame. The project publishes the measured optical frame as
> `camera_optical_calibrated`, a static child of `rgb_camera_link`. Perception
> code must tag PnP poses with that frame, never with `rgb_camera_link` or
> `depth_cam_color_optical_frame`.

Evidence:
[`evidence/phase2/frames_20261117/frame_contract.json`](../../evidence/phase2/frames_20261117/frame_contract.json)

## Decision

| Question | Decision |
|---|---|
| Which frame does `CameraInfo` use? | `rgb_camera_link` (driver body frame) |
| Which frame is the OpenCV optical frame? | `camera_optical_calibrated`, published by this project |
| Can we use `depth_cam_color_optical_frame`? | **No.** It is identity to `rgb_camera_link` in the live TF; it is a body frame despite its name |
| Can we use the model's `camera_optical_frame`? | **No.** It is a nominal REP-103 frame in the mock URDF and is not published on the real robot |
| Where is the measured transform stored? | `ros2_ws/src/armpi_fiducial_localizer/config/camera_optical_frame.yaml` |
| Who publishes it? | `camera_frame_publisher`, started by `fiducial_localizer.launch.py` (`publish_optical_frame:=true`) |

## Live chain

```text
link4
└── camera_connet_link          (vendor URDF mount)
    └── depth_camera_link       (driver static: x=0.038 m, pitch=+90°)
        └── rgb_camera_link     (identity; CameraInfo frame)
            ├── depth_cam_color_frame
            │   └── depth_cam_color_optical_frame   (identity; NOT optical)
            ├── depth_cam_depth_optical_frame       (identity; NOT optical)
            └── camera_optical_calibrated           (measured, project-published)
```

Frame roles:

| Frame | Role | Source | Use for PnP? |
|---|---|---|---|
| `rgb_camera_link` | RGB body frame | Aurora driver + driver static TF | **No** |
| `depth_camera_link` | depth body frame | Aurora driver | **No** |
| `depth_cam_color_optical_frame` | identity to `rgb_camera_link` | driver static TF | **No** |
| `camera_optical_calibrated` | OpenCV optical (x right, y down, z forward) | measured hand-eye + live body TF | **Yes** |
| `camera_optical_frame` | nominal REP-103 model frame | `armpi_ultra_description` mock URDF | **No** (not published on hardware) |

## Measured transform

```text
parent: rgb_camera_link
child:  camera_optical_calibrated
translation: [0.005408, 0.010058, -0.027380] m
quaternion:  [0.008838, -0.009541, -0.705740, 0.708351]  (x, y, z, w)
RPY:         [1.489, -0.060, -89.789] deg
angle:       89.798 deg
```

Composition:

```text
T_link4_rgb_body  = live tf2 link4 -> rgb_camera_link (full precision)
T_link4_camera    = validated chessboard hand-eye (TSAI)
T_rgb_optical     = inv(T_link4_rgb_body) @ T_link4_camera
```

The composition reproduces `T_link4_camera` to `1.4e-17`, so the adapter is
consistent with the calibrated extrinsic.

### Why this differs from the older numbers

An earlier `tf2_echo` snapshot was printed to three decimals and gave
`link4 → rgb_camera_link` translation `[-0.050, -0.019, 0.087] m`. The
full-precision live TF is `[-0.039995, -0.019122, 0.086003] m`; the x value
differs by **10 mm**. The hand-eye fit did not use the snapshot (it used URDF
FK), so the calibration result is unchanged. The old body→optical numbers
(89.34°, `[15.4, 10.0, -28.3] mm`) were derived from that snapshot and are
**superseded** by the values above (89.798°, `[5.4, 10.1, -27.4] mm`).

## Code contract

1. **Pose frame**: the localizer's `camera_frame` parameter defaults to
   `camera_optical_calibrated`. The pose header uses it, not the incoming
   `Image`/`CameraInfo` frame.
2. **TF output**: set `output_frame:=base_link` to get a base-frame target;
   the node transforms through the published static frame.
3. **Launch**:

   ```bash
   ros2 launch armpi_fiducial_localizer fiducial_localizer.launch.py \
     image_topic:=/depth_cam/rgb/image_raw \
     camera_info_topic:=/depth_cam/rgb/camera_info \
     output_frame:=base_link
   ```

   This starts `camera_frame_publisher` by default. If you deliberately use a
   different frame source, set `publish_optical_frame:=false` and
   `camera_frame:=...` explicitly.
4. **Standalone publisher**:

   ```bash
   ros2 run armpi_fiducial_localizer camera_frame_publisher \
     --ros-args -p config_file:=/path/to/camera_optical_frame.yaml
   ```
5. **Do not** replace the URDF static TF with the optical transform. The
   vendor body frame is what the driver publishes; the optical frame is a
   separate, measured child.

## Verification

```bash
# on the robot or a machine on the same DDS domain
ros2 run tf2_ros tf2_echo rgb_camera_link camera_optical_calibrated
ros2 run tf2_ros tf2_echo base_link camera_optical_calibrated
ros2 topic echo /depth_cam/rgb/camera_info --once --qos-profile sensor_data
```

Expected:

- `rgb_camera_link → camera_optical_calibrated` has the transform above.
- The `CameraInfo.header.frame_id` remains `rgb_camera_link`; that is the
  driver's body frame, not an error.
- The localizer publishes `target_pose` with
  `frame_id: camera_optical_calibrated` (or `base_link` when requested).

## Limitations

- The hand-eye is valid only for the current camera mount; remounting the
  camera invalidates this contract.
- The ~3 mm board scatter is an open-loop result, not an absolute metrology
  claim.
- `white_area_pose_world` and the vendor `hand2cam_tf_matrix` are **not**
  substitutes; see
  [vendor_calibration_baseline.md](vendor_calibration_baseline.md).
- The model's nominal `camera_optical_frame` should be kept for mock/RViz
  only. Do not publish two frames with the same name on one DDS domain.
