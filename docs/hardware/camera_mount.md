# Camera Mount and Optical Frame

> Status: **camera operational (2026-11-17)** after a restart. The
> RGB/depth streams and intrinsics are verified; the `link4 → camera` hand-eye
> extrinsic is **not measured**.

## Hardware

| Item | Value | Status |
|---|---|---|
| Model | Deptrum Aurora 930 | `[S]` |
| USB ID | `3251:1930` | `[V]` |
| Serial | `HY400523001036827G00179` | `[V]` |
| Firmware / SDK | 2.0.8 / 1.1.19 | `[V]` |
| Link speed | **USB 2.0, 480 Mbps** (Bus 003, Port 1) | `[V]`; bandwidth ceiling |
| Driver | `deptrum-ros-driver-aurora930`, launched by `bringup vision.launch.py` | `[V]` |
| RGB stream | 8.0 Hz, 640×400, `bgr8`, frame `rgb_camera_link` | `[V]` |
| Depth stream | 8.2 Hz, 640×400, `mono16`, frame `depth_camera_link` | `[V]` |

The camera publishes sensor-data QoS (best effort). A `nohup ... &` inside
`docker exec` was reaped and left a zombie node; start it persistently with:

```bash
sudo docker exec -d -u ubuntu -w /home/ubuntu ArmPiUltra /bin/zsh -c \
  'source ~/.zshrc >/dev/null 2>&1; \
   exec ros2 launch bringup vision.launch.py > /tmp/vision.log 2>&1'
```

## Frames

```text
link4
└── camera_connet_link          (vendor URDF mount)
    └── depth_camera_link       (driver static: x=0.038 m, pitch=+90°)
        └── rgb_camera_link     (identity)
            ├── depth_cam_color_frame
            │   └── depth_cam_color_optical_frame
            └── depth_cam_depth_optical_frame
```

| Transform | Value | Status |
|---|---|---|
| `link4 → camera_connet_link` | vendor xacro: `xyz=(-0.040352, -0.019122, 0.048005) m`, `rpy=(0, -1.5614, 0) rad` | `[S]` |
| `camera_connet_link → depth_camera_link` | driver static: `xyz=(0.038, 0, 0) m`, `rpy=(0, +1.5708, 0) rad` | `[V]` |
| `depth_camera_link → rgb_camera_link` | identity | `[V]` |
| `rgb_camera_link → depth_cam_color_frame` | identity | `[V]` |
| `depth_cam_color_frame → depth_cam_color_optical_frame` | identity | `[V]` |
| `rgb_camera_link → depth_cam_depth_optical_frame` | identity | `[V]` |

The project model's `camera_link` / `camera_optical_frame` are **not** published
by the robot. Use `rgb_camera_link` for the RGB stream (it matches the RGB
`camera_info`) and `depth_camera_link` for the depth stream.

## Intrinsics (driver-reported, 2026-11-17)

RGB (`/depth_cam/rgb/camera_info`, 640×400, `plumb_bob`):

```text
K = [418.0542,   0.0000, 308.7825
       0.0000, 419.1000, 187.1342
       0.0000,   0.0000,   1.0000]
D = [0.00960977, -0.05017150, 0.00056402, -0.00070902, 0.03192810]
```

Depth (`/depth_cam/depth/camera_info`, 640×400, `plumb_bob`):

```text
K = [423.0235,   0.0000, 320.0960
       0.0000, 423.8290, 199.9040
       0.0000,   0.0000,   1.0000]
D = [0.06023980, -0.12238700, 0.00141327, -0.00030765, 0.06893350]
```

Driver-reported IR-to-RGB extrinsic:

```text
R ≈ [ 0.999969,  0.00784129,  0.000758029
     -0.00783905, 0.999966,  -0.00291617
     -0.000780932, 0.00291014, 0.999995]
t ≈ [-9.86464, 0.136594, 0.5402]   (units reported by the driver; likely mm)
```

The TF chain treats `depth_camera_link` and `rgb_camera_link` as coincident.
The ~9.86 mm IR-RGB offset from the driver log is **not represented in TF**.
For RGB PnP, use the RGB intrinsics and `rgb_camera_link`; for depth-registered
points, treat the registration accuracy as unverified.

## Stream and depth checks

`tools/phase1/camera_stream_check.py` (run inside the container) measured:

```text
rgb:   40 msgs in 5.0 s (8.0 Hz), 640x400 encoding=bgr8   step=1920 frame=rgb_camera_link
depth: 41 msgs in 5.0 s (8.2 Hz), 640x400 encoding=mono16 step=1280 frame=depth_camera_link
       depth values: min=0 max=2551 median=2257.0 zeros=23.11%
```

Depth values are in **millimetres** (max 2551 mm ≈ 2.55 m). The 23% zero
fraction is invalid/no-return pixels, not a valid 0 mm reading.

## Status and limitations

- The first launch failed to open the device (`open device failed ret=0x21003`,
  error 135171). After the restart the same device opened and streamed.
  Root cause is unresolved; treat it as a transient device/USB state until
  reproduced.
- The camera enumerates at USB 2.0 (480 Mbps), not USB 3.0. 8 Hz at 640×400 is
  adequate for static AprilTag localization but not for high-rate visual
  servoing.
- The first saved frame at the current arm pose looks at a white surface with
  dark edges (arm/ceiling), not a work surface. Aim the arm down before
  collecting calibration data.
- Intrinsics are driver-reported, not independently calibrated. Do not use them
  as metrology without a calibration check.
- Mount rigidity, repeatability and the real `link4 → camera` extrinsic are
  unknown.

## Calibration note

The camera is mounted on `link4`, before `joint5`. Orientation diversity during
hand-eye calibration is physically limited; the calibration plan must maximize
arm elevation, viewing direction, distance and image-position diversity and
state this limitation explicitly.

## Next steps

1. Reconnect the direct Ethernet/power and confirm the camera with the
   persistent launch command.
2. Print an AprilTag 36h11 and run `armpi_fiducial_localizer` with
   `image_topic=/depth_cam/rgb/image_raw`,
   `camera_info_topic=/depth_cam/rgb/camera_info` and camera frame
   `rgb_camera_link`.
3. Measure the real `link4 → camera` transform; do not treat the vendor static
   TF as a calibrated extrinsic.
4. If higher frame rate/resolution is needed, check whether the camera can be
   moved to a USB 3 port/cable.
