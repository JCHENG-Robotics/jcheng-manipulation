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
- The persistent launch later exited with **SIGKILL** while the static TF
  publishers were logging DDS multicast errors. It was restarted over WiFi and
  published `camera_info` again. Check `/aurora/aurora` before relying on the
  stream; restart with the persistent command above if it is missing.
- The camera enumerates at USB 2.0 (480 Mbps), not USB 3.0. 8 Hz at 640×400 is
  adequate for static AprilTag localization but not for high-rate visual
  servoing.
- The first saved frame at an early pose looked at a white surface with dark
  edges (arm/ceiling). With the chessboard in front, the RGB stream is usable
  for detection; aim the arm at the work surface before collecting data.
- Intrinsics are driver-reported. A first chessboard refinement (8 close-range
  views) gave fx 446 vs the driver's 418 and is **inconclusive**; use the
  driver K/D until a better intrinsic dataset exists.
- Mount rigidity and repeatability are still open; the first `link4 → camera`
  hand-eye estimate is in the next section, not yet a validated calibration.

## First hand-eye baseline (2026-11-17)

- Board: 11×8 inner corners, 20 mm squares; 8 usable poses out of 13.
- `cv2.calibrateHandEye` PARK: `T_link4_camera` translation
  `[0.0053, -0.0090, 0.0326] m`, RPY `[-3.823, -0.527, -101.586]°`.
- Board pose in `base_link`: std **13.4 mm / 2.06°**; corner RMS 0.06–0.34 px.
- The vendor static TF `link4 → rgb_camera_link` is the **body** frame. The
  image/PnP optical frame differs by roughly **101° roll**, so the static TF
  alone must not be used as the measured optical extrinsic.
- The fit is limited by the open-loop arm: the same base command before/after a
  joint1 move differed 14–24 mm / 2.5°; a **joint1-only pre-pose**
  (`target − 0.08 rad`, then target) reduced the same-command difference to
  0.8 mm. The full pre-pose dataset was interrupted by a robot WiFi drop.
- Evidence: [`evidence/calibration/20261117/`](../../evidence/calibration/20261117/)
  and [`docs/phase2/CALIBRATION.md`](../phase2/CALIBRATION.md).

## Calibration note

The camera is mounted on `link4`, before `joint5`. Orientation diversity during
hand-eye calibration is physically limited; the calibration plan must maximize
arm elevation, viewing direction, distance and image-position diversity and
state this limitation explicitly.

## Next steps

1. Reconnect the direct Ethernet/power and confirm the camera with the
   persistent launch command.
2. Secure the chessboard and verify stationary frames before/after capture.
3. Rerun the full capture with `--pre-offset 0.08 --pre-joint1-only` over
   Ethernet; solve with `tools/phase2/chessboard_hand_eye.py`; target a board
   residual below 5 mm / 2°.
4. Place a 36h11 AprilTag at a measured `base_link` position and use
   `armpi_fiducial_localizer` to validate the calibrated transform
   independently.
5. If higher frame rate/resolution is needed, check whether the camera can be
   moved to a USB 3 port/cable.
