# Camera Mount and Optical Frame

> Status: **camera operational and optical extrinsic validated (2026-11-17)**.
> The RGB/depth streams and intrinsics are verified; the chessboard hand-eye
> gives `T_link4_camera` (camera optical frame) with a 2.94 mm / 0.39° board
> scatter. The static TF below remains the vendor **body** frame.

## Hardware

| Item | Value | Status |
|---|---|---|
| Model | Deptrum Aurora 930 | `[S]` |
| USB ID | `3251:1930` | `[V]` |
| Serial | `HY400523001036827G00179` | `[V]` |
| Firmware / SDK | 2.0.8 / 1.1.19 | `[V]` |
| Link speed | **USB 2.0, 480 Mbps** (Bus 003, Port 1) | `[V]`; bandwidth ceiling |
| Driver | `deptrum-ros-driver-aurora930`, launched by `bringup vision.launch.py` | `[V]` |
| RGB stream | **~12 Hz cross-host**, 640×400, `bgr8`, frame `rgb_camera_link` | `[V]` |
| Depth stream | 8.2 Hz in-container, 640×400, `mono16`, frame `depth_camera_link` | `[V]` |

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
        └── rgb_camera_link     (identity; CameraInfo frame)
            ├── depth_cam_color_frame
            │   └── depth_cam_color_optical_frame
            ├── depth_cam_depth_optical_frame
            └── camera_optical_calibrated   (measured, project-published)
```

| Transform | Value | Status |
|---|---|---|
| `link4 → camera_connet_link` | vendor xacro: `xyz=(-0.040352, -0.019122, 0.048005) m`, `rpy=(0, -1.5614, 0) rad` | `[S]` |
| `camera_connet_link → depth_camera_link` | driver static: `xyz=(0.038, 0, 0) m`, `rpy=(0, +1.5708, 0) rad` | `[V]` |
| `depth_camera_link → rgb_camera_link` | identity | `[V]` |
| `rgb_camera_link → depth_cam_color_frame` | identity | `[V]` |
| `depth_cam_color_frame → depth_cam_color_optical_frame` | identity | `[V]` |
| `rgb_camera_link → depth_cam_depth_optical_frame` | identity | `[V]` |
| `rgb_camera_link → camera_optical_calibrated` | measured: `[0.005408, 0.010058, -0.027380] m`, quaternion `[0.008838, -0.009541, -0.705740, 0.708351]` | `[V]` |

The project model's `camera_link` / `camera_optical_frame` are **not** published
by the robot. Use `rgb_camera_link` for the RGB stream (it matches the RGB
`camera_info`) and `depth_camera_link` for the depth stream. Use
`camera_optical_calibrated` for PnP poses and TF; see
[camera_frames.md](camera_frames.md).

## Validated optical extrinsic (2026-11-17)

The chessboard hand-eye result for the **camera optical** frame (image/PnP
frame), from 14 poses with measured bus-servo joint angles:

```text
T_link4_camera
translation = [-0.0348, -0.0091, 0.0586] m
RPY         = [0.951, -0.058, -89.789] deg
```

The calibrated frame is the OpenCV/PnP optical frame. Important: the live
`depth_cam_color_optical_frame` and `depth_cam_depth_optical_frame` are
**identity to `rgb_camera_link`** in TF, so they are body frames, not the
OpenCV optical frame. The measured body→optical adapter is published as
`camera_optical_calibrated`; use it in TF, or use the calibrated transform
directly in perception code. See [camera_frames.md](camera_frames.md).

Validation: board-pose scatter 2.94 mm / 0.39° (max 5.44 mm / 0.74°),
held-out final base 3.29 mm / 0.43°, 2-fold cross-validation 2.5–3.2 mm.
See [`docs/phase2/CALIBRATION.md`](../phase2/CALIBRATION.md) and
[`evidence/calibration/20261117/`](../../evidence/calibration/20261117/).

`inv(T_link4_rgb_body) @ T_link4_camera` with the **full-precision live TF**
(`link4 → rgb_camera_link` translation `[-0.039995, -0.019122, 0.086003] m`)
is an **89.798° rotation about -z** with translation
`[5.4, 10.1, -27.4] mm`. An earlier three-decimal `tf2_echo` snapshot
(`[-0.050, -0.019, 0.087] m`) gave the superseded 89.34° /
`[15.4, 10.0, -28.3] mm` numbers; the hand-eye fit did not use that snapshot.
Do **not** replace the URDF static TF with this optical transform; publish it
as a separate frame.

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

**Rate correction (2026-11-17 Session 4).** The in-container subscription
undercounts large images. A cross-host subscriber on the laptop measured
11.83 Hz and `ros2 topic hz` measured ~12.0 Hz for the same RGB topic, while an
in-container subscriber measured 8.33 Hz (and in-container `ros2 topic hz`
~7.0 Hz). The camera publishes at about **12 Hz**; use a cross-host measurement
for FPS and state the location. The raw log above is unchanged.

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
- The camera enumerates at USB 2.0 (480 Mbps), not USB 3.0. The observed
  cross-host rate is ~12 Hz at 640×400; that is adequate for static AprilTag
  localization but not for high-rate visual servoing. Do not use the
  in-container 8 Hz number as the sensor rate.
- The first saved frame at an early pose looked at a white surface with dark
  edges (arm/ceiling). With the chessboard in front, the RGB stream is usable
  for detection; aim the arm at the work surface before collecting data.
- Intrinsics are driver-reported. The validated 14-pose chessboard refinement
  agrees with the driver (fx 418.31 vs 418.05, RMS 0.060 px); the earlier
  8-view refinement (fx 446) was an outlier from the first run.
- The camera mount is rigid; the validated hand-eye is in the section above.

## First hand-eye baseline (superseded)

The first 8-pose run used cached `joint_states` and PARK:
`T_link4_camera` `[0.0053, -0.0090, 0.0326] m`, RPY
`[-3.823, -0.527, -101.586]°`, board scatter not validated at
13.4 mm / 2.06°. It is superseded by the 14-pose measured-servo run
(2.94 mm / 0.39°). The first run's repeatability finding still applies: the
same base command differed 14–24 mm after a joint1 move, and a **joint1-only
pre-pose** (`target − 0.08 rad`, then target) reduced the same-command
difference to 0.8 mm. Evidence:
[`evidence/calibration/20261117/`](../../evidence/calibration/20261117/) and
[`docs/phase2/CALIBRATION.md`](../phase2/CALIBRATION.md).

## Calibration note

The camera is mounted on `link4`, before `joint5`. Orientation diversity during
hand-eye calibration is physically limited; the calibration plan must maximize
arm elevation, viewing direction, distance and image-position diversity and
state this limitation explicitly.

## Next steps

1. Place a 36h11 AprilTag at a measured `base_link` position and use
   `armpi_fiducial_localizer` (with `camera_frame:=camera_optical_calibrated`)
   to validate the calibrated transform independently.
2. Wire the validated optical frame into the grasp planner; the live
   `depth_cam_color_optical_frame` is not the OpenCV optical frame. Use the
   project `camera_frame_publisher` / `camera_optical_calibrated` frame.
3. If higher frame rate/resolution is needed, check whether the camera can be
   moved to a USB 3 port/cable.

## Off-board front camera (RealSense D435i, 2026-11-20)

A second camera is mounted on the chassis front, below the arm base, looking
down at the table. It is a plain UVC depth camera, not a vendor device, so it
does not use `bringup vision.launch.py`.

| Item | Value |
|---|---|
| Model | Intel RealSense D435i, USB ID `8086:0b3a`, serial `308243060489` |
| Bus | **USB 3.0 (5000 Mbps)**; the USB 2.0 bus it started on shared 480 Mbps with the Aurora and dropped to 15 fps colour with 0.4% valid depth |
| Colour | UVC 1280x720 YUYV @ 30 fps |
| Depth | 640x480 Z16 through librealsense 2.55.1 (patched, see below): ~62% valid, median 0.31 m, `/camera/camera/depth/image_rect_raw`; IMU unavailable because the kernel has no `CONFIG_HID_SENSOR_HUB` |
| ROS path (colour) | **`armpi-ext-cam.service`** runs `tools/phase2/ext_cam_node.py`, the single owner of `/dev/video4`: `/ext_cam/color/image_raw` at 1280x720/15 fps plus latched `camera_info` and the `base_link -> ext_cam_optical` static TF. The studio gateway follows that topic (`ARMPI_GATEWAY_AUX_CAMERA_TOPIC`) instead of opening the device |
| ROS path (depth) | **`armpi-rs-depth.service`** runs `realsense2_camera` (`/home/ubuntu/rs_ws`) inside the container: `/camera/camera/depth/image_rect_raw` + `camera_info` at 640x480 Z16 |

Calibration (2026-11-20, [`evidence/calibration/20261120/`](../../evidence/calibration/20261120/)):

```text
base_link -> ext_cam_optical
translation = [0.136, 0.027, 0.038] m
pitch = 31.6 deg below horizontal, yaw = -1.1 deg
fx = fy = 903.5 px, cx/cy = 644.9/363.7, RMS 0.37 px
```

Two constraints drive how this camera may be used:

- **It is a planning-scene object, not a URDF link.** The live robot runs the
  vendor description, so the camera is published the same way as the measured
  table: `tools/phase2/ext_cam_collision.py` adds a 90 x 25 x 26 mm box at
  `base_link` (0.145, 0.027, 0.032) with the calibrated orientation. With that
  box in the scene the five validated operating poses (upright, table-down,
  adopted-low, deep-low, 150 mm lift) stay collision-free; a control run with
  the box padded by 0.3 m reported contacts against `link1`-`link3`, so the box
  really is in the checker.
  The box protects **MoveIt plans only**. It has to be republished after a
  MoveIt restart (run it like `table_collision.py`), and direct joint moves -
  including `move_to_pose.py` - bypass it; `armpi_safety` does not see scene
  objects either. Keep a visual check for unplanned/low motions until the
  geometry is added to the safety node.
- **Depth starts at ~0.2 m.** The grasp band is ~0.13 m in front of the lens, so
  that region is colour-only. The far half of the table (0.25-0.45 m) has depth
  and is the camera's real contribution: far-object measurement, base-drive
  feedback and post-release verification.

## Depth over librealsense (2026-11-20, unblocked)

The SDK path works: `rs2::context` finds the D435i, the depth stream runs at
640x480 Z16 and `realsense2_camera` publishes it. Two separate blockers had to
be cleared, and the first documented next step was the wrong one:

1. **`/run/udev` was not the problem.** With a fresh copy of the host
   `/run/udev` database inside the container the result was unchanged, and the
   library's own enumeration already worked: it listed the six UVC nodes
   (`Enumerating UVC video0..5`), and both libusb and libudev saw `8086:0b3a`.
   Recreating the container with `-v /run/udev:/run/udev:ro` would not have
   helped. (It is also not free: the container's writable layer is 15 GB with
   5.8 GB free, so there is no room to `docker commit` and recreate it.)
2. **Root cause - the D400 HID gate.** `d400-factory.cpp` only accepts a
   HID-sensor model (D435i/D455) when the kernel reports `>= 2` HID sensors
   (gyro + accelerometer). Linux exposes those through the **IIO** subsystem,
   and this kernel (`6.6.20+rpt-rpi-2712`) is built with
   `# CONFIG_HID_SENSOR_HUB is not set`: `/sys/bus/iio/devices` does not exist,
   the camera's HID interface binds to `hid-generic` / `hidraw0`, and
   `query_hid_devices()` returns 0. The camera was rejected before any stream
   was opened. The gate is at `src/ds/d400/d400-factory.cpp:1154`
   (`all_sensors_present &= (hids.size() >= 2)`).

Patch: `tools/phase2/patches/librealsense-2.55.1-d400-hid-gate.patch` accepts a
device that reports no HID sensors at all (`hids.size() >= 2 || hids.empty()`)
and still rejects a partial set. It was applied inside the vendor container and
rebuilt incrementally against the existing `/home/ubuntu/lrs_build` tree
(`gmake -j4 realsense2 && gmake install`, ~40 s); no ABI change, so
realsense-ros did not need a rebuild. **Consequence: the IMU is unavailable**
(`No HID info provided, IMU is disabled`) - it cannot work on this kernel
without `CONFIG_HID_SENSOR_HUB`. Depth and colour are unaffected.

Verified (evidence: [`evidence/calibration/20261120/depth_unblock/`](../../evidence/calibration/20261120/depth_unblock/)):

| Metric | Value |
|---|---|
| `rs2::context` | `Found 1 RealSense devices`, `Intel RealSense D435I sn=243522070388` |
| depth profile | 640x480 Z16, depth scale 1 mm |
| depth content | 61.5% valid pixels, median 0.312 m, range 0.154-25.7 m |
| depth intrinsics (SDK) | fx=fy 386.445, cx/cy 322.186/233.991 |
| ROS | `/camera/camera/depth/image_rect_raw` (16UC1, `camera_depth_optical_frame`), started by **`armpi-rs-depth.service`** |
| rate | 10.6 Hz depth only; 4-8 Hz with the colour stream running (post-boot measurement: 5.0 Hz depth / 2.4 Hz colour while the Aurora vision launch was still loading, 4-9 Hz once it settled) |
| aligned depth | `/camera/camera/aligned_depth_to_color/image_raw` at 11.3 Hz with colour enabled |

Deployment (`armpi-rs-depth.service`, see
[`deploy/README.md`](../../armpi-debug-studio/ros2_gateway/deploy/README.md)).
Three ordering rules, each of which failed first:

- **After the vendor bringup.** `start_node.sh` runs `~/.stop_ros.sh`, which
  `kill -9`s every process whose command line contains "ros", ~15 s after
  systemd reports `start_node.service` as started. The runner therefore waits
  for the `bringup.launch.py` process; otherwise the node dies with status 137
  and systemd has to restart it (`NRestarts` goes to 1 on every boot).
- **Before `armpi-ext-cam.service`.** If the colour node claims the UVC device
  first, librealsense fails `VIDIOC_S_FMT` with `EBUSY`. The depth unit is
  `Before=` the colour unit, the colour unit is `After=` the depth unit, and its
  `ExecStartPre` waits (best effort, 40 s) for the depth topic - ordering alone
  does not cover the node's ~5 s start-up.
- **IR wake-up before the node.** A freshly powered D435i can stay silent until
  its stereo unit is nudged; the runner streams the IR node (`/dev/video2`) for
  three frames, retrying for up to 90 s, before starting the node. Without it a
  cold boot ends at `Frames didn't arrived within 5 seconds` with no depth.

Measured after a clean reboot: `NRestarts=0`, wake-up on attempt 1, node up 45 s
after boot, depth and colour both streaming.

Other facts worth keeping:

- **A stale node wedges the next start.** `docker exec` children survive the
  unit's death, so a leftover `realsense2_camera_node` keeps `/dev/video0` open
  and everything afterwards reports `EBUSY` or no frames. The unit's `ExecStop`
  kills the launch and the node and escalates to `SIGKILL`.
- **IR and depth do not stream together with colour.** Depth + IR + colour drop
  to ~3 Hz with stalls; depth + colour is the usable combination, so IR is used
  only as the wake-up.
- **EPROTO on a stalled depth interface.** The kernel logs
  `Non-zero status (-71) in video completion handler` when the stereo unit is
  not delivering; the IR wake-up clears it. Repeated `USBDEVFS_RESET` on the
  device made things worse - recover with a power cycle, or by toggling
  `/sys/bus/usb/devices/2-1/authorized`.
- **SDK colour intrinsics caution.** At 640x480 the SDK reports
  fx=fy 606.5, cx/cy 325.0/249.2, a much narrower FOV than the
  chessboard-refined 1280x720 model (fx 903.5); the two disagree by ~34%, so the
  640x480 colour mode is probably a crop. Re-solve the board at 640x480 before
  using SDK colour intrinsics for geometry.

## Reboot test (2026-11-20, PASS)

`sudo systemctl reboot` with the table clear. SSH answered 25 s after the
reboot; the container came up, `armpi-ext-cam` and `armpi-gateway` each
restarted once while waiting for it, and the vendor `armpi-vision` (Aurora)
finished ~2.5 min after boot. After the boot `/health` reports
`camera_ready` and `aux_camera_ready` true, both snapshots return 200,
`base_link -> ext_cam_optical` is `[0.136, 0.027, 0.038]`, and the Pi clock
sits 0.9 s ahead of the laptop.
