# D435i depth unblocked through librealsense — 2026-11-20

> Status: **PASS on the depth path, root cause corrected.** The previous note
> blamed a missing `/run/udev` runtime in the vendor container. That was wrong:
> with a fresh host udev database inside the container the enumeration still
> reported 0 devices. The real blocker is the D400 factory's HID gate combined
> with a kernel that has no `CONFIG_HID_SENSOR_HUB`.

## What was tested and why /run/udev is not it

`tools/phase2/rs_probe.cpp` reproduces librealsense's own discovery steps
(libudev enumeration, then `rs2::context`) and `tools/phase2/rs_internal_probe.cpp`
calls the library's internal helpers (`query_uvc_devices`,
`filter_by_product`, `group_devices_by_unique_id`, `mi_present`,
`d400_info::pick_d400_devices`) from the 2.55.1 source headers.

| Step | Result |
|---|---|
| `udev_new()` / `usb` + `DEVTYPE=usb_device` enumerate | 7 USB devices, incl. `8086:0b3a` bus=2 addr=2 |
| `usb`, no property filter | 19 nodes (device + interface level) |
| `rs2::context` before patch | `Found 0 RealSense devices` |
| fresh host `/run/udev` copied into the container | still `Found 0 RealSense devices` |
| `query_uvc_devices()` | 3 streaming nodes (video0 + md video1, video2 + md video3, video4 + md video5) |
| `query_hid_devices()` | **0** |
| internal replica of the factory decision (without the HID clause) | `mi_present(0)=1`, would pick 1 |
| `d400_info::pick_d400_devices` | **0** |

The difference between the replica and the real call is the HID clause:
`d400-factory.cpp:1154` is

```cpp
if (is_device_hid_sensor)
    all_sensors_present &= (hids.size() >= 2);
```

`RS435I_PID` is in `ds::d400_hid_sensors_pid`, so a D435i is only accepted when
two HID sensors (gyro + accelerometer) are present. On Linux those come from
the IIO subsystem via `hid-sensor-hub`; this host runs
`6.6.20+rpt-rpi-2712` with `# CONFIG_HID_SENSOR_HUB is not set`,
`/sys/bus/iio/devices` does not exist, and the camera's HID interface binds to
`hid-generic` (`hidraw0`). Hence `hids.size() == 0` and the camera is rejected
before a single stream is opened.

## Patch

`tools/phase2/patches/librealsense-2.55.1-d400-hid-gate.patch` (also copied
here as `d400-factory.patch`):

```diff
-                all_sensors_present &= (hids.size() >= 2);
+                all_sensors_present &= (hids.size() >= 2 || hids.empty());
```

A partial HID set (exactly one sensor) is still rejected. Applied inside the
vendor container to
`/home/ubuntu/librealsense-2.55.1/src/ds/d400/d400-factory.cpp` and built
incrementally in the existing tree:

```bash
cd /home/ubuntu/lrs_build
gmake -j4 realsense2 && gmake install
```

The IMU is unavailable as a consequence (`No HID info provided, IMU is
disabled`); it cannot work on this kernel without `CONFIG_HID_SENSOR_HUB`.

Artifacts and hashes: `lib_sha256.txt` (installed `librealsense2.so.2.55.1` and
the patched source file).

## Verified results

`rs_probe_after.txt` — enumeration:

```text
INFO (context.cpp:128) Found 1 RealSense devices (0xfe requested & 0xff from device-mask in settings)
rs2 devices: 1
WARNING (ds-motion-common.cpp:455) No HID info provided, IMU is disabled
  Intel RealSense D435I sn=243522070388
```

`depth_smoke.txt` — ten frames through `rs2::pipeline`, depth only:

```text
depth scale: 0.001000 m/unit
intrinsics: fx=386.445 fy=386.445 ppx=322.186 ppy=233.991 w=640 h=480 model=4
frame 10: 146175/307200 valid (47.6%), z 0.154..65.535 m
frames=10
```

`ros_depth_only.txt` — `realsense2_camera` in the vendor container:

```text
/camera/camera/depth/image_rect_raw, .../camera_info, .../metadata
depth hz: 7.0 (8.4 with the colour node up, 10.6 depth-only)
depth K: [386.445 0 322.186; 0 386.445 233.991; 0 0 1]
```

Single-frame check with `rclpy` (colour node also running):

```text
depth frame (480, 640) valid 189396 of 307200 = 61.7%
median 0.312 m, range 154..25727 mm, frame_id camera_depth_optical_frame
```

Aligned depth (`align_depth.enable:=true`, colour enabled) ran at 11.3 Hz.

## Deployment notes

- `realsense2_camera` must start **before** `armpi-ext-cam.service`; with the
  colour node already streaming, `VIDIOC_S_FMT` on the depth node fails with
  `EBUSY`. The reverse order works and both streams then run together
  (depth 7-8.4 Hz, colour 6-11 Hz).
- A depth interface that has never delivered a frame can return `EPROTO` (-71)
  in the URB completion handler; bringing up `/dev/video2` (IR) once clears it
  in-session, and `USBDEVFS_RESET` on `/dev/bus/usb/002/002` also
  re-enumerates the device.
- The depth node currently has no systemd unit; it is a hand-started
  `ros2 launch` and does not survive a container restart.
- The SDK colour intrinsics at 640x480 (`fx=fy 606.5`) disagree by ~34% with the
  chessboard-refined 1280x720 model (`fx 903.5`); treat the 640x480 colour mode
  as cropped and re-solve the board at that size before using it for geometry.
