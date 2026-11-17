# Camera Frame Contract — 2026-11-17

Measured `rgb_camera_link → camera_optical_calibrated` adapter for the real
Aurora 930. No robot motion.

## Inputs

| Input | Source |
|---|---|
| `T_link4_camera` (optical) | [`../calibration/20261117/calibration_servo_20261117.json`](../../calibration/20261117/calibration_servo_20261117.json), TSAI |
| `T_link4_rgb_body` | Full-precision `tf2` lookup inside the robot container during the still capture (`link4 → rgb_camera_link`) |

## Result

```text
parent: rgb_camera_link
child:  camera_optical_calibrated
translation: [0.005408131, 0.010058266, -0.027380067] m
quaternion:  [0.008837590, -0.009540883, -0.705740445, 0.708351108]  (x, y, z, w)
RPY:         [1.489, -0.060, -89.789] deg
angle:       89.798 deg
```

`T_link4_rgb_body @ T_rgb_optical` reproduces `T_link4_camera` to
`1.4e-17`, so the adapter and the hand-eye are consistent.

## Superseded snapshot

An earlier three-decimal `tf2_echo` snapshot gave
`link4 → rgb_camera_link` translation `[-0.050, -0.019, 0.087] m`. The
full-precision live TF is `[-0.039995, -0.019122, 0.086003] m`; the x value
differs by 10 mm. The hand-eye fit did not use the snapshot, so the calibration
is unaffected, but the old body→optical numbers (89.34°,
`[15.4, 10.0, -28.3] mm`) are superseded.

## Publisher check

A laptop-side isolated ROS graph was used to verify the publisher itself:

```bash
ros2 run armpi_fiducial_localizer camera_frame_publisher
ros2 run tf2_ros tf2_echo rgb_camera_link camera_optical_calibrated
```

The publisher logged
`published static frame rgb_camera_link -> camera_optical_calibrated`; TF
reported translation `[0.005, 0.010, -0.027] m`, quaternion
`[0.009, -0.010, -0.706, 0.708]`, RPY `[1.489, -0.060, -89.789]°`.
The first `Invalid frame ID "rgb_camera_link"` warning is expected in the
isolated graph: no robot `robot_state_publisher` was running, but the direct
static transform is still served. On the robot the driver publishes
`rgb_camera_link`, so the chain is complete.

A full launch smoke test (`ros2 launch ...`) started both nodes; the localizer
logged `camera_frame=camera_optical_calibrated`, so the pose-frame parameter is
wired through the launch file.

Logs: `frame_publisher_20261117.txt`, `tf2_echo_static_20261117.txt`,
`launch_smoke_20261117.txt`.

## Contract

- `rgb_camera_link`: driver body frame (matches RGB `CameraInfo`).
- `camera_optical_calibrated`: measured OpenCV/PnP optical frame; canonical
  PnP frame, published as a static child of `rgb_camera_link`.
- `depth_cam_color_optical_frame`: identity to `rgb_camera_link`; a body frame
  despite its name.
- `camera_optical_frame`: model-only nominal frame in the mock URDF; not
  published on hardware.

See [`docs/hardware/camera_frames.md`](../../../docs/hardware/camera_frames.md).

## Limitations

- Valid only for the current camera mount.
- The hand-eye is an open-loop chessboard result (3 mm class), not absolute
  metrology.
- The adapter inherits the hand-eye uncertainty; it is not an independent
  measurement of the mount.

## Files

- `frame_contract.json` — full-precision inputs, composition, old/new
  comparison and contract roles.
- `frame_publisher_20261117.txt` — static publisher startup log.
- `tf2_echo_static_20261117.txt` — TF readback of the published transform.
