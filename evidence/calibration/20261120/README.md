# Off-board RealSense D435i — intrinsics and chassis extrinsic — 2026-11-20

> Status: **intrinsics provisional PASS, chassis extrinsic PASS.** The front
> camera is an Intel RealSense D435i (`8086:0b3a`) on the USB 3 bus, mounted on
> the chassis front under the arm base, looking down at the table. Colour runs
> on the UVC path; depth needs librealsense.

## Device and bus

| Item | Value |
|---|---|
| Model | Intel RealSense D435i, serial `308243060489` |
| Bus | **USB 3.0, 5000 Mbps** (`lsusb -t`: Bus 02 Port 1) |
| Streams (UVC) | colour 1280x720 YUYV @ 30 fps (measured 60 frames / 2 s) |
| Depth over raw v4l2 | 640x480 Z16, ~50-60% valid pixels |
| ROS integration | none yet; the studio gateway previews it as `uvc:/dev/video4` at 640x480 @ 9 fps |

The camera first enumerated on the USB 2.0 bus next to the Aurora 930
(480 Mbps shared, 720p at 15 fps, depth 0.4% valid). Moving it to the USB 3
port doubled the colour rate and made the depth stream usable.

## Intrinsics

Sweep: one 60 s hand-held chessboard pass at 0.3-0.5 m (11x8 inner corners,
20 mm squares), captured with ffmpeg on the Pi and solved on the laptop by
`tools/phase2/uvc_chessboard_intrinsics.py`.

| Metric | Value |
|---|---|
| Frames / detected / selected | 1800 / 353 / 9 |
| Model | pinhole + plumb_bob, **fx = fy fixed** |
| fx = fy | **892.1 px** (FOV 71.3 x 44.0 deg) |
| cx, cy | 648.0, 380.2 px |
| RMS | **0.507 px**, per-view 0.23-0.80 px |
| Split-half spread | fx 8.1 px, cx 0.9 px, cy 16.0 px |

Two limits are known and documented rather than hidden:

- the free fit gives fx 929 / fy 1032 (22% apart) because the sweep kept the
  board large and centred, so the vertical focal length is weakly constrained;
  fixing fx = fy costs 0.001 px of RMS, so the constrained model is used;
- the camera's working volume is a wedge (0.09-0.45 m on the table with at most
  ~0.07 m of clearance above it at 0.3 m), so a hand sweep cannot vary the
  board's distance much. A board propped at 15-30 deg would firm up `cy` and
  the distortion; until then the intrinsics are **provisional**.

## Chassis extrinsic

Solved with `tools/phase2/front_camera_extrinsic.py`: the same board is seen by
the wrist camera at the validated table-down pose (`[0, 0.2974, -1.14773,
-1.55404, -0.01676]`) and by the front camera with the arm still in place (the
arm stays above the front camera's frame, so the board is unoccluded).

```text
T_base_link_front_optical
translation = [0.139, 0.027, 0.037] m
pitch below horizontal = 32.6 deg
yaw = -1.3 deg
height above the board top = 119 mm
```

| Check | Value | Independent expectation |
|---|---|---|
| Board plane tilt in `base_link` | **0.8 deg** | board lies flat on the table |
| Camera pitch | **32.6 deg** | 30-36 deg from the depth plane fit |
| Camera height above table | **115 mm** | 0.12-0.14 m from the depth plane fit |
| Wrist board spread | 2.8 mm / 0.68 deg | 2.7 mm / 0.42 deg at the 2026-11-18 table-down fit |
| Front board spread (static board) | 0.0 mm / 0.01 deg | — |

Sensitivity: using the 2026-11-17 `camera_forward` hand-eye instead of the
table-down one moves the camera 10 mm in x, 2.4 deg in pitch and tilts the board
3.0 deg away from flat; the table-down fit is the one that matches a flat board,
so it is the adopted result.

Note the board's symmetric grid is ambiguous up to a 180 deg relabelling. The
solver scores both branches against the known chassis-front mount and the
rejected branch would have placed the camera 0.3 m beyond the board; the
reported solve is the plausible one, and the three checks above confirm it.

## Consequences for use

- The camera is **0.139 m forward, 0.027 m right, 0.115 m above the table**, so
  it sits just in front of and above the arm's shoulder area. The link2 origin
  at the low poses is at z = 0.087 m, i.e. within ~2 cm of the mount: the
  camera is **not in the collision model** and any low arm pose needs a visual
  check.
- D435i minimum range is ~0.2 m, so the **grasp band (0.13 m in front of the
  lens) has no depth**; colour only. Depth is usable from ~0.25 m out.
- The 720p colour frame covers roughly 0.09-0.45 m of table in front of the
  lens with the top edge on the floor beyond the table.
