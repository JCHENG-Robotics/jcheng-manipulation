# Depth table plane and absolute table alignment — 2026-11-18

> Status: **PASS.** A RANSAC + SVD fit of
> `/depth_cam/depth_registered/points` at the table-down pose gives a
> **147,626-inlier plane with 0.7 mm RMS**, normal **3.66°** from `base_link`
> +z, and table z = **−0.0787 m** under the camera (−0.0810 m at
> `(0.34, 0)`). A ruler check of the gripper-fingertip height (**215 mm**,
> TCP FK z = 0.1366 m) gives table z = **−0.0784 m**, **1.3 mm** from the
> depth plane. The RGB chessboard plane is **6.1 mm above** the depth plane,
> but that is **not** the board thickness: the board is **11 mm** (calipered)
> and is invisible to the depth (holes filled with the table). A same-frame
> matte-AprilTag test measured a **−4.6 mm same-surface RGB-vs-depth gap**;
> see [rgbd_same_surface_20261118.md](rgbd_same_surface_20261118.md). The
> earlier "physical table z ≈ −0.024 m" chassis estimate is wrong by ~54 mm:
> the table is at **z ≈ −0.078 m**.

## Why the earlier interpretation was wrong

The 2026-11-18 table-down report treated
`−0.024 m = 17.54 mm chassis + 6.46 mm base-mesh bottom` as the physical table
and concluded that the RGB PnP absolute z was biased by ~46 mm. The direct
measurements here contradict that:

| Quantity | Value |
|---|---:|
| Gripper fingertip height above the table (ruler) | **215 mm** |
| TCP FK z (measured bus-servo joints) | **0.1366 m** |
| Table z from the ruler | **−0.0784 m** |
| Depth plane z at the TCP XY | −0.0797 m |
| Difference | **1.3 mm** |

The 215 mm is consistent with the depth plane (216–222 mm predicted from the
depth geometry) and inconsistent with the chassis estimate (~161–167 mm). The
chassis/table-height assumption, not the RGB PnP, was the outlier. The
17.54 mm figure is the wheel/chassis clearance; the robot is evidently mounted
higher above this table than that number implied.

## Setup

| Item | Value |
|---|---|
| Arm pose | table_down, measured pulses `joint1..joint5 = 497, 565, 774, 122, 493` |
| Measured joints | `[−0.01257, 0.27227, −1.14773, −1.58336, −0.02932] rad` |
| Cloud | `/depth_cam/depth_registered/points`, 202,617 points, `point_step=16`, `height=1` (unorganized) |
| Frame label | `depth_camera_link` |
| Hand-eye | table-view TSAI `T_link4_camera` (`evidence/calibration/20261118/table_down/`) |
| Tool | [`tools/phase2/depth_plane_fit.py`](../../../tools/phase2/depth_plane_fit.py) |

## Frame-convention finding (important)

The driver labels the cloud `depth_camera_link`, but the point layout is the
**OpenCV optical** convention (x right, y down, z forward), the same as the RGB
PnP:

- Transforming the fitted plane with the live `link4 → depth_camera_link`
  **body** TF gives a normal **54.2°** from `base_link` +z — not a table.
- Using the measured hand-eye `T_link4_camera` gives **3.66°** — a level table.

The tool therefore defaults to `--convention optical` and records the label
mismatch in the report. Do not use the cloud `frame_id` as a plain TF frame for
geometry. This matches the earlier finding that the driver's
`*_optical_frame` names are identity to the body frame; for the cloud the
label/geometry mismatch is the opposite direction.

## Plane fit

```text
topic            /depth_cam/depth_registered/points
range filter     0.15 … 0.60 m  (149,321 points)
inlier threshold 4 mm
inliers          147,626
RMS              0.734 mm
plane (optical)  n = [−0.06952, 0.60642, 0.79210], d = −0.29441
plane (base)     n = [ 0.01031, −0.06301, 0.99796], d =  0.07728
angle to +z      3.66°
```

| Reference `(x, y)` | Depth plane z |
|---|---:|
| under camera (0.1848, 0.0095) | **−0.0787 m** |
| `(0.34, 0.0)` | **−0.0810 m** |
| board centre (0.4487, 0.0853) | −0.0767 m |
| tag A (0.3312, 0.0032) | −0.0807 m |

Depth statistics: median range 0.3684 m, p10 0.3195 m, p90 0.4428 m; inlier
median 0.3676 m. The optical axis in `base_link` is
`(0.5989, −0.0128, −0.8007)` (≈53.2° depression); the camera optical origin is
`(0.1848, 0.0095, 0.2163) m`.

## RGB chessboard comparison

Sixteen table-down samples (measured joints, 20.0 mm squares) give the board
top plane in `base_link`:

| Metric | Value |
|---|---:|
| Board plane z mean | **−0.07054 m** |
| Board plane z std | 1.65 mm |
| Depth plane at the board centre | −0.07664 m |
| Board top − depth plane | **+6.1 mm** |

**Corrected 2026-11-18:** the +6.1 mm is **not** the board thickness. The
board is **11 mm** thick (calipered), and the chessboard is **invisible to the
depth camera** — the raw depth inside the board is a filled table plane with
no step, even though the IR pattern is visible. A same-frame matte-AprilTag
test (tag sheet on the board and flat on the table) measured a **−4.25 /
−4.65 mm same-surface RGB-vs-depth gap**; the RGB absolute z is biased −3 to
−5 mm at this view. The 6.1 mm is the RGB board top minus the depth table with
that bias, not the board thickness. There is no 40–50 mm RGB depth bias at
this table view, but the RGB/depth agreement is ~4.6 mm, not 6.1 mm. See
[rgbd_same_surface_20261118.md](rgbd_same_surface_20261118.md). The depth table
plane remains the absolute-z reference.

## AprilTag absolute-z caveat

Recomputing the saved A/B/C tag frames (forward TSAI, as recorded) against the
depth plane at each tag `(x, y)`:

| Tag | RGB base z | Depth table z | RGB − depth |
|---|---:|---:|---:|
| A | −0.0936 | −0.0807 | **−12.9 mm** |
| B | −0.0926 | −0.0869 | **−5.6 mm** |
| C | −0.0989 | −0.0885 | **−10.4 mm** |

The tag's PnP plane is tilted **8.4°** from vertical while the table is 3.7°, so
the small sticker's absolute z is biased low by 6–13 mm at this view. This does
not affect the 100.67 mm lateral scale result (0.67 %), which is measured from
the same tag and cancels the constant offset. **Do not use the tag absolute z
for table height.** The chessboard (large, refined corners) and the depth plane
are the reliable absolute references.

## Table-z correction

For a target **on the table surface**, use the depth plane:

```text
z_table(x, y) = −(0.01031 x − 0.06301 y + 0.07728) / 0.99796   [m]
```

At the TCP XY this is −0.0797 m, 1.3 mm from the 215 mm ruler check. The RGB
PnP absolute z is not used for the table surface:

- board top: RGB z is the table + 6.1 mm (board thickness);
- tag: RGB z is the table − 5.6…−12.9 mm (tag-view bias);
- the table-view TSAI is the correct hand-eye for this view; using the
  camera_forward TSAI at the table pose changes the absolute z by ~10 mm.

## Absolute table-alignment plan (plan-only)

A table-aligned pre-grasp target 9 cm above the table at `(0.34, 0.0)`:

```text
target = (0.34, 0.0, 0.009) m
```

| Stage | Count |
|---|---:|
| Candidates generated | 44 |
| Sanity | 44 |
| IK / FK | 22 / 22 |
| Collision-free | 2 |
| Plan-only OK | **1** |
| Safety approved | **1 / 1** |

The single plan (`js_0016`) reaches TCP `(0.325, 0.000, 0.0331)`, 28.4 mm from
the requested point (inside the 30 mm sampler radius), 75 points, 7.38 s. Its
joints are `[−0.001, −0.662, −1.401, −0.192, −0.092]`: **joint2 is exactly at
the adopted low bound −0.662 rad**. The plan is valid but sits on a limit, so it
was **not executed**. A future sampler should reject poses within a joint-limit
margin; until then, do not execute this candidate.

**Resolved 2026-11-18:** the sampler and evaluator now enforce a 0.03 rad
joint-limit margin, and `js_0016` is no longer generated. See
[joint_limit_margin_20261118.md](joint_limit_margin_20261118.md). The target
still has no margin-safe plan in the mock, so it must move inward/upward before
a table grasp; do not execute `js_0016`.

## Files

| File | Content |
|---|---|
| `depth_plane_optical_20261118.json` | Full fit report: cloud/joints/transforms, plane, references, 2000-point sample |
| `table_alignment_20261118.json` | Table plane, ruler check, board and tag comparisons |
| `../../../tools/phase2/depth_plane_fit.py` | Reusable read-only fitter (optical convention) |

## Reproduction

```bash
# Inside the Humble container, arm at table_down, camera running:
python3 /repo_tools/phase2/depth_plane_fit.py \
  --output /tmp/depth_plane.json \
  --hand-eye /tmp/hand_eye_table_20261118.json \
  --joints-pulses 497,565,774,122,493
```

The ruler check is manual: measure the vertical distance from the table surface
to the lowest gripper fingertip while the arm is stationary at table_down.
