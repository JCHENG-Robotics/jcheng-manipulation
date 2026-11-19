# Chessboard Camera / Hand-Eye Calibration

> Status: **Validated on 2026-11-17 (second run); re-validated table-down
> 2026-11-18; used for real grasps 2026-11-19.** Using the physical
> bus-servo positions in FK, 14 chessboard poses give a board-pose scatter of
> **2.94 mm / 0.39°** (max 5.44 mm / 0.74°), the held-out final base is
> **3.29 mm / 0.43°**, 2-fold cross-validation gives **2.5–3.2 mm /
> 0.31–0.43°**, and leave-one-out gives **3.41 mm / 0.44°** std (max
> 6.52 mm / 0.81°). The 5 mm / 2° board gate passes. The first run (cached
> `joint_states` only) reached 13.4 mm / 2.06° and is kept as the baseline in
> [`evidence/calibration/20261117/`](../../evidence/calibration/20261117/).
> A table-down 15-pose fit gives 2.7 mm / 0.42°; real grasps later measured a
> configuration-dependent FK error of up to 15 mm at the low grasp pose, which
> is outside this calibration volume.

## Goal and frames

The camera is mounted on `link4`. The calibrated transform is

```text
T_link4_camera : camera optical frame -> link4
```

The image/PnP pose is in the OpenCV **optical** frame (x right, y down, z
forward). The live `depth_cam_color_optical_frame` and
`depth_cam_depth_optical_frame` are **identity to `rgb_camera_link`** in the
driver TF; they are body frames, not the OpenCV optical frame despite their
names. The live static TF `link4 → rgb_camera_link` is the vendor **body**
frame. The measured body→optical adapter is
`inv(T_link4_rgb_body) @ T_link4_camera` with the full-precision live TF:
**89.798° about -z**, translation `[5.4, 10.1, -27.4] mm`. It is published as
`camera_optical_calibrated`; see
[`docs/hardware/camera_frames.md`](../hardware/camera_frames.md). Use
`T_link4_camera` directly in perception code, or use the published frame in
TF. Do not replace the URDF static TF with the optical transform; add a
separate optical frame if the URDF needs one.

> The earlier 89.34° / `[15.4, 10.0, -28.3] mm` numbers came from a
> three-decimal `tf2_echo` snapshot with x = -0.050 m. The full-precision live
> TF has x = -0.040 m. The hand-eye fit did not use the snapshot, so the
> calibration result is unchanged; only the body→optical adapter changed.

## Target and procedure

| Item | Value |
|---|---|
| Board | OpenCV chessboard, 11×8 inner corners, 20 mm squares |
| Camera | Aurora 930 RGB, 640×400, `bgr8`, `rgb_camera_link` |
| Capture base | `[0.0, 0.59888, -1.25664, -0.71209, 0.0]` (joint1–joint5), i.e. `camera_forward` with joint2 +0.18 rad to centre the board |
| Robot pose | link4 FK from the project URDF, using **measured bus-servo positions** |
| Approach | pre-pose `target − 0.08 rad` on joint1 only, then target (takes up the largest backlash) |
| Capture | 4 s single-point trajectories; 0.5 s settle, 3 warm-up frames, then 3 frames per pose combined by median |
| Detector | `findChessboardCorners` + `cornerSubPix` + `solvePnP(IPPE)` + `solvePnPRefineLM` |
| Servo feedback | `/ros_robot_controller/bus_servo/get_state`, 3 reads per end, median |
| Poses | 15 targets: base, joint1 ±0.12, joint2 ±0.06, joint3 ±0.06, joint4 ±0.12 and mixed-sign combinations; 14 detected |

Servo mapping and conversion (vendor `servo_controller.yaml`):

```text
joint1..joint5 = servo IDs 6, 5, 4, 3, 2
radians = (pulse - 500) * 240/360 * 2*pi / 1000     # joint3 is inverted:
radians = (500 - pulse) * 0.00418879                # pulse 1000 -> -2.094 rad
```

The controller's `/joint_states` and `/controller_manager/servo_states` are
cached commands. The bus-servo read is a hardware position read; at the same
commanded pose it differed from the command by up to **2.16°** (RMS 1.13°),
which is exactly the scatter that limited the first calibration.

Tools:

- `tools/phase2/chessboard_capture.py` — runs inside the robot container.
  Moves the arm, reads the bus servos, saves images, commands, measured joints,
  camera K/D and board PnP to a private directory.
- `tools/phase2/chessboard_hand_eye.py` — runs on the laptop with the Jazzy
  workspace sourced. Computes link4 FK, prefers `measured_joints`, checks
  board-pose consistency, runs all five `cv2.calibrateHandEye` methods,
  compares with the static TF as information, refines the chessboard
  intrinsics and checks a held-out `final_base` sample.
- `tools/phase2/calibration_pose_selector.py` — offline selector for 20–30
  diverse, executable poses; see
  [CALIBRATION_POSES.md](CALIBRATION_POSES.md). The slim `poses.json` output
  is consumed by `chessboard_capture.py --poses-file`.

## Pose selection (Session 2, 2026-11-17)

The validated 14-pose result used hand-picked small offsets. Session 2
replaced that with an explicit selection: 14,176 evaluated poses (the 14,175
grid points plus the exact base), 3,093 feasible poses, and a 24-pose
farthest-point set including the exact base pose. The
set has 33.5° maximum view-angle spread, 0.096 m maximum camera-position
spread, 15.1–25.0% board image area, and passes the rotation-excitation and
non-coplanarity checks (rotation-axis rank ratio 0.067, planarity ratio
0.283). It has **not** been executed yet. The full pose list, metrics and
on-site command are in [CALIBRATION_POSES.md](CALIBRATION_POSES.md); evidence
is in [`evidence/calibration/20261117/pose_selection/`](../../evidence/calibration/20261117/pose_selection/).

Reproduction (inside the robot container):

```bash
sudo docker cp tools/phase2/chessboard_capture.py ArmPiUltra:/tmp/
sudo docker exec -d -u ubuntu -w /home/ubuntu ArmPiUltra /bin/zsh -c \
  'source ~/.zshrc >/dev/null 2>&1; exec python3 /tmp/chessboard_capture.py \
     --output-dir /tmp/chessboard_calib_servo_20261117 \
     --base-pose 0.0,0.59888,-1.25664,-0.71209 \
     --delta 0.12 --pre-offset 0.08 --pre-joint1-only \
     --final-base --servo-reads 3 > /tmp/calib_servo_run.log 2>&1'
```

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 tools/phase2/chessboard_hand_eye.py \
  --samples ~/jcheng/calibration/20261117/chessboard_calib_servo_20261117/samples.json \
  --output evidence/calibration/20261117/calibration_servo_20261117.json
```

## Validated result (14 poses)

```text
T_link4_camera
translation = [-0.0348, -0.0091, 0.0586] m
RPY         = [0.951, -0.058, -89.789] deg
```

| Metric | Value | Gate |
|---|---|---|
| Board pose in `base_link`, translation std | **2.94 mm** | ≤ 5 mm |
| Board pose in `base_link`, rotation std | **0.39°** | ≤ 2° |
| Board max translation error | 5.44 mm | — |
| Board max rotation error | 0.74° | — |
| Held-out `final_base` translation / rotation | **3.29 mm / 0.43°** | ≤ 5 mm / 2° |
| Leave-one-out board pose, std | **3.41 mm / 0.44°** (max 6.52 mm / 0.81°) | ≤ 5 mm / 2° (std) |
| Chessboard intrinsic refinement | fx 418.31 / fy 418.34 vs driver 418.05 / 419.10; RMS **0.060 px** | ≤ 1 px |
| Corner RMS | 0.09–0.19 px | — |
| Servo shift during a capture | 0–1 pulse (≤ 0.24°) | — |

Method agreement (`board_translation_std_m` / `board_rotation_std_deg`):

| Method | translation (m) | RPY (deg) | board std |
|---|---|---|---|
| **TSAI (selected)** | `[-0.0348, -0.0091, 0.0586]` | `[0.95, -0.06, -89.79]` | 2.94 mm / 0.39° |
| PARK | `[-0.0343, -0.0091, 0.0589]` | `[0.40, -0.11, -89.93]` | 2.97 mm / 0.39° |
| HORAUD | `[-0.0343, -0.0091, 0.0589]` | `[0.40, -0.11, -89.93]` | 2.97 mm / 0.39° |
| DANIILIDIS | `[-0.0360, -0.0083, 0.0600]` | `[0.96, -0.01, -89.94]` | 3.00 mm / 0.39° |
| ANDREFF | `[0.0292, -0.0187, -0.0008]` | `[0.49, -0.11, -89.87]` | 6.31 mm / 0.39° (outlier) |

2-fold cross-validation (fit on 7 poses, evaluate the other 7):

| Split | TSAI held-out | PARK held-out | HORAUD held-out |
|---|---|---|---|
| even | 2.54 mm / 0.31° | 2.51 mm / 0.30° | 2.51 mm / 0.30° |
| odd | 3.16 mm / 0.42° | 3.19 mm / 0.43° | 3.19 mm / 0.43° |

The static-TF comparison is **informational**, not a pass gate. With the
full-precision live TF the solver reports **29.7 mm / 89.798°**, and the
measured body→optical adapter is **89.798° / `[5.4, 10.1, -27.4] mm`** (see
[`docs/hardware/camera_frames.md`](../hardware/camera_frames.md)). The earlier
33.7 mm / 89.34° numbers used a three-decimal `tf2_echo` snapshot with
x = −0.050 m and are superseded. The board scatter, held-out base and
cross-validation are the meaningful checks.

## Independent AprilTag displacement validation (2026-11-18)

The hand-eye was checked independently of the chessboard with a flat 36h11
sticker (ID 3, **24.9 mm** black square) on the table. The arm was held at
`[0.0, 0.2974, -1.14773, -1.55404, -0.01676]` (table-down, outside the
camera_forward calibration pose range; the same view was later calibrated
separately, below), with measured bus-servo joints. The tag edge was kept
against a steel L-square; five frames were captured at each position.

| Move | Predicted | Ruler | Error |
|---|---:|---:|---:|
| A→B, lateral (camera right) | **100.67 mm** | 100.0 mm | **+0.67 mm (0.67 %)** |
| B→C, table-forward | **104.87 mm** | 100.0 mm | +4.87 mm (4.87 %) |
| Angle between A→B and B→C | **92.32°** | 90° | +2.32° |

Detection was 5/5 at each position (max reprojection 0.43 px; per-position
std ≤0.7 mm). The lateral/image-plane scale is validated to <1 %. The
table-forward move is ~5 % high: it moves the tag farther from the camera
(0.36 → 0.42 m) and toward the image edge, where the PnP depth axis is weak at
the ~48° incidence. Orthogonality is good.

**Absolute table-pose depth (corrected 2026-11-18):** the tag predicts
`base_link` z ≈ −0.09 m. The original interpretation compared this with a
chassis estimate (`−0.024 m`) and concluded that the RGB PnP was biased by
~40–50 mm. A later depth point-cloud plane and a direct ruler measurement
overturned that: the table is at **z ≈ −0.078 m**. The depth plane gives
**−0.0787 m** under the camera, a 215 mm ruler check on the gripper fingertip
gives **−0.0784 m** (1.3 mm apart). The RGB chessboard board top
(**−0.0705 m**) is 6.1 mm above the depth plane, but the board is **11 mm**
thick (calipered 2026-11-18) and the chessboard is **not visible to the depth
camera** (high-contrast/glossy board → depth holes filled with the table
plane). The 6.1 mm is therefore **not** the board thickness: a same-frame
matte-AprilTag test measured a **−4.6 mm RGB-vs-depth same-surface gap** at
this view. The tag's own absolute z is biased 6–13 mm low at this view (its
fitted plane is tilted 8.4° while the table is 3.7°); the lateral 0.67 % scale
result stands. Do **not** use the tag absolute z or the chassis estimate for
table height; use the depth plane. Evidence:
[`evidence/phase2/onsite_20261118/depth_plane_20261118.md`](../../evidence/phase2/onsite_20261118/depth_plane_20261118.md),
[`evidence/phase2/onsite_20261118/rgbd_same_surface_20261118.md`](../../evidence/phase2/onsite_20261118/rgbd_same_surface_20261118.md)
and
[`evidence/phase2/onsite_20261118/tag_displacement_validation/`](../../evidence/phase2/onsite_20261118/tag_displacement_validation/).

## Table-down calibration (2026-11-18)

A second 15-pose chessboard capture was run at the table-down pose with the
board flat on the table (20.0 mm squares, measured with a caliper). The
table-view TSAI fit gives `T_link4_camera` = `[-0.03194, -0.00808, 0.05918] m`,
RPY `[-1.478, 0.331, -90.187]°`; board scatter **2.7 mm / 0.42°**, held-out
final base 2.2 mm / 0.29°, leave-one-out 3.1 mm / 0.45° std, intrinsic RMS
0.052 px. It passes the 5 mm / 2° gate and differs from the camera_forward TSAI
by `[2.9, 1.0, 0.6] mm` and ~2.5°; both are internally consistent. The
table-view fit is the correct hand-eye for this view, but its RGB absolute z
carries the **−3 to −5 mm** view bias measured by the same-frame RGB-D test;
the board top is **11 mm** above the table, not the 6.1 mm implied by the
filled depth plane. Use the depth table plane for table z. The transform does
**not** replace the camera_forward transform for near-normal perception.
Evidence:
[`evidence/calibration/20261118/table_down/`](../../evidence/calibration/20261118/table_down/).

## Low-configuration error survey (2026-11-19)

The chessboard cannot be used at the grasp configuration: the camera sits
0.11–0.14 m above the floor there, so the 220 × 160 mm board does not fit in
the view (a 0.20 m camera height is needed, which forces `j2 ≈ 0` — 0.6 rad
away from the grasp pose). The error was therefore measured **relative to a
fixed AprilTag** (36h11 ID 0, 60 mm) that is observed from two reference poses
inside the validated table-down volume and from eight poses of the low family.

The result is that the low-configuration error is a **constant vector**, not a
configuration-dependent one: the per-pose spread is 0.40–0.99 mm and the
residual after a single constant correction is at most 2.4 mm over `j1 ±0.06`,
`j2 +0.10`, `j3 +0.05` and `j4 −0.29 … −0.545`.

| Model used for the correction | Correction (x, y, z) | Mean \|e\| | Residual after correction |
|---|---|---:|---:|
| Table-down fit (used by the 2026-11-19 grasps) | **(+11.6, +4.0, +8.5) mm** | 15.0 mm | 2.4 mm |
| Camera-forward fit (published as `camera_optical_calibrated`) | **(+5.2, +1.8, +5.8) mm** | 8.0 mm | 2.0 mm |

The two columns differ because the two fits disagree by ~2.5°; **the
correction must be applied against the same model that measures the target**.

Two further results:

- The bus-servo register is a faithful joint measure. The same commanded
  `joint1 = 0` settles at register 495 or 506 depending on the approach
  direction (11 pulses = 4.6°), and the image moves by the same 4.6°, so the
  measured-joint FK absorbs the servo dead band (predicted tag centre moves
  1.1 mm). The approach direction must still be reproduced at grasp time.
- A whole-node restart moves the arm and leaves `/joint_states` stale; the
  return pose of any capture sequence must come from the bus servos.

Evidence, including the conflict analysis against the single-tag pass-5
correction (`e = (−8.0, −15.3, −5.8) mm`), which was made with both tags within
8 px of the frame edge:
[`evidence/calibration/20261119/low_config_error/`](../../evidence/calibration/20261119/low_config_error/).

**Grasp validation (2026-11-19).** The correction was applied to a cube
measured from a tilted table-down view (`j4 = −1.70`, camera 0.22 m up):
`(+13.7, +3.9, +8.9) mm` on the cube centroid, fitted joints
`[−0.015, −0.6618, −1.8012, −0.340]`, close `r_joint −1.6 → −0.63`. The cube
was gripped, lifted 150 mm and released **1.3 mm** from the measurement. That
settles the sign: at the grasp configuration the model **overestimates** x by
~14 mm relative to the tilted view, the opposite of the pass-5 vector. The same
session measured a **~13 mm z error** at the tilted pose (the depth plane
through the model reads −0.0918 m against the calibrated −0.0784 m), so the
tilted view is outside the validated volume and the x/y correction is the
load-bearing quantity. Evidence:
[`evidence/phase2/onsite_20261119/low_config_grasp_validation/`](../../evidence/phase2/onsite_20261119/low_config_grasp_validation/).

## Why the first run failed

The first dataset used the controller's cached `joint_states` as if they were
physical joint angles. The bus-servo read shows the physical positions differ
from those commands by up to 2.16° (RMS 1.13°), depending on the pose. At a
0.32 m board distance, 2° is about 11 mm, which matches the 13.4 mm board
scatter of the first run. A controlled intermediate run with the same centered
base and 15-pose set but cached commands gave 9.9 mm / 1.38°; replacing only
the joint source with measured servo positions reduced the scatter to 2.9 mm.
The camera, board, pose set and detector were unchanged.

The first run's other findings still hold: the board and camera are stable when
the arm does not move (0.02–0.03 mm over repeated frames), and the joint1-only
pre-pose is the validated approach protocol. The earlier claim that
`servo_states` is not feedback was correct for
`/controller_manager/servo_states`; the bus-servo service is the separate
hardware read.

## Limitations

- The arm is open-loop; the bus-servo position is the servo's own register, not
  an independent external metrology. There are no joint encoders.
- Two board placements were used: a vertical stand at `camera_forward` and a
  flat table at the table-down pose. The relative scale was checked with a
  ruler (0.67 % lateral); the 215 mm fingertip-height check plus the depth
  plane now anchor the table z in `base_link` to ~1 mm, but there is still no
  surveyed 3D point in `base_link`.
- Absolute table z is **−0.078 m**, not the chassis estimate −0.024 m. The
  depth plane gives −0.0787 m and the 215 mm ruler check −0.0784 m (1.3 mm
  apart). The RGB absolute z at the table view is biased **−3 to −5 mm**, and
  the same-surface RGB-vs-depth gap is **≈ −4.6 mm** (matte-AprilTag test);
  the board is **11 mm** thick, and the 6.1 mm RGB-board-vs-depth-table number
  is **not** the board thickness (the chessboard is invisible to the depth and
  the hole is filled with the table). Use RGB XY + the depth table plane for z,
  with a ≥5 mm vertical margin. The AprilTag absolute z is also biased 6–13 mm
  low at the table view (tag-plane tilt 8.4° vs table 3.7°); use the tag only
  for relative displacement.
- The vendor RGB-D registration is not calibrated here: the same-surface gap
  is view-dependent and was measured at one table pose. Do not assume it at
  other views.
- Link geometry comes from the vendor URDF. Small link/zero errors are absorbed
  into the hand-eye transform and may grow outside the calibration volume
  (roughly 0.25–0.40 m in front of the camera).
- The transform is valid only for the current camera mount. Remounting the
  camera invalidates it.
- Raw RGB images stay private and are not committed.
- This is a calibration result, not an absolute metrology claim. Use it for
  visual grasping with a safety margin. Real grasps were run on 2026-11-18/19;
  the low-configuration FK error is recorded in
  [`evidence/phase2/onsite_20261119/grasp_150mm_attempt5/`](../../evidence/phase2/onsite_20261119/grasp_150mm_attempt5/).

## Next

1. ~~Add a joint-limit margin to the grasp sampler.~~ **Done (2026-11-18):**
   the sampler and evaluator enforce `--joint-limit-margin 0.03 rad` and the
   evaluator rejects boundary IK solutions as `JOINT_LIMIT_MARGIN`; see
   [GRASP_CANDIDATE_SEARCH.md](GRASP_CANDIDATE_SEARCH.md) §5.1.
2. ~~Do a same-instant RGB-D capture with the chessboard / measured-thickness
   target.~~ **Done (2026-11-18):** the matte-AprilTag test measured a −4.6 mm
   same-surface RGB-vs-depth gap; the board is 11 mm and the old 6.1 mm was
   not the board thickness. See
   [`evidence/phase2/onsite_20261118/rgbd_same_surface_20261118.md`](../../evidence/phase2/onsite_20261118/rgbd_same_surface_20261118.md).
3. If better than ~3 mm absolute metrology is required, add an external
   measurement (e.g. a dial indicator or a second fixed camera) rather than
   more chessboard poses; the remaining error is the open-loop arm and the
   view-dependent PnP depth axis.
