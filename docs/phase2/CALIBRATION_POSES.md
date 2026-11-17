# Calibration Pose Selection

> Status: **Implemented and offline-validated (2026-11-17, Session 2)**.
> 24 poses were selected from 14,176 evaluated poses (14,175 grid points plus
> the exact base pose) using the validated `T_link4_camera` and the measured
> chessboard pose in `base_link`. The set
> passes the diversity, rotational-excitation and coplanarity checks. It has
> **not** been executed on the robot yet.

## Why select poses

The first validated hand-eye run used 15 hand-picked small offsets around
`camera_forward` (`joint1`/`joint4` ±0.12 rad, `joint2`/`joint3` ±0.06 rad).
That set happened to work, but it was not chosen for observability:

- all poses were close to one view direction;
- `joint5` cannot change the camera view because the camera is mounted on
  `link4`, so only `joint1..joint4` matter;
- a hand-eye fit needs rotational excitation about non-parallel axes; a
  nearly coplanar or single-axis set is underdetermined even when every image
  detects the board.

The selector makes the choice explicit and repeatable.

## Inputs

| Input | Source in this run |
|---|---|
| Hand-eye transform `T_link4_camera` | `hand_eye.methods.TSAI.T_link4_camera` from [`calibration_servo_20261117.json`](../../evidence/calibration/20261117/calibration_servo_20261117.json) |
| Board pose `T_base_board` | `hand_eye.methods.TSAI.board_mean_transform` from the same report |
| Camera intrinsics | `intrinsics.driver_k` / `driver_d` (the values the driver publishes) |
| Robot model | Project URDF, `ArmKinematics(tip_link='link4')` |
| Base pose | `[0.0, 0.59888, -1.25664, -0.71209, 0.0]` (the validated capture base) |

The board pose is the mean of the 14 validated samples, so it contains the
same ~3 mm / 0.4° uncertainty as the calibration. The selector treats it as
nominal; on-site detection decides whether a pose is usable.

## Method

1. **Grid.** Sample `joint1..joint4` at 0.05 rad around the base pose, inside
   the URDF limits with a 0.03 rad margin. `joint5` is held at 0.
2. **Feasibility.** Keep a pose only when all four inner-corner board corners
   project inside the 640×400 image with at least 25 px margin, the board
   centre is 0.18–0.60 m away, the incidence angle is at most 60°, and the
   projected board area is at least 2% of the image.
3. **Diversity feature.** For each feasible pose, build a weighted vector:
   camera position in the board frame, camera viewing direction in the board
   frame, image roll (`cos`, `sin`) and normalised image centre.
4. **Selection.** Seed with the exact validated base pose, then greedily add
   the pose that maximises the minimum feature-space distance to the selected
   set (farthest-point sampling). Ties are broken by a quality score that
   prefers image margin, board area and near-mid-range distance.
5. **Checks.** Report camera-position spread, view-angle spread, image-centre
   coverage, per-joint spread, rotation-axis rank ratio, planarity ratio and
   a duplicate-view score.

The search envelope is deliberately conservative: `joint1` ±0.35,
`joint2` ±0.18, `joint3` ±0.15, `joint4` ±0.35 rad. The mechanical limits are
still provisional. Widen the ranges with `--joint1-range` etc. only after the
joint-limit survey.

## Selected set (24 poses)

| # | joint1 | joint2 | joint3 | joint4 | joint5 | distance m | incidence deg | image centre px | margin px |
|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|
| 0 | 0.000 | 0.599 | -1.257 | -0.712 | 0.000 | 0.326 | 1.16 | (317, 247) | 48.5 |
| 1 | 0.250 | 0.719 | -1.357 | -0.962 | 0.000 | 0.335 | 5.19 | (455, 120) | 25.1 |
| 2 | -0.250 | 0.769 | -1.407 | -0.912 | 0.000 | 0.334 | 4.52 | (182, 140) | 25.6 |
| 3 | 0.300 | 0.779 | -1.107 | -1.012 | 0.000 | 0.372 | 1.53 | (466, 270) | 26.0 |
| 4 | -0.250 | 0.779 | -1.107 | -1.012 | 0.000 | 0.365 | 0.42 | (194, 268) | 38.9 |
| 5 | 0.000 | 0.519 | -1.157 | -0.962 | 0.000 | 0.312 | 2.55 | (317, 134) | 26.5 |
| 6 | 0.200 | 0.419 | -1.407 | -0.412 | 0.000 | 0.295 | 6.44 | (443, 207) | 42.8 |
| 7 | -0.200 | 0.469 | -1.407 | -0.412 | 0.000 | 0.298 | 4.46 | (197, 235) | 25.5 |
| 8 | 0.150 | 0.769 | -1.157 | -1.062 | 0.000 | 0.359 | 1.60 | (390, 212) | 91.3 |
| 9 | -0.100 | 0.779 | -1.207 | -1.062 | 0.000 | 0.352 | 1.64 | (267, 188) | 90.7 |
| 10 | -0.150 | 0.419 | -1.407 | -0.512 | 0.000 | 0.286 | 6.23 | (224, 152) | 26.5 |
| 11 | 0.050 | 0.419 | -1.407 | -0.462 | 0.000 | 0.289 | 5.45 | (349, 181) | 69.8 |
| 12 | 0.300 | 0.779 | -1.257 | -1.012 | 0.000 | 0.355 | 3.15 | (472, 185) | 28.8 |
| 13 | 0.200 | 0.569 | -1.207 | -0.712 | 0.000 | 0.329 | 2.87 | (429, 261) | 26.0 |
| 14 | 0.150 | 0.419 | -1.107 | -0.862 | 0.000 | 0.304 | 3.68 | (406, 161) | 56.2 |
| 15 | -0.250 | 0.619 | -1.107 | -0.962 | 0.000 | 0.336 | 1.54 | (183, 215) | 34.0 |
| 16 | -0.100 | 0.419 | -1.107 | -0.762 | 0.000 | 0.303 | 1.05 | (259, 213) | 78.8 |
| 17 | -0.100 | 0.769 | -1.107 | -1.012 | 0.000 | 0.364 | 0.43 | (268, 264) | 42.6 |
| 18 | 0.050 | 0.779 | -1.107 | -1.012 | 0.000 | 0.368 | 0.87 | (341, 270) | 35.2 |
| 19 | 0.050 | 0.569 | -1.107 | -0.962 | 0.000 | 0.327 | 1.42 | (344, 192) | 92.2 |
| 20 | 0.100 | 0.779 | -1.307 | -1.062 | 0.000 | 0.344 | 3.77 | (368, 131) | 36.3 |
| 21 | 0.100 | 0.419 | -1.307 | -0.462 | 0.000 | 0.298 | 3.50 | (379, 246) | 36.7 |
| 22 | -0.150 | 0.669 | -1.407 | -0.562 | 0.000 | 0.328 | 2.06 | (236, 270) | 25.2 |
| 23 | -0.100 | 0.779 | -1.307 | -1.062 | 0.000 | 0.341 | 3.53 | (265, 130) | 27.2 |

`joint5` stays at 0 in every pose because it rotates the gripper, not the
`link4` camera.

## Diversity and checks

| Metric | Value | Criterion |
|---|---|---|
| Evaluated / feasible / selected | 14,176 / 3,093 / 24 | 20–30 selected |
| Board distance | 0.286–0.372 m (mean 0.330) | 0.18–0.60 m |
| Incidence angle | 0.42–6.44° (mean 2.88) | ≤ 60° |
| Projected board area | 15.1–25.0% of the image | ≥ 2% |
| Minimum image margin | 25.1 px | ≥ 25 px |
| Camera-position spread | 0.003–0.096 m (mean 0.039) | max ≥ 0.05 m |
| View-angle spread | 2.3–33.5° (mean 14.5) | max ≥ 10° |
| Image-centre std | 92.8 / 50.6 px | ≥ 20 px |
| Joint std (`joint1..4`) | 0.171 / 0.149 / 0.121 / 0.231 rad | ≥ 0.02 rad |
| Rotation-axis singular values | 4.111 / 2.277 / 0.275 | rank ratio 0.067 ≥ 0.05 |
| Camera-centre planarity ratio | 0.283 | ≥ 0.01 (not coplanar) |
| Duplicate-view score | 15.5 | > 1 (no duplicates) |

The minimum pairwise camera distance is **3.2 mm** and the minimum view-angle
separation is **2.3°**; those close pairs differ strongly in the other
features, so the combined feature distance stays large. These are diagnostics,
not pass/fail gates. The pass/fail checks are the spread, coverage, excitation
and coplanarity rows above.

## Reproduce

On the laptop with the Jazzy workspace sourced:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 tools/phase2/calibration_pose_selector.py \
  --calibration-json \
    evidence/calibration/20261117/calibration_servo_20261117.json \
  --output evidence/calibration/20261117/pose_selection \
  --count 24 --plot
```

Outputs:

| File | Content |
|---|---|
| `pose_selection_20261117.json` | Full report: inputs, search, 24 poses with predicted metrics, diversity, checks |
| `poses.json` | Slim list consumed by `chessboard_capture.py --poses-file` |
| `pose_selection_20261117.png` | Camera positions, predicted image centres, joint offsets |

## On-site execution

Copy the capture tool and the slim pose file into the robot container, then
run the capture. The arm moves to each absolute target; `--pre-offset` and
`--pre-joint1-only` keep the validated backlash take-up.

```bash
sudo docker cp tools/phase2/chessboard_capture.py ArmPiUltra:/tmp/
sudo docker cp \
  evidence/calibration/20261117/pose_selection/poses.json \
  ArmPiUltra:/tmp/poses.json
sudo docker exec -it -u ubuntu -w /home/ubuntu ArmPiUltra /bin/zsh -c \
  'source ~/.zshrc >/dev/null 2>&1; \
   python3 /tmp/chessboard_capture.py \
     --output-dir /tmp/chessboard_calib_selected \
     --poses-file /tmp/poses.json \
     --pre-offset 0.08 --pre-joint1-only \
     --final-base --servo-reads 3'
```

Then solve as before:

```bash
python3 tools/phase2/chessboard_hand_eye.py \
  --samples ~/jcheng/calibration/<date>/chessboard_calib_selected/samples.json \
  --output evidence/calibration/<date>/calibration_selected.json
```

The capture script validates each target against the provisional joint limits,
records `pose_source: poses_file`, and writes the usual `samples.json` with
`measured_joints` and board PnP. A pose where the board is not detected is
retained in the dataset and excluded by the solver's RMS filter; the operator
can also rerun just that pose.

## Limitations

- **No collision checking.** The selector checks joint limits and board
  visibility only. The on-site operator must confirm the path and keep the
  workspace clear. The conservative envelope is near the already-executed
  capture region, but it is not a collision guarantee.
- **Provisional mechanical limits.** The URDF limits are vendor values, not
  measured stops. The 0.03 rad margin is a software margin, not a mechanical
  one.
- **Board pose is nominal.** If the board or camera mount moves, re-run the
  selector with the new `T_base_board` (or validate a fixed board fixture).
- **Circular prior.** Visibility is predicted with the current validated
  `T_link4_camera`. A badly wrong prior would mis-rank poses; the actual capture
  still detects the board per pose.
- **Joint5 is fixed.** This is correct for a `link4`-mounted camera, but a
  different mount that puts the camera after `joint5` would need the joint
  included in the search.

## Verification

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 -m pytest tools/phase2/tests -q
```

The suite covers the synthetic hand-eye recovery, the board-projection math,
the greedy selection and the real validated board/camera pair (20-pose gate).
