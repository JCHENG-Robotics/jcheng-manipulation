# Calibration Pose Selection — 2026-11-17 (Session 2)

Offline selection of diverse, executable chessboard poses. **No robot motion
was commanded for this evidence.** The selector uses the already-validated
hand-eye calibration and the measured board pose; it only evaluates FK and
projected board visibility.

## Command

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
python3 tools/phase2/calibration_pose_selector.py \
  --calibration-json \
    evidence/calibration/20261117/calibration_servo_20261117.json \
  --output evidence/calibration/20261117/pose_selection \
  --count 24 --plot
```

## Files

| File | Content |
|---|---|
| `pose_selection_20261117.json` | Full report: inputs, search grid, 24 poses with predicted distance/incidence/image metrics, diversity report, pass/fail checks |
| `poses.json` | Slim 24-pose list consumed by `chessboard_capture.py --poses-file` |
| `pose_selection_20261117.png` | Camera positions in `base_link`, predicted board centres, selected joint offsets |

## Result

- Evaluated 14,176 poses (14,175 grid points plus the exact base) → feasible
  3,093 → selected 24 (including the exact validated base pose).
- Board distance 0.286–0.372 m; incidence 0.42–6.44°; projected board area
  15.1–25.0%; minimum image margin 25.1 px.
- Camera-position spread 0.003–0.096 m; view-angle spread 2.3–33.5°;
  image-centre std 92.8 / 50.6 px.
- Rotation-axis singular values 4.111 / 2.277 / 0.275 (rank ratio 0.067);
  camera-centre planarity ratio 0.283 (not coplanar).
- All criteria pass: count, `joint5` fixed, no duplicate views, camera spread,
  view spread, image coverage, joint spread, rotation excitation, not
  coplanar.

## Scope and limitations

- The board pose is the mean of the 14 validated samples; if the board or
  camera mount moves, re-run with the new `T_base_board`.
- Visibility is predicted with the current validated `T_link4_camera`, so the
  selection is a prior for the next capture, not a detection result.
- Joint limits are provisional and the selector does **not** collision-check.
  The search envelope is deliberately conservative around the executed
  `camera_forward` region; the operator must still supervise the motion.
- `joint5` is held at 0 because the camera is mounted on `link4`.

Procedure and interpretation: [`docs/phase2/CALIBRATION_POSES.md`](../../../../docs/phase2/CALIBRATION_POSES.md).
