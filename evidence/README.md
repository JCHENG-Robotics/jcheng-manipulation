# Evidence Index

This directory mixes **hardware**, **synthetic** and **mock** evidence. Each
dated folder states its scope; do not treat a mock or synthetic result as
hardware validation. Raw MCAP bags, Studio SQLite data and raw RGB frames are
gitignored or kept private.

## Hardware evidence

| Path | Content | Scope |
|---|---|---|
| `phase1/20261117/` | Robot/ROS/servo/TF/camera baseline, camera retry, RGB frame, logs | Real hardware, read-only + one supervised `reset_servo` |
| `calibration/20261117/` | Chessboard hand-eye datasets, per-method results, validation | Real hardware; 2.94 mm / 0.39° board scatter |
| `armpi_moveit/real_camera_forward_20261117/` | `camera_forward` plan-only check from the live pose | Real hardware, plan-only (not executed) |
| `phase2/joint_limits_20261117/` | Read-only bus-servo positions / torque / battery | Real hardware, read-only |
| `phase2/frames_20261117/` | Full-precision `link4 → rgb_camera_link`, body→optical composition | Real hardware, no motion |
| `phase2/real_background_20261117/` | 8 real RGB frames: negative control + semi-synthetic marker accuracy | Real camera frames; marker composited |
| `phase2/vendor_calibration_20261117/` | Static decoding of vendor `transform.yaml` / `hand2cam_tf_matrix` | Vendor config/source analysis |
| `studio_gateway/20261117/` | Persistent gateway live checks | Real hardware, read-only; execute disabled |
| `studio_e2e/` | Playwright browser tests | Mock mode |

## Model / mock / synthetic evidence

| Path | Content |
|---|---|
| `m2_2_v2/` | 2-DOF `simple_arm` controller study (MCAP analysis, snapshots, Gate 2) |
| `armpi_moveit/` | Model URDF, `check_urdf`, MoveIt smoke logs, Phase 0 plan-only results |
| `armpi_grasp/20261116/` | 5-DoF grasp candidate search (mock, plan-only) |
| `fiducial/20261116/` | Synthetic fiducial localization and ROS TF wiring |
| `armpi_moveit/phase0_20261116/` | Phase 0 constrained-planning acceptance |

## Rules

1. Do not edit raw logs/bags; add a new dated report instead.
2. `mcap` files and raw RGB frames are ignored or private; do not
   `git add -f` without a size/LFS/privacy decision.
3. Mock/Gazebo results must never be labelled hardware-validated.
4. Every claimed number needs a command, log or JSON in the matching folder.
5. Semi-synthetic results state that the target/marker was composited, not
   physically present.
