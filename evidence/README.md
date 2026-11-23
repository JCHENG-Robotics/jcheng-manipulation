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
| `calibration/20261117/pose_selection/` | 24-pose selection report, slim pose list, diversity/coplanarity plot | Offline computation from validated hardware calibration; no motion |
| `armpi_moveit/real_camera_forward_20261117/` | `camera_forward` plan-only check from the live pose | Real hardware, plan-only (not executed) |
| `phase2/joint_limits_20261117/` | Read-only bus-servo positions / torque / battery | Real hardware, read-only |
| `phase2/frames_20261117/` | Full-precision `link4 → rgb_camera_link`, body→optical composition | Real hardware, no motion |
| `phase2/real_background_20261117/` | 8 real RGB frames: negative control + semi-synthetic marker accuracy | Real camera frames; marker composited |
| `phase2/vendor_calibration_20261117/` | Static decoding of vendor `transform.yaml` / `hand2cam_tf_matrix` | Vendor config/source analysis |
| `studio_gateway/20261117/` | Persistent gateway live checks | Real hardware, read-only; execute disabled |
| `studio_g5/20261118/` | G5 no-motion + supervised `camera_forward` recorder captures, G2 imports, live `/tf` vs `tf2_echo`, gated hardware Playwright | Real hardware; one supervised 0.892 s motion |
| `phase3/session4_20261117/no_camera/`, `with_camera/` | Host + ROS/DDS benchmark on laptop and robot; camera run started/stopped for measurement | Real hardware, read-only; no motion |
| `studio_validation_20261123/` | Cache resync, `/safety_stop` latch/hold/409/clear, recovery, joint2 round trip, CameraCache/telemetry | Real hardware; supervised motion |
| `armpi_safety_install_20261123/` | Real `armpi_safety` node install and status/reset/validate/FK checks | Real hardware, no motion |
| `fast_path_regression_20261123/` | 4.0/20 view→upright fast-path regression | Real hardware; supervised motion |
| `g8_acceptance_20261123/` | Full 8-phase G8 task: one bounded chassis pulse, 11 bus-verified commands, grip verify, put-back | Real hardware; supervised motion |
| `g8_smoke_20261123/` | Post-8766/backend-auth G8 smoke: one 15.6 mm base pulse, 7 phases / 10 bus-verified commands, put-back checked | Real hardware; supervised motion |
| `studio_e2e/` | Playwright browser tests | Mock mode |

## Build / reproducibility evidence

| Path | Content | Scope |
|---|---|---|
| `ros_colcon_20261123/` | `colcon build/test` for 7 public packages on aarch64 Humble; 118 tests, 0 failures, 1 local skip | Target architecture; no motion |

## Operations evidence

| Path | Content | Scope |
|---|---|---|
| `gateway_service_20261123/` | Read-only gateway unit switched from root to `pi:docker`; health checks after the restart | Real robot host; no motion |
| `gateway_pose_service_20261123/` | 8766 pose/motion unit installed but disabled; explicit start, health and pointcloud verified | Real robot host; no motion |
| `backend_auth_20261123/` | Optional backend token auth, non-loopback startup guard, test evidence | Laptop backend; no motion |

## Model / mock / synthetic evidence

| Path | Content |
|---|---|
| `m2_2_v2/` | 2-DOF `simple_arm` controller study (MCAP analysis, snapshots, Gate 2) |
| `armpi_moveit/` | Model URDF, `check_urdf`, MoveIt smoke logs, Phase 0 plan-only results |
| `armpi_grasp/20261116/` | 5-DoF grasp candidate search (mock, plan-only) |
| `fiducial/20261116/` | Synthetic fiducial localization and ROS TF wiring |
| `phase2/session3_20261117/` | Synthetic marker → localizer → grasp bridge → safety veto E2E, safety service negative controls, measurement dry-run/simulate reports |
| `phase3/session4_20261117/` | Partial error budget, paired-trial plan/simulation/analysis, host/DDS benchmark reports |
| `armpi_moveit/phase0_20261116/` | Phase 0 constrained-planning acceptance |

## Rules

1. Do not edit raw logs/bags; add a new dated report instead.
2. `mcap` files and raw RGB frames are ignored or private; do not
   `git add -f` without a size/LFS/privacy decision.
3. Mock/Gazebo results must never be labelled hardware-validated.
4. Every claimed number needs a command, log or JSON in the matching folder.
5. Semi-synthetic results state that the target/marker was composited, not
   physically present.
