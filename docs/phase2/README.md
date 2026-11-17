# Phase 2 — Calibration and Visual Grasping

> Phase 1 hardware baseline captured 2026-11-17; the real RGB/depth camera is
> available. Phase 2 hand-eye calibration is **validated** (2.94 mm / 0.39°
> board scatter, 3.29 mm / 0.43° held-out). The measured optical frame is
> wired into the localizer as `camera_optical_calibrated`. A 24-pose diverse
> set is selected offline for the next capture; visual grasping integration is
> next.

| Document | Purpose |
|---|---|
| [GRASP_CANDIDATE_SEARCH.md](GRASP_CANDIDATE_SEARCH.md) | 5-DoF constrained grasp-candidate generation, MoveIt screening, mock evidence, limitations |
| [FIDUCIAL_TARGET_LOCALIZATION.md](FIDUCIAL_TARGET_LOCALIZATION.md) | AprilTag/ArUco detection, 6D pose, measured optical frame, real-background semi-synthetic validation |
| [CALIBRATION.md](CALIBRATION.md) | Chessboard camera/hand-eye procedure, tools, validated result and open-loop limitations |
| [CALIBRATION_POSES.md](CALIBRATION_POSES.md) | Offline selection of 24 diverse calibration poses, diversity/coplanarity checks and on-site command |
| [../hardware/camera_frames.md](../hardware/camera_frames.md) | Body/optical frame contract and `camera_optical_calibrated` |

## State

| Item | State | Note |
|---|---|---|
| 5-DoF grasp candidate generation | **Implemented, mock-validated** | `armpi_grasp_planner`; Cartesian family + joint-space reachability sampler |
| IK → FK → collision → plan screening | **Implemented, mock-validated** | full-pose IK, FK verification, state validity, plan-only Pilz PTP |
| Fiducial target localization | **Implemented, synthetic + real-background semi-synthetic validated** | 15/15 semi-synthetic poses, 1.69 mm / 0.53° max; 0 false detections on 8 real frames; LM pose refinement |
| Camera optical frame | **Implemented, validated** | `camera_optical_calibrated` = measured body→optical adapter (89.798°, 5.4/10.1/−27.4 mm) |
| Fiducial → grasp planner hand-off | **Documented, not integrated** | the planner accepts the same target point; no live bridge yet |
| Hand-eye calibration | **Validated (2026-11-17)** | 14 poses with measured bus-servo joints; 2.94 mm / 0.39° board scatter; held-out 3.29 mm / 0.43°; leave-one-out 3.41 mm / 0.44° std |
| Calibration pose selection | **Implemented, offline-validated (2026-11-17)** | 24 poses from 14,176 evaluated poses / 3,093 feasible; diversity, rotation-excitation and coplanarity checks pass; not executed |
| Real visual grasping | **Not started** | no real grasp; calibration validated, integration pending |
| Grasp execution and verification | **Not started** | mock runs are plan-only by design |

Evidence:

- [`evidence/armpi_grasp/20261116/`](../../evidence/armpi_grasp/20261116/)
- [`evidence/fiducial/20261116/`](../../evidence/fiducial/20261116/)
- [`evidence/calibration/20261117/`](../../evidence/calibration/20261117/)
- [`evidence/calibration/20261117/pose_selection/`](../../evidence/calibration/20261117/pose_selection/)
- [`evidence/phase2/real_background_20261117/`](../../evidence/phase2/real_background_20261117/)
- [`evidence/phase2/frames_20261117/`](../../evidence/phase2/frames_20261117/)
- [`evidence/phase2/vendor_calibration_20261117/`](../../evidence/phase2/vendor_calibration_20261117/)

> The hand-eye transform is validated against the 5 mm / 2° board gate, a
> held-out final base and 2-fold cross-validation. The measured optical frame
> is now wired into the localizer. The next calibration capture will use the
> 24 selected poses instead of the hand-picked 15; that set is planned offline
> and has not moved the robot. Visual grasping is still not started because
> the fiducial → grasp bridge and a physical marker/target test are pending.
