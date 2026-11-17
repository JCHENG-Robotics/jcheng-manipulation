# Phase 2 — Calibration and Visual Grasping

> Phase 1 hardware baseline captured 2026-11-17; the real RGB/depth camera is
> available. Phase 2 hand-eye calibration is **validated** (2.94 mm / 0.39°
> board scatter, 3.29 mm / 0.43° held-out); visual grasping integration is next.

| Document | Purpose |
|---|---|
| [GRASP_CANDIDATE_SEARCH.md](GRASP_CANDIDATE_SEARCH.md) | 5-DoF constrained grasp-candidate generation, MoveIt screening, mock evidence, limitations |
| [FIDUCIAL_TARGET_LOCALIZATION.md](FIDUCIAL_TARGET_LOCALIZATION.md) | AprilTag/ArUco detection, 6D pose, TF to `base_link`, synthetic accuracy and ROS integration |
| [CALIBRATION.md](CALIBRATION.md) | Chessboard camera/hand-eye procedure, tools, validated result and open-loop limitations |

## State

| Item | State | Note |
|---|---|---|
| 5-DoF grasp candidate generation | **Implemented, mock-validated** | `armpi_grasp_planner`; Cartesian family + joint-space reachability sampler |
| IK → FK → collision → plan screening | **Implemented, mock-validated** | full-pose IK, FK verification, state validity, plan-only Pilz PTP |
| Fiducial target localization | **Implemented, synthetic-validated** | `armpi_fiducial_localizer`; real RGB/depth streams now available |
| Fiducial → grasp planner hand-off | **Documented, not integrated** | the planner accepts the same target point; no live bridge yet |
| Hand-eye calibration | **Validated (2026-11-17)** | 14 poses with measured bus-servo joints; 2.94 mm / 0.39° board scatter; held-out 3.29 mm / 0.43° |
| Real visual grasping | **Not started** | no real grasp; calibration validated, integration pending |
| Grasp execution and verification | **Not started** | mock runs are plan-only by design |

Evidence:

- [`evidence/armpi_grasp/20261116/`](../../evidence/armpi_grasp/20261116/)
- [`evidence/fiducial/20261116/`](../../evidence/fiducial/20261116/)
- [`evidence/calibration/20261117/`](../../evidence/calibration/20261117/)

> The hand-eye transform is validated against the 5 mm / 2° board gate, a
> held-out final base and 2-fold cross-validation. The ~89° static-TF
> difference is the expected body/optical frame convention, not an error.
> Visual grasping is still not started: the transform is not yet wired into the
> fiducial/grasp pipeline.
