# Phase 2 — Calibration and Visual Grasping

> Phase 1 hardware baseline captured 2026-11-17; the real RGB/depth camera is
> available. Phase 2 calibration started, but the first hand-eye fit is **not
> validated**.

| Document | Purpose |
|---|---|
| [GRASP_CANDIDATE_SEARCH.md](GRASP_CANDIDATE_SEARCH.md) | 5-DoF constrained grasp-candidate generation, MoveIt screening, mock evidence, limitations |
| [FIDUCIAL_TARGET_LOCALIZATION.md](FIDUCIAL_TARGET_LOCALIZATION.md) | AprilTag/ArUco detection, 6D pose, TF to `base_link`, synthetic accuracy and ROS integration |
| [CALIBRATION.md](CALIBRATION.md) | Chessboard camera/hand-eye procedure, tools, first result and open-loop repeatability limit |

## State

| Item | State | Note |
|---|---|---|
| 5-DoF grasp candidate generation | **Implemented, mock-validated** | `armpi_grasp_planner`; Cartesian family + joint-space reachability sampler |
| IK → FK → collision → plan screening | **Implemented, mock-validated** | full-pose IK, FK verification, state validity, plan-only Pilz PTP |
| Fiducial target localization | **Implemented, synthetic-validated** | `armpi_fiducial_localizer`; real RGB/depth streams now available |
| Fiducial → grasp planner hand-off | **Documented, not integrated** | the planner accepts the same target point; no live bridge yet |
| Hand-eye calibration | **In progress — first baseline** | 8 chessboard poses; 13.4 mm / 2.06° board consistency; open-loop repeatability limit |
| Real visual grasping | **Not started** | no real grasp; calibration not validated |
| Grasp execution and verification | **Not started** | mock runs are plan-only by design |

Evidence:

- [`evidence/armpi_grasp/20261116/`](../../evidence/armpi_grasp/20261116/)
- [`evidence/fiducial/20261116/`](../../evidence/fiducial/20261116/)
- [`evidence/calibration/20261117/`](../../evidence/calibration/20261117/)

> The camera is real now, but the hand-eye result still has a 13.4 mm / 2.06°
> board residual and a ~101° body-vs-optical frame difference. Do not label
> this as visual grasping or as a validated calibration.
