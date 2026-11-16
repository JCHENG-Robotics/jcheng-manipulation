# Phase 2 — Calibration and Visual Grasping (preparation)

> **Hardware is not connected.** Phase 2 proper starts after the Phase 1
> hardware baseline. This directory covers the hardware-independent entry
> point that can be built and checked now.

| Document | Purpose |
|---|---|
| [GRASP_CANDIDATE_SEARCH.md](GRASP_CANDIDATE_SEARCH.md) | 5-DoF constrained grasp-candidate generation, MoveIt screening, mock evidence, limitations |
| [FIDUCIAL_TARGET_LOCALIZATION.md](FIDUCIAL_TARGET_LOCALIZATION.md) | AprilTag/ArUco detection, 6D pose, TF to `base_link`, synthetic accuracy and ROS integration |

## State

| Item | State | Note |
|---|---|---|
| 5-DoF grasp candidate generation | **Implemented, mock-validated** | `armpi_grasp_planner`; Cartesian family + joint-space reachability sampler |
| IK → FK → collision → plan screening | **Implemented, mock-validated** | full-pose IK, FK verification, state validity, plan-only Pilz PTP |
| Fiducial target localization | **Implemented, synthetic-validated** | `armpi_fiducial_localizer`; synthetic images and a synthetic ROS image stream, no real camera |
| Fiducial → grasp planner hand-off | **Documented, not integrated** | the planner accepts the same target point; no live bridge yet |
| Hand-eye calibration | **Not started** | requires hardware |
| Real visual grasping | **Not started** | requires hardware |
| Grasp execution and verification | **Not started** | mock runs are plan-only by design |

The 2026-11-16 evidence is in
[`evidence/armpi_grasp/20261116/`](../../evidence/armpi_grasp/20261116/) and
[`evidence/fiducial/20261116/`](../../evidence/fiducial/20261116/).

> A **synthetic** marker target is available and exercised through ROS. No real
> camera image, intrinsic or extrinsic exists yet. Do not label this as visual
> grasping.
