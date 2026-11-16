# Phase 2 — Calibration and Visual Grasping (preparation)

> **Hardware is not connected.** Phase 2 proper starts after the Phase 1
> hardware baseline. This directory covers the hardware-independent entry
> point that can be built and checked now.

| Document | Purpose |
|---|---|
| [GRASP_CANDIDATE_SEARCH.md](GRASP_CANDIDATE_SEARCH.md) | 5-DoF constrained grasp-candidate generation, MoveIt screening, mock evidence, limitations |

## State

| Item | State | Note |
|---|---|---|
| 5-DoF grasp candidate generation | **Implemented, mock-validated** | `armpi_grasp_planner`; Cartesian family + joint-space reachability sampler |
| IK → FK → collision → plan screening | **Implemented, mock-validated** | full-pose IK, FK verification, state validity, plan-only Pilz PTP |
| Fiducial target localization | **Not started** | requires the real camera and measured `link4 → camera` extrinsic |
| Hand-eye calibration | **Not started** | requires hardware |
| Real visual grasping | **Not started** | requires hardware |
| Grasp execution and verification | **Not started** | mock runs are plan-only by design |

The 2026-11-16 evidence is in
[`evidence/armpi_grasp/20261116/`](../../evidence/armpi_grasp/20261116/).

> A camera target pose is **not** available yet. The joint sampler is therefore
> validated against a point taken from the robot model, not from a real
> perception pipeline. Do not label this as visual grasping.
