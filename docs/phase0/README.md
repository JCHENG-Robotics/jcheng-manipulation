# Phase 0 — Model, Planning and Acceptance

> Current state: **Phase 0 complete for the simulation/mock scope.**
> No hardware was used in Phase 0. Phase 1/2 have since run on the real robot
> (hardware baseline 2026-11-17; formal five-joint survey, 3/5 first-series
> attempts and a 24-pair trial 12/24 → 24/24 on 2026-11-19; D435i front camera,
> dual-camera grasp, fast motion and the completed error budget on 2026-11-20);
> see [`PHASE0_ACCEPTANCE.md`](PHASE0_ACCEPTANCE.md) §1 for the current
> acceptance summary and the repository README for per-workstream status.

| Document | Purpose |
|---|---|
| [PHASE0_ACCEPTANCE.md](PHASE0_ACCEPTANCE.md) | Current capstone acceptance summary, Phase 0 checklist, evidence, known issues, gap vs the master plan |
| [ARMPI_5DOF_CONSTRAINTS.md](ARMPI_5DOF_CONSTRAINTS.md) | 5-DoF task-space limits, grasp policy, camera geometry |
| [JER-39-audit-and-plan.md](JER-39-audit-and-plan.md) | Historical: initial read-only audit. Its “plan files do not exist” / “no ArmPi assets” statements are **outdated** |
| [JER-59-sprint01-audit.md](JER-59-sprint01-audit.md) | Historical Sprint 01 audit |

Evidence lives in `evidence/armpi_moveit/` (2026-11-15 smoke test and
2026-11-16 Phase 0 acceptance run).

The historical documents are kept for traceability; do not use them as current
status. Use `PHASE0_ACCEPTANCE.md` and the repository README instead.
