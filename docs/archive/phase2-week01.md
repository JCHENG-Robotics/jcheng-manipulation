# Archived: Panda Phase II Week 1 branch

> Status: **not merged, runtime regression not executed.** Preserved for
> traceability only. See the audit decision D-05 (option A: archive).

Branch: `origin/phase2/week01-execution-reliability`

| Commit | Message |
|---|---|
| `b73f804` | Fix pick place readiness and launch overrides |
| `a2c51bf` | Stabilize gripper MoveGroup execution |
| `fdca2e0` | Gate execution on controller action readiness |
| `e596c5a` | Document Phase II Week 1 reliability work |

- Base: `eb08b9b` (`v0.1.0`).
- Diff vs `main`: 10 files, 483 insertions, 161 deletions.
- Contents: pick/place pose parameters + launch overrides, seven-element
  validation, typed controller action-server readiness gate, plan/execute
  failure exits, `CHANGELOG.md`, `docs/evolution/phase2/week01.md`,
  `docs/learning-roadmap.md`, ROADMAP restructuring.
- Branch document states: "Remaining: Run default, planning-failure, and
  execution-failure regressions in a ROS 2 Jazzy + MoveIt 2 environment."

`origin/agent/JER-18-remediation` (`1bbb136`, 2 commits) is a code subset of this
branch and is also unmerged.

`main` contains a narrower 13-line failure-stop change (`616ac80`). The frozen
`panda-baseline-v1` tag points to `435f8ec` and does **not** include the branch
above. Do not cherry-pick or merge without a new Board decision and a runtime
regression.
