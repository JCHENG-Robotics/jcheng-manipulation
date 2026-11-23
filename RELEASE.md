# JCHENG Manipulation — release notes

## v1.0.0-rc.1 (2026-11-23)

**This is a release candidate, not production `v1.0.0`.** It is the
feature-frozen, hardware-validated-on-the-gateway baseline for the JCHENG
Robotics ecosystem. The full task-level G8 acceptance passed **after** this
tag (2026-11-23, see below); the reproducible ROS build remains open (see
known limitations).

### Tags

| Tag | Commit | Meaning |
|---|---|---|
| `v0.9.0-ff` | `30799df` (2026-11-21) | feature-freeze baseline |
| `v1.0.0-rc.1` | this commit | release candidate, offline + gateway hardware validation |

### Supported path

- **Studio gateway G6 / G7 / G8** is the supported operator path; it uses the
  `profiles.step` / `profiles.pose` / `profiles.fast` tiers and bus-verified
  completion.
- **MoveIt CLI** uses `profiles.moveit` (1.2 rad/s / 0.5 rad/s²) through
  `armpi_safety`.
- `tools/phase2`/`phase3` runners are **research-only**; the fast CLI is an
  explicit `profiles.fast` path with bus settle and strict joint2, not a
  production surface.
- `armpi_safety` is installed in the persistent
  `/home/ubuntu/shared/safety_ws` overlay in the slim container; it is a
  **manual process**, not a boot service.

### Verification (2026-11-23)

| Suite | Result |
|---|---|
| Studio backend | 144 collected; exit 0 |
| Studio gateway | 93 passed |
| `armpi_analysis` | 34 passed |
| safety + planner + fiducial | 59 passed |
| tools (phase2/3/4 + shared/parity/config) | 242 passed / 7 skipped |
| config consistency | `CONFIG OK`; 4 limit profiles |
| frontend Playwright | not rerun in this release; CI retains 10 mock + 3 pose/task + 3 hardware |

Hardware evidence:

- `evidence/studio_validation_20261123/` — cache resync, `/safety_stop`
  latch/hold/409/clear, recovery, joint2 500→539→500, CameraCache/telemetry.
- `evidence/armpi_safety_install_20261123/` — real safety node install,
  status/reset/validate/FK.
- `evidence/fast_path_regression_20261123/` — 4.0/20 view→upright, settle
  2 pulses.

### Post-rc.1 acceptance update (2026-11-23)

A supervised **full G8 task sequence** passed after `v1.0.0-rc.1` was cut:

- cube at `0.304 m` → one bounded open-loop chassis pulse (`75.3 mm`
  nominal) → re-plan `VALID`;
- 8 phases / 11 arm and gripper commands, every terminal event `Completed`
  with `verification: bus` (gripper commands timed, final bus read back);
- grip verify `held: true` (empty cube box; the one segmented cluster was
  outside the box and rejected as `y_out_of_region`);
- put-back confirmed by a read-only re-plan at `0.236 m`, inside the
  validated `0.220–0.255 m` band;
- `/safety_stop=false`; the real safety node reported `valid=true`,
  `stop_latched=false`, no violations.

Evidence: [`evidence/g8_acceptance_20261123/`](evidence/g8_acceptance_20261123/README.md).
This closes the **full G8 hardware acceptance** part of the `v1.0.0` gate;
the rc.1 tag itself predates the run, so `v1.0.0` must include this evidence.

### Post-rc.1 reproducible build update (2026-11-23)

- `.github/workflows/ros-ci.yml` builds and tests the seven public ROS
  packages in a Humble container with an explicit dependency list.
- A target-architecture run in the `ArmPiUltra` container built 7 packages
  and ran **118 tests: 0 errors, 0 failures, 1 skipped** (the local `xmllint`
  schema fetch; CI has network). Evidence:
  [`evidence/ros_colcon_20261123/`](evidence/ros_colcon_20261123/README.md).
- `requirements/ci.txt` and the backend `pyproject.toml` pin Python tooling
  and application dependencies; `requires-python` is capped at `<3.13`.
- `docs/hardware/vendor_baseline_manifest.json` pins the private vendor
  source tree (SHA-256) without publishing it.
- The CI base image and the Dockerfile default are digest-pinned
  (`ros:humble-ros-base@sha256:dfc94c85…`); CI run 45098657645 pulled it,
  built the 7 packages and passed.

### Post-rc.1 operations update (2026-11-23)

- [`docs/operations/PREFLIGHT.md`](docs/operations/PREFLIGHT.md) and
  [`RUNBOOK.md`](docs/operations/RUNBOOK.md) cover link/clock/gateway/safety
  checks, G8 runs, safety reset, stale-cache recovery and rollback.
- `tools/check_evidence_manifest.py` verifies that every evidence path in
  this manifest exists and has a README.
- The gateway installer now uses `User=pi` / `Group=docker` for the host
  wrapper; the live 8765 service was reinstalled on 2026-11-23 and verified
  (`evidence/gateway_service_20261123/`).
- The backend remains localhost-only without authentication; use an SSH
  tunnel for remote access.

### Known limitations (accepted for rc.1)

- No e-stop, brake, self-collision model, independent encoder or odometry.
- The fast CLI bypasses `armpi_safety` **by design**; it is an explicit
  `profiles.fast` tier, not the MoveIt veto. The real safety node was not
  fault-injected (state-fault latch is covered by unit tests).
- Reproducible build: `ros-ci.yml` is digest-pinned, exact Python pins and
  the vendor source-tree manifest are in place, and the 2026-11-23 CI run
  pulled the digest, built 7 packages and passed all tests.
- The Studio backend has no authentication and is localhost-only by policy;
  use an SSH tunnel for remote access. The gateway token comparison is
  constant-time. The 8765 host wrapper was switched to `pi:docker` and
  verified on 2026-11-23; the 8766 pose/motion gateway remains a manual
  process.
- The rc.1 tag itself predates the full G8 acceptance; the post-tag run
  passed on 2026-11-23 (see above). The fast-path regression is one direction
  only.
- The gripper is left open in the validation pose; no automatic boot-time
  homing.

`v1.0.0` requires: closure or explicit acceptance of the remaining P1 items
(backend authentication policy and the manual 8766 service) and inclusion of
the 2026-11-23 G8/build/ops evidence. The colcon CI is digest-pinned and green
on both aarch64 and GitHub. Do not present rc.1 as a production safety
certification.

### Upgrade / rollback

```bash
git checkout v1.0.0-rc.1
# backend
cd armpi-debug-studio/backend
.venv/bin/pip install -e '.[dev]'
.venv/bin/pip install -e ../../packages/jcheng_core --no-deps
# gateway + safety overlay: re-deploy the current package to
# /home/pi/docker/tmp/armpi_gateway_pose and /home/pi/docker/tmp/safety_ws
```

Rollback to `v0.9.0-ff` restores the pre-rc freeze baseline; the robot-side
gateway/safety packages must be re-deployed from that tag as well.

### Release gate

```bash
scripts/release_check.sh
```

The gate runs the offline suites, the config-profiler check and the shared
package import; it refuses a dirty tree unless `RELEASE_ALLOW_DIRTY=1`.
