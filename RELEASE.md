# JCHENG Manipulation — release notes

## v1.0.0 (2026-11-23)

**Production `v1.0.0` baseline** for the JCHENG Robotics ecosystem. It
includes the feature freeze, the supervised full G8 acceptance, the
digest-pinned reproducible ROS build, the real safety node, the non-root
gateway services and the post-change G8 smoke. The platform still has the
hard safety limits listed below; this is not a safety certification.

### Tags

| Tag | Commit | Meaning |
|---|---|---|
| `v0.9.0-ff` | `30799df` (2026-11-21) | feature-freeze baseline |
| `v1.0.0-rc.1` | `d9b34b5` (2026-11-23) | release candidate |
| `v1.0.0` | this commit | production baseline with G8/build/ops acceptance |

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
| Studio backend | 148 collected; exit 0 |
| Studio gateway | 93 passed |
| `armpi_analysis` | 34 passed |
| safety + planner + fiducial | 59 passed |
| tools (phase2/3/4 + shared/parity/config) | 242 passed / 7 skipped |
| config consistency | `CONFIG OK`; 4 limit profiles |
| frontend Playwright | 10 mock + 3 token-mode dashboard passed; hardware/pose not rerun |
| CI | `studio` 45100626118 and `ros-colcon` 45098657645 passed |

Hardware evidence:

- `evidence/studio_validation_20261123/` — cache resync, `/safety_stop`
  latch/hold/409/clear, recovery, joint2 500→539→500, CameraCache/telemetry.
- `evidence/armpi_safety_install_20261123/` — real safety node install,
  status/reset/validate/FK.
- `evidence/fast_path_regression_20261123/` — 4.0/20 view→upright, settle
  2 pulses.
- `evidence/g8_smoke_20261123/` — post-8766/auth G8 smoke: 7 phases /
  10 bus-verified commands, one 15.6 mm base pulse, put-back locate valid.

### G8 acceptance included in v1.0.0 (2026-11-23)

A supervised **full G8 task sequence** passed before this release:

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
This is the **full G8 hardware acceptance** included in `v1.0.0`; the rc.1
tag predated it, and the v1.0.0 tag includes both.

### Reproducible build included in v1.0.0 (2026-11-23)

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

### Operations included in v1.0.0 (2026-11-23)

- [`docs/operations/PREFLIGHT.md`](docs/operations/PREFLIGHT.md) and
  [`RUNBOOK.md`](docs/operations/RUNBOOK.md) cover link/clock/gateway/safety
  checks, G8 runs, safety reset, stale-cache recovery and rollback.
- `tools/check_evidence_manifest.py` verifies that every evidence path in
  this manifest exists and has a README.
- The gateway installer now uses `User=pi` / `Group=docker` for the host
  wrapper; the live 8765 service was reinstalled on 2026-11-23 and verified
  (`evidence/gateway_service_20261123/`).
- `armpi-gateway-pose.service` wraps the 8766 gateway; the installer leaves
  it disabled and the operator starts it explicitly. It was verified with an
  explicit start (`evidence/gateway_pose_service_20261123/`).
- The backend gained an optional `ARMPI_BACKEND_TOKEN`; a non-loopback bind
  is refused at startup without it. The default remains localhost/no token.
  Live token/401/guard checks are in `evidence/backend_auth_20261123/`.
- A post-change G8 smoke passed on the new 8766 unit and auth code: 7 phases /
  10 bus-verified commands, one 15.6 mm base pulse, grip verify and post-run
  locate valid (`evidence/g8_smoke_20261123/`).

### Known limitations (accepted for v1.0.0)

- No e-stop, brake, self-collision model, independent encoder or odometry.
- The fast CLI bypasses `armpi_safety` **by design**; it is an explicit
  `profiles.fast` tier, not the MoveIt veto. The real safety node was not
  fault-injected (state-fault latch is covered by unit tests).
- Reproducible build: `ros-ci.yml` is digest-pinned, exact Python pins and
  the vendor source-tree manifest are in place, and the 2026-11-23 CI run
  pulled the digest, built 7 packages and passed all tests.
- The Studio backend defaults to localhost and no token. A non-loopback
  bind is refused at startup unless `ARMPI_BACKEND_TOKEN` is set; the token
  is embedded in the built frontend, so remote use needs TLS at a reverse
  proxy. Gateway token comparisons are constant-time. The 8765 host wrapper
  runs as `pi:docker`; the 8766 pose/motion unit is installed but **not
  enabled** and must be started explicitly.
- The full G8 acceptance and the post-change smoke passed before this tag;
  the fast-path regression is one direction only.
- The gripper is left open in the validation pose; no automatic boot-time
  homing.

`v1.0.0` includes the full G8 acceptance, digest-pinned colcon CI, non-root
8765 service, disabled-by-default 8766 unit, backend token/non-loopback guard
and the post-change G8 smoke. It is a production baseline, **not** a safety
certification; the physical limits above still apply.

### Upgrade / rollback

```bash
git checkout v1.0.0
# backend
cd armpi-debug-studio/backend
.venv/bin/pip install -e '.[dev]'
.venv/bin/pip install -e ../../packages/jcheng_core --no-deps
# gateway + safety overlay: re-deploy the current package to
# /home/pi/docker/tmp/armpi_gateway_pose and
# /home/pi/docker/tmp/armpi_gateway_deploy, and the safety overlay if changed
```

Rollback to `v1.0.0-rc.1` restores the candidate; rollback to `v0.9.0-ff`
restores the pre-freeze baseline. Re-deploy the robot-side gateway/safety
packages from the chosen tag as well.

### Release gate

```bash
scripts/release_check.sh
```

The gate runs the offline suites, the config-profiler check and the shared
package import; it refuses a dirty tree unless `RELEASE_ALLOW_DIRTY=1`.
