# JCHENG Manipulation — release notes

## v1.0.0-rc.1 (2026-11-23)

**This is a release candidate, not production `v1.0.0`.** It is the
feature-frozen, hardware-validated-on-the-gateway baseline for the JCHENG
Robotics ecosystem. The full task-level acceptance and the reproducible ROS
build remain open (see known limitations).

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

### Known limitations (accepted for rc.1)

- No e-stop, brake, self-collision model, independent encoder or odometry.
- The fast CLI bypasses `armpi_safety` **by design**; it is an explicit
  `profiles.fast` tier, not the MoveIt veto. The real safety node was not
  fault-injected (state-fault latch is covered by unit tests).
- Reproducible build is not complete: no ROS `colcon build/test` CI, no
  Python/Docker/vendor workspace pinning, `>=` Python dependencies, and the
  vendor workspace is gitignored.
- The Studio backend has no authentication and is localhost-only by policy;
  the gateway token comparison is constant-time; the systemd wrapper runs the
  container entrypoint as root.
- The full G7/G8 task sequence was not rerun for this candidate; the fast-path
  regression is one direction only.
- The gripper is left open in the validation pose; no automatic boot-time
  homing.

`v1.0.0` requires: a supervised full G8 hardware acceptance, a reproducible
ROS CI build, and closure of the remaining P1 operational items. Do not
present rc.1 as a production safety certification.

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
