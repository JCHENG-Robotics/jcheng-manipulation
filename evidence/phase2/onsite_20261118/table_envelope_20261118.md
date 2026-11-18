# Table envelope and collision object — 2026-11-18

Status: **fixed and mock-verified.** No hardware motion; the live
`armpi-humble` stack was not restarted or touched. This closes leftover issue 2
from the 2026-11-18 onsite session ("safety workspace lower z is 0.00, the
table is at −0.078, so a table grasp is vetoed").

## Origin

`armpi_safety/config/safety_limits.yaml` had:

```yaml
workspace:
  lower: [-0.05, -0.30, 0.00]
```

and `docs/phase2/SAFETY_LAYER.md` recorded the consequence: a table-surface
grasp target is outside the envelope and is vetoed. The leftover asked for the
table to become a collision object or for the envelope to be re-reviewed.

## Why a flat z bound is not enough

The 2026-11-18 depth fit gives a table plane at **z ≈ −0.078 m** with a
**3.66°** tilt and normal
`[0.010305, −0.063014, 0.997959]`. Across the `y = ±0.30 m` workspace the table
surface spans about **−0.096 … −0.063 m** (33 mm). Lowering the flat z bound
until a grasp is allowed would therefore be too low on the high side and too
high on the low side, and it would give no geometry guard at all.

A random FK search (**120,000** samples over the adopted limits, `x` in
`[−0.04, 0.42]`, `|y| < 0.28`) found a lowest TCP z of **−0.0445 m**, about
**33 mm above the table**. The TCP cannot reach the table surface, so the dead
band `0.00 … −0.0445 m` was blocking real table work while the surface itself
was never reachable.

## Fix

1. **Calibrated table plane in the safety policy.** `SafetyConfig` now has an
   optional `table_plane` section. The plane equation is
   `normal . p + offset = 0`; the signed height above the table is
   `normal . p + offset`. The normal must point up and the frame must match the
   workspace frame. `SafetyMonitor.validate_trajectory` computes the TCP height
   at every waypoint and rejects a value below `clearance_m` with
   `TABLE_CLEARANCE` (fail closed when FK is unavailable). The report now
   includes `table_checked` and the plane that was used.
2. **Coarse hard floor.** `workspace.lower[2]` moved from `0.00` to **−0.10**.
   This is only a gross floor below the table, not the table guard; the plane
   check is the operative one. The config comment says so explicitly.
3. **Table collision object.** New
   `tools/phase2/table_collision.py` publishes the measured plane as an
   oriented box (`table_surface`, 1.0 × 1.0 × 0.05 m, top face on the plane)
   to `/collision_object`, republishing at 1 Hz so a MoveIt restart picks it up;
   `--remove` clears it. This is the geometry guard for link3–5 and the
   gripper, which the TCP-only safety check cannot see.

## Verification

Unit tests:

```text
ros2_ws/src/armpi_safety:  23 passed (policy + flake8 + pep257)
tools/phase2/tests:        33 passed, 1 skipped (8 new geometry tests)
```

Service test: isolated Jazzy mock on `ROS_DOMAIN_ID=42`, three safety nodes
with the same 2-point trajectory. The trajectory's TCP is at
`z = −0.0317 m`, i.e. **47.9 mm above the measured table plane**, and the
workspace box uses the new floor:

| Config | Result |
|---|---|
| new (`table_plane.enabled: true`, clearance 0.01, z floor −0.10) | `approved=True`, `code=OK`, `table_checked=True` |
| old (reconstructed: z floor 0.00, table off) | `approved=False`, `code=WORKSPACE` — reproduces the leftover |
| strict test (clearance 0.06) | `approved=False`, `code=TABLE_CLEARANCE`, height 0.0476–0.0479 m |

The strict row is a deliberately tightened copy used only to exercise the
rejection path at a reachable height; the shipped clearance is 0.01 m.

Planning-scene check after running `table_collision.py` against the same mock:

```text
objects: 1
id table_surface frame base_link dims [1.0, 1.0, 0.05]
object pose center [-0.001054, 0.006445, -0.102074]
object pose quat   [0.031523, 0.005155, 0.0, 0.99949]   (~3.6 deg tilt)
```

The first `--once` design published a single message and the object was not
observed in the scene; `--once` now republishes at 2 Hz for `--timeout`
seconds and was verified to add `table_surface`. For planning/execution, keep
the continuous 1 Hz publisher alive rather than relying on a bounded run.

Reproduction (isolated domain, no hardware):

```bash
export ROS_DOMAIN_ID=42
python3 tools/phase2/table_collision.py \
  --plane-json evidence/phase2/onsite_20261118/depth_plane_optical_20261118.json
ros2 run armpi_safety safety_supervisor --ros-args \
  -p config_file:=$PWD/ros2_ws/src/armpi_safety/config/safety_limits.yaml \
  -p joint_state_topic:=/test_joint_states
# publish a fresh /test_joint_states at 10 Hz, then call
# /armpi_safety/validate_trajectory with the trajectory above
```

## Operating requirements

- The live `armpi_safety` node in the `armpi-humble` container was started
  before this change and still runs the old config. **Restart it** before any
  table work; the source YAML and the installed share are symlinked, so no
  rebuild is needed.
- Run `table_collision.py` before the grasp search and keep it alive for the
  execute path. Without it the safety plane check still protects the TCP, but
  the elbow/wrist/gripper are not protected from the table.
- The plane is valid only for the base pose at which it was measured. Do not
  move the mecanum chassis and then reuse the constant; re-measure or
  re-localize the table.
- The safety check is TCP-only. The table collision object is the geometry
  guard, and MoveIt's plan is the only self/environment collision check.
- No hardware validation yet: the real safety node restart, the real scene
  object and a supervised table approach remain to be done with the operator.
