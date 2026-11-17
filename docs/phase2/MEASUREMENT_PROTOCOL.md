# Phase 3 Measurement Protocols

> Status: **Implemented and mock-validated (2026-11-17, Session 3).** The four
> protocols have dry-run schedules and synthetic analysis reports. No hardware
> measurement has been collected, so no repeatability, backlash, visual or
> gripper number here is a robot result.

## Why schedules before motions

The error budget needs four kinds of measured input:

1. arm repeatability at a target;
2. backlash/hysteresis between approach directions;
3. visual relative-displacement error against a known fixture;
4. the gripper's tolerated placement envelope.

All four require the robot, a person and (for some) a fixture. To avoid
inventing an execution path and accidentally moving hardware, the tool has
three explicit modes:

| Mode | Effect | ROS import? |
|---|---|---|
| `dry-run` (default) | writes the trial schedule and validates the target against provisional limits | no |
| `simulate` | generates samples with known noise/bias, runs the same analyzers as hardware data | no |
| `analyze` | reads a hardware measurement JSON and runs the analyzers | no |

The hardware runner is deliberately **not** part of this tool. A supervised
runner will execute the schedule only after the joint-limit survey, the safety
review and a physical support/stop check.

## Protocols

| Protocol | Schedule | Analysis | Pass gate in this version |
|---|---|---|---|
| `repeatability` | 30 visits to one 5-joint target | per-axis and radial spread, nominal error | radial RMS ≤ 5 mm (placeholder) |
| `backlash` | 30 visits alternating pre-approach direction on one joint | forward/reverse mean offset | hysteresis magnitude ≤ 2 mm (placeholder) |
| `visual_displacement` | ±`max_offset` grid in x/y, repeated | per-axis bias, RMSE, max error vs known displacement | max error ≤ 5 mm (placeholder) |
| `gripper_envelope` | x/y/z offset grid plus a yaw grid, repeated | success rate per offset/yaw; largest passing linear offset and yaw | success threshold 0.8 |

The pass gates are initial analysis thresholds, not yet the master-plan task
tolerance. They will be replaced by the measured task tolerance after the
gripper/object tests.

## Joint limits (separate physical tool)

Joint-limit measurement is not part of this analyzer because it is a
torque-off, hand-moved, interactive survey. It remains in
`tools/phase2/joint_limit_survey.py`:

```bash
# read-only: position + torque registers, no command
python3 tools/phase2/joint_limit_survey.py --mode read --servo-reads 5

# on-site only, with support: torque off, hand to both stops, torque restore
python3 tools/phase2/joint_limit_survey.py --mode survey \
  --i-have-support --servo-reads 5
```

The procedure, the CycloneDDS requirement and the adoption rules are in
[`docs/phase2/JOINT_LIMITS.md`](JOINT_LIMITS.md). There is no mock dry-run for
the torque-off survey; the read-only mode is the safe pre-check.

## Commands

```bash
# Schedule only - no motion, no ROS.
python3 tools/phase2/measurement_protocol.py repeatability \
  --mode dry-run --trials 30 --json-out /tmp/repeatability.json

# Mock analysis path: injected 0.3 mm noise and 1 mm bias.
python3 tools/phase2/measurement_protocol.py repeatability \
  --mode simulate --json-out /tmp/repeatability_sim.json

python3 tools/phase2/measurement_protocol.py backlash \
  --mode simulate --json-out /tmp/backlash_sim.json

python3 tools/phase2/measurement_protocol.py visual_displacement \
  --mode simulate --json-out /tmp/visual_sim.json

python3 tools/phase2/measurement_protocol.py gripper_envelope \
  --mode simulate --json-out /tmp/gripper_sim.json

# Analyze a hardware file later.
python3 tools/phase2/measurement_protocol.py repeatability \
  --mode analyze --input /path/to/measurement.json --json-out /tmp/analysis.json
```

The target defaults to the validated calibration base pose
`[0.0, 0.59888, -1.25664, -0.71209, 0.0]`; `--target-joints` changes it. The
tool rejects targets outside the provisional limits before writing a schedule.

## Data schema

A hardware measurement file is a JSON object with the protocol name and the raw
samples. The analyzers accept:

```json
{
  "protocol": "repeatability",
  "nominal": [0.20, 0.0, 0.10],
  "samples": [
    {"trial": 1, "position_m": [0.2001, 0.0002, 0.1000]},
    {"trial": 2, "position_m": [0.1998, -0.0001, 0.0999]}
  ]
}
```

For `backlash`, `samples` is `{"forward": [...], "reverse": [...]}`. For
`visual_displacement`, each sample has `nominal_displacement_m` and
`measured_displacement_m`. For `gripper_envelope`, each record has `offset_m`,
`trials` and `successes`.

## Definitions

- **Per-axis spread**: sample standard deviation of x/y/z positions, in mm.
- **Radial RMS**: `sqrt(mean(||p_i - mean(p)||²))`, in mm.
- **Radial max**: largest distance from the mean, in mm.
- **Hysteresis**: `mean(forward) - mean(reverse)`; magnitude is the norm.
- **Visual error**: `measured_displacement - nominal_displacement` in mm.
- **Gripper envelope**: for each axis, the largest `|offset|` on that axis with
  the other axes at zero whose success rate is at least the threshold; the
  angular envelope is the largest `|yaw|` at zero translation with the same
  success threshold.

## Mock results (2026-11-17)

Evidence: [`evidence/phase2/session3_20261117/`](../../evidence/phase2/session3_20261117/).
Command log: `measurement_protocol.txt`; reports: `measurement_*_dry_run.json`
and `measurement_*_simulate.json`.

| Protocol | Schedule | Synthetic analysis |
|---|---|---|
| repeatability | 30 trials | radial RMS 0.483 mm, radial max 0.919 mm, nominal-error mean 1.130 mm (1 mm injected x bias) |
| backlash | 30 trials | hysteresis 0.939 mm (1 mm injected) |
| visual displacement | 21 placements | RMSE 1.139 mm, max 1.876 mm |
| gripper envelope | 42 offsets (33 linear + 9 yaw) | 6.0 mm on x, y and z; 10.0° yaw (injected boundary) |

These numbers validate that the schedules and analyzers run end to end. They
are **not** measurements of the ArmPi.

## Limitations

- **No hardware data.** A dry-run schedule is not a repeatability result.
- **Joint-space backlash probe.** The current schedule pre-positions one joint
  (±0.1 rad) before returning to the target. That measures joint-space
  hysteresis, not a Cartesian approach-direction effect.
- **Synthetic noise is simple.** Gaussian noise does not model servo dynamics,
  compliance, temperature, gear backlash or camera lighting.
- **No automatic trial execution.** The operator/runner must record positions,
  pass/fail and any aborted trial. The analyzer cannot recover a missing sample.
- **Placeholder gates.** 5 mm/2 mm are analysis defaults, not the final task
  tolerance envelope.
- **Angular envelope is yaw-only.** The gripper schedule varies yaw about the
  approach axis; pitch/roll and object-shape effects are not covered.
- **No error-budget aggregation yet.** The reports are inputs to the Phase 3
  error budget; they are not the budget itself.

## Hardware prerequisites

1. Joint-limit survey complete and reviewed; `mapping_verified` decision made.
2. Safety layer reviewed; conservative velocity/acceleration and workspace
   limits confirmed; physical stop/power cut accessible.
3. For repeatability/backlash: a repeatable target and a way to read the
   endpoint (measured joints + FK, or an external marker).
4. For visual displacement: a fixture with known displacements (grid, linear
   guide or caliper jig) and a camera/marker setup.
5. For gripper envelope: a safe object, a defined grasp and a success criterion.
