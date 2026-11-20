# Grasp execution, gripper placement and fixture pose — 2026-11-20

> Status: **measurement session complete.** 20 valid pick–lift–put-back
> records (two further put-backs were re-run because their landings were
> clipped) and 20 clean fixture placements. The three error-budget sources that were still
> unmeasured on 2026-11-19 are now measured. The component budget is
> **COMPLETE** but **fails** the 6 mm / 2° bound gates (translation RSS
> **13.325 mm**, worst case **24.014 mm**; rotation RSS **1.858°**, worst
> **3.322°**). The empirical placement mean is **3.080 mm**, inside the 6 mm
> tolerance; its tail (max **11.514 mm**) is not.

## 1. Setup

- HiWonder ArmPi Ultra, the same 39.4 mm cube, the validated view pose
  `[0, 0.2574, -1.18773, -1.70, -0.01676]`.
- New fixture: a **radial yellow tape strip** on the floor, no x-direction
  stop. The operator placed the cube on it by hand; the tape is the fixture.
- The grasp family is the validated deep family (joint2 −0.6618, joint3
  −1.8012, joint5 0) with the constant correction `(+13.7, +3.9, +8.9) mm`.
- The initial placement was at x = 0.262 m; the fit put the TCP **23.4 mm**
  above the corrected target, so it was refused before any motion. The
  accepted envelope is x 0.215–0.240 m with a fitted z residual below 10 mm
  and a complete top-face view; the physical gate is the z residual, not the
  window.
- Instruments: the same RGB-D segmenter as the paired trial (real frames,
  hand–eye `evidence/calibration/20261118/table_down/`), the bus-servo
  `get_state` service for the true joint registers, and the KDL model for
  forward kinematics.

## 2. Object-fixture pose (20 placements)

The operator lifted the cube and put it back 20 times; the top-face point was
measured at the fixed view pose after every stable placement. Three further
placements landed off the tape (|y − median| > 8 mm) and are excluded from the
estimate; they are kept in `discarded_fixture_off_tape.jsonl`. The raw
capture (`fixture.jsonl`, 18 records) and the clean set (`fixture_clean.jsonl`,
20 records) are both retained.

| Quantity | Value |
|---|---:|
| Clean placements | 20 |
| Radial RMS about mean | **5.057 mm** |
| Radial max | **14.815 mm** |
| Mean position | (+0.2321, −0.0146, −0.0501) m |
| Per-axis SD | (5.112, 0.884, 0.362) mm |
| Per-axis range | (22.990, 3.783, 1.076) mm |

The tape is a line, not a stop: the x scatter (5.1 mm SD) is the operator
placing the cube at different distances along the strip, while y and z are
stable (0.9 mm and 0.4 mm SD). This is the fixture's real repeatability, not
a jig tolerance.

## 3. Grasp execution (20 grasps)

Execution compares the **commanded** joint vector at the grasp pose with the
bus-servo registers read back at that same pose, mapped through the same KDL
model. The achieved vector is reconstructed from pulse differences, so a
constant servo offset cancels.

| Metric | Mean | SD | Max |
|---|---:|---:|---:|
| Translation | **4.220 mm** | 0.628 mm | **5.386 mm** |
| Rotation | **1.069°** | 0.207° | **1.495°** |

The translation term is systematic, not noise: joint1 and joint2 settle about
4 pulses (17 mrad / 17 mrad) from the command, which at the 0.23 m lever is
~4 mm. The empirical constant correction was fitted with this offset present,
so the component RSS and the corrected task outcome answer different
questions (see §5).

## 4. Gripper placement (20 put-backs)

Placement is the segmented landing point minus the pre-grasp measurement, in
xy. All 20 grasps held and were put back.

| Metric | Value |
|---|---:|
| Successful placements | 20 |
| Mean radial displacement | **3.080 mm** |
| SD | 2.748 mm |
| RMS | 4.082 mm |
| Max | **11.514 mm** |
| Mean xy vector | (−0.987, +1.634) mm |

Per trial:

| # | exec (mm) | exec (deg) | placement (mm) | placement vector (mm) |
|---:|---:|---:|---:|---|
| 1 | 3.977 | 1.109 | 6.273 | (+0.51, +6.25) |
| 2 | 4.061 | 0.882 | 2.853 | (+0.23, +2.84) |
| 3 | 3.746 | 0.904 | 0.928 | (+0.18, +0.91) |
| 4 | 4.754 | 1.327 | 0.503 | (+0.32, +0.39) |
| 5 | 4.432 | 1.061 | 0.504 | (−0.40, +0.31) |
| 6 | 4.743 | 1.200 | 2.824 | (−2.82, +0.19) |
| 7 | 5.386 | 1.495 | 0.283 | (−0.11, −0.26) |
| 8 | 5.014 | 1.335 | 3.135 | (−3.14, +0.00) |
| 9 | 4.294 | 1.039 | 3.269 | (−3.26, +0.24) |
| 10 | 5.044 | 1.388 | 6.696 | (−0.16, +6.69) |
| 11 | 3.361 | 0.710 | 3.034 | (−0.11, +3.03) |
| 12 | 4.332 | 0.978 | 11.514 | (−11.00, −3.39) |
| 13 | 4.412 | 1.105 | 3.955 | (+0.65, +3.90) |
| 14 | 4.030 | 0.890 | 0.736 | (−0.45, −0.58) |
| 15 | 4.509 | 1.144 | 4.629 | (+1.68, +4.31) |
| 16 | 3.734 | 0.962 | 1.636 | (−1.57, +0.45) |
| 17 | 4.605 | 1.191 | 1.581 | (−0.48, +1.50) |
| 18 | 3.014 | 0.837 | 1.775 | (−0.85, +1.56) |
| 19 | 3.315 | 0.988 | 0.702 | (−0.69, −0.13) |
| 20 | 3.634 | 0.842 | 4.776 | (+1.72, +4.46) |

Two patterns matter:

- **Trial 12 is a tail event**: one put-back shifted the cube 11.0 mm in −x
  and 3.4 mm in −y. The grip check and the landing gates both passed, so it
  is a valid placement, not a miss. It alone sets the 11.514 mm bound.
- **Human re-placement changes the release bias.** The trials immediately
  after an operator re-placement (1, 10, 15, 20) all shifted the cube +4.3 to
  +6.7 mm in y; the automatic put-backs in between were smaller. The
  segmenter does not measure the cube's yaw, so this is an unmodelled
  orientation effect, not a pure gripper bias.

## 5. Error budget

The component aggregates are in
[`error_budget.md`](error_budget.md) (pipeline) and
[`error_budget_fixture.md`](error_budget_fixture.md) (fixture-inclusive).
Configurations:
[`error_budget_onsite_20261120.json`](../../../ros2_ws/src/armpi_analysis/config/error_budget_onsite_20261120.json)
and
[`error_budget_fixture_20261120.json`](../../../ros2_ws/src/armpi_analysis/config/error_budget_fixture_20261120.json).

| Aggregate | Translation RSS | Translation worst | Rotation RSS | Rotation worst |
|---|---:|---:|---:|---:|
| Pipeline (no fixture term) | **13.325 mm** (fail) | **24.014 mm** (fail) | **1.858°** (pass) | **3.322°** (fail) |
| Fixture-inclusive | **19.926 mm** (fail) | **38.829 mm** (fail) | 1.858° (pass) | 3.322° (fail) |

The dominant translation source is `gripper_placement` (74.7 % of the
pipeline RSS), followed by `grasp_execution` (16.3 %); the fixture-inclusive
aggregate is dominated by `object_fixture_pose`. The dominant rotation source
is `grasp_execution_rotation` (64.7 %).

The two statements that must be read together:

- **Bound-level component budget fails.** Every source is a measured maximum
  treated as a bound; their RSS (13.3 mm) and linear worst case (24.0 mm)
  both exceed the measured 6 mm gripper tolerance.
- **Empirical task-level placement still fits on average.** The 20 put-backs
  mean 3.080 mm (RMS 4.082 mm), below 6 mm; the paired trial's 24 corrected
  placements mean 3.31 mm. The tail (max 11.5 mm) is what pushes the bound
  budget over.

The executor offset is systematic and the constant correction was fitted with
it present, so the component RSS is not additive to the corrected task error.
The task-level placement is the empirical check; the budget is the
conservative envelope.

## 6. Deviations

| Code | What happened | Handling |
|---|---|---|
| D1 | Initial cube placement at x = 0.262 m | Refused before motion: fitted TCP 23.4 mm above target. Re-placed. |
| D2 | Three fixture placements off the tape (y off by 12, 43 and 54 mm) | Excluded from the fixture estimate; kept in `discarded_fixture_off_tape.jsonl`. |
| D3 | Cube drifted −x during the trial series and left the camera view at trials 15 and 20 | Both landings clipped (30.8 and 34.5 mm visible extent); records rejected and kept as `discarded_trial15_landing.json` / `discarded_trial20_landing.json`. The trials were re-run after the operator recentred the cube. |
| D4 | Trial 12 shifted the cube 11.5 mm | Kept as a valid placement and reported as the tail. |
| D5 | The first fixture run waited 240 s for a second placement and timed out after one record | No motion; the series was restarted and resumed. |
| D6 | Operator recentred the cube four times (before trial 10, 15, 20 and the initial placement) | Each trial's placement is measured relative to its own pre-grasp point, so recentring does not bias the placement value. |

## 7. Files

| File | Contents |
|---|---|
| `fixture.jsonl` | First fixture capture, 18 placements (includes 3 off-tape) |
| `fixture_clean.jsonl` | 20 clean placements used for the estimate: the 15 kept from the first capture plus 5 appended after the split |
| `discarded_fixture_off_tape.jsonl` | 3 excluded placements with the rule |
| `fixture_report.json` / `.md` | Fixture statistics |
| `trials.jsonl` | 20 trial records (execution + placement) |
| `trial_report.json` / `.md` | Execution and placement statistics |
| `discarded_trial15_landing.json`, `discarded_trial20_landing.json` | Clipped landing reports |
| `error_budget.json` / `.md` / `.svg` | Pipeline budget (no fixture term) |
| `error_budget_fixture.json` / `.md` / `.svg` | Fixture-inclusive budget |
| `*_run*.log` | Supervised run logs |

The measurement tool is
[`tools/phase3/execution_fixture_run.py`](../../../tools/phase3/execution_fixture_run.py)
(13 unit tests in `tools/phase3/tests/test_execution_fixture.py`).

## 8. Limits

- One session, one cube, one tape strip, one battery state; not a population
  estimate.
- All sources are maxima over one session and are treated as bounds. No
  distribution was fitted, so no 95 % interval is reported.
- The fixture estimate includes the segmenter's perception noise (already a
  separate budget source); the fixture term is a task-level bound.
- The fixture is a nuisance in this pipeline because every trial re-measures
  the cube at the view pose; the fixture-inclusive aggregate is reported for
  completeness, not as the pipeline's task error.
- Placement is a task-level landing displacement; it overlaps the execution
  term and the residual model correction. The components are not independent
  in the RSS sense.
- The cube's yaw is not measured. Human re-placement changes the release bias
  (+4 to +7 mm in y after a re-placement), which the current model does not
  capture.
- The trial series needed operator recentring because the put-backs drift the
  cube; this is a property of the repeated put-back protocol, not of a
  single grasp-and-place.
