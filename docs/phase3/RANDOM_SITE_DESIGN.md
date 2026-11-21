# Random-placement 24-pair trial — pre-registered design

> Written **before** the first random-placement trial. Any change after data
> collection starts is recorded as a deviation at the bottom of this file.
>
> Pre-registered: 2026-11-21, on the ArmPi Ultra floor setup. Operator present;
> all motion supervised.
>
> **Supersedes, before any trial, the fixed 24-mark fan** in
> [`SITE_DIVERSITY_DESIGN.md`](SITE_DIVERSITY_DESIGN.md). The fixed-fan
> pre-registration (commit `1ee5217`) was never run and its evidence directory
> contains no trial records. The operator asked for the 24 positions to be
> random rather than a pre-planned grid; this file is the replacement
> pre-registration.

## 1. Question

The 2026-11-19 paired trial placed every pair on one fixture. The fixed-fan
follow-up would have placed one pair on each of 24 pre-marked sites. This
experiment instead tests the chain on **unknown placements**: the operator
puts the cube at a new, haphazard point inside a pre-registered region before
every pair, and the robot must locate it, drive to it and complete the
validated grasp/put-back.

1. **Capability.** At how many of the 24 random placements can the robot
   locate the cube (wrist view, joint1 yaw sweep or chassis front camera),
   bring it into the 0.226-0.234 m grasp band, and complete the pair?
2. **Method.** Under this random-placement regime, does the grasp-validated
   constant low-configuration correction `e = (+13.7, +3.9, +8.9) mm` still
   change the grasp outcome relative to the raw FK/hand-eye target?

The realized point, measured by the robot, is the recorded site. The operator
intent is not used in the analysis.

## 2. Region and placement rule

The region is the **sensor-covered fan from the home pose**, not a design
grid:

```text
radius   0.25-0.55 m
bearing  -20 ... +25 deg   (base +y is left)
```

Why these bounds:

- **Near half (radius <= ~0.46 m).** The validated view-pose wrist camera
  sees the floor band; a joint1 yaw sweep covers the wider bearings. The
  2026-11-19 out-of-view demo found a cube at 0.469 m, bearing 38.8 deg with
  a +0.28 rad yaw.
- **Far half (0.46-0.55 m).** The wrist image cuts a cube top near 0.46 m
  (the 2026-11-19 demo saw 0.469 m only at the upper image edge). The chassis
  D435i sees this far band from the home pose; its calibrated optical axis is
  at base y = +0.027 m, yaw -1.1 deg, which is why the bearing bounds are
  asymmetric. The far bearing limits are the calibrated far fan, not an
  arbitrary choice: a 0.55 m point at +25 deg sits about 29.5 deg from the
  front optical axis (inside the 35 deg half-field), and -20 deg sits about
  29.4 deg on the other side. A point outside this region has **no locator
  from the home pose**, so it would be a guaranteed capability failure
  rather than a measurement of the method.

**Placement rule (operator):** before every pair, place the cube at a new,
haphazard point inside the region. Do not reuse a previous spot, do not line
the points up, and keep at least **30 mm** from the previous realized point.
The operator must not aim at the image centre or the region boundary. The
operator's placement is the randomisation; the robot does not know the point
in advance.

The region, rule, 24 fixture labels (`r01`...`r24`) and the seeded pair/method
plan (seed **17**) are in
[`region.json`](../../evidence/phase3/onsite_20261121_random/region.json) and
[`paired_trials_plan.json`](../../evidence/phase3/onsite_20261121_random/paired_trials_plan.json).

## 3. Design

**24 matched pairs = 48 trials.** For pair `pNNN` the operator places the cube
at a fresh random point; both methods run back to back at that realized point:

| Method | Target sent to the joint fit |
|---|---|
| `baseline` | `C_view` — the strict wrist top-face centroid at the view pose |
| `intervention` | `C_view + (+13.7, +3.9, +8.9) mm` |

Randomization:

1. the pair order and the within-pair method order come from the seeded plan
   (seed 17);
2. the 24 placements are the operator's haphazard random draws.

The pair-to-fixture label is fixed (`p001` <-> `r01`, ..., `p024` <-> `r24`);
the label is only an identifier. The realized point is recorded.

## 4. Locator chain

For every sighting (the first one and after each base pulse):

1. **Wrist wide region.** The validated markerless segmenter at the view pose
   with the measured bus-servo registers, `x 0.15-0.60`, `|y| <= 0.40`,
   cluster `xy <= 0.12`. A top face rejected only for `partial_top_view` is
   accepted as an **approach** point, never as the final target.
2. **joint1 yaw sweep.** `+0.14, -0.14, +0.28, -0.28, +0.42, -0.42, +0.56,
   -0.56 rad`, measuring at each step. This is the 2026-11-19 out-of-view demo
   path; the arm is at the raised view pose, never the low grasp pose.
3. **Chassis D435i.** One colour + depth capture in the vendor container,
   copied to the laptop, then the pose-independent top-face measurement
   (`ext_cam_measure.py`: depth support plane, top plane, rectified colour,
   saturated blob). Its `point_base` is used **only as a base-drive target**;
   the front camera is not a grasp measurement. The 0.30-0.55 m far-field
   top-face rectification is the new, less-validated part; the pre-session
   check (Section 9) gates it.

If all three return no point, the pair aborts before any trial and is a
**capability failure**, recorded in `approaches.jsonl` with the reason.

The final grasp target always comes from the **strict wrist measurement in the
band** (placement gate 30-50 mm height and extents). The front camera can
never supply the grasp target.

## 5. Procedure per pair

1. The runner prints the pair and the region/rule. The operator places the
   cube at a fresh random point and presses Enter.
2. The arm moves to the validated view pose
   `[0, 0.2574, -1.18773, -1.70, -0.01676]`.
3. **Locate** with the chain above. The first measured point must be inside
   the region (10 mm / 1 deg margin); if the operator placed it outside, the
   runner asks for a re-place and re-locates (up to 3). A point outside after
   that is a capability failure, not a trial.
4. **Approach.** Closed-loop base pulses (vector duty 20) toward
   `(0.230, 0)`; after each pulse the full locator chain is re-run. Up to 5
   pulses; stop below 15 mm remaining planar travel.
5. **Final measurement and envelope guard.** The strict wrist measurement at
   `joint1 = 0`; the target fit must stay inside the prior 24/24 empirical
   envelope: measured `x` 0.215-0.240 m and `|joint1| <= 0.16 rad` (the
   2026-11-19 paired trial gripped 24/24 with the intervention over measured
   points x 0.2169-0.2388 m and |joint1| <= 0.142 rad across both methods).
   Up to 2 base trims; still outside means abort before any grasp.
6. **Trial A** (the plan's first method); the validated
   `low_config_grasp_validate.py` sequence, grip check in the 40 mm cube box,
   put-back and landing gates. The operator re-places the cube on the **same
   realized spot** for **Trial B**.
7. **Return home** by the reverse of the commanded translation with the same
   `duty * 4.9 mm/s` model that sized the outbound pulses; the speed scale
   cancels. The residual is absorbed by the next pair's locator chain. This is
   not a localization claim.

The runner writes the approach sidecar (`approaches.jsonl`) **and** the
validated paired-trial records. The final `stop` after a pair (including an
aborted one) always drives the reverse home pulse with the arm wherever the
abort left it; the operator keeps clear and the power switch reachable.

## 6. Outcomes

| Outcome | Definition | Role |
|---|---|---|
| Located | the locator chain returns a cube point before the first base pulse | capability (descriptive) |
| Region-valid | the first measured point is inside the pre-registered region | capability (descriptive) |
| Approached | the strict final measurement passes and the empirical envelope passes | capability (descriptive; a pair is attempted only then) |
| Grasp success | after the close and rise, the cube's 40 mm floor box is empty | **primary** (paired) |
| Placement error | for a held cube, the xy landing displacement from the pre-grasp measurement | secondary (paired, continuous) |
| Failure code | `MISS`, `NO_GRIP`, `AMBIGUOUS` | descriptive |
| Approach cost | locator used, initial radius/bearing, base pulses, final x/|joint1|, wall time | descriptive |

**Capability failure handling.** A point that cannot be located or approached
after one operator retry is a capability failure; the pair is not recorded as
a grasp trial, the reason is in `approaches.jsonl`, and the pair is excluded
from the paired grasp analysis with `n_complete` reported. It is never
silently dropped and never converted into a `MISS`.

The binary success is primary because a baseline miss has no release landing
to measure, exactly as in the 2026-11-19 design.

## 7. Sample size and analysis

24 pairs = 48 trials. With a baseline success near zero, the exact McNemar
test has two-sided p < 1e-5 for 17 or more intervention-only discordant pairs;
the 24 intervention trials estimate the success rate to about +/-10 % (95 %
Wilson). The 24 placements also give a capability rate with a descriptive
+/-10 % Wilson interval.

Analysis:

- `armpi_analysis.paired_trials analyze` on the completed pairs: exact McNemar;
  Wilson intervals; paired continuous difference with a seeded bootstrap 95 %
  interval, exact sign test, Cohen's `dz`; order split.
- `tools/phase3/site_report.py`: the capability table (pairs, sidecar
  attempts, retries, located/approached, locator, pulse count, realized
  radius/bearing, final envelope, abort reason).
- The realized placement distribution (radius and bearing of the first
  measured points) is reported descriptively. It is not a seeded sample and
  is not claimed to be uniform; the operator's draws are the randomisation.
- **No pooling** with the 2026-11-19 single-fixture trial or with the
  superseded fixed-fan design. The three are separate experiments.

## 8. Safety

- Supervised session; power switch within reach; no e-stop or brake; the arm
  falls if power is cut.
- The arm stays at the view pose during normal base pulses. If a trial aborts
  mid-motion, the runner's `finally` still drives the reverse home pulse with
  the arm wherever the abort left it; that pulse is a horizontal translation
  away from the cube; the operator keeps clear.
- The base moves toward the cube, not the operator; the operator stays behind
  or at the side.
- Adopted EEPROM-bounded joint limits; base vector duty <= 20; a pulse <= 3 s;
  the approach <= 5 pulses; the return <= 3 s.
- The front camera only observes; it is never a grasp target.
- Abort on any trajectory failure, action rejection, implausible pulse read,
  lost cube after a pulse, failed final measurement, or envelope violation.
- A battery swap or power loss is resumable: the pending pair is re-located
  from the current base pose; a pair aborted before its return still gets the
  finally-return, so the next locate starts near home.

## 9. Pre-session checks (required)

1. **Front-camera far field.** Place the cube at (0.55, 0) and at the far
   corners (0.55, -20 deg) and (0.55, +25 deg). With `--precheck`, confirm the
   front-camera locator returns a point and the region check passes. If the
   far corners fail, **do not start** the 24-pair session: the far region has
   no locator; record the failure and revise the region before data (a
   pre-data design change, Section 11).
2. **Wrist far edge.** Place the cube at (0.45, 0) and at (0.30, +/-20 deg);
   confirm the wrist wide measurement or the yaw sweep finds them at the view
   pose.
3. **Base direction.** One short forward and one short left pulse in the
   cleared workspace; confirm the wrist-measured point shifts in the expected
   direction; return with the reverse.
4. **Cube and floor.** The cube is flat (39.3-39.6 mm) and is the only
   saturated object within 0.70 m; no tape marks or other saturated clutter.
5. **Robot.** Vendor stack, wrist camera, D435i depth, Humble container,
   SSH key and battery voltage recorded; the base origin on the home mark and
   the forward axis on the taped line.

The `--precheck` mode runs checks 1-2 without moving the base and writes the
front-camera measurement JSONs to `front/`.

## 10. Threats to validity and limits

- **The randomisation is the operator, not a random-number generator.** The
  24 realized points are recorded, but the sample is not reproducible and the
  operator can unconsciously avoid hard spots. The placement rule (new spot,
  no lines, >= 30 mm) is enforced by instruction, not by the robot; the
  realized distribution is reported.
- **The front camera's far-field top-face measurement is not fully
  validated.** It is used only as a drive target; a bias of a few millimetres
  at 0.5 m is tolerable because the closed loop re-measures and the wrist
  takes over. The pre-session check gates the far field before the 48-trial
  budget is spent.
- **The home return is open-loop.** The model-cancelled reverse cancels the
  unknown speed scale, not wheel slip or yaw. The next locate is relative and
  absorbs it; no global-pose claim is made.
- **The region is the sensor envelope, not the whole floor.** A random point
  outside it would be unlocatable from home; the placement rule keeps the
  draws inside.
- **One cube, one operator, one session, one battery state.** The cube is the
  same 39.4 mm wooden cube; the lighting and operator are fixed.
- **Site count vs power.** The 24 placements are for coverage, not a power
  calculation; a site-level effect is described, not tested.

## 11. Deviations

None: no random-placement trial has been run yet. A **pre-data** change (for
example a region revision forced by the pre-session front-camera check) is not
a deviation: it regenerates the region and plan, updates the evidence hashes
and records the reason in the evidence README, still before the first valid
record. After the first valid record, any change is listed here
chronologically.

The fixed 24-mark fan in `SITE_DIVERSITY_DESIGN.md` was superseded by this
file before data; its evidence directory has no trials.

## 12. Artifacts and commands

- Region: `tools/phase3/random_region.py`; the pre-registered region/rule in
  `evidence/phase3/onsite_20261121_random/region.json`.
- Runner: `tools/phase3/random_site_session.py` (subclasses the validated
  `paired_trial_session.Session`; the grasp/grip/landing/recorder path is
  unchanged).
- Plan: `evidence/phase3/onsite_20261121_random/paired_trials_plan.json`
  (seed 17, fixtures r01...r24, generated before data).
- Trials: `evidence/phase3/onsite_20261121_random/trials.jsonl`;
  sidecar: `approaches.jsonl`; front-camera measurement JSONs: `front/`.
- Session log: `evidence/phase3/onsite_20261121_random/session_*.log`.

Precheck (no base motion, no trial; use the operator's actual far points)::

    sg docker -c "python3 tools/phase3/random_site_session.py --precheck \
        --region evidence/phase3/onsite_20261121_random/region.json \
        --plan evidence/phase3/onsite_20261121_random/paired_trials_plan.json \
        --trials evidence/phase3/onsite_20261121_random/trials.jsonl"

On-site::

    sg docker -c "python3 tools/phase3/random_site_session.py \
        --region evidence/phase3/onsite_20261121_random/region.json \
        --plan evidence/phase3/onsite_20261121_random/paired_trials_plan.json \
        --trials evidence/phase3/onsite_20261121_random/trials.jsonl" \
        2>&1 | tee evidence/phase3/onsite_20261121_random/session_$(date +%Y%m%d_%H%M).log

After the session::

    PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m \
        armpi_analysis.paired_trials analyze \
        --input evidence/phase3/onsite_20261121_random/trials.jsonl \
        --json-out evidence/phase3/onsite_20261121_random/analysis.json \
        --markdown-out evidence/phase3/onsite_20261121_random/analysis.md

    python3 tools/phase3/site_report.py \
        --approaches evidence/phase3/onsite_20261121_random/approaches.jsonl \
        --trials evidence/phase3/onsite_20261121_random/trials.jsonl \
        --markdown-out evidence/phase3/onsite_20261121_random/site_report.md
