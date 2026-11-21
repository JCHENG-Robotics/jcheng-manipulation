# 24-site paired trial — pre-registered design

> **Superseded before data (2026-11-21).** The user replaced the fixed
> 24-mark fan with operator-random placements in a sensor-covered region.
> See [RANDOM_SITE_DESIGN.md](RANDOM_SITE_DESIGN.md). The fixed fan was never
> run; its evidence directory has no trial records.
>
> Written **before** the first 24-site trial. Any change after data collection
> starts is recorded as a deviation at the bottom of this file.
>
> Pre-registered: 2026-11-20, on the ArmPi Ultra floor-grasp setup. Operator
> present, all motion supervised. The 2026-11-19 single-fixture paired trial
> is a separate, completed experiment; this file does not re-analyse it.

## 1. Question

The 2026-11-19 paired trial placed every pair on the same floor fixture. Its
result (baseline 12/24 vs intervention 24/24, McNemar p = 0.00049) answers
whether the constant correction works **at that fixture**, not whether the
system can find the object at all when the object is elsewhere.

This experiment asks two questions:

1. **Capability.** Can the robot locate the same 39.4 mm cube at **24 distinct,
   pre-registered floor sites**, drive the base to each site until the cube is
   in the validated grasp band, and complete the validated grasp/put-back
   sequence?
2. **Method.** Under this site diversity, does the grasp-validated constant
   low-configuration correction `e = (+13.7, +3.9, +8.9) mm` still change the
   outcome of a floor-cube grasp, compared with the raw FK/hand-eye target?

The first question is descriptive and is answered per site. The second keeps
the matched-pair design and the exact McNemar test.

## 2. Design

**24 matched pairs = 48 trials, one floor site per pair.** For pair `pNNN`
the operator places the cube on the mark of site `sNNN`; both methods run back
to back at that site:

| Method | Target sent to the joint fit |
|---|---|
| `baseline` | `C_view` — the cube top-face centroid measured at the view pose |
| `intervention` | `C_view + (+13.7, +3.9, +8.9) mm` |

Randomization (seeded, reproducible; seed **17**):

1. the pair order (which site is visited first, ...);
2. the method order inside each pair.

The plan is generated once with `armpi_analysis.paired_trials.build_plan`,
written to
[`paired_trials_plan.json`](../../evidence/phase3/onsite_20261120_sites/paired_trials_plan.json),
and is append-only from then on. The pair-to-site assignment is fixed
(`p001` ↔ `s01`, ..., `p024` ↔ `s24`); the plan shuffles the visit order.

The site is the block: each site contributes exactly one pair, so the site is
not a repeated measure. Randomizing the method order inside the pair protects
the method comparison from within-pair time drift; the visit order does not
align the method with time.

## 3. Sites

The pre-registered layout is a fan read from the validated view pose
([`site_layout.json`](../../evidence/phase3/onsite_20261120_sites/site_layout.json);
operator sheet
[`site_layout.md`](../../evidence/phase3/onsite_20261120_sites/site_layout.md)):

| Site | Radius (m) | Bearing (deg) | Forward x (m) | Left y (m) |
|---|---:|---:|---:|---:|
| s01 | 0.30 | −10 | 0.295 | −0.052 |
| s02 | 0.30 | −4 | 0.299 | −0.021 |
| s03 | 0.30 | +2 | 0.300 | +0.010 |
| s04 | 0.30 | +8 | 0.297 | +0.042 |
| s05 | 0.30 | +14 | 0.291 | +0.073 |
| s06 | 0.30 | +20 | 0.282 | +0.103 |
| s07 | 0.35 | −10 | 0.345 | −0.061 |
| s08 | 0.35 | −4 | 0.349 | −0.024 |
| s09 | 0.35 | +2 | 0.350 | +0.012 |
| s10 | 0.35 | +8 | 0.347 | +0.049 |
| s11 | 0.35 | +14 | 0.340 | +0.085 |
| s12 | 0.35 | +20 | 0.329 | +0.120 |
| s13 | 0.40 | −10 | 0.394 | −0.069 |
| s14 | 0.40 | −4 | 0.399 | −0.028 |
| s15 | 0.40 | +2 | 0.400 | +0.014 |
| s16 | 0.40 | +8 | 0.396 | +0.056 |
| s17 | 0.40 | +14 | 0.388 | +0.097 |
| s18 | 0.40 | +20 | 0.376 | +0.137 |
| s19 | 0.45 | −10 | 0.443 | −0.078 |
| s20 | 0.45 | −4 | 0.449 | −0.031 |
| s21 | 0.45 | +2 | 0.450 | +0.016 |
| s22 | 0.45 | +8 | 0.446 | +0.063 |
| s23 | 0.45 | +14 | 0.437 | +0.109 |
| s24 | 0.45 | +20 | 0.423 | +0.154 |

Why this fan:

- **The wrist sees the nominal sites from the view pose.** The view-pose FK,
  the measured table-down hand-eye and the commanded view joints project every
  nominal top-face square inside the 640 x 400 wrist image; the tightest
  observed corner margin at 0.45 m is about 20 px (nominal projection, to be
  confirmed at s21 and s24 in the pre-session check). The robot can therefore
  measure the initial sighting without an arm yaw.
- **The front D435i also sees them.** The fan is centred on the front camera
  (mounted at `base_link` y = +0.027 m, yaw −1.1 deg), because its 35 deg
  horizontal half-field is asymmetric about the robot's forward axis. Every
  nominal top-face square keeps more than 30 px to the front-camera border.
  The front camera is **not** in the grasp loop in this experiment (its
  far-field top-face rectification is untested at the oblique 0.30-0.45 m
  views), but the layout leaves it available as a future independent check.
- **The radii stop at 0.45 m.** The wrist view image cuts a cube top near
  0.46 m; the prior out-of-view demo measured a cube at 0.469 m only at the
  upper image edge. 0.45 m keeps a margin.
- **Adjacent sites do not overlap badly.** At 0.30 m, adjacent 6 deg bearings
  are 31 mm apart and radii are 50 mm apart; the site marks are small dots,
  and only one cube is on the floor at a time.

The site coordinates are nominal. The operator places the cube on the mark;
the robot measures the **realized** initial point and records it. The
realized point is not claimed to equal the nominal point, because the base
return is not a localization system (Section 8).

## 4. Procedure per pair (automated, operator-gated)

1. The runner prints the pair, site id, radius, bearing and mark coordinates.
   The operator moves the single cube from the previous site to the prompted
   mark, centres it, and presses Enter.
2. The arm moves to the validated view pose
   `[0, 0.2574, −1.18773, −1.70, −0.01676]`.
3. **Locate.** The wrist camera segments the wide region
   (`x 0.15-0.60`, `|y| <= 0.40`, cluster `xy <= 0.12`) with the measured
   bus-servo registers. A valid top face is accepted directly; a top face the
   segmenter rejects only for `partial_top_view` is still accepted as an
   **approach** point (the demo used that path at 0.469 m), never as the final
   target. If no point appears, `joint1` is yawed through
   `+0.14, −0.14, +0.28, −0.28, +0.42, −0.42, +0.56, −0.56 rad`, measuring at
   each step; the first sighting wins.
4. **Approach.** A closed-loop base pulse
   (`base_pulse_vector.py`, vector duty 20, mecanum signs confirmed
   2026-11-18) drives the cube toward `(0.230, 0)` in the base frame. After
   every pulse the cube is re-measured (wide region; yaw sweep if the pulse
   lost it). Up to 5 pulses; the loop stops when the remaining planar travel
   is below 15 mm.
5. **Final measurement and envelope guard.** With the arm at the view pose
   (`joint1 = 0`), the strict segmenter and the placement gate (30-50 mm
   height, 30-50 mm horizontal extents) produce `C_view`. The correction
   target is fitted. The point must lie inside the **prior 24/24 empirical
   envelope**: measured `x` in 0.215-0.240 m and `|joint1| <= 0.16 rad`.
   The 2026-11-19 paired trial (which gripped 24/24 with the intervention)
   ran over measured points x 0.2169-0.2388 m and |y| <= 24.5 mm across both
   methods; the 24/24 intervention subset spans x 0.2169-0.2326 m,
   |y| <= 22.0 mm and |joint1| <= 0.142 rad. The guard's margins are 1.9 mm
   below and 1.2 mm above the prior x extremes, and 0.007 rad above the prior
   maximum |joint1|. If outside, up to 2 base trim pulses (target 0.230 m,
   5 mm tolerance) and re-measures; if still outside, the pair aborts before
   any grasp.
6. **Trial A** (the plan's first method): the validated
   `low_config_grasp_validate.py` sequence with the fast profile — open,
   approach from above (`joint4 = 0`), descend, close in two steps, rise to
   the view pose. The grip check asks whether a cluster sits in a 40 mm box
   around the measured cube (D2). If held, the cube is put back and the
   landing is measured; the landing must pass the same cube gates (D4).
7. The operator re-places the cube on the **same** site mark and presses
   Enter; **Trial B** runs with the same base pose.
8. **Return home.** The base drives the reverse of the commanded approach
   translation. The travel model is the same `duty * 4.9 mm/s` that sized the
   outbound pulses (not the pulse report's `duty / 0.182` nominal), so the
   base's unknown speed scale cancels: the return duration is the same as the
   outbound duration. The residual (wheel slip, possible yaw) is absorbed by
   the next pair's wide measurement or yaw sweep. The return is **not** an
   odometry or localization claim.

The grasp target, the grip classifier, the landing gates and the recorder are
exactly the validated 2026-11-19 ones; only the locate/approach phase and the
home return are new.

## 5. Outcomes

| Outcome | Definition | Role |
|---|---|---|
| Site located | the wide measurement or the yaw sweep returns a cube cluster before any base pulse | capability (descriptive) |
| Site approached | the strict final measurement passes after the base loop and the empirical envelope guard passes | capability (descriptive; a pair is only attempted when true) |
| Grasp success | after the close and the rise to the view pose, the cube's 40 mm box on the floor is empty | **primary** (paired) |
| Placement error | for a held cube, the xy distance between the release landing and the pre-grasp measurement, in mm | secondary (paired, continuous) |
| Failure code | `MISS` (cube still on the floor), `NO_GRIP` (no cluster and no grip), `AMBIGUOUS` (a cluster remains but the operator confirms the cube was held) | descriptive |
| Approach cost | initial radius/bearing, number of base pulses, commanded travel, final x and joint1, search yaw, wall time | descriptive |

**Capability failure handling.** If a site cannot be located or approached
after one operator retry (the runner aborts, the operator re-checks the cube
and mark and re-runs the same pair), the pair is **not** recorded as a grasp
trial. It is reported as a capability failure in the session summary and
excluded from the paired grasp analysis, with its reason. It is never
silently dropped and never converted into a `MISS`. The capability result is
therefore "sites approached / 24", and the grasp analysis reports `n_complete`
pairs.

**Why the binary outcome is primary.** The baseline's measured target error
(13.7 mm in x) is larger than the ±4 mm grip margin, so a baseline miss has no
release landing to measure; the continuous outcome is undefined for a miss.
The binary McNemar test is the primary comparison, as in the 2026-11-19
design. The placement error is reported as a one-sample estimate for the
intervention and as a paired difference if any baseline trial grips.

## 6. Sample size and analysis

**24 pairs = 48 trials.** The 24 sites are fixed by the design; they are not
chosen for power. With a baseline success near zero, the exact McNemar test
has a two-sided p < 1e-5 for 17 or more intervention-only discordant pairs.
The 24 intervention trials estimate the success rate to about ±10 % (95 %
Wilson). The same 24 sites give 24 locate/approach attempts, i.e. a capability
rate with a ±10 % Wilson interval; that is a descriptive precision, not a
power claim.

Analysis:

- `armpi_analysis.paired_trials analyze` on the completed pairs: exact
  two-sided McNemar on the success table; Wilson intervals; paired continuous
  difference with a seeded bootstrap 95 % interval, exact sign test and
  Cohen's `dz`; the discordant-pair split by which method ran first.
- A site table built from `approaches.jsonl` and the trial records by
  `tools/phase3/site_report.py`: `site_id`, initial radius/bearing, number of
  base pulses, final `x`/`|joint1|`, search yaw, success by method, placement
  error and the last-attempt state/locate/approach counts. This is
  descriptive; no site-level hypothesis test is pre-registered.
- The capability summary: sites located / 24, sites approached / 24, retries,
  and any capability failure with its reason.
- **No pooling** with the 2026-11-19 single-fixture trial. The two experiments
  are reported side by side: the first establishes the method effect at one
  fixture; this one asks whether the effect and the locate/approach chain
  survive site diversity. A pooled McNemar would double-count the same
  correction and hide the different sampling units.

## 7. Safety

- Supervised session; power switch within reach; the robot has **no e-stop
  and no brake** and falls if power is cut. The workspace is the floor.
- The arm stays at the view pose during every base pulse of a **normal**
  approach. If a trial aborts mid-motion, the runner's `finally` still drives
  the reverse home pulse with the arm wherever the abort left it; that pulse
  is a horizontal translation away from the cube, and the operator keeps
  clear and the power switch reachable. Completed records are kept.
- The base moves toward the cube, not toward the operator; the operator stays
  behind/at the side of the robot and keeps the power switch reachable.
- Adopted EEPROM-bounded joint limits; the grasp family stays inside the
  2026-11-19 validated joints. Base vector duty is capped at 20; a pulse is
  capped at 3 s; the approach is capped at 5 pulses and the return at 3 s.
- The 24 site marks are matte, low-saturation tape. The cube is the only
  saturated object inside 0.70 m, so the wrist segmenter and any front-camera
  blob selection cannot confuse a mark with the cube.
- Abort on any trajectory failure, action rejection, implausible pulse read,
  lost cube after a pulse, failed final measurement, or envelope violation.
  Completed records are kept.
- A battery swap or power loss stops the session. The robot's base pose is
  unchanged; the arm falls. Restart the stack, re-run the session: the
  pending pair is re-located from the current base pose, not replayed from a
  stale one. The session is resumable by construction.

## 8. Threats to validity and limits

- **The home return is open-loop.** It reverses the commanded translation
  with the plan's speed model so the unknown speed scale cancels, but wheel
  slip and any base yaw are not measured. The next pair's wide measurement or
  yaw sweep absorbs the residual. No claim is made that the robot returns to
  the same world pose; the initial sighting is recorded in the robot frame at
  that moment, not in a home frame.
- **The sites are nominal.** The operator places the cube on a mark; the
  robot measures the realized point. The pair's site identity is the mark and
  the operator's placement, not an independent global measurement.
- **The wrist far sighting is oblique.** At 0.30-0.45 m the top face can be
  partially clipped, so the segmenter may reject it for `partial_top_view`.
  The approach accepts that rejected point (the 2026-11-19 demo path); the
  **final target** always comes from a strict measurement in the band. A
  clipped initial centroid biases the approach direction, which the
  closed-loop re-measurements correct.
- **The correction envelope is empirical.** The documented tag-survey
  envelope is `joint1` ±0.06 rad; the 2026-11-19 paired trial itself gripped
  24/24 at `|joint1| <= 0.142 rad`. This experiment enforces the **realized**
  envelope (x 0.215-0.240 m, |joint1| <= 0.16) and records `x`/`|joint1|` per
  trial, so the outcome can be checked against the envelope. It does not
  claim a new yaw-envelope measurement.
- **One cube, one session, one battery state.** The cube is the same 39.4 mm
  wooden cube. Floor lighting and the operator are fixed. The battery state
  is recorded at the session start/end; a long session may need a mid-session
  charge (Section 7).
- **The front camera is not a second locator here.** Its near-band
  measurement agrees with the wrist to 2.6 mm, but the far-field 0.30-0.45 m
  top-face rectification is untested; using it in the loop would add an
  unvalidated measurement to a grasp experiment. It remains available as a
  future independent check.
- **Site count vs statistical power.** The 24 sites are a design choice for
  spatial coverage, not a power calculation. A site-level effect (e.g. a far
  site is systematically harder) is described, not tested.

## 9. Pre-session checks

1. **Camera envelope.** Place the cube at s21 and s24; at the view pose,
   confirm the wide measurement returns a point for both (s21 is the tightest
   top-edge corner at 0.45 m; s24 is the tightest side corner). If either is
   missed, do not start; record the failure and re-check the view
   pose/calibration. If the camera check is otherwise correct and the site is
   still missed, revise the layout **before** any trial (Section 10).
2. **Base direction.** One short forward and one short left pulse (duty 10,
   0.5 s) in the cleared workspace; confirm the wrist-measured cube moves
   toward the robot on the forward pulse and that its measured point shifts
   in the expected direction. Return with the reverse. (The 2026-11-18 signs
   are operator-confirmed; this is a session-start sanity check.)
3. **Cube and marks.** The cube is flat (39.3-39.6 mm high), the 24 marks are
   low-saturation, and the floor within 0.70 m has no other saturated object.
4. **Home mark.** The base origin is on the home mark and the forward axis is
   on the taped line; record the initial `C_view` of a cube at s09 as the
   session reference.
5. **Stack.** The vendor nodes, the wrist camera, the Humble container and
   the SSH key work; the battery voltage is recorded. The arm is at the view
   pose, `/joint_states` staleness is irrelevant (the runner reads the
   bus-servo registers).

## 10. Deviations

None: no 24-site trial has been run yet. This section is filled in
chronologically if the procedure, sites, envelopes or outcomes change after
the first valid record. A **pre-data** design change (for example a layout
revision forced by the pre-session camera check) is not a deviation: it
regenerates the layout and plan, updates the evidence hashes and records the
reason in the evidence README, still before the first valid record. The
pre-registered artifacts are the layout JSON, the operator markdown, this file
and the plan JSON; their hashes/commit are recorded with the first session
log.

## 11. Artifacts and commands

- Layout: `tools/phase3/site_layout.py`, 24 sites, 4 radii x 6 bearings.
- Session runner: `tools/phase3/site_diversity_session.py`; it subclasses the
  validated `paired_trial_session.Session` and appends through the same
  recorder.
- Site report: `tools/phase3/site_report.py`; it joins the approach sidecar
  and the trial records into the capability/outcome table (retries, locate,
  approach, pulse count, final envelope, abort reason).
- Plan: `evidence/phase3/onsite_20261120_sites/paired_trials_plan.json`
  (seed 17, generated before data).
- Trials: `evidence/phase3/onsite_20261120_sites/trials.jsonl` (append-only,
  created on the first valid trial).
- Approach sidecar: `evidence/phase3/onsite_20261120_sites/approaches.jsonl`
  (one locate/approach/return record per attempted pair; written even when a
  pair aborts, so the capability and return data exist independently of the
  trial records).
- Session log: `evidence/phase3/onsite_20261120_sites/session_*.log` (the
  operator tees the terminal output; a failed locate attempt is visible there
  even though it writes no trial record).

Dry run (no robot; writes to a temporary directory, not the evidence dir)::

    python3 tools/phase3/site_layout.py --output /tmp/site_layout.json \
        --markdown /tmp/site_layout.md
    python3 tools/phase3/site_diversity_session.py --dry-run \
        --sites /tmp/site_layout.json \
        --plan /tmp/site_plan.json --trials /tmp/site_trials.jsonl

On-site::

    sg docker -c "python3 tools/phase3/site_diversity_session.py \
        --sites evidence/phase3/onsite_20261120_sites/site_layout.json \
        --plan evidence/phase3/onsite_20261120_sites/paired_trials_plan.json \
        --trials evidence/phase3/onsite_20261120_sites/trials.jsonl" \
        2>&1 | tee evidence/phase3/onsite_20261120_sites/session_$(date +%Y%m%d_%H%M).log

After the session::

    PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m \
        armpi_analysis.paired_trials analyze \
        --input evidence/phase3/onsite_20261120_sites/trials.jsonl \
        --json-out evidence/phase3/onsite_20261120_sites/analysis.json \
        --markdown-out evidence/phase3/onsite_20261120_sites/analysis.md

The dry run is a control-flow check only; its synthetic records must never be
copied into the evidence directory.
