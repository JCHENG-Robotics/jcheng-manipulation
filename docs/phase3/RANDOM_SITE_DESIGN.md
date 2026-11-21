# Random-placement 24-pair trial — pre-registered design

> Written **before** the first random-placement trial; the free-placement
> change was pre-data. One old-flow attempt (the site put-back) was then
> discarded (D1, Section 11); the **home-return flow** in this file is the
> protocol for the valid data. Any change after the first valid new-flow
> record is recorded as a deviation at the bottom of this file.
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
puts the cube by eye at a new spot anywhere in the forward workspace before
every pair, and the robot must locate it, drive to it when it is far, and
complete the validated grasp/put-back. The radius/bearing envelope is a guide
to where the robot can see the cube, not a gate; the operator does not
measure.

1. **Capability.** At how many of the 24 free placements can the robot
   locate the cube (wrist view, chassis front camera or joint1 yaw sweep),
   bring it into the 0.226-0.234 m grasp band when needed, and complete the
   pair? A placement already inside the band is grasped directly.
2. **Method.** Under this free-placement regime, does the grasp-validated
   constant low-configuration correction `e = (+13.7, +3.9, +8.9) mm` still
   change the grasp outcome relative to the raw FK/hand-eye target?

The realized point, measured by the robot, is the recorded site. The operator
intent is not used in the analysis.

## 2. Free placement and reference envelope

The radius/bearing envelope is the **sensor-covered fan from the home pose**,
a guide, not a design grid or a gate:

```text
guide only:  radius ~0.25-0.55 m
             bearing ~-20 ... +25 deg   (base +y is left)
```

Why this guide:

- **Near half (radius <= ~0.46 m).** The validated view-pose wrist camera
  sees the floor band; a joint1 yaw sweep covers the wider bearings. The
  2026-11-19 out-of-view demo found a cube at 0.469 m, bearing 38.8 deg with
  a +0.28 rad yaw.
- **Far half (0.46-0.55 m).** The wrist image cuts a cube top near 0.46 m
  (the 2026-11-19 demo saw 0.469 m only at the upper image edge). The chassis
  D435i sees this far band from the home pose; its calibrated optical axis is
  at base y = +0.027 m, yaw -1.1 deg, which is why the bearing bounds are
  asymmetric. The far bearing limits are the calibrated far fan: a 0.55 m
  point at +25 deg sits about 29.5 deg from the front optical axis (inside the
  35 deg half-field), and -20 deg sits about 29.4 deg on the other side. A
  point much farther or wider may not be locatable; the operator then moves
  it a little closer/centred.

**Placement rule (operator):** before every pair, place the cube **by eye at
a new spot in the forward workspace; no marks, no measurements, no fixed
points.** Do not reuse the same spot. The robot reports the realized point and
records it. If the measured point is already inside the 0.226-0.234 m band,
the robot grasps directly; otherwise it drives the base toward it (one or more
closed-loop pulses) and then grasps. The region membership is reported
descriptively, not enforced.

The rule, reference envelope, 24 fixture labels (`r01`...`r24`) and the
seeded pair/method plan (seed **17**) are in
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
2. **Chassis D435i.** One colour + depth capture in the vendor container,
   copied to the laptop, then the pose-independent **depth top-face cluster**
   (`ext_cam_measure.py --depth-cluster`): RANSAC the support plane, select
   the points 30-50 mm above it inside the region's bounding box, cluster them
   on a 5 mm x/y grid and take the largest component. The depth top face is
   used instead of the rectified colour because at the oblique far view the
   wooden top can be over-exposed and the fixed colour thresholds miss it
   (the 2026-11-21 precheck frame: a far cube at 0.43 m was measured by the
   depth cluster, not by the colour path). The `point_base` is used **only as
   a base-drive target**; the front camera is not a grasp measurement. It is
   tried before the yaw sweep because one capture is shorter than the
   eight-step sweep; it covers the far half (0.46-0.55 m) that the wrist
   cannot see at any yaw. The 0.30-0.55 m far-field depth is the new,
   less-validated part; the pre-session check (Section 9) gates it.
3. **joint1 yaw sweep.** `+0.14, -0.14, +0.28, -0.28, +0.42, -0.42, +0.56,
   -0.56 rad`, measuring at each step, for the near wide bearings that the
   front camera cannot see. This is the 2026-11-19 out-of-view demo path; the
   arm is at the raised view pose, never the low grasp pose.

If all three return no point, the pair aborts before any trial and is a
**capability failure**, recorded in `approaches.jsonl` with the reason.

The final grasp target always comes from the **strict wrist measurement in the
band** (placement gate 30-50 mm height and extents). The front camera can
never supply the grasp target.

## 5. Procedure per trial (two trials per pair)

For each of a pair's two methods, back to back at the same realized spot:

1. **Trial A.** The runner prints the pair; the operator places the cube by
   eye at a new spot in the forward workspace (no measuring) and presses
   Enter.
2. The arm moves to the validated view pose
   `[0, 0.2574, -1.18773, -1.70, -0.01676]`.
3. **Locate** with the chain above. The realized point is reported and its
   region membership is recorded descriptively; there is **no** placement gate
   and no re-place prompt. If the locator finds no point, the cube is usually
   visible in the image but outside the segmenter's 3-D region (farther than
   x 0.60 m, closer than 0.15 m, or wider than |y| 0.40 m); the runner prompts
   the operator to move it to about 0.3-0.5 m, centred, and retries up to 2.
   A second failure aborts the trial before any motion.
4. **Approach or direct grasp.** If the measured point is already inside the
   0.226-0.234 m band, no pulse is sent and the robot grasps directly (the
   near case). Otherwise closed-loop base pulses (vector duty 20) drive toward
   `(0.230, 0)`; after each pulse the full locator chain is re-run. Up to 5
   pulses; stop below 15 mm remaining planar travel (the far case).
5. **Final measurement and envelope guard.** The strict wrist measurement at
   `joint1 = 0`; the target fit must stay inside the prior 24/24 empirical
   envelope: measured `x` 0.215-0.240 m and `|joint1| <= 0.16 rad` (the
   2026-11-19 paired trial gripped 24/24 with the intervention over measured
   points x 0.2169-0.2388 m and |joint1| <= 0.142 rad across both methods).
   Up to 2 base trims; still outside means abort before any grasp.
6. **Grasp and return home with the cube.** The validated
   `low_config_grasp_validate.py` sequence and the grip check in the 40 mm cube
   box. If held, the base **drives the reverse of the commanded approach while
   holding the cube** (the 2026-11-20 far-run return: 245.3 mm reversed,
   11.06 mm landing against the fitted target), then the arm lowers and
   releases the cube **at the start**. The operator picks it up there. If the
   trial missed, the base returns empty and the cube stays at the spot; the
   operator adjusts it there for Trial B.
7. **Home landing check.** The landing is measured with the wide region; it
   must pass the cube gates and lie within **60 mm** of the fitted release
   target (the 2026-11-20 release was 11.06 mm). A missing cube or a farther
   landing means the grip claim was false — the cube stayed at the spot — and
   the trial is recorded as **NO_GRIP**, not a crash. A **clipped top face**
   at the image border is accepted as a success when the cluster still passes
   the cube height/size gates; the release error is then a biased secondary
   value, recorded as `landing_partial` with the segmenter's real rejection
   reason. The operator re-checks the cube at the spot for Trial B.
8. **Trial B.** The operator takes the cube from the start (held case) or
   adjusts it at the spot (miss case) and places it at the **same spot**; the
   base approach is repeated from the start. The second trial has the same
   locate/approach/grasp/return-home/release sequence.
8. After both trials the base is at the start; the next pair's placement
   begins there. The sidecar records one locate/approach/return record per
   trial (two per completed pair).

The runner writes the approach sidecar (`approaches.jsonl`) **and** the
validated paired-trial records. The final `stop` after a trial (including an
aborted one) always drives the reverse home pulse with the arm wherever the
abort left it; the operator keeps clear and the power switch reachable.

## 6. Outcomes

| Outcome | Definition | Role |
|---|---|---|
| Located | the locator chain returns a cube point before the first base pulse | capability (descriptive) |
| Inside reference envelope | the first measured point is inside the radius/bearing guide | descriptive (not a gate) |
| Direct grasp | the first measured point is already inside the 0.226-0.234 m band, so no base pulse is sent | descriptive (near case) |
| Approached | the strict final measurement passes and the empirical envelope passes | capability (descriptive; a trial is attempted only then) |
| Grasp success | after the close and rise, the cube's 40 mm floor box is empty | **primary** (paired) |
| Home release error | for a held cube, the xy landing displacement from the fitted release target (nominal TCP) after the base returns home; it measures the release precision at the start, not a put-back at the original spot | secondary (paired, continuous) |
| Failure code | `MISS` (the grip check still sees the cube at the spot), `NO_GRIP` (the grip claim was false: the cube is absent at the start or more than 60 mm from the release target), `AMBIGUOUS` | descriptive |
| Approach cost | locator used, initial radius/bearing, base pulses, final x/|joint1|, wall time, per trial | descriptive |

**Capability failure handling.** A trial that cannot be located or approached
after the two prompted operator retries when the cube was outside the 3-D
region is a capability failure; it is not recorded as a grasp trial, the
reason is in `approaches.jsonl`, and the pair is excluded from the paired
grasp analysis with `n_complete` reported. It is never silently dropped and
never converted into a `MISS`.

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
- `tools/phase3/site_report.py`: the capability table (pairs, approach
  records one per trial, located/approached, locator, pulse count, realized
  radius/bearing, final envelope, abort reason).
- The paired continuous outcome is the **home release error** (the landing
  vs the fitted release target after the return), not the original site
  put-back; it is reported only when both methods held the cube. The primary
  success comparison is unchanged.
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
- **The return home while holding the cube** is the 2026-11-20 far-run path
  (245.3 mm reversed with the cube held, 11.06 mm landing against the fitted
  target). The base reverses with the arm at the view pose; the arm then
  lowers and releases at the start. The operator stays clear of the arm path
  and picks the cube up only after the release. The base return is open-loop
  (centimetre-level), not a localization.
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

1. **Free near/far warm-up.** With `--precheck`, place the cube **by eye**
   once near (~0.3 m) and once far (~0.5 m); no coordinates, no marks. The
   near placement should be located by the wrist (or the chain); the far one
   by the chassis front camera. If the far one is not located, move it a little
   closer/centred and repeat; **do not start** the 24-pair session with a far
   locator that never sees a far cube. The precheck does not move the base and
   writes no trial.
2. **Base direction.** One short forward and one short left pulse in the
   cleared workspace; confirm the wrist-measured point shifts in the expected
   direction; return with the reverse.
3. **Cube and floor.** The cube is flat (39.3-39.6 mm) and is the only
   saturated object within 0.70 m; no tape marks or other saturated clutter.
4. **Robot.** Vendor stack, wrist camera, D435i depth, Humble container,
   SSH key and battery voltage recorded; the base origin on the home mark and
   the forward axis on the taped line.

The `--precheck` mode runs the free near/far warm-up without moving the base
and writes the front-camera measurement JSONs to `front/`.

## 10. Threats to validity and limits

- **The randomisation is the operator, not a random-number generator.** The
  24 realized points are recorded, but the sample is not reproducible and the
  operator can unconsciously avoid hard spots. The free-placement rule (new
  spot by eye, no measuring) is enforced by instruction only; the realized
  distribution is reported.
- **The front camera's far-field depth top-face measurement is not fully
  validated.** It is used only as a drive target; a bias of a few millimetres
  at 0.5 m is tolerable because the closed loop re-measures and the wrist
  takes over. The free near/far warm-up checks it once; the in-run locator
  chain is the real test.
- **The home return is open-loop.** The model-cancelled reverse cancels the
  unknown speed scale, not wheel slip or yaw. The next locate is relative and
  absorbs it; no global-pose claim is made.
- **The radius/bearing envelope is a guide, not the whole floor.** A point
  much farther or wider may be unlocatable from the home pose; the operator
  then moves it a little closer. No coordinate is measured by the operator.
- **The continuous outcome is the home release error.** The cube is released
  at the start, not at the original spot, so the 2026-11-19 site put-back
  metric does not apply. The home release error measures the release precision
  at the start; the primary success comparison is unchanged. The two methods'
  fits differ slightly, so the release targets differ slightly.
- **One old-flow trial precedes this protocol.** Before the operator requested
  the home-return flow, one intervention trial was recorded with the site
  put-back (success, 4.38 mm). It is preserved in the discarded file and
  excluded; `p020` restarts under this flow (D1).
- **A cube left near the start can look carried.** The home landing accepts a
  cube within 60 mm of the release target; a false grip that leaves the cube
  within that radius of the start is indistinguishable from a carried cube.
  The free spots should stay a few centimetres away from the start release
  area; the near-direct case (cube already in the band) is the ambiguous one.
  The operator's visual check and the recorded landing/error values are the
  audit.
- **The first new-flow attempt was a false grip.** The cube at 0.2265 m was
  pushed out of the check box, the robot returned empty, and the start
  landing found no cube; the trial was not recorded. The runner now records
  that case as `NO_GRIP` instead of aborting the session, and retries the
  landing with the wide region. No valid new-flow trial preceded the fix.
- **The start top view can be clipped.** The release places the cube near the
  image border at the start; the top-view extent gate may reject it while the
  height/size gates pass. The success is accepted with `landing_partial`; the
  release error is biased and must not be read as a millimetre-grade placement
  value. A rejected landing always prints the segmenter's real reason (the
  p003 lesson).
- **A cube in the image can be outside the 3-D region.** The segmenter filters
  the point cloud by `x 0.15-0.60`, `|y| <= 0.40` (wide region) and the front
  box `x 0.212-0.565`; a cube farther, closer or wider is visible in the
  colour image but has no point. The retry prompt tells the operator to move
  it to about 0.3-0.5 m; the guide is not a sensor guarantee.
- **One cube, one operator, one session, one battery state.** The cube is the
  same 39.4 mm wooden cube; the lighting and operator are fixed.
- **Site count vs power.** The 24 placements are for coverage, not a power
  calculation; a site-level effect is described, not tested.

## 11. Deviations

**D1 — 2026-11-21, the home-return release flow.** The first recorded trial
(`p020` / intervention, success, 4.38 mm) used the site put-back: the robot
released the cube at the measured spot. The operator then asked for the robot
to carry the cube back to the start and release it there, so the operator can
recover it at the start and re-place it at the same spot for the second
method. The trial is preserved in
`discarded_site_release_attempt.jsonl` (with the aborted baseline attempt and
the old approach records in `discarded_site_release_approaches.jsonl`) and is
**not** part of the new-flow analysis. `p020` restarts from its first method
under the home-return flow. The data collection before this change is one
discarded attempt, not a valid pair. The change is also recorded in the
evidence README.

**D2 — 2026-11-21, the release gripper open duration.** At `p015` /
intervention the grip held, but the release open used the tool's 0.45 s
interpolation default: the register stopped at 421 (open reads about 183)
and the cube stayed between the fingers, so the start landing found no cube
and the trial was recorded as `NO_GRIP`. The operator saw the cube still in
the gripper. The trial is preserved in
`discarded_release_stall_trial.jsonl` and excluded; `p015` restarts from its
first method. The release now commands `--release-open-duration 3.5` (the
servo tracks at about 100 pulses/s) and the runner records
`release_open_register` and warns the operator when it stays above 230. This
changes only the release open timing, not the grasp, the release target or
any measurement.

**Pre-data amendment (2026-11-21, before any trial).** The operator asked for
free placement without measuring. The fixed far precheck points and the
region re-place gate were removed: the operator now places the cube by eye
anywhere in the forward workspace, the robot reports the realized point and
decides direct grasp (already in the band) vs base approach (far). The region
JSON is a sensor-coverage guide, not a gate; the far front camera is checked
by a free near/far warm-up. No trial record existed at this change.

The fixed 24-mark fan in `SITE_DIVERSITY_DESIGN.md` was superseded by this
file before data; its evidence directory has no trials.

## 12. Artifacts and commands

- Region: `tools/phase3/random_region.py`; the free-placement rule and
  reference envelope in
  `evidence/phase3/onsite_20261121_random/region.json`.
- Runner: `tools/phase3/random_site_session.py` (subclasses the validated
  `paired_trial_session.Session`; the grasp/grip/recorder path is the
  validated one; the runner adds the per-trial approach, the return home while
  holding the cube, and the release at the start).
- Plan: `evidence/phase3/onsite_20261121_random/paired_trials_plan.json`
  (seed 17, fixtures r01...r24, generated before data).
- Trials: `evidence/phase3/onsite_20261121_random/trials.jsonl`;
  sidecar: `approaches.jsonl` (one record per trial); front-camera
  measurement JSONs: `front/`.
- Discarded (excluded from analysis):
  `discarded_site_release_attempt.jsonl` (the one old-flow trial),
  `discarded_site_release_approaches.jsonl` (its approach records).
- Session log: `evidence/phase3/onsite_20261121_random/session_*.log`.

Precheck (free near/far warm-up; no coordinates, no base motion, no trial)::

    source /opt/ros/jazzy/setup.bash
    source ros2_ws/install/setup.bash
    sg docker -c "python3 tools/phase3/random_site_session.py --precheck \
        --region evidence/phase3/onsite_20261121_random/region.json \
        --plan evidence/phase3/onsite_20261121_random/paired_trials_plan.json \
        --trials evidence/phase3/onsite_20261121_random/trials.jsonl"

On-site::

    source /opt/ros/jazzy/setup.bash
    source ros2_ws/install/setup.bash
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
