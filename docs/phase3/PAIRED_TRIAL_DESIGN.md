# Paired hardware trial — pre-registered design

> Written **before** the first trial. Any change after data collection starts is
> recorded as a deviation at the bottom of this file.
>
> Planned: 2026-11-19, on the ArmPi Ultra floor-grasp setup. Operator present,
> all motion supervised.

## 1. Question

Does the grasp-validated constant low-configuration correction
`(+13.7, +3.9, +8.9) mm` change the outcome of a floor-cube grasp, compared
with the raw FK/hand–eye target that failed passes 3–4 on 2026-11-19?

The mechanism is measured: at the low grasp configuration the model places the
TCP 13.7 mm short in `x` and 8.9 mm low in `z`, which is outside the ±4 mm grip
margin. The intervention subtracts that measured offset from the commanded
target. A physical model for the offset was tested and rejected
([`model_selection.md`](../../evidence/calibration/20261119/low_config_error/model_selection.md)),
so the correction is a constant, valid inside the tested envelope
(tool-down, `j1` within ±0.06 rad of 0, view pose `j4 −1.58…−1.66`). The
fixture keeps every trial inside that envelope.

## 2. Design

Matched pairs. For each pair the operator places the same 39.4 mm wooden cube
at the same floor fixture and two trials run back to back:

| Method | Target sent to the joint fit |
|---|---|
| `baseline` | `C_view` — the cube top-face centroid measured at the view pose, no correction |
| `intervention` | `C_view + (+13.7, +3.9, +8.9) mm` |

Randomization (seeded, reproducible):

1. the method order inside each pair;
2. the order of the pairs.

The plan is generated with `armpi_analysis.paired_trials plan` and is
append-only from then on.

## 3. Outcomes

| Outcome | Definition | Role |
|---|---|---|
| Grasp success | after the close and the rise to the view pose, the floor has **no** cluster above it (the cube is held) | **primary** |
| Placement error | for a held cube, the xy distance between the release landing and the pre-grasp measurement, in mm | secondary (continuous) |
| Failure code | `MISS` (cube still on the floor), `NO_GRIP` (no cluster and no grip), `AMBIGUOUS` (a cluster remains but the operator confirms the cube was held) | descriptive |

**Why the binary outcome is primary here.** The project framework
([`PAIRED_TRIALS.md`](../../docs/phase3/PAIRED_TRIALS.md)) lists the continuous
error first. That ordering assumes both methods produce a grip. The baseline
does not: its measured target error (13.7 mm in `x`) is more than three times
the ±4 mm grip margin, so a baseline trial is expected to miss and therefore
has **no release landing to measure**. The continuous outcome is undefined for
a miss, and a paired continuous test would have almost no complete pairs. The
binary McNemar test is therefore the primary comparison; the placement error is
reported as a one-sample estimate for the intervention and as a paired
difference if any baseline trial does grip. This choice is made before data
collection and is not changed after seeing the result.

## 4. Sample size

**24 pairs = 48 trials.** With an expected baseline success near zero, the
exact McNemar test has a two-sided p < 1e-5 for 17 or more discordant
intervention-only pairs. The same 24 intervention trials estimate the success
rate to about ±10 % (95 % Wilson) and the placement error distribution.

The session is resumable: records are appended one trial at a time, and an
interrupted session reruns only the missing entries.

## 5. Fixture and envelope

- Fixture: a marked square on the floor (`floor_square_A`) that places the cube
  at x ≈ 0.228–0.232 m on the robot's forward axis. This is the validated
  measurement band: at x < 0.226 the top face is clipped by the frame bottom,
  and the deep family reaches only x ≈ 0.245 at that height.
- The same cube (39.4 mm) all session; the operator re-places it on the mark
  after every trial, including after a successful release.
- `joint5 = 0` for every trial: at the view pose a non-zero roll puts the
  fingers across the camera ray and biases the centroid.
- If the pre-trial segmentation rejects the top face (clipped view), the
  operator re-places the cube and the measurement is retried; no trial is
  recorded until a valid pre-trial measurement exists.

## 6. Procedure per trial (automated)

1. Operator places the cube on the fixture mark and presses Enter.
2. Read the five bus-servo registers at the view pose.
3. Segment the registered point cloud with the measured hand–eye, giving
   `C_view`.
4. Target = `C_view` (+ the correction for the intervention).
5. Fit `joint1` and `joint4` of the deep family (`joint2 = -0.6618`,
   `joint3 = -1.8012`, `joint5 = 0`) so the model TCP lands on the target in
   `x`/`y`.
6. Open, approach from above (`joint4 = 0`), descend to the grasp joints, close
   in two steps, rise to the view pose.
7. Segment at the view pose: no cluster above the floor ⇒ held.
8. If held: lower to the grasp joints, open, rise to the view pose, segment
   again and record the landing displacement.
9. Append the record to `trials.jsonl` through the validated recorder.

The sequence is exactly the validated 2026-11-19 one; only the target and the
joint fit are method-dependent.

## 7. Analysis

- Exact two-sided McNemar test on the paired success table.
- Per-method success rates with 95 % Wilson intervals.
- Paired differences of the continuous error where both methods succeeded
  (bootstrap 95 % interval, sign test, Cohen's `dz`), rendered by
  `armpi_analysis.paired_trials analyze`.
- Order effect: the discordant-pair split by which method ran first
  (descriptive).
- Every record keeps the measured point, target, joints, fitted residuals and
  segmentation reports in `notes`.

## 8. Safety

- Supervised session; power cut within reach; the robot has no e-stop or brake
  and falls if power is cut.
- Adopted EEPROM-bounded joint limits; `joint4` in the grasp family stays
  between −0.28 and −0.45 rad, inside the adopted range.
- No base motion, no yaw: the fixture stays inside the validated envelope.
- Abort on any trajectory failure, action rejection or unexpected pulse read;
  completed records are kept.

## 9. Deviations

**D4 — 2026-11-19, trial p003/intervention: the landing measurement picked
up a different object.** The post-release report selected a cluster **21.7 mm**
above the floor (the cube is 39.4 mm) at `(0.216, 0.055, −0.066) m`, i.e. a
neighbouring low object, giving a meaningless 38.4 mm "error". The grasp itself
was held (the floor was empty at the grip check). The runner now applies the
same 30–50 mm height / 30–50 mm extent gate to the landing and refuses to
record a trial whose landing does not look like the cube. The p003/intervention
record was withdrawn (preserved in
[`trials_with_p003_artifact.jsonl`](../../evidence/phase3/onsite_20261119/trials_with_p003_artifact.jsonl))
and the trial was re-run after the session. All other 11 successful landings
measured 39.4–39.6 mm above the floor.

**D3 — 2026-11-19, trials p019/baseline and p023/baseline: placement
contamination and a negative-argument crash.** A tape roll (and possibly a
spacer) was left inside the grasp region. The p019 baseline pre-measurement put
the cube's top face **59.3 mm** above the floor (the flat cube is 39.3–39.6 mm),
i.e. the cube sat on a ~20 mm object; that trial gripped and dragged the cube
19.9 mm. The p019 intervention trial was flat (39.4 mm) and is retained. The
p023 baseline pre-measurement merged the cube with a neighbour into a
**53.7 × 72.4 mm** cluster, and the runner then aborted before motion because a
negative joint list was passed as `--grasp -0.1762,...` and argparse read it as
an option.

Fixes: the runner now rejects a pre-trial measurement whose height is outside
**30–50 mm** or whose horizontal extents are outside **30–50 mm**, and passes
`--grasp=`/`--verify-pose=` so negative values parse. The invalid p019 baseline
record is kept in
[`discarded_protocol_violation_attempt.jsonl`](../../evidence/phase3/onsite_20261119/discarded_protocol_violation_attempt.jsonl);
the p019 pair is incomplete and its baseline is re-run. The operator was asked
to clear the grasp region to flat tape only.

**D2 — 2026-11-19, trial p006/intervention: the grip classifier misread a
success as a miss.** The segmenter counts every cluster above the floor, and
its report included a background cluster at `(0.499, −0.249, −0.083) m`
(`x_out_of_region`, 16 mm high). The runner's first grip rule
(`clusters == 0`) therefore recorded a miss even though the cube was held: a
re-measurement showed **0 clusters** and the arm was still holding it. The cube
was released at the grasp joints and landed **2.9 mm** from the pre-grasp
point. The trial is retained with its correct outcome (success, 2.9 mm), the
original misclassified record is kept in
[`discarded_grip_classifier_attempt.jsonl`](../../evidence/phase3/onsite_20261119/discarded_grip_classifier_attempt.jsonl),
and the grip test now asks whether a cluster lies in a **40 mm box around the
measured cube**, the same "cube box" check the validated runs used. The change
touches only the outcome classifier, not the grasp execution or the measured
continuous error.

**D1 — 2026-11-19, before any valid trial: a unit error discarded the first
pair.** The runner added the correction in millimetres to a target in metres,
so the first intervention target became `(13.9319, 3.9220, 8.8507) m`. The
joint fit saturated (`joint4 = 0.8987`, xy error 14 144 mm), the arm executed
the bogus pose, and the trial recorded a miss. The following baseline trial
then recorded a miss against the cube's new position. Both records are kept for
transparency in
[`discarded_unit_error_attempt.jsonl`](../../evidence/phase3/onsite_20261119/discarded_unit_error_attempt.jsonl)
and are **not** part of the analysis; the plan restarts at `p006`. Fixes:
the correction is converted to metres, and `validate_target` now refuses a
target outside 0.10–0.30 m, a joint-fit xy error above
`--max-xy-error-mm` (default 3.0), or a `joint4` low-limit saturation —
before any motion.
