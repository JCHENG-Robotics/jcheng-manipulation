# Random-placement 24-pair trial — pre-registered artifacts

> Status: **pre-registered on 2026-11-21; the new-flow session is starting.**
> The operator places the cube **by eye, without measuring**, at a new spot in
> the forward workspace before every pair; nothing is pre-taped or pre-drawn.
> The radius/bearing envelope is a guide to where the robot can see the cube,
> not a gate. One old-flow attempt (the site put-back) was recorded and then
> discarded when the operator asked for the **home-return flow**: the robot
> carries the cube back to the start and releases it there between the two
> trials. See `discarded_site_release_*.jsonl` and D1 in the design.
>
> This directory supersedes the fixed 24-mark fan in
> `../onsite_20261120_sites/` **before any trial**. The fixed-fan directory
> has no trial records; see `docs/phase3/RANDOM_SITE_DESIGN.md`.

## Files

| File | SHA-256 | Contents |
|---|---|---|
| `region.json` | `26428246f342a210321250141b7fc7852c8d06d1c1c020016151c5fe7f3d3f1f` | Free-placement rule, reference envelope (r ~0.25-0.55 m, bearing ~-20..+25 deg), 24 fixture labels, seed, locator order |
| `paired_trials_plan.json` | `02a4ac0ddf9f15f9c96f9678dc1778536ddc6c20dc4e9b18976afb17746e0bea` | Seeded plan (seed 17): pair order, within-pair method order, fixtures r01..r24 |
| `trials.jsonl` | — | Created on the first valid new-flow trial; append-only |
| `approaches.jsonl` | — | One locate/approach/return record per trial (two per completed pair), including aborts |
| `discarded_site_release_attempt.jsonl` | — | The one old-flow intervention trial (site put-back), excluded (D1) |
| `discarded_site_release_approaches.jsonl` | — | Its approach/abort records, excluded (D1) |
| `front/` | — | Front-camera measurement JSONs used by the far locator (and the precheck) |
| `site_report.md` | — | Capability/outcome table rendered from the two JSONL files |
| `analysis.json`, `analysis.md` | — | Paired analysis after the session |
| `session_*.log` | — | Operator terminal logs, including the discarded attempt and prechecks |

The hashes are the pre-registration. The free-placement change was pre-data;
the home-return flow is D1 (after one discarded old-flow trial). After the
first valid new-flow record, any change is a deviation.

## Free placement and reference envelope

```text
guide only:  radius ~0.25-0.55 m
             bearing ~-20 ... +25 deg   (base +y is left)
```

The near half is the validated view-pose wrist band or the joint1 yaw sweep;
the far half (0.46-0.55 m) is outside the wrist image and is covered by the
chassis D435i. The bearing bounds are the calibrated front-camera far fan.
They are a **guide** to where the robot can see the cube, not an enforced
sampling region.

**Operator rule:** before every pair, place the cube by eye at a new spot in
the forward workspace. **No marks, no measurements, no fixed points.** Do not
reuse the same spot. The robot reports the realized point and records it; if
the measured point is already in the 0.226-0.234 m band it grasps directly,
otherwise it drives the base toward it and then grasps. If the cube is too far
or too wide to locate, move it a little closer/centred; no coordinate is
needed.

**Between the two trials:** if the robot held the cube, it carries it back to
the start and releases it there; the operator takes it from the start and
places it at the **same spot** for the second trial. If the trial missed, the
cube stays at the spot; the operator adjusts it there. After the second trial
the robot returns to the start again (with the cube if held), ready for the
next pair.

## Plan

`paired_trials_plan.json` was generated before data with
`armpi_analysis.paired_trials.build_plan(pair_ids, fixtures=r01..r24,
seed=17)`:

- 24 pairs `p001`...`p024`, paired with `r01`...`r24`;
- the visit order is shuffled (first pair `p020` at `r20`);
- the method order inside each pair is shuffled independently;
- both methods of a pair are adjacent in the plan; the two trials use the
  same realized spot, and the robot returns home with the cube between them.

The plan is append-only. The trial records go through
`armpi_analysis.paired_trials.append_record`.

## Operator procedure

1. Run the free near/far warm-up in
   [`docs/phase3/RANDOM_SITE_DESIGN.md`](../../../docs/phase3/RANDOM_SITE_DESIGN.md)
   §9 with `--precheck`: place the cube by eye once near (~0.3 m) and once far
   (~0.5 m). **No coordinates, no marks.** The far placement should be located
   by the wrist or the front camera; if not, move the cube a little closer and
   repeat. The precheck does not move the base and writes no trial.
2. Start the session:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
sg docker -c "python3 tools/phase3/random_site_session.py \
    --region evidence/phase3/onsite_20261121_random/region.json \
    --plan evidence/phase3/onsite_20261121_random/paired_trials_plan.json \
    --trials evidence/phase3/onsite_20261121_random/trials.jsonl" \
    2>&1 | tee evidence/phase3/onsite_20261121_random/session_$(date +%Y%m%d_%H%M).log
```

The runner prints the pair. Place the cube by eye at a fresh spot for the
first method. If the robot held it, the robot has carried it back to the start
and released it there; take it from the start and place it at the **same
spot** for the second method. If the first method missed, the cube is still at
the spot; adjust it there. The runner reports the realized point and decides:
near (already in the 0.226-0.234 m band) grasps directly, far drives the base
toward it and then grasps. No measuring is required. After the second method
the robot returns to the start again.

If the runner aborts at a trial (no locator point, lost cube, final
measurement, envelope), re-check the cube and re-run the same pair. The
attempt is in the session log and `approaches.jsonl` but writes no trial
record. A trial that fails after one retry is a capability failure; it is
reported, not converted into a `MISS`.

**First new-flow attempt (2026-11-21, aborted).** The wrist wide measurement
missed the cube; the front camera found no depth cluster at that distance; the
joint1 yaw sweep found it at 0.481 m. Two base pulses brought it to the band,
the grasp check claimed held, the robot returned home, but the cube had been
pushed out of the 40 mm check box and was still at the spot; the start landing
found no cube and the trial aborted. The runner now records that case as
`NO_GRIP` (and retries the landing with a wide region) instead of stopping the
session. No trial record came from the attempt; its approach/error is in
`approaches.jsonl`.

## After the session

```bash
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m \
    armpi_analysis.paired_trials analyze \
    --input evidence/phase3/onsite_20261121_random/trials.jsonl \
    --json-out evidence/phase3/onsite_20261121_random/analysis.json \
    --markdown-out evidence/phase3/onsite_20261121_random/analysis.md

python3 tools/phase3/site_report.py \
    --approaches evidence/phase3/onsite_20261121_random/approaches.jsonl \
    --trials evidence/phase3/onsite_20261121_random/trials.jsonl \
    --markdown-out evidence/phase3/onsite_20261121_random/site_report.md
```

The paired continuous outcome is the **home release error** (the landing vs
the fitted release target after the return home); the 2026-11-19 site put-back
metric is not used in this flow. The `discarded_site_release_*` files are
excluded.

Record the capability summary (located/approached, approach records, aborts,
realized point distribution) in this README, and add the outcome to
`docs/phase3/RANDOM_SITE_DESIGN.md`, the phase-3 README and the CHANGELOG. Do
not pool with the 2026-11-19 single-fixture trial or the superseded fixed-fan
design.
