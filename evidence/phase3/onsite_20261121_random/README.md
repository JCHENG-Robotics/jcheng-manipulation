# Random-placement 24-pair trial — pre-registered artifacts

> Status: **pre-registered on 2026-11-21; no trial has been run.** The
> operator places the cube at a fresh random point inside the region before
> every pair; nothing is pre-taped or pre-drawn. These files fix the region,
> the placement rule, the pair/method plan and the analysis before data.
>
> This directory supersedes the fixed 24-mark fan in
> `../onsite_20261120_sites/` **before any trial**. The fixed-fan directory
> has no trial records; see `docs/phase3/RANDOM_SITE_DESIGN.md`.

## Files

| File | SHA-256 | Contents |
|---|---|---|
| `region.json` | `3857dae9c9855210374a18209afaa315d66dd9e879768666b30b127d6bea6efd` | Region (r 0.25-0.55 m, bearing -20..+25 deg), placement rule, 24 fixture labels, seed |
| `paired_trials_plan.json` | `02a4ac0ddf9f15f9c96f9678dc1778536ddc6c20dc4e9b18976afb17746e0bea` | Seeded plan (seed 17): pair order, within-pair method order, fixtures r01..r24 |
| `trials.jsonl` | — | Created on the first valid trial; append-only |
| `approaches.jsonl` | — | One locate/approach/return record per attempted pair, including aborts |
| `front/` | — | Front-camera measurement JSONs used by the far locator (and the precheck) |
| `site_report.md` | — | Capability/outcome table rendered from the two JSONL files |
| `analysis.json`, `analysis.md` | — | Paired analysis after the session |
| `session_*.log` | — | Operator terminal logs, including aborted prechecks |

The hashes are the pre-registration. A pre-data region revision (for example
after the front-camera precheck) regenerates the region and plan and updates
the hashes before the first trial; after the first trial it is a deviation.

## Region and placement rule

```text
radius   0.25-0.55 m
bearing  -20 ... +25 deg   (base +y is left)
```

The near half is the validated view-pose wrist band or the joint1 yaw sweep;
the far half (0.46-0.55 m) is outside the wrist image and is covered by the
chassis D435i. The asymmetric bearing bounds are the calibrated front-camera
far fan, not an arbitrary choice. A point outside the region has no locator
from the home pose and would be a guaranteed capability failure.

**Operator rule:** before every pair, place the cube at a new, haphazard point
inside the region. Do not reuse a spot, do not line the points up, keep
>= 30 mm from the previous realized point. The robot measures and records the
realized point; that measured point is the site.

## Plan

`paired_trials_plan.json` was generated before data with
`armpi_analysis.paired_trials.build_plan(pair_ids, fixtures=r01..r24,
seed=17)`:

- 24 pairs `p001`...`p024`, paired with `r01`...`r24`;
- the visit order is shuffled (first pair `p020` at `r20`);
- the method order inside each pair is shuffled independently;
- both methods of a pair are adjacent, so a pair runs back to back at the
  same realized point.

The plan is append-only. The trial records go through
`armpi_analysis.paired_trials.append_record`.

## Operator procedure

1. Run the pre-session checks in
   [`docs/phase3/RANDOM_SITE_DESIGN.md`](../../../docs/phase3/RANDOM_SITE_DESIGN.md)
   §9, starting with `--precheck` at the far region: (0.55, 0), (0.55, -20
   deg), (0.55, +25 deg). The front-camera far field must return a point at
   all three; if not, stop and revise the region before data.
2. Start the session:

```bash
sg docker -c "python3 tools/phase3/random_site_session.py \
    --region evidence/phase3/onsite_20261121_random/region.json \
    --plan evidence/phase3/onsite_20261121_random/paired_trials_plan.json \
    --trials evidence/phase3/onsite_20261121_random/trials.jsonl" \
    2>&1 | tee evidence/phase3/onsite_20261121_random/session_$(date +%Y%m%d_%H%M).log
```

The runner prints the pair and the region. Place the cube at a fresh random
point, press Enter, and repeat the placement on the same spot after the first
trial. The runner locates, drives the base into the band, runs both methods
and returns home.

If the runner aborts at a pair (no locator point, lost cube, final
measurement, envelope, or region), re-check the cube and re-run the same pair.
The attempt is in the session log and `approaches.jsonl` but writes no trial
record. A pair that fails after one retry is a capability failure; it is
reported, not converted into a `MISS`.

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

Record the capability summary (located/approached, retries, aborts, realized
point distribution) in this README, and add the outcome to
`docs/phase3/RANDOM_SITE_DESIGN.md`, the phase-3 README and the CHANGELOG. Do
not pool with the 2026-11-19 single-fixture trial or the superseded fixed-fan
design.
