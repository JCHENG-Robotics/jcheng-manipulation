# 24-site paired trial — pre-registered artifacts

> Status: **pre-registered on 2026-11-20; no trial has been run.** These files
> fix the sites, the randomization and the analysis before data collection.
> Do not edit them after the first valid record; use
> `docs/phase3/SITE_DIVERSITY_DESIGN.md` §10 (Deviations) instead.

This directory is the evidence root of the follow-up to the 2026-11-19
single-fixture paired trial. The design, questions, outcomes, envelope guard,
safety rules and analysis are in
[`docs/phase3/SITE_DIVERSITY_DESIGN.md`](../../../docs/phase3/SITE_DIVERSITY_DESIGN.md).

## Files

| File | SHA-256 | Contents |
|---|---|---|
| `site_layout.json` | `70b2cce8aaa7010b89de3b2e5e70743a683a04855ebaab6331feb2b4554255f5` | 24 sites: 4 radii x 6 bearings, base-frame coordinates, envelope |
| `site_layout.md` | `9f232cb12decbe1232a111d84d08a8dc272cb9b4b5b4b665d6832e82890409ad` | Operator floor-taping table (24 marks) |
| `paired_trials_plan.json` | `5e9d128bd44048f2cf0f65d15b9b54c884cbaa6f1412bdcea804ecb489b882f8` | Seeded plan (seed 17): pair order, within-pair method order, one site per pair |
| `trials.jsonl` | — | Created on the first valid trial; append-only |
| `approaches.jsonl` | — | One locate/approach/return record per attempted pair (initial sighting, base pulses, final envelope, home return, abort reason) |
| `site_report.md` | — | Capability/outcome table rendered from the two JSONL files |
| `analysis.json`, `analysis.md` | — | Created by the paired analysis after the session |
| `session_*.log` | — | Operator terminal log, including any aborted locate/approach attempt |

The hashes are the pre-registration. If a layout or plan file is regenerated,
its `created_at` field changes and the hash must be updated before the first
trial; after the first trial, a regeneration is a deviation.

## Sites

The operator sheet is [`site_layout.md`](site_layout.md). The 24 marks are a
fan read from the validated view pose:

```text
radii    0.30, 0.35, 0.40, 0.45 m
bearings -10, -4, +2, +8, +14, +20 deg   (base +y is left)
```

Sites are radius-major: `s01` = (0.30 m, −10 deg) ... `s24` = (0.45 m,
+20 deg). The bearings are asymmetric because the fan is centred on the front
D435i's optical axis (`base_link` y = +0.027 m, yaw −1.1 deg); the wrist also
sees every nominal site from the view pose. Adjacent sites at 0.30 m are
31 mm apart, so the marks are small dots and only one cube is on the floor.

## Plan

`paired_trials_plan.json` was generated with
`armpi_analysis.paired_trials.build_plan(pair_ids, fixtures=site_ids,
seed=17)` **before** any trial:

- 24 pairs `p001`...`p024`, paired with `s01`...`s24` in layout order;
- the visit order is shuffled (first pair `p020` at `s20`);
- the method order inside each pair is shuffled independently;
- both methods of a pair are adjacent, so a pair runs back to back at one
  site.

The plan is append-only. The trial records go through
`armpi_analysis.paired_trials.append_record`, which validates the schema and
refuses duplicate pair/method records.

## Procedure and operator prompts

Run the session from the laptop:

```bash
sg docker -c "python3 tools/phase3/site_diversity_session.py \
    --sites evidence/phase3/onsite_20261120_sites/site_layout.json \
    --plan evidence/phase3/onsite_20261120_sites/paired_trials_plan.json \
    --trials evidence/phase3/onsite_20261120_sites/trials.jsonl" \
    2>&1 | tee evidence/phase3/onsite_20261120_sites/session_$(date +%Y%m%d_%H%M).log
```

The runner prints the pair, site id, radius, bearing and mark `(x, y)`. The
operator moves the single cube to that mark, presses Enter, and repeats the
placement after the first trial of each pair. The runner locates the cube,
drives the base into the band, runs the pair's two methods and returns the
base home.

If the runner aborts at a site (no cube found, lost cube, failed final
measurement, envelope violation), the operator re-checks the cube and the
mark and re-runs the same pair. The attempt is visible in the session log but
writes no trial record. A site that fails after one retry is a capability
failure; it is reported in the session summary, not converted into a `MISS`.

## Pre-session checks (required)

1. Place the cube at `s21` and `s24`; confirm the view-pose wide measurement
   returns a point for both (`s21` is the tightest top-edge corner, `s24` the
   tightest side corner).
2. One short forward and one short left base pulse in the cleared workspace;
   confirm the cube moves toward the robot and the wrist measurement shifts
   in the expected direction; return with the reverse.
3. The cube is flat (39.3-39.6 mm), the marks are matte and low-saturation,
   and no other saturated object is inside 0.70 m.
4. The base origin is on the home mark and the forward axis on the taped
   line; record the initial view-pose measurement of a cube at `s09`.
5. Vendor stack, wrist camera, Humble container and SSH key work; record the
   battery voltage.

## After the session

```bash
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m \
    armpi_analysis.paired_trials analyze \
    --input evidence/phase3/onsite_20261120_sites/trials.jsonl \
    --json-out evidence/phase3/onsite_20261120_sites/analysis.json \
    --markdown-out evidence/phase3/onsite_20261120_sites/analysis.md

python3 tools/phase3/site_report.py \
    --approaches evidence/phase3/onsite_20261120_sites/approaches.jsonl \
    --trials evidence/phase3/onsite_20261120_sites/trials.jsonl \
    --markdown-out evidence/phase3/onsite_20261120_sites/site_report.md
```

Then record the capability summary (sites located/approached, retries,
capability failures) and the site table in this README, and add the outcome
to `docs/phase3/SITE_DIVERSITY_DESIGN.md` (results section) and the phase-3
README/CHANGELOG. Do not pool the result with the 2026-11-19 single-fixture
trial.
