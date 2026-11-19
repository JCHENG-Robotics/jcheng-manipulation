# Paired hardware trial — 2026-11-19 on-site session

> Status: **in progress.** The design is pre-registered in
> [`docs/phase3/PAIRED_TRIAL_DESIGN.md`](../../../docs/phase3/PAIRED_TRIAL_DESIGN.md);
> the randomization plan was committed before data collection (`4871e70`).

## Files

| File | Content |
|---|---|
| `paired_trials_plan.json` | 24-pair plan, seed 7, fixture `floor_square_A` (committed before data) |
| `trials.jsonl` | append-only records from the session runner |
| `discarded_unit_error_attempt.jsonl` | the first attempt, discarded because of a runner unit error (see design note D1) |
| `seg_*.json`, `grasp_*.json`, `release_*.json` | per-trial container reports (copied from `/tmp` after the session) |

## Protocol

See the design note. Per trial: measure the cube at the view pose, fit the
deep-family `joint1`/`joint4`, grasp, check the floor, release a held cube,
measure the landing, append the record.

## Session commands

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
sg docker -c "python3 tools/phase3/paired_trial_session.py \
  --plan evidence/phase3/onsite_20261119/paired_trials_plan.json \
  --trials evidence/phase3/onsite_20261119/trials.jsonl"
```

Analysis:

```bash
PYTHONPATH=ros2_ws/src/armpi_analysis python3 -m armpi_analysis.paired_trials \
  analyze --input evidence/phase3/onsite_20261119/trials.jsonl \
  --json-out evidence/phase3/onsite_20261119/analysis.json \
  --markdown-out evidence/phase3/onsite_20261119/analysis.md
```
