# Full G8 acceptance — 2026-11-23

Scope: one supervised, end-to-end G8 pick-and-place on the real ArmPi Ultra,
including one bounded open-loop chassis approach. This is the hardware
acceptance run for the `v1.0.0` blocker **full G8 task sequence**.

## Setup

- Backend `1.0.0-rc.1`, hardware mode, pose scope, DB
  `/tmp/armpi_studio_hardware_20261123.db`.
- 8765 read-only gateway and 8766 pose/motion gateway running on
  `10.42.0.218`; the motion gateway owns
  `/depth_cam/depth_registered/points`.
- Real `armpi_safety` node running from `/home/ubuntu/shared/safety_ws`.
- Workspace cleared; the arm was supported while it was placed at the view
  pose. The cube started at `0.304 m`, outside the validated
  `0.220–0.255 m` approach band, so the run was allowed to perform **one**
  bounded chassis pulse.

## Command

```bash
python3 tools/studio/g8_task_run.py \
  --backend http://127.0.0.1:8000 \
  --session g8-acceptance-20261123 \
  --out evidence/g8_acceptance_20261123/g8_run.json \
  --execute --i-have-clear-workspace
```

## Result

- Planner: `VALID`, no errors. Locate point
  `[0.2317, 0.0030, -0.0497] m`; corrected target
  `[0.2454, 0.0069, -0.0408] m`; constant correction
  `[+13.7, +3.9, +8.9] mm`; FK xy error `0.000003 mm`.
- One chassis pulse: `vx 0.985598`, `vy -0.169103`, duty `10.0`,
  `1.536 s`, nominal travel `75.3 mm`. Re-plan after the pulse:
  `VALID`.
- 11 motion commands across the 8 phases, all terminal `Completed` with
  `verification: bus` (gateway read back the actual bus registers within
  ±12 pulses; gripper commands are timed but their final positions were
  also read back).

| Phase | Command | Target pulses | Actual bus pulses | Deviation |
|---|---|---|---|---|
| open_gripper | open_gripper | r 170 | r 184 | timed |
| approach | approach | 509/342/930/500/500 | 504/339/929/495/500 | -5/-3/-1/-5/0 |
| grasp | descend | 509/342/930/466/500 | 505/338/929/469/500 | -4/-4/-1/+3/0 |
| grasp | grasp | 509/342/930/430/500 | 504/339/929/433/500 | -5/-3/-1/+3/0 |
| close | close_pre | r 318 | r 306 | timed |
| close | close_grip | r 550 | r 426 | timed; the cube stopped the jaws |
| lift | lift | 500/561/784/120/496 | 504/555/793/119/497 | +4/-6/+9/-1/+1 |
| release | release_descend | 509/342/930/466/500 | 506/338/932/460/500 | -3/-4/+2/-6/0 |
| release | release_final | 509/342/930/430/500 | 505/339/930/434/499 | -4/-3/0/+4/-1 |
| release | release_open | r 170 | r 226 | timed |
| return_view | return_view | 500/561/784/120/496 | 505/558/792/120/497 | +5/-3/+8/0/+1 |

- The gripper closed on the cube at `426` and opened to `226` after the
  put-back; `close_grip` is intentionally timed, so the `550` target is not
  expected to be reached while an object is between the jaws.
- `/safety_stop` stayed `false`; the real safety node reported
  `valid=true`, `stop=false`, `stop_latched=false`, no violations, and its
  last validation approved.

## Grip verification and put-back

`verify.held=true`, `verified=true`, `clusters=1`. The only segmented
cluster was **outside** the cube box at the view
(`x 0.478–0.557 m`, `y -0.252…-0.187 m`) and was rejected as
`y_out_of_region`, so the segmentation report has `success=false`. This is
the expected empty-box result while the cube is held: `held=true` here means
"no cube cluster inside the box", not a positive observation of the cube in
the gripper.

After the run, a read-only re-plan (`g8_postcheck.json`) found the cube back
at `[0.236019, 0.002949, -0.050847] m`, inside the `0.220–0.255 m` band,
with `VALID` and no warnings. That independently confirms the put-back
landing; the G8 runner itself does not score the final placement.

## Post-run note

At `2026-11-23T01:42:04Z`, a later 1.0 s view-pose command from another
session (`1c1bdca1-5434-4af4-b8a4-d80d3ba6d176`) was dispatched outside this
scripted run. It completed bus-verified at `504/558/792/120/497`. The G8
phase evidence ends at `01:39:27Z` and is unaffected.

## Evidence files

- `g8_run.json` — full runner report: plan, approach response, previews,
  dispatches, grip verification.
- `g8_postcheck.json` — read-only re-plan and cube location after the run.
- `runtime_snapshot.json` — live gateway health, last bus state and real
  safety status captured after the run.
- `safety_status.txt` — raw `/safety_status` and `/safety_stop` echo from the
  real `armpi_safety` node.
- `acceptance.json` — compact machine-readable acceptance record, including
  the terminal event for every command.

## Limitations

- One supervised run on one cube and one fixture; not a statistical sample.
- The chassis approach is open-loop (no odometry); the 75.3 mm nominal pulse
  is not a measured displacement.
- The gripper is timed, not position-servoed; bus readback is evidence of
  where it stopped, not a force or grasp-quality measurement.
- No e-stop, brake, self-collision model or independent encoder; the arm was
  supported during setup and the workspace was clear.
- The real safety node is a manual process in `/home/ubuntu/shared/safety_ws`,
  not a systemd unit.
