# Supervised G8 smoke after 8766 unit + backend auth — 2026-11-23

Scope: re-run the G8 view→grasp→put-back after the two P1 changes — the 8766
systemd unit and the backend token-auth code. This is a smoke of the new
runtime, not a replacement for the earlier 8-phase acceptance evidence.

## Preflight

- Backend `1.0.0-rc.1`, hardware/pose, live on `127.0.0.1:8000`.
- 8765 and 8766 `status=ok`; 8766 `motion_enabled=true`,
  `motion_state=idle`, `pointcloud_configured=true`.
- Real safety node `valid=true`, `stop_latched=false`, no violations;
  `/safety_stop=false`.
- Read-only locate: point `0.2456 m`, corrected target `0.2593 m`, outside
  the `0.220–0.255 m` band (`APPROACH_REQUIRED`).

## Run

One bounded chassis pulse was approved by the standing "continue" instruction:

- `vx=0.999778`, `vy=0.021093`, duty `10.0`, `0.318 s`,
  `15.6 mm` nominal, `verification=open_loop`.
- Re-plan: `VALID`, point `0.228004 m`, corrected target `0.241704 m`.

The 7 phases / 10 commands all completed with `verification: bus`:

| Phase | Command | Actual bus pulses |
|---|---|---|
| approach | approach | 503/337/932/494/500 |
| grasp | descend | 504/337/932/460/500 |
| grasp | grasp | 503/337/932/426/500 |
| close | close_pre | r 308 |
| close | close_grip | r 424 |
| lift | lift | 503/556/794/118/497 |
| release | release_descend | 503/338/932/452/500 |
| release | release_final | 503/339/932/426/500 |
| release | release_open | r 173 |
| return_view | return_view | 502/558/792/119/497 |

`grip_verify`: `held=true`, `verified=true`, `clusters=0` (the cube box at the
view was empty). The gripper closed at `424` and released to `173`.

## Post-run check

A read-only re-plan after the run found the cube at `0.218571 m` with
corrected target `0.232271 m`; the plan is `VALID` and the corrected target
is inside `0.220–0.255 m`. The raw point is about 1.4 mm below the raw band
lower edge, but the controller uses the corrected target; the landing is
still inside the validated grasp band.

## Runtime after the smoke

- 8766 service: `User=pi`, `Group=docker`, `active`, `disabled` at boot.
- `/safety_stop=false`; 8765 `camera_ready=true`; backend healthy.
- Final bus: `joint1 502, joint2 558, joint3 792, joint4 119, joint5 497`
  (gripper last read 173).

## Files

- `g8_smoke.json` — full runner report.
- `g8_postcheck.json` — post-run read-only locate.
- `runtime.txt` — unit identity, safety and health after the run.
- `acceptance.json` — compact phases/actuals record.
