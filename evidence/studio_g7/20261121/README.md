# G7 pose takeover — hardware verification (2026-11-21)

Status: **PASS with real motion and bus-servo verification.** A full sequence
ran on the robot — `view_table → grasp_low → view_table`, then gripper
open/close — and every arm completion is a `GetBusServoState` read of the
actual registers, not the vendor command cache.

> **Key discovery.** `/controller_manager/servo_states` is **not** measured
> feedback. The vendor `ServoManager.set_position` writes the commanded pulse
> into the cache that both `/joint_states` and `/servo_states` publish. A
> cache-based `Completed` appeared ~0.12 s after dispatch even for a 2.5 s
> interpolation. The actual registers are only available through
> `GetBusServoState`, and they differ from the cache by up to 8 pulses; the
> settled gripper close is ~645 while the cache says 700. This affects G6's
> cache-based completion and G5's "measured feedback" label.

## Setup

| Item | Value |
|---|---|
| Robot | ArmPi Ultra at `10.42.0.218`, vendor Humble stack in `ArmPiUltra` |
| Read-only gateway | `8765`, left running and untouched |
| Pose gateway | a separate deploy dir on `8766`, `--enable-motion` with the pose contract, `verification=true`, started with **`/bin/zsh -lc 'source ~/.zshrc'`** (CycloneDDS) so `GetBusServoState` is reachable |
| Pose contract | `configs/takeover.yaml`; verification ±12 pulses, settle 0.5 s, read margin 25 |
| Studio backend | laptop `127.0.0.1:8000`, hardware mode, `ARMPI_MOTION_SCOPE=pose`, the fixed acknowledgement |
| Battery | 7.572 V before the runs |

The pose gateway was started from `/home/pi/docker/tmp/armpi_gateway_pose`
(removed after the session); the live read-only deployment was not restarted
during the motion runs. **After** the verification, the live 8765 package was
updated to the same code (still read-only) so its `/state` also labels the
cache `truth=commanded`; its unit's `ExecStop` now targets only `--port 8765`
(with `pkill --`) so restarting the read-only service does not kill the pose
instance on 8766.

## First run (cache completion) — the discovery

The first sequence used the original cache-based completion. The arm physically
performed the sequence (operator: "正常下探并返回，夹爪紧闭"), but every
`Completed` arrived ~0.12 s after dispatch:

```text
view_table      dispatched 08:31:58.939  observed 08:31:59.066  (0.13 s for a 1.5 s move)
grasp_low       dispatched 08:31:59.471  observed 08:31:59.579  (0.11 s for a 2.5 s move)
view_table      dispatched 08:32:00.008  observed 08:32:00.144  (0.14 s for a 2.5 s move)
```

A direct bus read after the run gave `joint1…joint5 = 501/555/792/123/496`,
`r_joint = 645`, while `/servo_states` reported `500/561/784/120/496` and
`r_joint = 700`. The arm had reached the pose, but the completion evidence was
the command cache.

Raw: [`first_motion_cache_completion.json`](first_motion_cache_completion.json)
(superseded; kept as the discovery record).

## Verified run (bus completion)

The gateway now waits `duration + 0.5 s` (1.2 s for a timed gripper), reads the
actual registers with `GetBusServoState`, and accepts an arm command only within
±12 pulses. The sequence was re-run:

| Step | Duration | Actual bus registers | Δ vs target (pulse) | Dispatch → Completed |
|---|---:|---|---:|---:|
| `view_table` | 1.5 s | 502 / 556 / 792 / 122 / 497 | +2 / −5 / +8 / +2 / +1 | 2.10 s |
| `grasp_low` | 2.5 s | **502 / 339 / 928 / 336 / 499** | +2 / −3 / −2 / −4 / −1 | 3.06 s |
| `view_table` return | 2.5 s | 502 / 555 / 792 / 122 / 497 | +2 / −6 / +8 / +2 / +1 | 3.14 s |

The arm reached the low grasp pose within **4 pulses (0.017 rad)**. The low-pose
actual (`339/928/336`) is the physical evidence the previous "measured"
completion lacked.

Raw: [`first_motion_verified.json`](first_motion_verified.json) (arm poses;
its gripper events were recorded before the timed-settle fix and are
superseded). A bus-verified hold (no motion) is in
[`hold_verified.json`](hold_verified.json).

### Gripper re-run (timed settle fix)

The first verified sequence read the gripper at `duration + 0.5 s` (the
transient; open 183, close 700). The gateway now waits 1.2 s for a timed
gripper, so the **settled** register is recorded:

| Step | Duration | Actual register | Dispatch → Completed |
|---|---:|---|---:|
| gripper open | 5.0 s timed | r_joint **184** (command 170) | 6.2 s |
| gripper close | 5.0 s timed | r_joint **645** (command 700) | 6.2 s |

Raw: [`gripper_verified.json`](gripper_verified.json). A direct read
immediately after the session:
`{"1": 646, "2": 497, "3": 122, "4": 792, "5": 555, "6": 501}` — the settled
close is 646, confirming the timed read's 645. These match the 2026-11-18
settled stops (open ~167–184, close ~645), not the command cache's 700.

## No-motion verification (final code)

`tools/studio/verify_takeover.py` against the final pose-scope backend and
gateway: **17/17 PASS**, measured pulses unchanged (delta 0).

Raw: [`no_motion_verification.json`](no_motion_verification.json).

```text
gateway_pose_mode          motion_enabled=true, motion_mode=pose, measured_ready=true
gateway_idle_contract      state=idle|completed, mode=pose, velocity<=0.5 rad/s
gateway_gripper_contract   r_joint 170..700, completion=timed, measured cache
health_pose_scope          motion_scope=pose, gripper_control=true, pose_gate=true
measured_arm_joints        five joints, truth=commanded (vendor cache)
measured_gripper           r_joint available, truth=commanded
safety_ready               ready=true, blockers=[]
pose_takeover_visible      supervisor exposes the pose scope
presets_available          5 presets, all limit_ok, gate_error=null
valid_preset_preview       grasp_low, pose.ok=true, clearance 34.8 mm
partial_joint_set_rejected POSE_JOINT_SET
low_pose_yaw_rejected      LOW_POSE_YAW at sample 9 (tcp_z=0.049, j1=0.225)
velocity_cap_rejected      VELOCITY_EXCEEDED (joint4 1.030 rad/s at 1.0 s)
gripper_preview_valid      r_joint close, 5.0 s, completion=timed
gripper_speed_rejected     VELOCITY_EXCEEDED (state-independent far endpoint)
gripper_range_rejected     GRIPPER_RANGE (+0.5 rad)
no_motion_dispatched       pulses before == after (max delta 0)
```

## How it works

1. **Pose gateway** — `mode=pose`, velocity 0.5 rad/s, five-joint-or-gripper,
   `verification=true`, started with the vendor zsh environment (CycloneDDS)
   so the `GetBusServoState` service is visible. The gateway refuses to start if
   the service is missing.
2. **Dispatch** — one `ServosPosition` with `position_unit=pulse` and the
   duration; the vendor controller interpolates.
3. **Verification timer** — `duration + 0.5 s` (or `+1.2 s` for a timed
   gripper), one guarded bus read. Joints whose last pulse is below 25 counts
   are not read (a negative register crashes the vendor node); an arm command
   that cannot be read becomes `Unknown`, never a false `Completed`.
4. **Backend** — `Completed` for an arm requires `verification="bus"` and
   carries `actual`, `deviations_pulses`; a cache-only completion is reported
   as `Unknown` and latches. The continuous `/api/state` position is labelled
   `commanded`; a fresh bus read is `measured_on_demand` for ~500 ms.
5. **Gripper** — timed completion (5 s) plus the settled bus register; no force
   claim.

## Commands

```bash
# generate the verified pose contract
cd armpi-debug-studio
backend/.venv/bin/python tools/export_gateway_motion_config.py \
  --mode pose --out /tmp/pose_limits_verified.json --overwrite

# copy the package + JSON to a separate Pi dir, then start on 8766 with the
# vendor zsh environment (see ros2_gateway/deploy/takeover.md for the exact
# docker exec command); the log must say verification=True bus=...

# backend
ARMPI_TAKEOVER_ACK=I_AM_AT_THE_ROBOT_WITH_A_CLEAR_WORKSPACE \
  tools/studio/start_studio.sh --takeover

# staged real-motion run (preview first; execute with the flag)
python3 tools/studio/g7_first_motion.py \
  --backend http://127.0.0.1:8000 \
  --session operator-onsite-20261121 \
  --out evidence/studio_g7/20261121/first_motion_verified.json \
  --execute --i-have-clear-workspace
```

## What this proves / does not prove

- Proven: the Studio can command the arm to a five-joint pose and the gripper,
  and every arm completion is a real bus-servo read; the arm reached `grasp_low`
  within 4 pulses; the gripper settled registers are 184 (open) and 645
  (close); the refusal paths (partial pose, low-pose yaw, velocity, gripper
  range/speed) fire.
- Not proven: self-collision freedom (the gate is TCP-only and nominal), grip
  force, table calibration after a base move, or continuous measured feedback
  (the actual is a one-shot read; the continuous stream is the command cache).
- The G5 dataset's `feedback` is the same cache; its tracking metrics are
  plan-vs-command, not servo tracking.
