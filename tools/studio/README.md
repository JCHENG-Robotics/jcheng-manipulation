# Start the ArmPi Debug Studio backend

`start_studio.sh` wraps `uvicorn` with the four checks that broke real runs
before (2026-11-17/18): a stale backend holding port 8000, a rotated gateway
token, a manual camera launch that had silently died, and the literal
`<token>` placeholder left in an exported variable.

It does not move the arm by itself. By default hardware motion stays disabled;
`execute` is rejected with `GATE_REJECTED`. The explicit `--takeover` option
starts the G7/G8 pose scope against a separately started pose gateway (see
below).

## Use

```bash
# hardware read-only (default; the robot is reachable at 10.42.0.218 over the
# direct Ethernet cable — office WiFi is outbound-only since 2026-11-18)
tools/studio/start_studio.sh

# mock only, no robot access
tools/studio/start_studio.sh --mock

# do not touch the camera launch even if it is down
tools/studio/start_studio.sh --no-camera

# G7 pose takeover against the pose gateway on 8766 (operator present)
ARMPI_TAKEOVER_ACK=I_AM_AT_THE_ROBOT_WITH_A_CLEAR_WORKSPACE \
  GATEWAY_PORT=8766 tools/studio/start_studio.sh --takeover
```

Then open <http://127.0.0.1:8000>. The script prints the backend PID; stop it
with `kill <pid>`.

## Pose takeover (G7)

`--takeover` exports `ARMPI_HARDWARE_MOTION=1`, `ARMPI_MOTION_SCOPE=pose` and
the acknowledgement, defaults the gateway to **8766**, and refuses to start
without the fixed phrase. The pose gateway must already be running with a
pose-mode motion contract; the exact procedure is
[`armpi-debug-studio/ros2_gateway/deploy/takeover.md`](../../armpi-debug-studio/ros2_gateway/deploy/takeover.md).

The Joint Control page then renders the **Pose Takeover** panel: preset library,
five-joint editor, FK/Joint preview, gripper open/close, lease, confirm modal,
hold request. The continuous joint stream is `truth=commanded` (the vendor
command cache); the command status card shows the **bus-verified actual**
registers after each move. The motion gateway must be started with the vendor
zsh environment (`source ~/.zshrc`, CycloneDDS) so `GetBusServoState` is
reachable; its config carries `verification=true`.

No-motion verification of a running pose backend (never calls execute):

```bash
python3 tools/studio/verify_takeover.py \
  --backend http://127.0.0.1:8010 \
  --gateway http://10.42.0.218:8766 \
  --out /tmp/g7_no_motion.json
```

The staged real sequence (preview first; `--execute` is the only motion path):

```bash
python3 tools/studio/g7_first_motion.py \
  --backend http://127.0.0.1:8000 \
  --session operator-onsite \
  --out /tmp/g7_first_motion.json \
  --execute --i-have-clear-workspace --steps view_table,grasp_low,view_table_return
```

It aborts on any rejected preview, `Failed` or `Unknown`; it never retries.

It checks the pose scope, measured joints and gripper, the 5 presets, one valid
preview, the partial-set / velocity / low-yaw / gripper rejections, and that
the measured pulses are unchanged. Evidence from the first run:
[`evidence/studio_g7/20261121/`](../../evidence/studio_g7/20261121/README.md).

## Task takeover (G8)

With the arm at the `view_table` preset and the pose gateway started with
`--pointcloud-topic /depth_cam/depth_registered/points`, one command plans the
**7-phase fast task** from a live registered cloud and runs it phase by phase.
Each pose phase is one dense S-curve (4.0 rad/s / 20 rad/s² free moves,
0.6/0.35 rad/s grasp descents, 1.4/0.8 rad/s release descents); each grip
phase is one timed raw bus-servo command (0.35/0.5/0.45 s):

```bash
# plan only (read-only: capture + locate + deep-family fit + validation)
python3 tools/studio/g8_task_run.py --backend http://127.0.0.1:8000 \
  --out /tmp/g8_plan.json

# execute (operator beside the robot; every phase bus-verified)
python3 tools/studio/g8_task_run.py --backend http://127.0.0.1:8000 \
  --session operator-g8 --out /tmp/g8_run.json \
  --execute --i-have-clear-workspace
```

The plan is refused unless the arm is within 0.12 rad of the view and every
command passes the joint limits, the profile velocity cap and the FK gate. A
`TOP_CLIPPED_RECOVERED` warning means the Aurora's min-range sphere (~0.25 m)
hid the near part of the top face and the centre was rebuilt from the intact
far edge with the measured 39.4 mm cube. The runner aborts on a rejected
preview or a non-`Completed` event and never retries. Evidence (the older
13-step plan): [`evidence/studio_g8/20261121/`](../../evidence/studio_g8/20261121/README.md).

### Out-of-band cube (G8.1)

If "定位并规划" answers `APPROACH_REQUIRED`, the run still starts: it pulses
the base toward `(0.230, 0)` (up to three bounded pulses, each followed by a
fresh cloud and a new plan) and continues with the arm phases automatically.
The standalone "底盘接近" button is gone, and
`tools/studio/g8_task_run.py --execute` performs the same loop. The G8.1
runner is still available for a base-only alignment:

```bash
# plan only (no motion): reports whether an approach is needed
python3 tools/studio/g8_approach.py --backend http://127.0.0.1:8000 \
  --out /tmp/g8_approach.json

# pulse until aligned (operator beside the robot, arm at the view)
python3 tools/studio/g8_approach.py --backend http://127.0.0.1:8000 \
  --session operator-g81 --out /tmp/g8_approach.json --execute \
  --i-have-clear-workspace
```

The base is open-loop: the pulse response is `verification: open_loop` and the
camera re-measurement is the only confirmation. Evidence:
[`evidence/studio_g81/20261121/`](../../evidence/studio_g81/20261121/README.md).

## What it does

1. **Port** — if `:8000` is busy it kills the holder only when its command line
   contains `app.main:create_app`; anything else is reported and the script
   exits instead of killing an unrelated process.
2. **Token** — reads `/home/pi/docker/tmp/armpi_gateway_deploy/token` from the
   robot over SSH (`ARMPI_GATEWAY_TOKEN` overrides this). Fails loudly rather
   than starting with a placeholder, because a wrong token turns both the
   camera and battery panels into `Unavailable` with
   `gateway returned HTTP 401`.
3. **Gateway** — probes `/health` with `X-Armpi-Token` and requires
   `status=ok` and `ros_ready=true`.
4. **Camera** — when `camera_ready=false`, starts the persistent
   `vision.launch.py` with the `docker exec -d ... exec` form from
   [`docs/hardware/ros_environment.md`](../../docs/hardware/ros_environment.md)
   and polls `/camera/status` for up to 40 s. A camera that stays down only
   warns: battery and joints keep working.
5. **Backend** — starts it detached, logs to `/tmp/studio_studio_8000.log`,
   waits for `/api/health`, then prints `mode`, battery voltage and camera
   geometry with their `truth=` labels.

`source_label=LIVE` in the header is only the mode label. Data is really live
only when each panel says `truth=measured`.

## Overrides

| Variable | Default |
|---|---|
| `PORT` | `8000` |
| `HOST` | `127.0.0.1` |
| `ROBOT_HOST` | `10.42.0.218` (gateway HTTP + SSH over the direct Ethernet cable; the same link carries DDS) |
| `GATEWAY_PORT` | `8765` |
| `SSH_KEY` | `~/.ssh/id_ed25519_armpi` |
| `SSH_USER` | `pi` |
| `CONTAINER` | `ArmPiUltra` |
| `ARMPI_GATEWAY_TOKEN` | read from the robot |
| `DB_PATH` | `/tmp/armpi_studio_hardware_<date>.db` |
| `LOG` | `/tmp/studio_studio_8000.log` |

Binding stays on `127.0.0.1`. The backend has no authentication; reach it from
another device with an SSH tunnel (`ssh -L 8000:127.0.0.1:8000 pi@10.42.0.218`),
never by binding `0.0.0.0`. The tunnel must ride the direct Ethernet link; a
WiFi client cannot reach the robot's SSH port anymore.
