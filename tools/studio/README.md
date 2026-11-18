# Start the ArmPi Debug Studio backend

`start_studio.sh` wraps `uvicorn` with the four checks that broke real runs
before (2026-11-17/18): a stale backend holding port 8000, a rotated gateway
token, a manual camera launch that had silently died, and the literal
`<token>` placeholder left in an exported variable.

It does not move the arm. Hardware motion stays disabled; `execute` is still
rejected with `GATE_REJECTED`.

## Use

```bash
# hardware read-only (default; needs the robot reachable at 10.51.128.226)
tools/studio/start_studio.sh

# mock only, no robot access
tools/studio/start_studio.sh --mock

# do not touch the camera launch even if it is down
tools/studio/start_studio.sh --no-camera
```

Then open <http://127.0.0.1:8000>. The script prints the backend PID; stop it
with `kill <pid>`.

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
| `ROBOT_HOST` | `10.51.128.226` (gateway HTTP + SSH, office WiFi); the DDS link is the wired `eth0 10.42.0.218` |
| `GATEWAY_PORT` | `8765` |
| `SSH_KEY` | `~/.ssh/id_ed25519_armpi` |
| `SSH_USER` | `pi` |
| `CONTAINER` | `ArmPiUltra` |
| `ARMPI_GATEWAY_TOKEN` | read from the robot |
| `DB_PATH` | `/tmp/armpi_studio_hardware_<date>.db` |
| `LOG` | `/tmp/studio_studio_8000.log` |

Binding stays on `127.0.0.1`. The backend has no authentication; reach it from
another device with an SSH tunnel (`ssh -L 8000:127.0.0.1:8000 pi@10.51.128.226`),
never by binding `0.0.0.0`.
