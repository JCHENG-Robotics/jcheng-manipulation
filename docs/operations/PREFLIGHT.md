# Hardware session preflight

Run this checklist before any supervised motion. It is written for the
current direct-Ethernet setup: laptop → `10.42.0.218`, vendor container
`ArmPiUltra`, read-only gateway `8765`, pose/motion gateway `8766`.

**Stop if any check fails.** Do not "try one move to see"; fix the link,
clock, cache or safety state first.

## 1. Physical

- Workspace clear; cube in the validated `0.220–0.255 m` band if a G8 run is
  planned, or the base approach is explicitly approved.
- Arm supported while it is placed at the view pose; no e-stop or brake
  exists.
- No cable strain at the wrist; the wrist camera is not touching the cube.
- Laptop and robot on the wired link (`eth0`), not Wi-Fi.

## 2. Link and clocks

```bash
ping -c 3 10.42.0.218
ssh -o IdentitiesOnly=yes -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 'hostname; uptime'
systemctl --user status armpi-clock-sync.timer --no-pager
```

The Pi must be reachable and the clock-sync timer active. A Pi clock behind
the laptop produces `CONTROL_FAILED` with no movement.

## 3. Containers and gateways

```bash
ssh -o IdentitiesOnly=yes -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'docker ps --format "{{.Names}} {{.Status}}"'
```

`ArmPiUltra` must be `Up`. Then, without printing the token:

```bash
TOKEN=$(cat /home/pi/docker/tmp/armpi_gateway_deploy/token)  # on the Pi
# or read ARMPI_GATEWAY_TOKEN from the backend environment on the laptop
curl -fsS -H "X-Armpi-Token: $TOKEN" http://10.42.0.218:8765/health
curl -fsS -H "X-Armpi-Token: $TOKEN" http://10.42.0.218:8766/health
curl -fsS http://127.0.0.1:8000/api/health
```

Required:

- `8765`/`8766` `status: ok`, `ros_ready: true`, `joints_ready: true`.
- `8766` `motion_state` is `completed` or `idle`, not an error.
- `safety_stop: false`.
- Backend `mode: hardware`, expected `version`, and the intended
  `motion_scope`.
- `pointcloud_configured: true` for a G8 run; check `pointcloud_last_capture`
  is fresh.

Confirm the backend is loopback-only. The default has no token; a
non-loopback bind is refused at startup unless `ARMPI_BACKEND_TOKEN` is set:

```bash
ss -ltnp | grep ':8000'
# expected: 127.0.0.1:8000
```

If it shows `0.0.0.0:8000`, confirm the process has `ARMPI_BACKEND_TOKEN` and
use an SSH tunnel or TLS reverse proxy; do not expose it directly.

## 4. Safety node

```bash
ssh -o IdentitiesOnly=yes -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'docker exec ArmPiUltra bash -lc "source /opt/ros/humble/setup.bash; \
   timeout 6 ros2 topic echo --once --full-length /safety_status; \
   timeout 6 ros2 topic echo --once /safety_stop"'
```

Required: `state.valid: true`, `stop: false`, `stop_latched: false`, no
violations, and `/safety_stop: false`. If the node is not running, start it
from `/home/ubuntu/shared/safety_ws` (manual process) and repeat.

## 5. Bus cache vs actual

The vendor `/controller_manager/servo_states` is the **last command cache**,
not measured feedback. Read the actual bus registers in the vendor
environment before a hold or a task:

- If the cache and actual differ, do **not** publish a hold at the cache;
  resync to the measured pulses as in
  [`evidence/studio_validation_20261123/README.md`](../../evidence/studio_validation_20261123/README.md).
- Never read a negative servo register; the vendor node crashes on a
  negative read.
- The gripper is servo ID 1; the arm joints are IDs 6..2.

```bash
curl -fsS -H "X-Armpi-Token: $TOKEN" http://10.42.0.218:8766/motion/status
```

`actual` is the bus read; `measured` is the cache.

## 6. Resources

```bash
ssh -o IdentitiesOnly=yes -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'df -h /; free -h'
```

Keep at least ~1 GiB free on `/`. A full root disk or stale shared-memory
segments can prevent the camera stack from starting.

## 7. Scope confirmation

- Normal Studio path: G6 `step` or G7/G8 `pose`; confirm the backend
  `motion_scope` and that the operator UI is the intended one.
- `profiles.fast` is an explicit 4.0/20 tier with bus verification; do not
  use it for a first pass or an unverified pose.
- Any chassis pulse invalidates the table calibration; re-locate before the
  next grasp.
