# Persistent gateway service — lifecycle evidence (2026-11-17)

> Status: **systemd service installed, enabled and stop/start verified.** A full
> reboot or vendor-container restart was not performed; both trigger the vendor
> `startup_check`, which moves the arm. The operator deferred them.

## Service

| Item | Value |
|---|---|
| Unit | `armpi-gateway.service` (`/etc/systemd/system/`) |
| Runner | `/usr/local/lib/armpi-gateway/armpi-gateway-run.sh` |
| Environment | `/etc/armpi-gateway/gateway.env` |
| Bind | `10.42.0.218:8765` |
| Token | `/home/pi/docker/tmp/armpi_gateway_deploy/token` (mode 0600, uid 1000; not in this repository) |
| Container | `ArmPiUltra`, `running=true restart=always` |

## Results

Raw log: [`service_lifecycle_20261117.txt`](service_lifecycle_20261117.txt).

```text
enabled=yes
active before stop=active
GET /health (token)            200, ros/battery/camera/joints ready, motion_enabled=false
GET /battery (token)           7424 mV, 51.2%, measured/valid, age ~45 ms
GET /camera/status (token)     640x400 bgr8, ~6.6-6.8 fps, measured/valid
GET /health (no token)         401
systemctl stop                 clean SIGTERM: "received signal 15, shutting down"
after stop                     inactive; 0 listening sockets on :8765; 0 in-container gateway processes
systemctl start                active; GET /health 200; 1 in-container gateway process
```

The stop sequence also logs `Deactivated successfully` and
`Stopped armpi-gateway.service`. The gateway's final `stopped` line is not
always flushed through `docker exec`; the SIGTERM line and the process/port
checks are the authoritative evidence.

## Backend hardware-mode re-check

Raw log:
[`backend_hardware_20261117.txt`](backend_hardware_20261117.txt).

A backend was started with `ARMPI_MODE=hardware`,
`ARMPI_GATEWAY_URL=http://10.42.0.218:8765`, a temporary SQLite database and the
service token (port 8010, stopped afterwards):

```text
GET /api/health         200; mode=hardware; source_label=LIVE; hardware_motion=false; hardware_read=true
GET /api/peripherals    200; battery 7.424 V measured/valid; camera 640x400 measured/valid
GET /api/connection     200; driver/gateway/feedback online
GET /api/safety         200; blockers mapping_verified, hardware_motion_enabled
POST /api/control/acquire  200
POST /api/motion/preview   200; valid, token issued
POST /api/motion/execute   403 GATE_REJECTED; same two blockers
POST /api/control/release  200
```

No motion was dispatched. The gateway has no motion endpoint, and the backend
rejected execute before any adapter dispatch.

## What was not tested

- A host reboot (`systemctl is-enabled` is `yes`; the unit waits for
  `docker.service` and for the container).
- A vendor-container restart (`Restart=always` should restart the unit; the
  vendor startup check moves the arm).
- Token rotation (`deploy/rotate-token.sh` was not exercised).

## Files

| File | Content |
|---|---|
| `service_lifecycle_20261117.txt` | status, health/battery/camera payloads, 401, stop/start, process and port counts, journal excerpts |
| `backend_hardware_20261117.txt` | hardware-mode backend health/peripherals/connection/safety, preview, `GATE_REJECTED` execute, lease release |
