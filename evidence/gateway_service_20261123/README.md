# Gateway service privilege drop — 2026-11-23

Scope: close the P1 item "the systemd wrapper runs the container entrypoint
as root" for the read-only gateway (**8765**). This did not touch the vendor
stack, the pose/motion gateway (**8766**), or the arm.

## Change

- `armpi-gateway.service` is now a template with
  `User=@ARMPI_SERVICE_USER@` / `Group=@ARMPI_SERVICE_GROUP@`.
- `install-gateway.sh` substitutes `pi` / `docker`, verifies that account can
  reach the Docker daemon, and installs `gateway.env` as `root:docker 0640`.
- The host wrapper still runs `docker exec`; the in-container gateway already
  ran as the vendor `ubuntu` user.

## Deployment

The current live service was backed up first:

- `/tmp/armpi-gateway.service.root.bak`
- `/tmp/gateway.env.root.bak`

Then the installer was run from
`/home/pi/docker/tmp/armpi_gateway_pkg_new/ros2_gateway/deploy/install-gateway.sh`
as root. It restarted only `armpi-gateway.service`.

## Result

- `systemctl show armpi-gateway -p User -p Group` → `User=pi`, `Group=docker`.
- `ActiveState=active`, `SubState=running`; the `docker exec` process runs as
  `pi`.
- `/etc/armpi-gateway/gateway.env` is `root:docker 0640`.
- 8765 `/health`: `status=ok`, `ros_ready=true`, `camera_ready=true`,
  `aux_camera_ready=true`, `joints_ready=true`, `battery_ready=true`.
- 8766 `/health`: `status=ok`, `motion_state=completed`,
  `safety_stop=false` (unchanged).
- Backend `/api/health`: `status=ok`, `mode=hardware`,
  `version=1.0.0-rc.1`.

No arm or chassis command was sent; `/safety_stop` stayed false.

## Residual

- The Studio backend still has **no authentication**; it is localhost-only
  by policy. The preflight page checks `ss -ltnp` and recommends an SSH
  tunnel for remote access. This is an accepted rc.1 limitation.
- The 8766 pose/motion gateway is still a manual process, not a systemd unit.
- The root-unit backups are in `/tmp` on the Pi and disappear on reboot; the
  repo files are the source of truth.

## Evidence files

- `runtime.txt` — captured systemd identity, env permissions and the three
  health responses.
- `acceptance.json` — compact record.
