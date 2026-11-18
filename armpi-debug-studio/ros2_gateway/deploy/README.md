# Persistent gateway deployment (robot host)

> Deployed on the Pi 5 on 2026-11-17. Lifecycle evidence (enabled, health,
> 401, clean SIGTERM stop, start, port/process checks):
> [`evidence/studio_gateway/20261117/persistent/`](../../../evidence/studio_gateway/20261117/persistent/).
> A reboot/container-restart test was deferred because the vendor startup check
> moves the arm.

The read-only gateway runs **inside the vendor container** `ArmPiUltra`, whose
ROS 2 stack owns the camera/battery/joint topics. A small host-side systemd unit
starts it with `docker exec`, waits for the container after a reboot, restarts
it if it exits, and stops it with SIGTERM (the gateway shuts rclpy and HTTP down
cleanly).

```text
armpi-gateway.service (host systemd)
  └── armpi-gateway-run.sh
        └── docker exec -u ubuntu ArmPiUltra python3 -m armpi_debug_gateway.main
              └── /health /battery /camera/... /state /tf /motion/status
                    (read-only; motion_enabled=false)
```

The systemd service is **always read-only**. The supervised write path uses a
separate, manually started instance on a second port; see
[`supervised-motion.md`](supervised-motion.md). The installer does not enable
motion and does not install a motion limits file.

## Files

| File | Installed as |
|---|---|
| `armpi-gateway.service` | `/etc/systemd/system/armpi-gateway.service` |
| `armpi-gateway-run.sh` | `/usr/local/lib/armpi-gateway/armpi-gateway-run.sh` |
| `gateway.env.example` | `/etc/armpi-gateway/gateway.env` (once) |
| `install-gateway.sh` | run from the copied `deploy/` directory |
| `rotate-token.sh` | run from the copied `deploy/` directory |

## Install

Copy the package and the deploy directory to the robot, then run the installer
as root:

```bash
# from the laptop, repository root
scp -i ~/.ssh/id_ed25519_armpi -r \
  armpi-debug-studio/ros2_gateway \
  pi@10.51.128.226:/home/pi/docker/tmp/armpi_gateway_pkg

ssh -i ~/.ssh/id_ed25519_armpi pi@10.51.128.226 \
  'sudo bash /home/pi/docker/tmp/armpi_gateway_pkg/ros2_gateway/deploy/install-gateway.sh'
```

The installer:

1. copies `armpi_debug_gateway/` to
   `/home/pi/docker/tmp/armpi_gateway_deploy/` (container:
   `/home/ubuntu/shared/armpi_gateway_deploy/`);
2. creates or preserves the token file next to it with mode `0600` (owned by
   uid 1000 so the container user can read it), using `openssl rand -hex 24`;
3. installs the unit and `/etc/armpi-gateway/gateway.env` (first install only);
4. enables and starts the service, then polls `/health` for up to 60 s.

It does **not** start, stop or modify the vendor container or the ROS stack.

## Configure the bind address

Edit `/etc/armpi-gateway/gateway.env`, then restart:

```bash
sudoedit /etc/armpi-gateway/gateway.env   # ARMPI_GATEWAY_BIND=...
sudo systemctl restart armpi-gateway
```

Use the address the backend actually reaches (office WiFi is the current link
since 2026-11-18; see `docs/hardware/ros_environment.md`):

| Link | Bind |
|---|---|
| Office WiFi (current) | `10.51.128.226` |
| Direct Ethernet (fallback) | `10.42.0.218` |
| Tailscale | the tailnet address |

If the address does not exist on the robot, the gateway fails to bind, systemd
restarts it every 5 s, and `journalctl -u armpi-gateway` shows
`Cannot assign requested address`. Update the env file rather than binding
`0.0.0.0`.

## Operations

```bash
sudo systemctl status armpi-gateway
sudo journalctl -u armpi-gateway -f
sudo systemctl restart armpi-gateway
sudo systemctl stop armpi-gateway
```

Expected startup journal:

```text
armpi_debug_gateway listening on http://10.51.128.226:8765
  battery: /ros_robot_controller/battery
  camera:  /depth_cam/rgb/image_raw
  joints:  /controller_manager/joint_states
  measured:/controller_manager/servo_states
  tf:      /tf + /tf_static
  motion:  DISABLED (read-only gateway)
```

A clean stop logs `received signal 15, shutting down` followed by
`armpi_debug_gateway: stopped`. If `docker exec` does not forward the signal,
the unit's `ExecStop` runs `pkill -TERM` inside the container before the
timeout.

## Rotate the token

```bash
ssh -i ~/.ssh/id_ed25519_armpi pi@10.51.128.226 \
  'sudo bash /home/pi/docker/tmp/armpi_gateway_pkg/ros2_gateway/deploy/rotate-token.sh'
```

The new token is written to the shared token file and not printed. Update the
backend's `ARMPI_GATEWAY_TOKEN` from that file and restart the backend.

The token file is at `/home/pi/docker/tmp/armpi_gateway_deploy/token` on the
host and `/home/ubuntu/shared/armpi_gateway_deploy/token` in the container. It
is not in this repository.

## Reboot and container restart behaviour

- `Requires=docker.service` and the startup wait loop cover a host reboot.
  Verify the vendor container itself starts on boot (its own restart policy or
  vendor service); the gateway waits up to
  `ARMPI_GATEWAY_CONTAINER_WAIT_S` (default 60 s) and then fails/restarts.
- If the container restarts, `docker exec` exits and systemd restarts the unit.
- Optionally make the container restart on boot as well:
  `docker update --restart unless-stopped ArmPiUltra`. This changes vendor
  container behaviour; confirm with the robot owner first.

## Uninstall

```bash
sudo systemctl disable --now armpi-gateway
sudo rm -f /etc/systemd/system/armpi-gateway.service
sudo rm -rf /usr/local/lib/armpi-gateway
sudo systemctl daemon-reload
# keep /etc/armpi-gateway/gateway.env and the token file if you plan to reinstall
```

## Security notes

- The token is a shared secret, not user authentication. Keep the port on a
  private link (direct Ethernet, private WiFi, Tailscale) and do not expose it
  to the internet.
- The unit runs the gateway as the container's `ubuntu` user. It does not need
  host root beyond installing the unit; the `docker` group grants effective
  root and is required for `docker exec`.
- The gateway has no motion endpoint. `POST /motion/dispatch` returns `501`.
