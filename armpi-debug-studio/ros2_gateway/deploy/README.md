# Persistent gateway deployment (robot host)

> Deployed on the Pi 5 on 2026-11-17. Lifecycle evidence (enabled, health,
> 401, clean SIGTERM stop, start, port/process checks):
> [`evidence/studio_gateway/20261117/persistent/`](../../../evidence/studio_gateway/20261117/persistent/).
> A full reboot test **passed on 2026-11-20** (`NRestarts=0` for the gateway,
> D435i depth and aux-camera units); a vendor-container restart still triggers
> the vendor startup check, which moves the arm.

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
  pi@10.42.0.218:/home/pi/docker/tmp/armpi_gateway_pkg

ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'sudo bash /home/pi/docker/tmp/armpi_gateway_pkg/ros2_gateway/deploy/install-gateway.sh'
```

The installer:

1. copies `armpi_debug_gateway/` to
   `/home/pi/docker/tmp/armpi_gateway_deploy/` (container:
   `/home/ubuntu/shared/armpi_gateway_deploy/`);
2. creates or preserves the token file next to it with mode `0600` (owned by
   uid 1000 so the container user can read it), using `openssl rand -hex 24`;
3. installs the unit with `User=pi` / `Group=docker` (override with
   `ARMPI_GATEWAY_SERVICE_USER` / `ARMPI_GATEWAY_SERVICE_GROUP`) and the
   `root:docker` `gateway.env` (first install only);
4. enables and starts the service, then polls `/health` for up to 60 s.

The host wrapper no longer runs as root: `pi` is in the Docker group and the
in-container gateway already runs as the vendor `ubuntu` user. The installer
refuses to proceed if that account cannot reach the Docker daemon. Verify an
installed unit with:

```bash
systemctl show armpi-gateway.service -p User -p Group
# User=pi
# Group=docker
```

It does **not** start, stop or modify the vendor container or the ROS stack.

## Configure the bind address

Edit `/etc/armpi-gateway/gateway.env`, then restart:

```bash
sudoedit /etc/armpi-gateway/gateway.env   # ARMPI_GATEWAY_BIND=...
sudo systemctl restart armpi-gateway
```

Use the address the backend actually reaches. Since the 2026-11-18 inbound
lockdown the direct Ethernet link is the only inbound path
(`docs/hardware/network_access.md`):

| Link | Bind |
|---|---|
| Direct Ethernet (current) | `10.42.0.218` |
| Office WiFi | not reachable; wlan0 is outbound-only |
| Tailscale | removed 2026-11-18 |

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
armpi_debug_gateway listening on http://10.42.0.218:8765
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
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
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

- The token is a shared secret, not user authentication. The gateway is bound
  to `10.42.0.218` and the robot's wlan0 is outbound-only; do not expose it to
  the internet and do not bind `0.0.0.0`.
- The unit runs the gateway as the container's `ubuntu` user. It does not need
  host root beyond installing the unit; the `docker` group grants effective
  root and is required for `docker exec`.
- The gateway has no motion endpoint. `POST /motion/dispatch` returns `501`.

## Off-board camera driver (`armpi-ext-cam.service`)

The chassis-mounted RealSense D435i has exactly one owner: `ext_cam_node`
(`tools/phase2/ext_cam_node.py`), started by `armpi-ext-cam.service`. It opens
`/dev/video4` at 1280x720 @ 15 fps and publishes:

- `/ext_cam/color/image_raw` (bgr8, sensor-data QoS)
- `/ext_cam/color/camera_info` (latched, from the 2026-11-20 calibration)
- `base_link -> ext_cam_optical` (static, same calibration)

The gateway follows that topic (`ARMPI_GATEWAY_AUX_CAMERA_TOPIC`) instead of
opening the device, so the studio preview and ROS consumers share one stream.

Install:

```bash
# node + calibration reports, into the container's shared directory
docker cp tools/phase2/ext_cam_node.py ArmPiUltra:/home/ubuntu/shared/armpi_ext_cam/
docker cp tools/phase2/ext_cam_frames.py ArmPiUltra:/home/ubuntu/shared/armpi_ext_cam/
docker cp evidence/calibration/20261120/front_camera_extrinsic.json \
  ArmPiUltra:/home/ubuntu/shared/armpi_ext_cam/
docker cp evidence/calibration/20261120/uvc_color_intrinsics_refined.json \
  ArmPiUltra:/home/ubuntu/shared/armpi_ext_cam/

# service
install -d -m 755 /usr/local/lib/armpi-ext-cam
install -m 755 deploy/ext-cam-run.sh /usr/local/lib/armpi-ext-cam/ext-cam-run.sh
install -m 644 deploy/armpi-ext-cam.service /etc/systemd/system/
systemctl daemon-reload
systemctl enable --now armpi-ext-cam.service
systemctl restart armpi-gateway.service
```

Verify: `/aux-camera/status` must report
`"source": "armpi_debug_gateway_ros"` and `"device": null`; `fuser /dev/video4`
must show exactly one process.

## Depth driver (`armpi-rs-depth.service`)

The D435i depth stream comes from `realsense2_camera` inside the container,
started by `armpi-rs-depth.service`. Depth only: colour stays with
`armpi-ext-cam.service`. It publishes `/camera/camera/depth/image_rect_raw`
(16UC1, `camera_depth_optical_frame`) at 640x480, measured 4-8 Hz with the
colour stream running and ~10.5 Hz alone.

librealsense inside the container carries a local patch
(`tools/phase2/patches/librealsense-2.55.1-d400-hid-gate.patch`) because this
kernel is built without `CONFIG_HID_SENSOR_HUB`; see
[`docs/hardware/camera_mount.md`](../../../docs/hardware/camera_mount.md). The
patch is already built and installed in the container - reinstall it there with
`cd /home/ubuntu/lrs_build && gmake -j4 realsense2 && gmake install` if the
container is ever recreated.

Install:

```bash
install -d -m 755 /usr/local/lib/armpi-rs-depth
install -m 755 deploy/rs-depth-run.sh /usr/local/lib/armpi-rs-depth/rs-depth-run.sh
install -m 644 deploy/armpi-rs-depth.service /etc/systemd/system/

# ordering: start colour only after the depth topic exists
install -d -m 755 /etc/systemd/system/armpi-ext-cam.service.d
install -m 755 deploy/wait-depth-topic.sh /usr/local/lib/armpi-ext-cam/wait-depth-topic.sh
install -m 644 deploy/armpi-ext-cam-after-depth.conf \
  /etc/systemd/system/armpi-ext-cam.service.d/20-after-depth.conf

systemctl daemon-reload
systemctl enable --now armpi-rs-depth.service
```

Three ordering rules make this work, and each one was a real failure first:

- **After the vendor bringup.** `start_node.sh` runs `~/.stop_ros.sh`, which
  `kill -9`s every process whose command line contains "ros". systemd reports
  `start_node.service` as started ~15 s before that kill lands, so the runner
  waits for the `bringup.launch.py` process itself; otherwise the node dies
  with status 137 and has to restart.
- **Before `armpi-ext-cam.service`.** If the colour node claims the UVC device
  first, librealsense fails `VIDIOC_S_FMT` with `EBUSY`. The unit declares
  `Before=armpi-ext-cam.service`, `armpi-ext-cam` declares
  `After=armpi-rs-depth.service`, and its `ExecStartPre` waits (best effort,
  40 s) for the depth topic, because ordering alone does not cover the node's
  ~5 s start-up.
- **IR wake-up before the node.** A freshly powered D435i can need a nudge
  before its stereo unit answers: the runner streams the IR node for three
  frames (retried for up to 90 s), and only then starts the node. Without it a
  cold boot ends in `Frames didn't arrived within 5 seconds` and no depth.

Verify after a reboot:

```bash
systemctl is-active armpi-rs-depth armpi-ext-cam
journalctl -u armpi-rs-depth -b | grep -E "wake-up|Is Up|didn't arrived"
systemctl show -p NRestarts --value armpi-rs-depth     # expect 0
```

Stopping the unit leaves no node behind (the unit kills the launch and the
node, escalating to `SIGKILL`, and a leftover node keeps `/dev/video0` open so
the next start fails with `EBUSY`).
