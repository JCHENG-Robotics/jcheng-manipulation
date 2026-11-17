# Studio read-only gateway — first hardware verification (2026-11-17)

> Status: **verified, read-only only**. The gateway served live battery, RGB
> camera and joint-command state from the real ArmPi Ultra Pi 5, and the
> FastAPI backend consumed it in `hardware` mode. No motion command was
> dispatched; execute was rejected by the safety gate. Raw camera images are
> private and are **not** in this repository.

## Setup

| Item | Value |
|---|---|
| Robot | Pi 5 (`raspberrypi`), direct Ethernet `eth0=10.42.0.218/24` |
| Laptop | `enx00e04ca373c7=10.42.0.1/24` (shared link), studio backend on `127.0.0.1:8000` |
| Robot OS state | root filesystem 87% full (44G/52G); factory AP on `wlan0` at the time of test |
| ROS runtime | vendor container `ArmPiUltra` (`ros:humble`, `--net=host`), vendor stack already running (`bringup` + `vision`) |
| Gateway host path | `/home/pi/docker/tmp/armpi_gateway_deploy` (container: `/home/ubuntu/shared/...`) |
| Gateway bind | `10.42.0.218:8765` (direct link only, shared token required) |
| Camera topic | `/depth_cam/rgb/image_raw`, 640×400 `bgr8` |
| Battery topic | `/ros_robot_controller/battery`, `std_msgs/UInt16` mV |
| Joint topic | `/controller_manager/joint_states` (command echo) |

The vendor container has OpenCV 4.10 and numpy 1.26 but **no `cv_bridge`**. The
gateway now decodes `bgr8`/`rgb8`/`mono8` frames directly when `cv_bridge` is
missing (`ros2_gateway/armpi_debug_gateway/imaging.py`); this run exercised that
fallback.

## Start (inside the robot container)

```bash
# laptop -> robot
scp -i ~/.ssh/id_ed25519_armpi /tmp/armpi_gateway.tgz pi@10.42.0.218:/home/pi/docker/tmp/
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'tar xzf /home/pi/docker/tmp/armpi_gateway.tgz -C /home/pi/docker/tmp/armpi_gateway_deploy'

# robot host: start detached in the vendor container
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  "docker exec -d -u ubuntu \
     -e ARMPI_GATEWAY_TOKEN=\$ARMPI_GATEWAY_TOKEN \
     -e PYTHONPATH=/home/ubuntu/shared/armpi_gateway_deploy \
     ArmPiUltra /bin/bash -lc \
     'source /opt/ros/humble/setup.bash; \
      source /home/ubuntu/ros2_ws/install/setup.bash; \
      cd /home/ubuntu/shared/armpi_gateway_deploy; \
      exec python3 -m armpi_debug_gateway.main --host 10.42.0.218 --port 8765'"
```

The token was generated per session with `openssl rand -hex 24`; it is not
stored in this repository.

## Gateway results

| Check | Result | Evidence |
|---|---|---|
| `GET /health` | `status=ok`, `ros_ready/battery_ready/camera_ready/joints_ready=true`, `motion_enabled=false` | `gateway_health.json` |
| `GET /battery` | 7424 mV, 51.2%, 2S, `status=ok`, `truth=measured`, `validity=valid` | `gateway_battery.json` |
| `GET /camera/status` | 640×400 `bgr8`, 8.5 fps, frame_count 15757, `truth=measured` | `gateway_camera_status.json` |
| `GET /camera/snapshot` | JPEG 640×400, 20415 bytes | `snapshot_meta.json` (image not committed) |
| `GET /state` | 5 arm joints + gripper, `truth=commanded`, sample age ~ms | `gateway_state.json` |
| Missing token | HTTP 401 | `gateway_unauthorized_status.txt` |
| `POST /motion/dispatch` | HTTP 501 `NOT_IMPLEMENTED` | `gateway_motion_disabled.txt` |
| Process / bind | PID 9496, LISTEN `10.42.0.218:8765` | `gateway_process.txt` |

## Backend results (`ARMPI_MODE=hardware`)

| Check | Result | Evidence |
|---|---|---|
| `GET /api/health` | `mode=hardware`, `source_label=LIVE`, `hardware_motion=false`, `hardware_read=true` | `backend_health.json` |
| `GET /api/peripherals` | battery + camera both `measured`/`valid`, 7.424 V, 640×400, 8.5 fps | `backend_peripherals.json` |
| `GET /api/peripherals/camera/snapshot` | JPEG 640×400 | collected locally; image private |
| `GET /api/connection` | driver/gateway/feedback `online`; serial `unknown` (vendor-owned) | `backend_connection.json` |
| `GET /api/state` | joints `commanded`/`valid`; limits provisional | `backend_state.json` |
| `GET /api/safety` | blockers `mapping_verified`, `hardware_motion_enabled` | `backend_safety.json` |
| `POST /api/motion/preview` (joint1 0.1 rad) | valid, token issued | `backend_preview_valid.json` |
| `POST /api/motion/execute` | `GATE_REJECTED`; blockers `mapping_verified`, `hardware_motion_enabled` | `backend_execute_rejected.json` |
| Dashboard UI | `LIVE`, camera `measured 640×400 bgr8` ~5.8–8.5 fps, battery `7.42 V / 51%`, connection chain online | browser DOM snapshot; screenshot kept private |

The UI camera panel rendered a live 640×400 RGB frame of the workspace
(chessboard visible), not the Mock SVG placeholder. The joint table showed
`commanded`/`valid` with `provisional` limits, and the safety gate showed the
two expected blockers.

## Limits

- This verifies the **read path** only. There is no motion write path in the
  gateway, and the backend rejected execute even with a valid preview token.
- `/controller_manager/joint_states` is the vendor command echo, not encoder
  feedback. Do not treat these positions as measured joint angles.
- The gateway was started manually for this test. It is not a systemd service
  and does not survive a robot reboot/container restart.
- The bind address is the direct-Ethernet IP. If the robot moves back to WiFi,
  bind the `wlan0` address or use an SSH tunnel.
- A shared token is the only authentication; keep the port on a private link.
- Camera fps varied 5.1–8.5 during the test (vendor stack load and startup);
  the stream is not rate-limited by the gateway.
- The Pi root filesystem is 87% full; do not add large artifacts without
  checking space.

## Files

| File | Content |
|---|---|
| `gateway_*.json` | raw gateway responses (health, battery, camera, state) |
| `backend_*.json` | raw backend responses (health, peripherals, connection, safety, state) |
| `backend_preview_valid.json` | preview result; the single-use token is redacted because the command was rejected |
| `backend_execute_rejected.json` | live hardware-mode safety rejection |
| `gateway_process.txt` | gateway PID and listening socket |
| `gateway_unauthorized_status.txt` | 401 without token |
| `gateway_motion_disabled.txt` | 501 for motion |
| `snapshot_meta.json` | JPEG size/SHA-256/dimensions; image not committed |
