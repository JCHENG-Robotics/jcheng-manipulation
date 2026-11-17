# ArmPi Debug Gateway (read-only)

This process runs **on the robot**, normally inside the vendor ROS 2 container
(`ArmPiUltra`), which owns the vendor stack. It exposes battery, camera, and
joint-command state to the Debug Studio backend over local HTTP. It has **no
motion write path**.

> Verified 2026-11-17 on the Pi 5 vendor container: `/health`, `/battery`,
> `/camera/status`, `/camera/snapshot`, `/state`, token 401 and motion 501. See
> [`docs/e7-gateway-verification.md`](../docs/e7-gateway-verification.md) and
> [`evidence/studio_gateway/20261117/`](../../evidence/studio_gateway/20261117/).

## Data sources

| Data | ROS 2 source | Notes |
|---|---|---|
| Battery | `/ros_robot_controller/battery` (`std_msgs/UInt16`, mV) | 2S pack; `6.4 V` empty, `8.4 V` full estimate |
| Camera | `/depth_cam/rgb/image_raw` (`sensor_msgs/Image`) | JPEG at the configured quality; Deptrum Aurora 930 |
| Joint state | `/controller_manager/joint_states` (`sensor_msgs/JointState`) | **command echo**, truth=`commanded`, not encoder feedback |

The gateway subscribes only. It never publishes to `/servo_controller`, never
calls `/ros_robot_controller/bus_servo/set_position`, and never opens the serial
port itself.

## Requirements

- ROS 2 Humble (the robot's vendor environment)
- `rclpy`, `sensor_msgs`, `std_msgs`
- OpenCV and numpy for camera JPEG encoding. `cv_bridge` is used when present;
  otherwise the gateway decodes `bgr8`/`rgb8`/`mono8` frames directly. The
  vendor `ArmPiUltra` container has OpenCV/numpy but **not** `cv_bridge`, so the
  fallback is the expected path there.
- The vendor stack already running, e.g. `ros_robot_controller` and
  `servo_controller` (do **not** start `bringup.launch.py`, which moves the arm
  during startup check)

## Run

```bash
source /opt/ros/humble/setup.bash
source <vendor_ws>/install/setup.bash

export ARMPI_GATEWAY_TOKEN='replace-with-a-long-random-token'
python3 -m armpi_debug_gateway.main \
  --host 127.0.0.1 \
  --port 8765 \
  --battery-topic /ros_robot_controller/battery \
  --camera-topic /depth_cam/rgb/image_raw
```

### Start inside the vendor container

The Pi 5 vendor stack runs in the `ArmPiUltra` container (`--net=host`). Copy
the package to a shared host directory (for example
`/home/pi/docker/tmp/armpi_gateway_deploy`, mounted at
`/home/ubuntu/shared/armpi_gateway_deploy`), then start it detached:

```bash
docker exec -d -u ubuntu \
  -e ARMPI_GATEWAY_TOKEN='replace-with-a-long-random-token' \
  -e PYTHONPATH=/home/ubuntu/shared/armpi_gateway_deploy \
  ArmPiUltra /bin/bash -lc \
  'source /opt/ros/humble/setup.bash; \
   source /home/ubuntu/ros2_ws/install/setup.bash; \
   cd /home/ubuntu/shared/armpi_gateway_deploy; \
   exec python3 -m armpi_debug_gateway.main --host <robot-eth0-ip> --port 8765'
```

This is a manual start; it does not survive a reboot or container restart.

For a private Tailscale/LAN setup, bind to the Tailscale address and keep the
token enabled:

```bash
python3 -m armpi_debug_gateway.main --host 0.0.0.0 --port 8765
```

Do **not** expose this port on the public internet. There is no user
authentication beyond the shared token, and the camera feed shows the workspace.

## Verify on the robot

```bash
TOKEN='replace-with-a-long-random-token'
curl -H "X-Armpi-Token: $TOKEN" http://127.0.0.1:8765/health
curl -H "X-Armpi-Token: $TOKEN" http://127.0.0.1:8765/battery
curl -H "X-Armpi-Token: $TOKEN" http://127.0.0.1:8765/camera/status
curl -H "X-Armpi-Token: $TOKEN" http://127.0.0.1:8765/camera/snapshot -o /tmp/frame.jpg
curl -H "X-Armpi-Token: $TOKEN" http://127.0.0.1:8765/state
```

Expected:

- `/health` has `ros_ready`, `battery_ready`, `camera_ready`, `joints_ready`,
  and `motion_enabled: false`.
- `/battery` returns `voltage_mv`, `voltage_v`, `percent_estimate`,
  `status` (`ok`/`low`/`critical`), `truth: measured`.
- `/camera/status` reports `640×400`, `fps` around 8, `truth: measured`.
- `/camera/snapshot` is a JPEG.
- `/motion/dispatch` and `/motion/stop` return `501 NOT_IMPLEMENTED`.

## Connect the backend

On the machine running the FastAPI backend:

```bash
export ARMPI_MODE=hardware
export ARMPI_GATEWAY_URL=http://<robot-host>:8765
export ARMPI_GATEWAY_TOKEN='replace-with-a-long-random-token'
uvicorn app.main:create_app --factory --host 127.0.0.1 --port 8000
```

If the robot is reached through an SSH tunnel, `ARMPI_GATEWAY_URL` can stay
`http://127.0.0.1:8765`:

```bash
ssh -N -L 8765:127.0.0.1:8765 <user>@<robot-host>
```

Then open the Debug Studio Dashboard. Camera and battery should switch from
`SIMULATION` to live `measured` data. Joint motion remains blocked; the backend
advertises `hardware_motion=false`.

## Safety boundaries

- The gateway is read-only. Do not add motion publishers until the joint
  mapping/limits are hardware-verified and the Board approves a guarded motion
  phase.
- The camera topic is RGB only. Depth/point-cloud streaming is intentionally
  out of scope for G1.
- `/controller_manager/joint_states` is a command echo. Do not present it as
  encoder feedback.
