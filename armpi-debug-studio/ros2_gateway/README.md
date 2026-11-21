# ArmPi Debug Gateway (read-only by default)

This process runs **on the robot**, normally inside the vendor ROS 2 container
(`ArmPiUltra`), which owns the vendor stack. It exposes battery, camera,
joint-command state and a TF edge snapshot to the Debug Studio backend over
local HTTP. The persistent service is **read-only** and has no motion write
path; the G6 supervised single-joint path exists only in a separately started
instance launched with `--enable-motion` (see
[`deploy/supervised-motion.md`](deploy/supervised-motion.md)).

> Verified 2026-11-17 on the Pi 5 vendor container: `/health`, `/battery`,
> `/camera/status`, `/camera/snapshot`, `/state`, token 401 and motion 501.
> `/tf` was verified on the robot on 2026-11-18 (11 dynamic + 8 static edges,
> `base_link→link5` within 0.4 mm of `tf2_echo`). It now runs as an enabled host
> systemd service; stop/start, clean SIGTERM and the 2026-11-20 reboot test
> pass. See
> [`docs/e7-gateway-verification.md`](../docs/e7-gateway-verification.md),
> [`deploy/README.md`](deploy/README.md) and
> [`evidence/studio_gateway/20261117/`](../../evidence/studio_gateway/20261117/).

## Data sources

| Data | ROS 2 source | Notes |
|---|---|---|
| Battery | `/ros_robot_controller/battery` (`std_msgs/UInt16`, mV) | 2S pack; `6.4 V` empty, `8.4 V` full estimate |
| Camera | `/depth_cam/rgb/image_raw` (`sensor_msgs/Image`) | JPEG at the configured quality; Deptrum Aurora 930 |
| Joint state | `/controller_manager/joint_states` (`sensor_msgs/JointState`) | **command echo**, truth=`commanded`, not encoder feedback |
| Servo state | `/controller_manager/servo_states` (`ServoStateList`) | **vendor command cache**, truth=`commanded`; the actual registers are only the `GetBusServoState` service (2026-11-21) |
| TF | `/tf` + `/tf_static` (`tf2_msgs/TFMessage`) | latest edge per parent/child; dynamic edges may derive from commanded joints |

The gateway subscribes only. It never publishes to `/servo_controller`, never
calls `/ros_robot_controller/bus_servo/set_position`, and never opens the serial
port itself.

## Requirements

- ROS 2 Humble (the robot's vendor environment)
- `rclpy`, `sensor_msgs`, `std_msgs`
- `tf2_msgs` for `/tf`; if it is missing the gateway still starts and `/tf`
  reports `unavailable` instead of crashing.
- OpenCV and numpy for camera JPEG encoding. `cv_bridge` is used when present;
  otherwise the gateway decodes `bgr8`/`rgb8`/`mono8` frames directly. The
  vendor `ArmPiUltra` container has OpenCV/numpy but **not** `cv_bridge`, so the
  fallback is the expected path there.
- The vendor stack already running, e.g. `ros_robot_controller` and
  `servo_controller` (do **not** start `bringup.launch.py`, which moves the arm
  during startup check)
- The gateway subscribes only, so it works under the default RMW (Fast DDS)
  even though the vendor nodes use CycloneDDS. **Services are not interoperable
  across RMW implementations**: if a future gateway feature calls a vendor
  service, start it with the vendor environment
  (`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
  `CYCLONEDDS_URI=file:///home/ubuntu/cyclonedds.xml`) or source `~/.zshrc`.
  See `docs/phase2/JOINT_LIMITS.md` for the field evidence.

## Run

```bash
source /opt/ros/humble/setup.bash
source <vendor_ws>/install/setup.bash

export ARMPI_GATEWAY_TOKEN='replace-with-a-long-random-token'
python3 -m armpi_debug_gateway.main \
  --host 127.0.0.1 \
  --port 8765 \
  --battery-topic /ros_robot_controller/battery \
  --camera-topic /depth_cam/rgb/image_raw \
  --tf-topic /tf --tf-static-topic /tf_static
```

For a service, prefer a token file over the environment variable so the
secret is not in the process environment:

```bash
install -m 600 /dev/null /path/to/gateway-token
openssl rand -hex 24 > /path/to/gateway-token
python3 -m armpi_debug_gateway.main --host 10.42.0.218 --token-file /path/to/gateway-token
```

Binding to a non-loopback address without a token now fails closed with exit
code 2. SIGTERM/SIGINT stop the HTTP server and rclpy cleanly.

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

This manual form does not survive a reboot or container restart. Use the
systemd deployment in [`deploy/README.md`](deploy/README.md) for a persistent
service (host unit → `docker exec` → container, token file, restart policy and
clean SIGTERM handling).

For a private LAN setup, bind to the direct-Ethernet address (`10.42.0.218`)
and keep the token enabled. Do not bind `0.0.0.0`: the robot's wlan0 is now
outbound-only, and a wildcard bind would also cover any future interface.

```bash
python3 -m armpi_debug_gateway.main --host 10.42.0.218 --port 8765
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
curl -H "X-Armpi-Token: $TOKEN" http://127.0.0.1:8765/aux-camera/status
curl -H "X-Armpi-Token: $TOKEN" http://127.0.0.1:8765/aux-camera/snapshot -o /tmp/aux.jpg
curl -H "X-Armpi-Token: $TOKEN" http://127.0.0.1:8765/state
curl -H "X-Armpi-Token: $TOKEN" http://127.0.0.1:8765/tf
```

Expected:

- `/health` has `ros_ready`, `battery_ready`, `camera_ready`,
  `aux_camera_ready`, `joints_ready`, and `motion_enabled: false`.
- `/battery` returns `voltage_mv`, `voltage_v`, `percent_estimate`,
  `status` (`ok`/`low`/`critical`), `truth: measured`.
- `/camera/status` reports `640×400`, `fps` around 8, `truth: measured`.
- `/camera/snapshot` is a JPEG.
- `/aux-camera/status` reports the off-arm RealSense D435i color stream read
  directly over UVC (`uvc:/dev/video4`, default `640×480` YUYV, `truth:
  measured`). It needs no ROS driver; `--no-aux-camera` disables it and the
  endpoints then report `available: false`. Depth/IR are not captured.
- `/aux-camera/snapshot` is a JPEG (503 `AUX_CAMERA_UNAVAILABLE` before the
  first frame).
- `/tf` reports `available`, `status` (`ok`/`static_only`/`stale`/`unavailable`),
  edge counts, `freshest_age_ms`, `max_dynamic_age_ms`, `max_static_age_ms` and
  `transforms[]` with parent/child/translation/quaternion/age. Static edges are
  latched and legitimately old; compare dynamic ages for liveness. Verified
  against the live robot TF graph on 2026-11-18 (11 dynamic + 8 static edges,
  `base_link→link5` within 0.4 mm of `tf2_echo`).
- `/motion/dispatch` and `/motion/stop` return `501 NOT_IMPLEMENTED` on the
  persistent 8765 service. A manually started `--enable-motion` instance on
  8766 exposes the supervised single-joint G6 path; see
  [`deploy/supervised-motion.md`](deploy/supervised-motion.md).

## Trajectory recorder (G5)

The package also contains a read-only recorder, but the systemd unit does not
start it. It subscribes to the command echo and to
`/controller_manager/servo_states` (the vendor **command cache**, not a bus
read) and writes a G2 dataset. It creates no publisher or service client:

```bash
sudo docker exec -u ubuntu ArmPiUltra /bin/bash -lc '
  source /opt/ros/humble/setup.bash
  source /home/ubuntu/ros2_ws/install/setup.bash
  cd /home/ubuntu/shared/armpi_gateway_deploy
  python3 -m armpi_debug_gateway.recorder --mode manual --duration 5 \
    --static-planned --capture-out /home/ubuntu/shared/g5_capture.json
'
```

See [`../docs/g5-trajectory-recorder.md`](../docs/g5-trajectory-recorder.md) for
the moving-capture workflow, plan JSON and dataset assembly.

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
