# Operator runbook

Scope: the current single-robot, direct-Ethernet setup. Hardware motion is
always supervised; there is no e-stop, brake, independent encoder or
odometry. Run [`PREFLIGHT.md`](PREFLIGHT.md) first.

## Start the read-only gateway

Installed as `armpi-gateway.service`; it waits for `ArmPiUltra`, then runs the
gateway in-container as `ubuntu`.

```bash
sudo systemctl start armpi-gateway
systemctl is-active armpi-gateway
curl -fsS -H "X-Armpi-Token: $TOKEN" http://10.42.0.218:8765/health
```

The installer replaces `@ARMPI_SERVICE_USER@`/`@ARMPI_SERVICE_GROUP@` with
`pi`/`docker`, so the host wrapper does not run as root. Re-run
`install-gateway.sh` to apply the unit to an existing robot; this restarts
the read-only gateway but not the vendor stack.

## Start the pose/motion gateway

The 8766 instance is manual (it owns the point cloud and motion topic):

```bash
# inside the vendor container
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
python3 -m armpi_debug_gateway.main \
  --host 10.42.0.218 --port 8766 --token-file /home/ubuntu/shared/armpi_gateway_deploy/token \
  --pointcloud-topic /depth_cam/depth_registered/points
```

The camera stays off in this instance; the read-only 8765 gateway owns the
cameras. Do not start two 8766 instances.

## Start the safety node

The real node is a manual process in the persistent overlay:

```bash
docker exec -it ArmPiUltra zsh
source /home/ubuntu/shared/safety_ws/install/setup.zsh
export PYTHONPATH=/home/ubuntu/shared/armpi_tools:$PYTHONPATH
ros2 run armpi_safety safety_supervisor
```

Verify `/safety_status` and `/safety_stop=false`. The node is **not** a boot
service.

## Backend

```bash
cd armpi-debug-studio/backend
BIND_HOST=127.0.0.1 \
ARMPI_MODE=hardware \
ARMPI_HARDWARE_MOTION=1 \
ARMPI_MOTION_SCOPE=pose \
ARMPI_TAKEOVER_ACK='I_AM_AT_THE_ROBOT_WITH_A_CLEAR_WORKSPACE' \
ARMPI_GATEWAY_URL=http://10.42.0.218:8766 \
ARMPI_CAMERA_GATEWAY_URL=http://10.42.0.218:8765 \
ARMPI_GATEWAY_TOKEN="$(cat gateway-token)" \
.venv/bin/python -m uvicorn app.main:create_app --factory \
  --host 127.0.0.1 --port 8000
```

The backend is localhost-only by policy and has no authentication. Do not
bind it to a non-loopback address; use an SSH tunnel for remote access and
keep the gateway token out of shell history. Backend edits need a restart
(there is no autoreload).

## Normal G8 run

Use the runner, not hand-built command sequences:

```bash
python3 tools/studio/g8_task_run.py \
  --backend http://127.0.0.1:8000 \
  --session g8-$(date +%Y%m%d-%H%M) \
  --out evidence/g8_acceptance_$(date +%Y%m%d)/g8_run.json \
  --execute --i-have-clear-workspace
```

If the planner returns `APPROACH_REQUIRED`, clear the workspace and approve
one bounded chassis pulse explicitly; a base move invalidates the table
calibration. After the run, re-plan read-only to confirm the cube is back in
the `0.220–0.255 m` band. Record the report and a README; do not rewrite old
evidence.

## Safety stop

`/safety_stop` is a transient-local `Bool`. The gateway latches it and refuses
arm/chassis commands with `SAFETY_STOP` (HTTP 409).

1. Stop issuing commands.
2. Find the cause in `/safety_status` (`violations`, `stop_reason`).
3. Clear the physical condition; support the arm if it is low.
4. Reset the safety node:

```bash
docker exec ArmPiUltra bash -lc \
  'source /opt/ros/humble/setup.bash; \
   ros2 service call /armpi_safety/reset_stop std_srvs/srv/Trigger'
```

5. Confirm `/safety_stop=false`; restart the gateway only if it still holds a
   stale latch.

## Command cache is stale after a restart

A node restart can move the servos and leave `/servo_states` at the old
command. Never hold at the cache. Read the actual bus, then publish
`ServosPosition(target=actual, duration=2.0, pulse)` and verify the actual bus
moved by no more than a few counts. The 2026-11-23 procedure and numbers are
in
[`evidence/studio_validation_20261123/README.md`](../../evidence/studio_validation_20261123/README.md).

## Vendor controller crash

A negative-register read can crash `ros_robot_controller`. Do not repeat the
read or restart the whole container. Relaunch only the vendor node and
re-check the bus/cache before motion. The incident record is in
[`docs/hardware/vendor_baseline.md`](../hardware/vendor_baseline.md).

## Motion does not start or reports `CONTROL_FAILED`

- Movement present: per-segment latency / timing; check the command durations
  and gateway log.
- No movement: check the Pi clock is ahead of the laptop, the `safety_stop`
  latch, and whether the adapter rejected the command before publication.
- Repeated vendor failures: stop retrying after three attempts and record the
  logs.

## Evidence capture

Every hardware claim needs a command, log or JSON in a dated
`evidence/<topic>_YYYYMMDD/` folder. Copy the runner JSON, the gateway
`/motion/status` actual values, and the safety status; write a README that
states what was and was not observed. Never label a mock or synthetic run as
hardware.

## Rollback

```bash
git checkout v1.0.0-rc.1   # last tagged candidate
# re-deploy the gateway package and, if the safety node changed,
# /home/pi/docker/tmp/safety_ws from the same tag
```

The vendor workspace is private and is restored from the archive recorded in
[`docs/hardware/vendor_baseline.md`](../hardware/vendor_baseline.md); verify
it with `python3 tools/vendor_baseline_manifest.py --check`.
