# Pose takeover gateway (operator procedure)

The persistent `armpi-gateway.service` stays read-only on port **8765**. Pose
takeover uses a **second, manually started** gateway instance on port **8766**
with a pose-mode motion config. The read-only console, the hardware Playwright
suite and the 8765 process stay untouched.

> Read [`docs/g7-pose-takeover.md`](../../docs/g7-pose-takeover.md) first.
> `mapping_verified=true` covers the operating envelope; there is **no e-stop,
> brake or independent encoder**, and the FK gate is not a collision model.
>
> Addresses use the direct Ethernet link `10.42.0.218`, the only inbound path
> since the 2026-11-18 lockdown (`docs/hardware/network_access.md`).

## 1. Generate the pose contract JSON

On the laptop:

```bash
cd armpi-debug-studio
backend/.venv/bin/python tools/export_gateway_motion_config.py \
  --mode pose --out /tmp/studio_takeover_limits.json --overwrite
```

Defaults in pose mode: the widest joint span as the step cap (3.61 rad),
0.5 rad/s velocity, 1.0 s floor, 3.0 s completion margin, five-joint poses and
the `r_joint` gripper contract from `configs/takeover.yaml`. Do not hand-edit
pulse bounds; change `takeover.yaml`/`joint_mapping.yaml`, regenerate, and
re-run the parser (the generator validates with the gateway's own parser).

## 2. Use a separate deploy directory

Do **not** overwrite the live `/home/pi/docker/tmp/armpi_gateway_deploy` while
the read-only service is running; the running process keeps its modules, but a
later restart would silently switch the read-only console to a new package.
For a verification run, use a sibling directory:

```bash
cd armpi-debug-studio
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'mkdir -p /home/pi/docker/tmp/armpi_gateway_pose'
tar -C ros2_gateway --exclude=__pycache__ -cf - armpi_debug_gateway |
  ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'tar -C /home/pi/docker/tmp/armpi_gateway_pose -xf -'
scp -i ~/.ssh/id_ed25519_armpi /tmp/studio_takeover_limits.json \
  pi@10.42.0.218:/home/pi/docker/tmp/armpi_gateway_pose/takeover_limits.json
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'sudo chown -R 1000:1000 /home/pi/docker/tmp/armpi_gateway_pose'
```

The vendor container mounts `/home/pi/docker/tmp` as `/home/ubuntu/shared`.
When the pose instance is adopted permanently, install the package with
`install-gateway.sh` instead and keep one deploy directory.

## 3. Start the pose instance

The pose instance must run with the **vendor zsh environment**
(`source ~/.zshrc`), because it calls the `GetBusServoState` service. The
vendor stack uses CycloneDDS; a `bash -lc` launch with only the ROS setup files
is Fast DDS and cannot reach the service (topics still work, which is why the
read-only 8765 gateway can stay Fast DDS). Start it on the robot host:

```bash
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'sudo docker exec -d -u ubuntu ArmPiUltra /bin/zsh -lc "
     source ~/.zshrc >/dev/null 2>&1;
     cd /home/ubuntu/shared/armpi_gateway_pose;
     exec python3 -m armpi_debug_gateway.main \
       --host 10.42.0.218 --port 8766 \
       --token-file /home/ubuntu/shared/armpi_gateway_deploy/token \
       --enable-motion \
       --motion-config /home/ubuntu/shared/armpi_gateway_pose/takeover_limits.json \
       --no-aux-camera --camera-topic /armpi_pose_camera_off \
       > /tmp/armpi_pose_gateway.log 2>&1"'
```

The gateway fails at startup if the bus service is not visible. Its log must
contain `verification=True bus=/ros_robot_controller/bus_servo/get_state`.
With the G8.1 approach, `configs/takeover.yaml` also carries a `chassis` block;
regenerate the JSON (the exporter copies it) and check `/health` reports
`"chassis_enabled": true` and the duty/duration caps. The chassis endpoint is
`POST /chassis/pulse` (token + `--enable-motion` + an enabled `chassis` block);
it publishes `MotorsSpeedControl` at 20 Hz for one pulse, waits for a matched
subscriber and then publishes four zero messages. A pulse can take up to
~5 s, so backend clients need a POST timeout above 8 s (the Studio uses 20 s).

`--no-aux-camera --camera-topic /armpi_pose_camera_off` keeps the motion
instance from decoding frames it does not need; the 8765 instance still serves
the camera. The pose page does not use the camera, so a takeover-mode Dashboard
reports the camera panels as unavailable. If you want the pose gateway to serve
the main camera too, omit the camera-off flag (this adds one JPEG encoder on the
Pi); keep the aux camera on the read-only instance. Record the PID:

```bash
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  "pgrep -af 'armpi_debug_gateway.main' | grep 8766"
```

## 4. Verify before the backend connects

```bash
TOKEN=$(ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'sudo cat /home/pi/docker/tmp/armpi_gateway_deploy/token')
curl -s -H "X-Armpi-Token: $TOKEN" http://10.42.0.218:8766/health
curl -s -H "X-Armpi-Token: $TOKEN" http://10.42.0.218:8766/motion/status
```

Expected:

```text
health:        motion_enabled=true, motion_mode=pose,
               motion_state=idle, measured_ready=true
motion/status: state=idle, mode=pose, max_velocity_rad_s=0.5,
               max_step_rad=3.61, single_joint_only=false,
               verification=true, verification_tolerance_pulses=12,
               gripper r_joint 170..700 completion=timed,
               measured (command cache) includes r_joint,
               actual is empty until the first completed command
```

After a command, `/motion/status` must show `verification="bus"`,
`state="completed"`, `actual={joint: raw_position}` and
`deviations_pulses` in the backend event. `verification="pending"` with
`state="completed"` is not accepted by the backend; it becomes `Unknown`.

Rejection probes (these fail validation **before** publication; the arm does
not move):

```bash
# partial pose: two joints only -> 400 MOTION_BAD_REQUEST
curl -s -X POST -H "X-Armpi-Token: $TOKEN" -H "Content-Type: application/json" \
  -d '{"command_id":"probe-partial","duration_s":2.0,"joints":[
       {"joint":"joint1","servo_id":6,"target_pulse":510,"target_rad":0.0419},
       {"joint":"joint2","servo_id":5,"target_pulse":500,"target_rad":0.0}]}' \
  http://10.42.0.218:8766/motion/dispatch

# out-of-limit pulse -> 400 MOTION_OUT_OF_LIMIT
curl -s -X POST -H "X-Armpi-Token: $TOKEN" -H "Content-Type: application/json" \
  -d '{"command_id":"probe-limit","duration_s":2.0,"joints":[
       {"joint":"joint1","servo_id":6,"target_pulse":1000,"target_rad":2.094}]}' \
  http://10.42.0.218:8766/motion/dispatch
```

Do not send a valid pose here. A valid dispatch publishes immediately.

## 5. Start the pose backend

On the laptop:

```bash
export ARMPI_GATEWAY_TOKEN=$(ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'sudo cat /home/pi/docker/tmp/armpi_gateway_deploy/token')
export ARMPI_TAKEOVER_ACK=I_AM_AT_THE_ROBOT_WITH_A_CLEAR_WORKSPACE
GATEWAY_PORT=8766 tools/studio/start_studio.sh --takeover
```

`--takeover` selects `ARMPI_MOTION_SCOPE=pose`, `ARMPI_HARDWARE_MOTION=1` and
the 8766 gateway; it refuses to start without the acknowledgement phrase.
Confirm:

- `/api/health`: `motion_scope=pose`, `gripper_control=true`,
  `pose_gate=true`, `max_velocity_rad_s=0.5`.
- `/api/state`: five joints `truth=measured`, plus a measured `gripper` block.
- `/api/diagnostics`: `safety.ready=true`, no blockers, and the
  `pose_takeover` check visible.
- `/api/motion/presets`: 5 presets, all `limit_ok`, current TCP present.

Run the read-only verification (no execute):

```bash
cd <repo>
python3 tools/studio/verify_takeover.py \
  --backend http://127.0.0.1:8010 \
  --gateway http://10.42.0.218:8766 \
  --out /tmp/g7_no_motion.json
```

## 6. First real pose motion

1. Support the arm or clear the workspace; the arm falls on power loss.
2. Load a preset, read the whole preview (per-joint from/to/raw, target TCP,
   table clearance, path samples), then confirm execute once.
3. Expected audit: `Requested → Validated → Dispatched → Observed →
   Completed`. Completion requires all five measured pulses within ±3 for
   three consecutive samples.
4. Stop publishes a hold at the last measured pulses; it is not a brake.
   Power/support remain the physical layer.
5. An `Unknown` outcome latches further motion until the backend is restarted
   after inspection.

## 7. Stop and clean up

```bash
# 1. find the 8766 PID on the robot and kill only that process
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  "pgrep -af 'armpi_debug_gateway.main' | grep 8766"
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'sudo kill <PID-from-the-line-above>'

# 2. confirm 8766 is closed and 8765 still listens
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 'ss -ltn | grep -E "8765|8766"'

# 3. remove a temporary verification deploy dir if used
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'sudo rm -rf /home/pi/docker/tmp/armpi_gateway_pose'
```
