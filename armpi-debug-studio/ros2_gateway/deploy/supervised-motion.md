# Supervised motion gateway (operator procedure)

The persistent `armpi-gateway.service` stays read-only on port **8765**. Motion
uses a **second** gateway instance on port **8766** with `--enable-motion`.
Since 2026-11-23 it is wrapped by `armpi-gateway-pose.service`, which is
installed but **not enabled**; start it explicitly (see the deploy README).
The manual commands below remain the fallback/diagnostic path.

> Read [`docs/g6-motion-write-path.md`](../../docs/g6-motion-write-path.md)
> before running this. The limits are verified for the operating envelope
> (`mapping_verified=true` from 2026-11-19); there is no e-stop or brake.
>
> The addresses below use the direct Ethernet link `10.42.0.218`, the only
> inbound path since the 2026-11-18 lockdown
> (`docs/hardware/network_access.md`). Start the supervised instance with
> `--host 10.42.0.218`; a WiFi bind is no longer reachable.

## 1. Generate and copy the limits JSON

On the laptop:

```bash
cd armpi-debug-studio
backend/.venv/bin/python tools/export_gateway_motion_config.py \
  --out /tmp/studio_step_limits.json --overwrite
```

Since 2026-11-21 the step config also carries `verification=true` (tolerance
8 pulses): `/controller_manager/servo_states` is the vendor command cache, not
a bus read, so the gateway verifies every step with `GetBusServoState`. Use a
**separate** deploy directory so the read-only 8765 package and JSON are not
touched:

```bash
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'mkdir -p /home/pi/docker/tmp/armpi_gateway_step'
tar -C ros2_gateway --exclude=__pycache__ -cf - armpi_debug_gateway |
  ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'tar -C /home/pi/docker/tmp/armpi_gateway_step -xf -'
scp -i ~/.ssh/id_ed25519_armpi /tmp/studio_step_limits.json \
  pi@10.42.0.218:/home/pi/docker/tmp/armpi_gateway_step/motion_limits.json
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 \
  'sudo chown -R 1000:1000 /home/pi/docker/tmp/armpi_gateway_step'
```

The generator validates the JSON with the gateway parser. Defaults: 0.10 rad
step cap, 1.0 s duration floor, one joint, ±8 pulse bus verification. Do not
hand-edit pulse bounds; change `configs/joint_mapping.yaml`, then regenerate.

## 2. Start the supervised instance

On the robot host (inside the vendor container; the read-only service on 8765
keeps running). The instance needs the vendor zsh environment for the bus
service:

```bash
sudo docker exec -d -u ubuntu ArmPiUltra /bin/zsh -lc '
  source ~/.zshrc >/dev/null 2>&1;
  cd /home/ubuntu/shared/armpi_gateway_step;
  exec python3 -m armpi_debug_gateway.main \
    --host 10.42.0.218 --port 8766 \
    --token-file /home/ubuntu/shared/armpi_gateway_deploy/token \
    --enable-motion \
    --motion-config /home/ubuntu/shared/armpi_gateway_step/motion_limits.json
'
```

The log must contain `verification=True bus=/ros_robot_controller/bus_servo/get_state`.
Record the PID from inside the container:

```bash
sudo docker exec -u ubuntu ArmPiUltra bash -lc \
  "pgrep -af 'armpi_debug_gateway.main' | grep 8766"
```

## 3. Verify before the backend connects

```bash
TOKEN=$(sudo cat /home/pi/docker/tmp/armpi_gateway_deploy/token)
curl -s -H "X-Armpi-Token: $TOKEN" http://10.42.0.218:8766/health
curl -s -H "X-Armpi-Token: $TOKEN" http://10.42.0.218:8766/motion/status
```

Expected:

```text
health:        motion_enabled=true, motion_mode=step,
               motion_state=idle, measured_ready=true
motion/status: state=idle, mode=step, max_step_rad=0.1, min_duration_s=1.0,
               single_joint_only=true, verification=true,
               verification_tolerance_pulses=8, limits for joint1..joint5
```

A rejected probe (no publication, no motion):

```bash
curl -s -X POST -H "X-Armpi-Token: $TOKEN" -H "Content-Type: application/json" \
  -d '{"command_id":"probe","duration_s":1.0,"joints":[{"joint":"joint1","servo_id":6,"target_pulse":950,"target_rad":1.885}]}' \
  http://10.42.0.218:8766/motion/dispatch
# 400 MOTION_OUT_OF_LIMIT
```

## 4. Start the supervised backend

On the laptop:

```bash
export ARMPI_GATEWAY_URL=http://10.42.0.218:8766
export ARMPI_GATEWAY_TOKEN_FILE=/tmp/armpi_gateway_token   # from the robot token file
export ARMPI_SUPERVISED_MOTION_ACK=I_HAVE_SUPPORT_AND_A_CLEAR_WORKSPACE
backend/.venv/bin/python tools/start-supervised-backend.sh
```

Open `http://127.0.0.1:8000` (keep it bound to localhost). Confirm:

- `/api/health`: `hardware_motion=true`, `supervised_motion=true`,
  `max_step_rad=0.096342`, `min_duration_s=1.0`.
- `/api/state`: five joints labelled `truth=commanded` (the vendor cache) with
  `source=/controller_manager/servo_states (command cache)`; a fresh bus read
  is `measured_on_demand` for ~500 ms after a move.
- `/api/diagnostics`: `safety.ready=true`, no blockers; the
  `mapping_verified` check is satisfied (verified 2026-11-19).

## 5. Execute one joint

Use Joint Control: acquire the lease, enter a target within 0.05 rad of the
measured position, duration ≥ 1.5 s, preview, read the rows, then confirm
execute. Keep the arm supported and the workspace clear.

Expected audit sequence: `Requested → Validated → Dispatched → Observed →
Completed`. With bus verification, `Observed` appears **after** the
interpolation + 0.5 s settle because it is a `GetBusServoState` read; the
actual pulse must be within ±8 of the target. A cache-only completion, a
network/status failure or a failed/unsafe bus read produces `Unknown` and a
**latched** fault; motion stays locked until the backend is restarted after
inspection.

`Stop` publishes a hold at the last measured pulses for 0.1 s and reports
`StopDispatched`. It is not a brake and not an emergency stop. Power/support
remain the physical safety layer.

## 6. Stop the supervised instance

Kill the PID recorded in step 2 (do **not** use `pkill -f '--port 8766'`: the
leading `--` is parsed as an option and the kill silently fails):

```bash
sudo docker exec -u ubuntu ArmPiUltra bash -lc 'kill -TERM <PID>'
```

Then confirm 8765 is still healthy and no 8766 process remains:

```bash
curl -s -H "X-Armpi-Token: $TOKEN" http://10.42.0.218:8765/health
sudo docker exec -u ubuntu ArmPiUltra bash -lc "pgrep -af armpi_debug_gateway.main"
```

The read-only service and its token are unchanged; the motion limits JSON can
stay in the deploy directory and is ignored unless `--enable-motion` is passed.
