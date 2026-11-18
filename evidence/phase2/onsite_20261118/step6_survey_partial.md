# Step 6 — arm joint-limit survey: partial result and incident

Date: 2026-11-18, after the torque mapping was corrected. Operator at the
robot throughout; the arm was supported. No object in the gripper.

## Status

**Partial.** The survey is unblocked (see
[step6_torque_mapping.md](step6_torque_mapping.md)) and two joints produced
usable data, but a vendor-node crash during joint3 stopped the run. Joints
3–5 are still unmeasured. `mapping_verified` stays false.

| Joint | Servo | Result | Valid? |
|---|---|---:|---|
| joint1 | 6 | first run: 501…503 (no movement); second run: positive side reached **1164**, guard aborted; servo clamped the recovery hold to **952…954** | partial: positive side beyond servo range; negative side unmeasured |
| joint2 | 5 | **322…873** (−0.746…1.562 rad, span 2.308), repeat spread 1 pulse | yes, but see EEPROM note |
| joint3 | 4 | pushed to **−41**, then the vendor node died | no |
| joint4 | 3 | not measured | no |
| joint5 | 2 | not measured | no |

## What the measurements mean

The hand-found mechanical stops are **outside the servo's commandable
range** on the positive sides:

- joint1 reached **1164** by hand; a position command at 1164 was clamped
  by the servo to **952…954** (EEPROM limit 950).
- joint2's high stop **873** is beyond its EEPROM limit **830**.
- joint2's low stop **322** is *inside* its EEPROM range 160…830, so there
  the mechanical stop is the binding limit.

The servo EEPROM position limits (read-only, captured in
[step6_blocked.md](step6_blocked.md)) are therefore the **effective
commandable limits**, and the provisional ±2.094 rad software range is
wider than the hardware allows. Do not adopt ±2.094.

### Effective limits implied by the EEPROM table

`rad = (pulse - 500) * 0.00418879` (joint3 inverted).

| Joint | EEPROM pulse | Effective rad | Measured mechanical stop |
|---|---|---|---|
| joint1 | 50…950 | −1.885…1.885 | positive ≥1164 (outside range) |
| joint2 | 160…830 | −1.424…1.382 | low 322 (inside, binds); high 873 (outside) |
| joint3 | 70…950 | −1.885…1.801 | not measured |
| joint4 | 100…900 | −1.676…1.676 | not measured |
| joint5 | 100…890 | −1.676…1.634 | not measured |

These are **not adopted** here. Tightening the URDF / MoveIt limits is a
separate reviewed change, and joint3–5 still need a measurement.

## Incident: vendor node exit code 1

During the joint3 window the operator moved joint3 past the servo's
reporting range (register **−41**). The next state read killed the vendor
node:

```
[1794968833.3967228] [ERROR] [ros_robot_controller-1]: process has died
[pid 1192, exit code 1, cmd '.../ros_robot_controller --ros-args']
```

The tool then lost the service and could not restore torque itself. The
arm was supported throughout. Recovery:

1. `ros_robot_controller` was restarted in place (no container restart, so
   the vendor `startup_check` was not run):
   `docker exec -d -u ubuntu ArmPiUltra /bin/zsh -c 'source ~/.zshrc;
   exec .../ros_robot_controller --ros-args'`
2. A read-back showed all five joints at torque 1 and inside the
   provisional ranges; joint3 was at 829.
3. The survey tool was hardened before retrying joint1:
   - sample **one servo per request** (the crash followed a five-servo
     read);
   - abort **before the next read** if the pulse leaves 0…1000;
   - clearer "vendor node not available" messages.

The hardened tool then sampled joint1 at **1164** and aborted cleanly — the
node survived — so the guard addresses the observed failure mode. The
exact SDK exception was not captured (the node's stderr is the container
TTY); treat out-of-range back-driving as unsafe.

## State after the session

Post-restart read in [`post_incident_state.json`](post_incident_state.json)
(02:28:45Z, battery 7645 mV): `joint1=498, joint2=597, joint3=829,
joint4=113, joint5=501`, all torque **1**. The base was then returned to
503 with a 3 s `ServosPosition` command; the other joints were held, so the
final pose is `joint1=503, joint2=596, joint3=830, joint4=113,
joint5=501`, all torque **1**.

## Safety rules confirmed

1. Support the arm before releasing any joint; joints 2–5 fall.
2. Stop at the **first** mechanical resistance; do not force a joint past
   the servo's 0…1000 range.
3. Keep the power switch in reach (no e-stop).
4. If the vendor node dies, keep supporting the arm and restart only that
   node; do not restart the container/robot while a joint is free.
5. A position hold at the current registers is the recovery path.

## Remaining work

- Measure joint3, joint4, joint5 with the hardened tool.
- Re-measure joint1's negative side and joint2's low stop repeatedly.
- Review and adopt limits (EEPROM-bounded, with margin) as a separate
  change; keep `mapping_verified=false` until then.
