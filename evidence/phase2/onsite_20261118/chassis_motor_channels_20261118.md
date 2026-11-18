# Chassis motor channels — first motion-verified connection (2026-11-18)

> Status: **ALL THREE SIGN TABLES AND THE TWO-POINT SPEED SCALE VERIFIED.**
> The wheel motors are not bus servos: they are four DC channels behind
> `set_motor_duty` (0x05) on the STM32. They have **no feedback path** in the
> vendor SDK (no ID, no position register, no ACK, no motor report in the
> protocol parser), so this test proves the connection by motion, not by a
> readback.

## Why this test exists

Before 2026-11-18 the chassis was only exercised with **zero** duty: a
`/cmd_vel` message was traced to `mecanum_chassis_node`, then to
`MotorsSpeedControl`, and the echoed command was zero. Nothing proved that the
motor packets reached the MCU or that the wheels were wired. The SDK has no
motor read function, and `MotorsState.msg` / `MotorState.msg` (id, rps) are
never published or subscribed in the vendor stack, so a readback test is
impossible by construction.

## Environment

| Item | Value |
|---|---|
| Robot | ArmPi Ultra / Pi 5, container `ArmPiUltra`, ROS 2 Humble |
| Link | direct Ethernet `10.42.0.218`, SSH as `pi` |
| Battery | 7846 mV at the read-only status check |
| Chassis node | **not running** (direct `MotorsSpeedControl` pulses) |
| Arm state | low/fallen pose (measured joint4 = 109 pulse, gripper = 224) |

## Method

`tools/phase2/motor_pulse.py` (new; motion needs `--yes`):

- publishes `MotorsSpeedControl` directly on
  `/ros_robot_controller/set_motor_speeds` at 20 Hz, bypassing
  `mecanum_chassis_node` and `/cmd_vel`;
- hard caps `|duty| <= 20`, `duration <= 1.0 s`; explicit zero afterwards and
  `echoed zero: True` verified on every run;
- the duty field is a percentage: 10 % was already visible on a free wheel.

## Results

### Per-channel pulses (10 %, 0.3 s, one ID at a time)

Wheels lifted; operator watched each pulse. The operator's viewing frame was
not recorded, so the rotation senses below are the raw report, not a sign table.

| ID | Wheel | Observed |
|---|---|---|
| 1 | right front | clockwise |
| 2 | right rear | counterclockwise |
| 3 | left front | counterclockwise |
| 4 | left rear | counterclockwise |

This matches the vendor `mecanum_2.py` order
`[right_front, right_rear, left_front, left_rear]` → IDs 1..4.

### Forward sign pattern (on the floor)

Command sent twice: ID1 **+**, ID2 **−**, ID3 **−**, ID4 **+** — the vendor
`mecanum_2.py` forward pattern (`invert_motor = {2, 3}`). First 10 % / 0.5 s
(too brief for the operator to see), then 20 % / 1.0 s.

Result: **the robot moved forward** (operator observation). This confirms both
the wheel map and the forward sign table.

### Measured duty → speed (tile floor, ruler)

Forward pattern, one 1.0 s pulse per point, ruler measured from a floor mark.
The arm was clear of the floor and the operator watched throughout.

| duty | displacement in 1.0 s | mean speed |
|---|---|---|
| 10 % | 55 mm | 0.055 m/s |
| 20 % | 110 mm | 0.110 m/s |

Two-point fit through the origin: **≈0.0055 m/s per duty unit** (5.5 mm/s per
unit), with no measurable deadband at or above 10 %. The vendor `m/s × 100`
mapping implies 0.010 m/s per unit, so the real base runs at **≈55 % of the
nominal vendor scale**; honoring an m/s command needs
`duty = speed_mps × 182` (e.g. 0.1 m/s → 18.2 duty).

Caveats: one run per point; 1.0 s includes spin-up and stop; tile floor,
battery ≈7.8 V, open loop, no odometry; ruler error ±5 mm. Below 10 % the
deadband is unmeasured.

## What this proves / does not prove

Proves:

1. All four motor channels and their wiring are live — `set_motor_duty` (0x05)
   reaches the MCU and drives the wheels.
2. Wheel map: ID 1 right front, ID 2 right rear, ID 3 left front, ID 4 left rear.
3. Sign tables (re-verified after the adjustment below): forward `[+,−,−,+]`,
   left `[+,+,+,+]`, counter-clockwise yaw `[+,−,+,−]`; reverse is the negation.
4. Open-loop speed scale on tile: duty ≈ 182 × v[m/s] (two points, linear
   through the origin above 10 %).

Does **not** prove:

- any motor feedback (there is none in the SDK/firmware report set);
- odometry, IMU yaw, or closed-loop speed control;
- that the motor command is acknowledged — the echo only shows the ROS node
  accepted a zero command;
- the sub-10 % deadband or the steady-state (spun-up) speed.

## Consequence for `mecanum.py` (resolved)

The vendor `mecanum.py` put wheel **rps** in the field the controller feeds to
`set_motor_duty`, so `linear.x = 0.1 m/s` published ≈0.33 (≈0.3 % duty), far
below the 10 % that visibly turns a wheel. The hardened node now uses the
measured map `duty = v × 182` with a 60-duty cap and keeps the sign table; the
`/cmd_vel` check below confirms the published duty.

## Safety note

The base moved on the floor. This invalidates the table-plane calibration
(`z ≈ −0.078 m` assumed a stationary base, `docs/phase2/SAFETY_LAYER.md`).
Re-measure before the next arm/table task. No e-stop and no brake exist.

## Re-verification after the chassis adjustment (2026-11-18, evening)

The operator adjusted the chassis wheel actuators after the first run. The
robot is on the floor, the arm is in the measured table-down pose
(498/596/831/113/500), and the link is office WiFi (`10.51.128.226`). The same
`motor_pulse.py` was used; every pulse echoed zero afterwards.

| Axis | Duty pattern (ID1..4) | Observation |
|---|---|---|
| +x | `[+,−,−,+]` | forward; −x returned to the start |
| +y | `[+,+,+,+]` | left (two 20 % pulses, ~220 mm total); `[−,−,−,−]` returned |
| +yaw | `[+,−,+,−]` | counter-clockwise (two 20 % pulses); `[−,+,−,+]` returned |

Scale re-measured with a ruler on the same floor: **10 % → 55 mm** in 1 s,
**20 % → 110 mm** in 1 s, cumulative 165 mm after both; the reverse pulses
returned to the origin. The scale is unchanged by the adjustment:
**`duty = v[m/s] × 182`**, no deadband at or above 10 %.

End-to-end `/cmd_vel` check with the updated node
(`DUTY_PER_MPS = 182`, `MAX_DUTY = 60`, watchdog 0.5 s @ 20 Hz):

| Command | Published duty | Result |
|---|---|---|
| `linear.x = +0.05` | `[9.1, −9.1, −9.1, 9.1]` | forward ≈55 mm |
| `linear.x = −0.05` | `[−9.1, 9.1, 9.1, −9.1]` | back to the start |

Both runs echoed an explicit zero. The node's log confirms
`mecanum duty scale: 182.0 duty per m/s, cap 60.0`.

## Next

1. Steady-state scale: a longer measured path (e.g. 0.3 m) to remove the
   spin-up bias from the 0.11 m/s estimate.
2. Re-measure the table plane before any table task (the base moved).
3. Add a `/cmd_vel` supervisor (speed cap, stale-command stop, e-stop path)
   before letting the Studio drive the base.
