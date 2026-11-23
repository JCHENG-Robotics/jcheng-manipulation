# Studio gateway hardware validation — 2026-11-23

Scope: validate the offline gateway refactors on the real ArmPi Ultra:
`/safety_stop` consumer, joint2 775-pulse strict bound, `CameraCache`,
`telemetry_format`, and the recovery procedure after a reboot. No new
algorithms and no autonomous operation.

## Setup

- Robot: ArmPi Ultra, direct Ethernet `10.42.0.218`, container `ArmPiUltra`.
- `armpi-gateway.service` on **8765** stayed read-only and untouched.
- The updated gateway package was copied to
  `/home/pi/docker/tmp/armpi_gateway_pose` and started on **8766** with
  `--enable-motion --motion-config .../takeover_limits.json --no-aux-camera`.
- A separate read-only instance on **8767** used the same package with
  `--camera-topic /depth_cam/rgb/image_raw --no-aux-camera` to exercise
  `CameraCache` and `telemetry_format`.
- `armpi_safety` is **not installed** in the slim container
  (`ros2 pkg prefix armpi_safety` → not found). The stop test therefore used a
  transient-local `std_msgs/Bool` publisher on `/safety_stop`, not the real
  supervisor. This validates the gateway consumer only.

## 1. Command cache was stale after reboot

The vendor `ServoManager` initializes `/controller_manager/servo_states` from
the `init` parameters, not from the actual registers.

| Servo | Joint | Actual bus (first read) | Cache before resync |
|---|---|---:|---:|
| 1 | r_joint | 216 | 500 |
| 2 | joint5 | 499 | 500 |
| 3 | joint4 | 105 | 500 |
| 4 | joint3 | 831 | 500 |
| 5 | joint2 | 600 | 500 |
| 6 | joint1 | 504 | 700 |

A hold at the cached pose would have commanded a large motion. It was **not**
sent.

## 2. Cache resync (no intentional motion)

Procedure: read the actual registers with `GetBusServoState`
(`GetBusServoCmd(id, get_position=1)`, IDs 1–6), then publish
`servo_controller_msgs/ServosPosition` to `/servo_controller` with
`duration=2.0`, `position_unit='pulse'` and **target = actual** (two publishes).
`ServoManager.set_position` updates the cache; the bus target is the current
pose.

- Actual at resync: `{1: 224, 2: 501, 3: 113, 4: 830, 5: 597, 6: 497}`
- Bus after hold: `{1: 224, 2: 501, 3: 113, 4: 834, 5: 597, 6: 497}` (≤4 counts)
- Cache after: `joint1 497, joint2 597, joint3 830, joint4 113, joint5 501,
  r_joint 224` — matches actual within 1–4 pulses.

## 3. `/safety_stop` consumer

Fake `Bool true` → gateway latched, published a hold at the corrected cache,
and `/motion/dispatch` returned `SAFETY_STOP` (409). `Bool false` cleared the
latch. Actual positions were unchanged by the hold.

- Log: `safety_stop latched by armpi_safety`,
  `hold command published; physical stop is not confirmed`.
- Status: `safety_stop: true`, then `false` after clear.
- This is a hold, not a brake or an e-stop.

## 4. Recovery to the initial pose

Studio preview/execute, five-joint pose, `duration_s=10.0`:

- Target: `joint1 700, joint2–5 500` (near-limit warning on joint3; accepted).
- Start TCP `[0.1853, -0.0013, 0.1137]`, table clearance `0.1928 m`;
  target TCP `[0.0732, 0.0081, 0.4185]`, clearance `0.2072 m`; 21 samples,
  no violations.
- Bus-verified actual: `joint1 694, joint2 500, joint3 504, joint4 495,
  joint5 501` (deviations −6…+4 pulses, within ±12).
- Gripper was left open at 225; it was not closed to 500.

## 5. joint2 small move

Studio preview/execute, `duration_s=3.0`:

- `500 → 540`: actual `539`, other joints stable; bus verification `bus`.
- `540 → 500`: actual `500`, other joints stable; bus verification `bus`.
- The on-container `fast_motion` strict check rejected a 789-pulse joint2
  target (`outside adopted (342, 775)`); the valid view pose still produced
  only the expected joint4 94 < 120 warning.

## 6. Camera / telemetry on 8767

- Battery: 7.739 V, 67 %, `ok`, `valid`, age 19 ms.
- Camera: 640×400, `bgr8`, 5.6 fps, 61 frames; snapshot HTTP 200, JPEG
  `ffd8`, 20,025 bytes.
- TF: 11 dynamic + 11 static edges, status `ok`, freshest 83 ms; static max
  10,389 ms (latched).
- `/state` measured: `joint1 700, joint2–5 500, r_joint 224`; all joints
  `valid`.

## Limitations

- No real `armpi_safety` node was running; the stop test used a fake Bool.
- No e-stop, brake, self-collision model, or independent encoder.
- 8766 and 8767 were stopped after the tests; only 8765 remains read-only.
- The arm was supported and the workspace clear throughout; the operator was
  present.

## Artifacts

- `validation.json` — structured numbers from this run.
