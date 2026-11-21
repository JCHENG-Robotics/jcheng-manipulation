# Studio G5 read-only trajectory recorder and live TF verification (2026-11-18)

> **Correction (2026-11-21).** The `feedback` stream in this dataset comes from
> `/controller_manager/servo_states`, which the vendor `ServoManager.set_position`
> writes as a **command cache**. It is not a bus-servo read; the historical
> `feedback.truth=measured` label and the "tracking" metrics are plan-vs-command,
> not servo tracking. The actual registers require `GetBusServoState`; see
> [`evidence/studio_g7/20261121/`](../../studio_g7/20261121/README.md). The raw
> dataset is unchanged; read `feedback` as `commanded`.

> Scope: laptop + robot. The first capture is a **static hold**; a later
> supervised `camera_forward` execute moved the arm for 0.892 s and was
> recorded with the same read-only recorder. The hardware Playwright spec
> attempts an execute and the safety gate rejects it. The gateway token is not
> stored here.

## Environment

- Laptop: Python 3.12.3, pytest 9.2.0, Node v24.22.3, npm 11.19.1,
  Playwright 1.65.1 (Chromium), 2026-11-18.
- Robot: ArmPi Ultra, Pi 5, vendor container `ArmPiUltra`, ROS 2 Humble,
  CycloneDDS vendor stack, gateway systemd service on `10.42.0.218:8765`.
- Robot battery during the run: 7.69 V (64.5%).
- Gateway `/tf`: 11 dynamic + 8 static edges, `status=ok`, freshest 15–50 ms.

## Results

```text
backend pytest            76 passed
gateway pytest            24 passed
Playwright mock E2E       10 passed
Playwright hardware E2E    3 passed
frontend build            tsc -b && vite build OK
recorder static capture    5 s, no motion
commanded samples         242 (~48.16 Hz)
measured samples          241 (~48.15 Hz)
G2 import                 accepted; feedback truth=measured; 0 rad error (static)

supervised moving capture  camera_forward, plan duration 0.892 s
commanded samples         712 (~47.37 Hz)
measured samples          711 (~47.38 Hz)
G2 tracking               max 0.008321 rad, RMSE 0.003962 rad,
                          mean 0.003659 rad, final 0.004048 rad
G2 command fidelity       max 0.008326 rad, RMSE 0.003963 rad
post-motion robot         7.562 V, all joints valid, no fault
```

The moving error is dominated by servo pulse quantization (`0.00419 rad` per
tick) and a one-controller-period start offset from the `0.005` arm threshold
used in that run; the recorder default is now `0.002`.

Hardware Playwright checks:

1. LIVE source; battery `measured` 7.69 V; camera `measured` 640×400; 5 joints
   truth `commanded`; `hardware_motion=false`.
2. G4 `/api/visual`: `tf.status=live`, `source=tf2_topics`, 11 dynamic /
   8 static edges; camera body `rgb_camera_link`, optical
   `camera_optical_calibrated`.
3. Lease → preview → execute: HTTP 403 `GATE_REJECTED`; event log has
   `Failed` and no `Dispatched`/`Completed`.

## Files

| File | Content |
|---|---|
| [`static_capture.json`](static_capture.json) | raw recorder capture: 242 commanded, 241 measured, 2-point static hold plan |
| [`static_dataset.json`](static_dataset.json) | G2 dataset built by the recorder; feedback truth `measured` |
| [`g2_import_response.json`](g2_import_response.json) | backend import + analysis of the static dataset |
| [`moving_capture.json`](moving_capture.json) | raw recorder capture of the supervised `camera_forward` move: 712 commanded, 711 measured |
| [`moving_dataset.json`](moving_dataset.json) | G2 dataset with the executed 10-waypoint plan and measured feedback |
| [`camera_forward_plan.json`](camera_forward_plan.json) | MoveIt plan written by `real_moveit_execute.py --trajectory-json` |
| [`g2_moving_import_response.json`](g2_moving_import_response.json) | backend import + tracking/command-fidelity analysis of the moving dataset |
| [`moving_record.log`](moving_record.log) | recorder log: armed at 0.0084 rad, 15 s capture |
| [`post_motion_health.json`](post_motion_health.json) | gateway health after the move |
| [`post_motion_state.json`](post_motion_state.json) | final commanded joint positions after the move |
| [`post_motion_battery.json`](post_motion_battery.json) | battery after the move (7.562 V) |
| [`hardware_health.json`](hardware_health.json) | hardware backend health (`LIVE`, motion disabled) |
| [`hardware_peripherals.json`](hardware_peripherals.json) | measured battery/camera |
| [`hardware_state.json`](hardware_state.json) | 5 joints, truth `commanded` |
| [`hardware_visual.json`](hardware_visual.json) | visual snapshot with live TF |
| [`hardware_playwright.txt`](hardware_playwright.txt) | gated hardware Playwright log |
| [`gateway_tf.json`](gateway_tf.json) | raw gateway `/tf` snapshot |
| [`tf2_echo.txt`](tf2_echo.txt) | `tf2_echo base_link link5` in the vendor container |
| [`tf_link5_composed.json`](tf_link5_composed.json) | gateway composition `base_link → link5` |
| [`hardware_dashboard.png`](hardware_dashboard.png) | LIVE dashboard with measured camera/battery (camera area blurred for privacy) |
| [`hardware_visual.png`](hardware_visual.png) | Visual Workbench with live TF (camera area blurred for privacy) |
| [`moving_trajectory_page.png`](moving_trajectory_page.png) | G2 Inspector showing the imported moving dataset; the top badge reads `SIMULATION` because this page was served by a mock-mode backend, while the dataset's own feedback truth is `measured` |

## Key values

```text
recorder streams          commanded + feedback; planned from --static-planned
feedback source           /controller_manager/servo_states (pulse -> rad)
static tracking error     0.0 rad (no motion; not tracking accuracy)
moving plan               10 waypoints, 0.892 s, execute_status=4,
                          execute_error_code=1
moving first command      [0.0, 0.414690, -1.252448, -0.716283, -0.012566] rad
moving final command      [0.0, 0.418879, -1.252448, -0.716283, -0.012566] rad
moving final measured     same as final command within one tick
gateway base_link->link5  [0.164158495, 0.000307417, 0.239816373] m
                          q=[-0.000094587, 0.706816809, 0.005552384, 0.707374837]
tf2_echo base_link->link5 [0.164, 0.000, 0.240] m
                          q=[-0.000, 0.707, 0.006, 0.707]
difference                < 0.4 mm; quaternion components < 0.001
```

The comparison is not an independent kinematic check: both readers subscribe to
the same vendor `/tf` graph. It verifies the gateway cache and the Workbench
composition. `tf2_echo` prints three decimals, so the comparison is
display-limited.

## Contamination found and cleared

The first hardware run showed `tf.status=degraded` with a duplicate live
`camera_optical_calibrated` frame and a 5-minute maximum age. Two causes:

- orphaned laptop mock nodes from 2026-11-17
  (`static_transform_publisher` publishing
  `base_link→camera_optical_calibrated`, plus a laptop
  `robot_state_publisher` and `camera_frame_publisher`) were still on the same
  DDS domain. They were stopped before the evidence run.
- the gateway reported the maximum age over all edges, including latched static
  edges. It now reports `max_dynamic_age_ms` and `max_static_age_ms` separately;
  the Workbench uses the dynamic maximum.

After cleanup: 11 dynamic + 8 static edges, no duplicate frame, `status=live`.

## Reproduce

```bash
# unit / API suites (laptop)
cd armpi-debug-studio/backend && .venv/bin/python -m pytest -o addopts="" -q
cd ../ros2_gateway && python3 -m pytest -q
cd ../frontend && npm run build && npm run test:e2e

# hardware, read-only, private gateway (not CI)
cd ../frontend
ARMPI_E2E_HARDWARE=1 \
ARMPI_GATEWAY_URL=http://10.42.0.218:8765 \
ARMPI_GATEWAY_TOKEN_FILE=/tmp/armpi_gateway_token \
npm run test:e2e:hardware

# static recorder run (inside the vendor container)
# see armpi-debug-studio/docs/g5-trajectory-recorder.md
```

## Supervised motion run

The operator confirmed a clear workspace and stayed at the robot. The laptop
Humble MoveIt container ran `real_moveit.launch.py use_rviz:=false`; the
recorder ran in the vendor container with its CycloneDDS environment. The
executed plan was `camera_forward`: 10 waypoints, 0.892 s, `execute_status=4`,
`execute_error_code=1`. The recorder armed on a commanded delta of 0.0084 rad
and captured 15 s. The MoveIt container was removed afterwards; the robot's
final state was 7.562 V with all five joints valid.

## Limits

- One small moving trajectory was captured. It is not a grasp, a repeatability
  study or a broad tracking benchmark; the measured error is dominated by
  servo pulse quantization and a one-period start offset.
- The raw capture is the source of truth; the dataset is derived and
  recomputable with `tools/build_g5_dataset.py`.
- The measured stream is bus-servo position, not an independent encoder.
- Joint mapping and limits remain provisional; `r_joint` is excluded.
