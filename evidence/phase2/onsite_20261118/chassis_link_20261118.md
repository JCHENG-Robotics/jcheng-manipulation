# Chassis link verification (zero motion) — 2026-11-18

> Status: **PARTIAL.** The ROS → STM32 path for the mecanum base is verified
> end to end with **zero wheel motion**: a `/cmd_vel` message reaches
> `mecanum_chassis_node`, which publishes a four-motor `MotorsSpeedControl`
> that the vendor controller accepts. The **speed scale and wheel directions
> are not verified** — they need a supervised non-zero pulse on the floor.
> No odometry, no IMU data, and no `/cmd_vel` safety supervision exist.

## Environment

| Item | Value |
|---|---|
| Robot | ArmPi Ultra / Pi 5, container `ArmPiUltra`, ROS 2 Humble |
| Link | direct Ethernet, robot `eth0 10.42.0.218/24`, ping 0.23 ms |
| Battery | **7691 mV** (`/ros_robot_controller/battery`) |
| Arm state | unchanged; only the chassis node was started and stopped |
| Chassis node | not running by default (matches "chassis not started" in `docs/armpi/g0/03_driver_interface_inventory.md`) |

## What was run

1. `ros2 pkg executables controller` → `controller mecanum`; prefix
   `/home/ubuntu/ros2_ws/install/controller`.
2. Started `ros2 run controller mecanum` detached in the container.
3. Published ten zero `geometry_msgs/Twist` messages at 5 Hz on `/cmd_vel`
   while echoing `/ros_robot_controller/set_motor_speeds`.
4. Stopped the node and re-checked the graph.

## Results

| Check | Result |
|---|---|
| Node start | `mecanum watchdog enabled: timeout=0.500s rate=20.0Hz` |
| `/cmd_vel` subscriber | 1 (`mecanum_chassis_node`) |
| Motor topic subscriber | 1 (`ros_robot_controller`) |
| Echoed command | `id 1: 0.0`, `id 2: -0.0`, `id 3: -0.0`, `id 4: 0.0` |
| MCU link | battery topic returns 7691 mV (same STM32 as the wheels) |
| Shutdown | node gone; `set_motor_speeds` publisher count back to 0 |
| `/cmd_vel` safety | none — `/armpi_safety` subscribes `/joint_states` only |

## Interface map

```
/cmd_vel (geometry_msgs/Twist)
  → mecanum_chassis_node (package controller)
      wheelbase 0.195 m, track 0.22 m, wheel diameter 0.0965 m
      rps = v / (π · d), signs (+,-,-,+), clamp ±(127/-126)
  → /ros_robot_controller/set_motor_speeds (MotorsSpeedControl, ids 1..4)
      → ros_robot_controller → SDK set_motor_duty() (sub-command 0x05)
      → STM32 over the CH340 serial link
```

The deployed `mecanum.py` is a hardened fork of the vendor file (watchdog
described in `docs/hardware/vendor_baseline.md` item 5, diff in
`docs/hardware/patches/controller_mecanum_hardened.patch`).

> **Superseded 2026-11-18:** the ground tests in
> [chassis_motor_channels_20261118.md](chassis_motor_channels_20261118.md)
> now verify the wheel map, all three sign tables and the duty scale; the
> node maps m/s to duty with `duty = v × 182`.

## Findings

1. **Unit mismatch, scale unknown.** The node computes wheel **rps** and puts
   it in `MotorSpeedControl.speed`, but the vendor callback passes that field
   to `set_motor_duty` (0x05). The sibling `mecanum_2.py` sends **percent ×
   100**. For `linear.x = 0.02` the node publishes `±0.065971`; under the rps
   assumption that is ≈0.02 m/s, under a duty-percent assumption it is
   ≈0.07 % and likely below the motor deadband.
2. **No odometry.** No encoder reads, no `/odom`, no `odom` frame, no running
   EKF (`driver/controller/config/ekf.yaml` references `odom_raw`, which
   nothing publishes). The chassis is open loop; the only displacement
   reference is the camera/TF.
3. **IMU silent.** `/ros_robot_controller/imu_raw` has a publisher but produced
   no message in 12 s, consistent with the known vendor issue; no yaw feedback.
4. **No safety coverage.** `/cmd_vel` has no speed cap, timeout outside the
   node, or e-stop. The node watchdog (0.5 s) only runs while the node lives;
   node death leaves the last duty on the MCU.
5. **Topic mismatch in vendor teleop.** Vendor teleop publishes
   `controller/cmd_vel` (0.2 m/s, 0.5 rad/s) while the chassis node subscribes
   `/cmd_vel`; a remap is required.
6. **Chassis motion invalidates the table calibration.** The measured table
   plane (`z ≈ −0.078 m`) assumes the base has not moved; any chassis travel
   requires re-measuring it (`docs/phase2/SAFETY_LAYER.md`).

## Not verified — requires a supervised ground test

Use `tools/phase2/chassis_jog.py` (run in the container). Preconditions: robot
on the floor, arm tucked, ≥1 m clear, power switch within reach (**no e-stop
and no brake**: cutting power drops the arm), battery ≥ 7.5 V.

| Step | Command | Pass criterion |
|---|---|---|
| 1 | `--status` | chassis node up, 1 motor subscriber, battery present |
| 2 | `--axis x --speed 0.02 --duration 0.5 --yes` | robot creeps forward; operator measures travel |
| 3 | same with `--speed -0.02` | reverses |
| 4 | `--axis y --speed 0.02 --duration 0.5 --yes` | strafes left/right as expected |
| 5 | `--axis yaw --speed 0.1 --duration 0.5 --yes` | rotates counter-clockwise |
| 6 | repeat step 2 at 0.05 / 0.1 | fit m/s per unit from measured travel |

The jog tool publishes at 20 Hz, sends an explicit zero afterwards, verifies
the echoed command is zero, and writes a JSON record to `/tmp`.
