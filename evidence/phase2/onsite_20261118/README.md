# Onsite hardware session — 2026-11-18

Supervised session at the robot, following the 7-step onsite runbook. Steps 1–6
are recorded here; an extra independent AprilTag relative-displacement
validation was added after step 6. Step 7 (first real visual grasp) is not part
of this bundle.

| Step | Scope | Result | Record |
|---|---|---|---|
| 1 | Post-reboot read-only health check | **PASS** | [step1_health.md](step1_health.md) |
| 2 | Physical power-cut / stop behavior | **PASS — no brake; arm falls immediately** | [step2_powercut.md](step2_powercut.md) |
| 3 | Gripper sign and range (no object) | **PASS — `-0.8378 … 0.0` rad, positive closes** | [step3_gripper.md](step3_gripper.md) |
| 4 | Real MoveIt Plan → Execute (`camera_forward`) | **Motion PASS; action reporting timed out and was fixed** | [step4_moveit_execute.md](step4_moveit_execute.md) |
| 5 | Real AprilTag detection | **PASS — 5/5, 0.322 m, ≤1.1 mm translation spread** | [step5_tag/README.md](step5_tag/README.md) |
| 6 | Arm joint-limit survey with support | **PARTIAL — joint4 measured; joint2 measured; joint3 observed by hand; joint5 not measurable (falls freely when released); three vendor-node crashes from negative reads, root cause identified** | [step6_torque_mapping.md](step6_torque_mapping.md), [step6_survey_partial.md](step6_survey_partial.md), [step6_survey_run2_20261118.md](step6_survey_run2_20261118.md) |
| V | Independent AprilTag relative-displacement validation (extra, after step 6) | **Partial PASS — lateral 100.67 mm vs 100.0 mm (0.67 %); table-forward 104.87 mm vs 100.0 mm (4.87 %); angle 92.3°; absolute table-pose z is ~90 mm low (FK extrapolation)** | [tag_displacement_validation/README.md](tag_displacement_validation/README.md) |

## Environment

- Robot: ArmPi Ultra / Pi 5, Debian 12, container `ArmPiUltra`, ROS 2 Humble.
- Link: direct Ethernet, laptop `10.42.0.1/24` ↔ robot `eth0 10.42.0.218/24`.
- DDS: vendor CycloneDDS on `eth0`; commands run through `zsh` + `~/.zshrc`.
- Battery after the power-cut reboot: **7704 mV**.
- Root filesystem: **87% used, 6.6 GiB free**.

## Safety facts established

1. There is **no electronic e-stop and no brake**. Cutting main power drops the
   arm immediately because joints 2–5 lose holding torque. Any motion test must
   be supervised, supported, and within reach of the power switch.
2. The vendor stack recovers automatically after power restore: the container
   and action servers return, and `joint1` re-homes to `+0.8378 rad`.
3. `/joint_states` is a **command echo**, not feedback. Physical contact/stall
   must be read from `/ros_robot_controller/bus_servo/get_state`.
4. The gripper's old SRDF `open=+1.2` was wrong and could drive the fingers into
   the open stop. It is replaced by the measured `-0.80` safe-open state.
5. The vendor SDK's torque boolean is **inverted** on this servo firmware:
   `enable_torque=[1, 1]` (0x0B) **releases** torque and `[1, 0]` (0x0C)
   **engages** it; the register read reflects the real state. A selective
   release was demonstrated on the empty gripper. A single publish right
   after startup can be lost to DDS discovery; the survey now waits for the
   subscription match.
6. Hand-found mechanical stops can be **outside the servo command range**:
   joint1 reached 1164 by hand, and a position command at 1164 was clamped
   to ~952. The servo EEPROM position limits are the effective commandable
   range; the provisional ±2.094 rad range is wider than the hardware.
   Back-driving a joint past 0…1000 is unsafe. **Exact cause:** the vendor
   `get_state` service puts the position register into a `uint16` field, so
   a **negative** position raises an `AssertionError` in the callback and
   the node exits with code 1 during that read. Positive out-of-range
   values are harmless. Keep the register positive; stop at the guard
   (`step6_survey_run2_20261118.md`).
7. A released arm joint is **not held by the mechanism**: joint5 fell 196
   pulses in 0.75 s when released with the arm extended. The survey now has
   a free-fall guard that aborts at 60 pulses between reads.
8. Final arm state after this session: torque **on** for all servos; pulses
   `joint1=498, joint2=596, joint3=834, joint4=109, joint5=500`, battery
   7629–7690 mV. `joint4` sits near its low EEPROM bound.

## Changed files from this session

- `ros2_ws/src/armpi_ultra_description/urdf/gripper.urdf.xacro` — measured
  `r_joint` limits.
- `ros2_ws/src/armpi_ultra_description/urdf/armpi_ultra.xacro` — adopted
  EEPROM-bounded arm limits (2026-11-18).
- `ros2_ws/src/armpi_ultra_moveit_config/config/joint_limits.yaml` — measured
  `r_joint` limits and adopted arm limits.
- `ros2_ws/src/armpi_ultra_moveit_config/config/armpi_ultra.srdf` — measured
  `open` / `close` named states.
- `ros2_ws/src/armpi_ultra_moveit_config/config/initial_positions.yaml` —
  simulation rest pose uses the safe open value.
- `ros2_ws/src/armpi_ultra_moveit_config/config/moveit_controllers.yaml` —
  execution duration scaling 1.2 → 1.6, margin 0.5 → 1.0 s.
- `armpi-debug-studio/configs/joint_mapping.yaml` — adopted raw limits,
  `mapping_version: eeprom-bounded-2026-11-18`, `verified: false`.
- `tools/phase2/real_moveit_execute.py`, `capture_real_frames.py`,
  `analyze_real_tag.py`, `servo_hold.py`, `joint_limit_survey.py`,
  `torque_probe.py`.
- `evidence/phase2/onsite_20261118/step6_torque_mapping.md`,
  `step6_survey_partial.md`, `step6_survey_run2_20261118.md`, and the
  `joint_limits_*_20261118.json` files recorded by this session.
- `docs/hardware/measured_joint_limits.md`,
  `docs/hardware/hardware_inventory.md`, `docs/phase2/JOINT_LIMITS.md`.
