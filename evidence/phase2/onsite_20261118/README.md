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
| V | Independent AprilTag relative-displacement validation (extra, after step 6) | **Partial PASS — lateral 100.67 mm vs 100.0 mm (0.67 %); table-forward 104.87 mm vs 100.0 mm (4.87 %); angle 92.3°.** The original absolute-z interpretation (RGB biased ~40–50 mm) is **superseded** by step D: the tag's small-plane absolute z is itself biased 6–13 mm low; the lateral scale stands. | [tag_displacement_validation/README.md](tag_displacement_validation/README.md) |
| T | Table-down chessboard calibration + depth check (extra) | **Relative PASS — 2.7 mm / 0.42° board scatter, 2.2 mm / 0.29° held-out, 3.1 mm / 0.45° LOO.** The absolute-depth conclusion is **superseded** by step D. The board is **11 mm** thick and invisible to the depth (holes filled); the same-surface RGB-D gap is −4.6 mm, so the 6.1 mm board-vs-table difference is **not** the board thickness. | [../../calibration/20261118/table_down/README.md](../../calibration/20261118/table_down/README.md) |
| A | Supervised object-free approach at `camera_forward` (plan-only + execution) | **PASS — 3/3 safety-approved plans; executed best plan to TCP (0.259, −0.001, 0.235), 1 mm from the plan.** Kinematic finding: the camera_forward view volume is not a grasp volume (max reach ~0.10 m in front of the lens). | [approach_20261118/README.md](approach_20261118/README.md) |
| D | Depth table plane + absolute table alignment | **PASS — 147,626-inlier plane, 0.7 mm RMS, normal 3.66°; table z = −0.0787 m (depth) vs −0.0784 m (215 mm ruler), 1.3 mm apart.** RGB board top is 6.1 mm above the depth plane, but that is **not** the board thickness (board = 11 mm, invisible to the depth); the same-surface RGB-D gap is −4.6 mm. Plan-only alignment target approved but its candidate sits on the joint2 bound, so it was not executed. | [depth_plane_20261118.md](depth_plane_20261118.md) |
| S | Same-frame RGB-D same-surface check with a matte AprilTag (extra) | **DONE — RGB tag vs depth paper −4.25 / −4.65 mm (0.37° / 0.76°); depth paper vs table +1.66 mm vs ~0.1 mm paper; RGB absolute z biased −3 to −5 mm.** The chessboard is not a depth target; use the depth table plane for z. | [rgbd_same_surface_20261118.md](rgbd_same_surface_20261118.md) |
| C | Chassis link, zero wheel motion (extra) | **PARTIAL — the ROS → STM32 path is verified: `/cmd_vel` → `mecanum_chassis_node` → four-motor `MotorsSpeedControl` (all 0.0), battery 7691 mV; node start/stop clean.** Speed scale, direction, odometry and `/cmd_vel` safety coverage are **not verified**; a supervised ground pulse is the next step. | [chassis_link_20261118.md](chassis_link_20261118.md) |
| W | Wired → office WiFi switch (extra) | **PASS with caveats — SSH, DDS (`/joint_states`, `/battery` 8289 mV, `/tf` 12.5 Hz) and the gateway HTTP all work over `wlp4s0`/`wlan0`; robot DDS `wlan0`, laptop `wlp4s0`, gateway bound to `10.51.128.226`.** The core restart left the arm unmoved; the camera launch was restarted (6.5 fps in-container) but cross-host image topics are only ~2 Hz over WiFi. The SSID is open, so the graph and gateway are LAN-exposed. | [wireless_switch_20261118.md](wireless_switch_20261118.md) |

## Environment

- Robot: ArmPi Ultra / Pi 5, Debian 12, container `ArmPiUltra`, ROS 2 Humble.
- Link: direct Ethernet, laptop `10.42.0.1/24` ↔ robot `eth0 10.42.0.218/24`,
  until the 2026-11-18 afternoon switch to office WiFi
  (`wlp4s0 10.51.128.246/23` ↔ `wlan0 10.51.128.226/23`, step W). The cable
  stays attached as the fallback.
- DDS: vendor CycloneDDS; `eth0` before step W, `wlan0` after; commands run
  through `zsh` + `~/.zshrc`.
- Battery after the power-cut reboot: **7704 mV**; **8289 mV (94.5 %)** after
  the step-W reboot.
- Root filesystem: **87% used, 6.6 GiB free**.

## Link and DDS fixes (2026-11-18, needed for the MoveIt graph)

The wired link was in NetworkManager `shared` mode. Shared mode drops inbound
UDP from the robot (measured **0/300** packets) while laptop→robot UDP works
(300/300), so the laptop's DDS participants never discovered the robot. Even
after switching to a plain static link, EEE (Energy-Efficient Ethernet) caused
intermittent loss (ping 23/30, SSH banner timeouts). Fixes:

1. Laptop `armpi-direct`: `ipv4.method manual`, `10.42.0.1/24`, no gateway;
   robot `Wired connection 1`: static `10.42.0.218/24`, no gateway.
2. Laptop `ethtool.eee-enabled off` (the robot's older NM has no EEE property;
   disabling it on one side disables EEE on the link). After this: ping 30/30,
   UDP 300/300.
3. The Humble container's `-v .../cyclonedds-laptop.xml:/workspace/cyclonedds-laptop.xml`
   mount resolved to a **0-byte root-owned placeholder** inside the `/workspace`
   bind mount. CycloneDDS then had no interface config. Use a workspace-local
   file (`/workspace/cyclonedds-wired.xml`) instead of mounting a file under
   `/workspace`.


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
