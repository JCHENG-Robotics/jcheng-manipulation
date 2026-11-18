# Gripper stop re-measurement — 2026-11-18 19:59

> Status: **the documented gripper open stop was wrong.** It recorded
> `r_joint = -0.8378` (raw 500, register 502) as fully open; that value was a
> **binding stall**. The real linkage stop is register **167** at command raw
> **~150**, so the adopted range doubles from 0.8378 to **2.22 rad**. Operator
> confirmed the new open position is the widest finger gap seen.

## Why this was re-measured

During the `init_pose` restore the operator reported the gripper "has a gap
but does not reach maximum". The servo registered 504–516 at `r_joint = -0.80`
and would not move toward 502, while hand force (torque released on servo ID 1
only) pushed it to register 299 and further. A staircase sweep then found
travel well beyond the documented stop.

## Method

`tools/phase2/gripper_stop_survey.py` — one command at a time on
`/ros_robot_controller/bus_servo/set_position`, wait for the register to
settle, record `command -> register`. Every other servo was held at its
measured register. The operator watched; the gripper was empty.

Note: the vendor path is a **serial interpolation** command (`duration` in
ms), not a queue. Publishing at 20 Hz re-baselines the interpolation from the
current position and produces a slow creep instead of a move — an earlier
20 Hz attempt moved the gripper only 14 counts in 2.2 s.

## Measurements

Close → open sweep (step 50, `gripper_sweep_fwd.json`):

| command raw | register | note |
|---:|---:|---|
| 700 | 640 | closed stop |
| 650 | 640 | stalled at the closed stop |
| 600 | 619 | |
| 550 | 567 | |
| 500 | 520 | the old "open stop" value — only ~60 % of the travel |
| 450 | 468 | |
| 400 | 417 | |
| 350 | 367 | |
| 300 | 315 | |
| 250 | 266 | |
| 200 | 216 | |
| 160 | 176 | near the stop |

Continuation (`gripper_sweep_low.json`) and fine pass
(`gripper_sweep_fine.json`):

| command raw | register | note |
|---:|---:|---|
| 160 | 176 | |
| 120 | 177 | no further travel |
| 80 | 176 | no further travel |
| 40 | 176 | no further travel |
| 0 | 167 | linkage stop |
| 200 → 120 (fine pass) | 167 | already at the stop, no movement |

Return direction: raw 400 read **383** coming back from the stop versus **417**
going toward it → **~34 counts of backlash**.

## Result

| Item | Value |
|---|---|
| Closed stop | raw **700** → register **640** (fingers closed) |
| Linkage open stop | register **167** at raw **~150** |
| Adopted open | raw **170** → register **188** → `r_joint = -2.22` |
| Range | **2.22 rad**, 530 raw counts |
| Command↔register | ratio ≈ 0.86, backlash ≈ 34 counts |
| Confirmation | operator: at raw 170 / register 188 the fingers are at their widest |

`r_joint = -2.22` uses the unchanged 0.00418879 rad/count scale:
`(170 − 700) × 0.00418879 = −2.220`.

## Files updated

| File | Change |
|---|---|
| `ros2_ws/src/armpi_ultra_description/urdf/gripper.urdf.xacro` | `r_joint` limit `-0.8378` → `-2.22`; five mimic joints `±1.57` → `±2.22` |
| `ros2_ws/src/armpi_ultra_moveit_config/config/joint_limits.yaml` | `r_joint` `min_position` → `-2.22` |
| `ros2_ws/src/armpi_ultra_moveit_config/config/armpi_ultra.srdf` | `open` state `-0.80` → `-2.22` |
| `ros2_ws/src/armpi_ultra_moveit_config/config/initial_positions.yaml` | simulation open `-0.800` → `-2.200` |
| `ros2_ws/src/armpi_grasp_planner/armpi_grasp_planner/evaluator.py` | `GRIPPER_OPEN_POSITION` `1.2` (stale, outside every range) → `-2.20` |
| `armpi-debug-studio/configs/joint_mapping.yaml` | `r_joint` exclusion reason now records the measured range |
| `docs/hardware/measured_joint_limits.md`, `docs/hardware/hardware_inventory.md`, `docs/phase2/JOINT_LIMITS.md` | corrected tables and a superseded note |
| `tools/phase2/gripper_stop_survey.py` | new instrument (requires `--yes`) |

## Verification after the rebuild (2026-11-18 20:05)

- `~/jcheng/humble_ws` is a **symlink-install** workspace whose `install/`
  entries point at `$WS/src/<pkg>` → the repo, so the edited YAML/SRDF/URDF are
  visible to the build without copying. The 20:05 relaunch loaded them.
- Plan-only `open` through the running `move_group`:
  `state=open group=gripper target r_joint=-2.22`, **108 waypoints, 10.62 s,
  final `r_joint = -2.210130`** — the old limits would have clamped this to
  −0.8378, so the new range is live.
- **An orphaned `move_group` from the 19:14 launch (PID 2417041) was still
  running** next to the new 20:05 instance: its launch parent had died and it
  had been reparented to the container's PID 1. Two instances with the same
  node name make `/plan_kinematic_path` routing nondeterministic (one holds the
  old limits). The orphan was terminated; one instance remains.
- The command echo was resynced: a single-point
  `/gripper_controller/follow_joint_trajectory` goal at `r_joint = -2.22`
  (2 s) returned `status=4 error_code=0` and `/joint_states` now reads
  `r_joint = -2.2201` with the linkage at register 183 — before that the echo
  still said −0.80 while the fingers were physically at −2.22.

### Open gap: the safety veto rejects gripper-only trajectories

`real_moveit_execute.py open` planned fine but the safety check returned
`approved=False code=WORKSPACE` with

```
trajectory is missing joint1; ...; trajectory is missing joint5;
FK failed at point 0: 'joint1'
```

`armpi_safety` validates a trajectory against the five arm joints and cannot
FK a gripper-only trajectory, so **`--execute` cannot run a MoveIt gripper
move** today. Gripper motion currently goes either through the vendor
`ServosPosition` topic (`tools/phase2/gripper_stop_survey.py`) or the
`/gripper_controller/follow_joint_trajectory` action, both without the safety
veto. Fixing this needs a safety-node change (skip the arm FK checks when the
trajectory touches only `r_joint`).

## Still to do

- ~~**Rebuild/restart the MoveIt stack**~~ — **done** 2026-11-18 20:05 and
  verified by the plan-only `open` above. For the record, the rebuild step is:

  ```bash
  cd /workspace
  source /opt/ros/humble/setup.bash
  colcon build --packages-select armpi_ultra_description armpi_ultra_moveit_config
  source install/setup.bash
  pkill -f 'real_moveit.launch.py'; pkill -f safety_supervisor   # kill BOTH
  ros2 launch armpi_ultra_moveit_config real_moveit.launch.py use_rviz:=false &
  ros2 run armpi_safety safety_supervisor &
  ```

  Check `pgrep -af moveit_ros_move_group/move_group` shows exactly one
  instance: a killed launch can leave its `move_group` orphaned, and two
  instances with the same node name split the service traffic.

  The robot's own deployed `armpi_ultra_description` is the **vendor** copy
  (all limits ±1.57, dated April) and was not modified; the vendor
  `servo_controller` maps `r_joint` through its own raw 0…1000 range
  (`pulse = 700 + r_joint × 238.73`), so it accepts raw 170 unchanged.
- **Inspect the rack/pinion.** The linkage bound at register 504–521 and
  needed hand force to free, and it shows ~34 counts of backlash. That is a
  mechanical symptom, not a command problem; re-check the grip width after any
  service.
- The gripper remains excluded from the gateway's supervised single-joint
  motion path (`Q-08`); no gripper motion path was enabled here.
