# armpi_safety real-node installation — 2026-11-23

Goal: install and run the real `armpi_safety` node in the slim `ArmPiUltra`
container, without modifying the vendor workspace or the 8765 read-only
gateway. This closes the "safety_stop test used a fake Bool" caveat from
`evidence/studio_validation_20261123/`.

## Dependencies found in the container

- `moveit_msgs`, `action_msgs`, `rclpy`, `sensor_msgs`, `std_msgs`, `std_srvs`:
  present in `/opt/ros/humble`.
- `PyKDL` and `xacro`: present.
- `urdf_parser_py`: **missing**, and `pip install` hung on the robot network.
  It was copied from the laptop's Jazzy install
  (`/opt/ros/jazzy/lib/python3.12/site-packages/urdf_parser_py`) to the
  persistent shared mount:
  `/home/ubuntu/shared/armpi_tools/urdf_parser_py` (host
  `/home/pi/docker/tmp/armpi_tools`). It is pure Python and loads under 3.10.
- `colcon`, `cmake`, `gcc`: present.

## Overlay install

The three packages were copied from the repo and built in a persistent
shared overlay, **not** in `/home/ubuntu/ros2_ws/install`:

```text
/home/ubuntu/shared/safety_ws        (host /home/pi/docker/tmp/safety_ws)
  src/{arm_interfaces,armpi_grasp_planner,armpi_safety}
  install/...
```

Build:

```bash
source /opt/ros/humble/setup.bash
source /home/ubuntu/ros2_ws/install/setup.bash
cd /home/ubuntu/shared/safety_ws
colcon build --symlink-install \
  --packages-select arm_interfaces armpi_grasp_planner armpi_safety
```

Result: **3 packages finished [26.7 s]**; only setuptools deprecation warnings.

`ros2 pkg prefix armpi_safety` →
`/home/ubuntu/shared/safety_ws/install/armpi_safety`.

## Start

Use **`setup.zsh`**, not `setup.bash`, inside zsh (the `.bash` setup does not
reliably set the overlay paths under zsh). The vendor `~/.zshrc` is required
for CycloneDDS:

```bash
docker exec -d -u ubuntu ArmPiUltra /bin/zsh -lc '
  source ~/.zshrc >/dev/null 2>&1;
  source /opt/ros/humble/setup.zsh;
  source /home/ubuntu/ros2_ws/install/setup.zsh;
  source /home/ubuntu/shared/safety_ws/install/setup.zsh;
  export PYTHONPATH=/home/ubuntu/shared/armpi_tools:$PYTHONPATH;
  exec ros2 run armpi_safety safety_supervisor > /tmp/armpi_safety.log 2>&1'
```

The node is running now; it is **not** a systemd service, so after a container
restart the command must be re-run. The overlay itself persists.

## Verification

- `ros2 node list` → `/armpi_safety`.
- `/safety_status` → `state.valid=true`, `stop=false`, `stop_latched=false`.
- `/safety_stop` → `false`.
- `ros2 service call /armpi_safety/reset_stop std_srvs/srv/Trigger` →
  `success=True`, `stop cleared`.
- Empty trajectory → `approved=false`, `code=TRAJECTORY_INVALID`.
- Current-pose 2-point trajectory (1 s) → `approved=true`, `code=OK`,
  `workspace_checked=true`, `table_checked=true`; log:
  `workspace FK ready: base_link -> end_effector_link`.
- The 8766 pose gateway was started and saw the real node's
  `/safety_stop=false`; it was stopped again afterwards.

## Limitations

- The real **state-fault latch** (invalid/stale `/joint_states` → true) was not
  fault-injected on hardware; it is covered by the unit tests
  (`armpi_safety/test/test_policy.py`). The gateway consumer was tested with a
  fake Bool in `evidence/studio_validation_20261123/`.
- No e-stop, brake, self-collision model or independent encoder.
- The node is a manual process, not a boot service.
- 8766 was stopped after the check; 8765 remains read-only. The arm is at the
  initial pose, gripper open.
