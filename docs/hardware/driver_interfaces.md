# Driver Interfaces — [S] source audit only

> Source: `docs/armpi/g0/02_sdk_api_audit.md`, `03_driver_interface_inventory.md`.
> No real Driver was started; no serial port was opened. Treat every runtime
> field as `[U]`.

## Known vendor interfaces (source-level)

| Interface | Type | Notes | Status |
|---|---|---|---|
| `/controller_manager/joint_states` | topic | last commanded pulse converted to rad; **not measured feedback** | `[S]` |
| `ros_robot_controller/bus_servo/get_state` | service | on-demand single-servo read | `[S]` |
| `bus_servo_set_position` | SDK call | one serial write; no ACK | `[S]` |
| `/servo_controller` | topic | pulse command path | `[S]` |
| `servo_controller` `FollowJointTrajectory` | action | open-loop; no feedback; `succeed()` without position check | `[S]` |
| `kinematics` services | services | IK/pose; optional for G1 | `[S]` |

## Known vendor defects (from G0)

- `get_torque_state` calls a non-existent method → `AttributeError`.
- `pub_button_data` uses an unimported `PacketReportKeyEvents` → `NameError`.
- `/joint_controller` callback parameter type is wrong.
- SDK reads use `queue.get(block=True)` with no timeout; single-threaded executor can freeze.
- Multiple paths can open `/dev/rrc`; no single-owner guarantee.

## Required before hardware use

- Read-only probe of device node and topics.
- D-10 position-read semantics test (torque off, mechanically fixed, hand-turn).
- Confirm whether `joint_states` is echo or measured.
- Measure feedback rate/latency.
