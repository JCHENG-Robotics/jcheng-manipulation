# Fast, smooth arm motion

> Status: implemented and hardware-verified for the grasp / put-back sequence on
> 2026-11-20 (no-cube cycle, **5.43 s** vs ~37 s). A real-cube re-grip with the
> new timing is still open. Evidence:
> [`evidence/phase2/onsite_20261120/fast_motion/`](../../evidence/phase2/onsite_20261120/fast_motion/).

## Why the old motion was slow and why it shook

The vendor `FollowJointTrajectory` controller forwards **one bus-servo command
per trajectory point** with that segment's duration
(`servo_controller/joint_trajectory_action_controller.py`), and the servo
interpolates linearly over the duration. Every previous move was a **single
point** lasting 6-9 s, so it was one constant-velocity move with an
instantaneous velocity step at the start and at the stop. At 0.06-0.10 rad/s
(3-6 deg/s) the joints also sit in the gear stiction band. Slow and shaky are
the same defect.

## The profile

`tools/phase2/motion_profile.py` (pure Python, unit-tested on the laptop):

* one scalar path parameter `s(t)`, a trapezoid with **half-cosine ramps**, so
  the acceleration is continuous and the jerk is bounded;
* every joint follows the same `s(t)`: a straight line in joint space, all
  joints start and stop together;
* `vmax` is the peak joint velocity and `amax` the peak joint acceleration of
  the fastest joint; both limits are hit exactly (or the move is triangular);
* waypoints are sampled at `dt <= 0.05 s`; the endpoints are exact;
* `dt >= 0.025 s`, because the vendor clamps a segment shorter than 20 ms up to
  20 ms and would otherwise stretch the move.

The vendor path has no meaningful per-segment cost: a zero-motion bench of
20/40/80/200 points over 2.0 s measured 0.27/0.10/0.07/0.01 ms of overhead per
segment. Dense waypoints are therefore free, and the S-curve is real.

## Measured settle, not a fixed sleep

`fast_motion.py` waits for the **bus-servo registers** to stop after every move
(`/ros_robot_controller/bus_servo/get_state`; `/joint_states` is only a command
echo). The criterion is a time window: no traced register may move more than
3 counts in the last 0.12 s. A per-read tolerance is not enough — at the ~60 Hz
read rate, "2 counts per read" is 120 counts/s.

The gripper needs one extra rule: its firmware **queues** a raw
`ServosPosition` command and starts ~0.1 s later, so the settle may not pass
before `duration + 0.15 s`. The measured close travel is ~420-460 counts/s with
a 13-19 count steady-state error under load.

Readback cost (measured): one joint 10 ms (100 Hz), two joints 17 ms (58 Hz),
five joints 39 ms (25 Hz). The settle traces the two joints that move most.

## Gripper

One raw `ServosPosition` per step on
`/ros_robot_controller/bus_servo/set_position`, because the gripper action
cannot express the measured stops (the action path clamps to the vendor
`servo_controller.yaml` range). The open is **sent in parallel** with the
approach move and completes during it.

## Measured speed envelope

Real path, view pose <-> approach pose (`0, -0.6618, -1.8012, 0, 0`):

| vmax (rad/s) | amax (rad/s^2) | return move (s) | settle (s) |
|---:|---:|---:|---:|
| 1.2 | 6 | 1.78 | 0.050 |
| 2.0 | 6 | 1.43 | 0.059 |
| 2.4 | 6 | 1.40 | 0.102 |
| 3.0 | 10 | 1.11 | 0.052 |
| 4.0 | 15 | 0.90 | 0.056 |
| 4.0 | 20 | 0.80 | 0.060 |

The settle time does not grow with speed and the post-stop tail jitter stays at
1-3 counts, so the servos are not saturating in this range (the old 9 s return
was 11x slower than the arm allows). **Production defaults: `vmax = 3.0`,
`amax = 12`.** The 4.0/20 rows are the tested ceiling on an unloaded arm, not
the default.

## Tools

| Tool | Purpose |
|---|---|
| `motion_profile.py` | S-curve planner (pure Python) |
| `vendor_servo_map.py` | pulse<->radian mapping, hard / adopted bounds |
| `fast_motion.py` | CLI: `--target`, `--sweep`, `--bench`, `--gripper`, `--hold-trace` |
| `low_config_grasp_validate.py` | grasp / put-back; fast by default, `--slow` for the old path |
| `move_to_pose.py` | view-pose move; fast by default, `--slow` for the old path |

All of them run inside the robot container with the vendor environment and need
`--i-have-clear-workspace` for any motion. `fast_motion.py --bench N` sends N
points at the current commanded pose — no motion — and reports the per-segment
overhead.

## Remaining work

* Re-run a real-cube grasp with the fast timing (targets, close values and the
  two-step close are unchanged; only the finger impact speed changes, 24 mm/s
  vs 6 mm/s).
* `--slow` was not re-run on hardware; it is a straight port of the old code.
* The 4.0 rad/s / 20 rad/s^2 ceiling is untested while holding the cube.
