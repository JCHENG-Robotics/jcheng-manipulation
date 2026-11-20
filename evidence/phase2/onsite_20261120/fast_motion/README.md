# Fast, measured arm motion — 2026-11-20

> Status: **PASS on hardware (no-cube cycle).** The grasp / put-back sequence
> went from **~37 s to 5.43 s** of motion (6.8×) with every step ending on a
> **measured** bus-servo settle instead of a fixed sleep, and the motion is a
> jerk-limited S-curve instead of one constant-velocity move with a hard start
> and stop. A real-cube re-grip with the fast timing has **not** been run yet;
> the grip targets and close values are unchanged.

## 1. What was slow, and what shook

The previous sequence (`low_config_grasp_validate.py`) sent **one trajectory
point per move** with a hand-set duration:

| Step | Old duration | Resulting joint speed |
|---|---:|---|
| Open gripper | 4 s | — |
| Approach (j4 -> 0) | 8 s | ~0.10 rad/s peak |
| Descend to grasp | 6 s | ~0.06 rad/s |
| Close step + close | 3 + 3 s | ~0.06 rad/s |
| Return to view | 9 s | ~0.10 rad/s |
| Fixed settles | 6 x 0.6 s | — |
| **Total** | **~37 s** | |

The vendor controller forwards **one bus-servo command per trajectory point**
with that segment's duration (`joint_trajectory_action_controller.py`), and the
servo interpolates linearly over it. A single point is therefore a
constant-velocity move with an instantaneous velocity step at the start and at
the stop. At 0.06-0.10 rad/s the motion also sits in the gear stiction band, so
the slow single-point move is both slow **and** the source of the visible
shake. `--slow` keeps that path for comparison.

## 2. What replaced it

`tools/phase2/motion_profile.py` plans a synchronised trapezoid with
**half-cosine (jerk-limited) ramps** in joint space: all joints share one path
parameter, the peak joint velocity is `vmax`, the peak acceleration is `amax`,
and the endpoints are exact. `tools/phase2/fast_motion.py` sends the dense
waypoint list to `/arm_controller/follow_joint_trajectory` with a 60 ms header
lead, then waits for the bus-servo registers to actually stop.

Two details matter:

1. **The vendor path has no per-segment cost.** A zero-motion bench of 20/40/
   80/200 points over 2.0 s measured 0.27/0.10/0.07/0.01 ms per segment
   (`bench_zero_motion_80pts.json`), so 25-50 ms segments are safe.
2. **The settle must be a time window, not a per-read delta.** At the ~60 Hz
   read rate, "two counts per read" is 120 counts/s and passed while the
   gripper was still interpolating. The criterion is now "no register moved
   more than 3 counts in the last 0.12 s", with a minimum wait of
   `command duration + 0.15 s` for the gripper (its firmware queues a command
   and starts ~0.1 s later; see `gripper_open_trace.json` and
   `gripper_close_trace.json`).

The gripper is driven with one raw `ServosPosition` per step on
`/ros_robot_controller/bus_servo/set_position`; the open is **sent in
parallel** with the approach move. Its measured travel is ~420-460 counts/s
(~1.8 rad/s) with a 13-19 count steady-state error under load.

## 3. Speed sweep (in the air)

Pose pair: view pose <-> `(j1 +0.45 rad, j4 +0.70 rad)`, `amax = 6`,
`low_config` S-curve, 1 s hold trace after each stop.
File: `sweep_air_vmax0.6-2.4_amax6.json`.

| vmax (rad/s) | planned (s) | wall (s) | settle (s) | tail p2p j1 (counts) | tail p2p j4 |
|---:|---:|---:|---:|---:|---:|
| 0.6 | 1.276 / 1.232 | 1.348 / 1.299 | 0.056 / 0.054 | 2 / 1 | 1 / 0 |
| 1.2 | 0.867 / 0.848 | 0.930 / 0.912 | 0.051 / 0.059 | 5 / 1 | 2 / 0 |
| 1.8 | 0.833 / 0.822 | 0.899 / 0.888 | 0.047 / 0.051 | 3 / 0 | 2 / 1 |
| 2.4 | 0.833 / 0.822 | 0.900 / 0.885 | 0.056 / 0.077 | 5 / 1 | 1 / 0 |

Each cell holds the outbound / return move. The tail is the last 0.4 s of the
1 s hold trace (after the move and the settle).

Above 1.8 rad/s the profile is **acceleration-limited** at `amax = 6`, so a
higher `vmax` no longer shortens this move. The wall time is the plan plus the
0.06 s lead plus ~10-20 ms, i.e. the vendor path tracks the schedule.

## 4. The real grasp path

View pose <-> approach pose `(0, -0.6618, -1.8012, 0, 0)`, `low_config` S-curve.
Files: `path_*.json`.

| Move | vmax | amax | planned (s) | wall (s) | settle (s) |
|---|---:|---:|---:|---:|---:|
| descend | 0.6 | 6 | 2.94 | 3.00 | 0.052 |
| descend | 1.0 | 8 | 1.87 | 1.93 | 0.050 |
| return | 1.2 | 6 | 1.71 | 1.78 | 0.050 |
| return | 2.0 | 6 | 1.36 | 1.43 | 0.059 |
| return | 2.4 | 6 | 1.33 | 1.40 | 0.102 |
| return | 3.0 | 10 | 1.03 | 1.11 | 0.052 |
| return | 4.0 | 15 | 0.84 | 0.90 | 0.056 |
| return | 4.0 | 20 | 0.73 | 0.80 | 0.060 |

Settle time does **not** grow with speed and the post-stop tail jitter stays at
1-3 counts on joint2, so the servos are not saturating in this range: the 9 s
return is 11× slower than the arm allows. The production default is
`vmax = 3.0 rad/s, amax = 12 rad/s^2` (the 4.0/20 rows are the tested ceiling,
not the default).

## 5. The full cycle (no cube)

`low_config_grasp_validate.py` with the validated grasp joints
`(-0.0227, -0.6618, -1.8012, -0.32)` and the view pose, no cube on the table.
File: `cycle_nocube_fast.json`.

| Step | Wall (s) | Settle (s) |
|---|---:|---:|
| open (parallel with the approach) | 0.80 | — |
| approach | 1.014 | 0.148 |
| descend to pre-grasp | 0.385 | 0.138 |
| final descent | 0.557 | 0.162 |
| close step (raw 318) | 0.574 | register 307 |
| close final (raw 550) | 0.893 | register 535 |
| return to view | 0.920 | 0.151 |
| **motion total** | **5.433** | |

Old sequence on the same poses: ~37 s. The gripper closed to register 535
against the 550 command (load error), matching the isolated close trace (537),
and the fingers stop at the 39 mm gap of an empty grip.

## 6. Open items

* **Real-cube re-grip with the fast timing** — the grip targets, close values
  and the two-step close are unchanged, so only the finger impact speed
  differs (24 mm/s vs 6 mm/s at the fingers), but it has not been re-run.
* `--slow` (the previous single-point path, now on the same client) was not
  re-run on hardware in this session; it is a straight port of the old code.
* The 4.0 rad/s / 20 rad/s^2 rows are a tested upper bound on an unloaded arm;
  the holding-cube return is untested above 3.0 rad/s.
