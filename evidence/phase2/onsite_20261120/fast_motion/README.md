# Fast, measured arm motion — 2026-11-20

> Status: **PASS on hardware, including a real-cube grasp and put-back.** The
> grasp / put-back sequence went from **~37 s to 5.43 s** of motion (6.8x) with
> every step ending on a **measured** bus-servo settle instead of a fixed
> sleep, and the motion is a jerk-limited S-curve instead of one
> constant-velocity move with a hard start and stop. The first real-cube run
> gripped the cube (0 clusters left in the cube box) and put it back
> **1.23 mm** from the measured start. The same bundle also holds the later
> in-view (**0.92 mm**) and far return-home (**11.06 mm** against the fitted
> target) runs.

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
return is 11× slower than the arm allows. This early run used
`vmax = 3.0 rad/s, amax = 12 rad/s^2`; the later throughput v2 default is
**4.0/20** (see `docs/phase2/FAST_MOTION.md`), and the 4.0/20 rows above are
the tested ceiling on the unloaded arm.

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

## 6. Real-cube grasp and put-back

File: `e2e_cube.json` (driver `tools/phase3/fast_motion_e2e.py`).

The cube sat at (0.2824, -0.0552) m from the base, outside the validated band
(x 0.226-0.234 m, y ~ 0). Two closed-loop base pulses (`base_pulse_vector.py`,
duty 10, 1.0 s then 0.58 s) brought it to **(0.2305, 0.0016) m** — the
2026-11-19 procedure, but driven by the measured point rather than by a search.

| Step | Result |
|---|---|
| Wrist measurement at the view pose | (0.2305, 0.0016, -0.0484) m, height 39.7 mm, 37.8 x 42.0 mm — placement gates pass |
| Target `C_view + e` | (0.2442, 0.0055, -0.0395) m |
| Joint fit | (0.0294, -0.6618, -1.8012, -0.3041), xy error 0.00 mm |
| Grasp motion | **5.447 s** (approach 1.021, descend 0.362, final 0.563, close 0.564 + 0.856, return 0.924) |
| Grip | close register **414** (535 on an empty grip), **0 clusters** in the cube box |
| Release motion | **6.216 s** (descent 2.677 at 0.6 rad/s, final 0.476, open 1.336, return 0.931) |
| Landing | (0.2293, 0.0013, -0.0499) m, **1.23 mm** planar (previous dual-camera run 3.65 mm) |

The 9.9 s / 10.7 s wall time of each grasp/release *command* includes ~4.5 s of
SSH plus container startup; the arm motion itself is the 5.4 / 6.2 s above.

## 7. Later runs: in-view and far return-home

Two further runs on the same driver use the **4.0/20** default and the v2
release profile:

| Run | Cube start | Base | Grasp | Release | Landing |
|---|---|---|---:|---:|---|
| In view (no base move) | (0.2369, 0.0003) m | none | 4.77 s | 4.44 s | **0.92 mm** (vs the cube's own start) |
| Far + return home | (0.4325, -0.1674) m, 0.464 m at -21 deg | 2 pulses, 294.7 + 32.4 mm | 4.76 s | 4.31 s | **11.06 mm** (vs the fitted target) |

Files: `e2e_cube_run2.json` and `e2e_far_return_home.json`. The far run's
`--return-home` reverses the commanded approach while the cube is still held
(one 2.50 s pulse, 245.3 mm measured net travel); it is open-loop, so the
11.06 mm landing is a base-frame placement error, not a home error. Both runs
used `joint1 <= 0.017 rad`, a close register of 422 / 421 against 535 on an
empty grip, and 0 clusters in the cube box before and after the return.

## 8. Open items

* `--slow` (the previous single-point path, now on the same client) was not
  re-run on hardware in this session; it is a straight port of the old code.
* The 4.0/20 envelope was validated on four real-cube runs (this bundle's two
  later runs plus the two throughput v2 runs); the earlier 5.45 s run used
  3.0/12, and the holding-cube return now uses 1.4 rad/s. The 4.0/20 rows
  above are still a tested upper bound on an unloaded arm.
