# 4.0/20 fast-path regression — 2026-11-23

Scope: one real-arm regression of the G8 fast profile after the
`profiles.fast` unification and zero-move change. `fast_motion.py` sends the
dense S-curve directly to `/arm_controller/follow_joint_trajectory`; it does
not pass through `armpi_safety` (by design), but it applies the strict joint2
adopted-bound check and waits for a measured bus-servo settle.

## Setup

- Real `armpi_safety` node running (persistent overlay): `/safety_stop=false`.
- 8765 read-only gateway running; 8766 motion gateway stopped.
- Actual bus before the move: `r_joint 224, joint1 502, joint2 557,
  joint3 788, joint4 122, joint5 497` (near the view pose).
- The command cache was resynced to the actual bus with
  `ServosPosition(target=actual)` before the move; cache then matched.

## Command

```bash
python3 /home/ubuntu/shared/armpi_tools/fast_motion.py \
  --target 0,0,0,0,0 --vmax 4.0 --amax 20 --i-have-clear-workspace
```

Target: upright initial pose. Start: the measured view pose.

## Result

- Tool `ok: true`; target reached without an action error.
- `pulses_after`: `joint1 502, joint2 500, joint3 500, joint4 497,
  joint5 500`.
- `settle_excursion_pulses`: `joint3 2, joint4 2` (≤3 counts).
- Post-move bus: `r_joint 224, joint1 503, joint2 501, joint3 500,
  joint4 498, joint5 500`.
- The real safety node stayed `/safety_stop=false`; no fault was latched.
- The gripper was not commanded and remains open (224).

## Limitations

- One direction only (view → upright); the full G7/G8 task sequence was not
  repeated.
- `fast_motion.py` bypasses `armpi_safety`; the fast profile is a separate
  explicit tier, not the MoveIt veto path.
- No e-stop, brake, self-collision model or independent encoder; the arm was
  supported and the workspace clear.
