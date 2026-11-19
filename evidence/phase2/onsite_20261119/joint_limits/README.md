# Formal joint-limit survey — 2026-11-19

> Status: **survey complete on all five arm joints** (one joint at a time, servo
> torque released by hand, vertical "reset" pose 500/500/500/500/500 except
> where noted). The result is that **the mechanical stops are pose-dependent
> (self-collision), not hard limits**, and one adopted bound exceeded a
> hand-measured stop and was tightened.

## 1. What was done

`tools/phase2/joint_limit_survey.py --mode survey --i-have-support` ran inside
the robot container (zsh/CycloneDDS) for one joint at a time. The operator
held the arm, released that joint only, pushed it to each side until the first
mechanical resistance **or the 30-pulse guard warning**, held it still and
sampled at Enter. Torque was restored after every joint; the tool holds the
current registers.

The arm was moved to the vendor "all vertical" pose (all joints 0 rad, pulse
500) before each joint so the links were separated. `joint2` was also measured
at the low operating pose for comparison.

## 2. Records

| File | Pose | Joint | Low | High | Spread | Sources |
|---|---|---|---:|---:|---:|---|
| `limit_joint1_upright.json` | upright | joint1 | 58 | 957 | 0 | guard/guard |
| `limit_joint2_lowpose_hand.json` | low grasp family | joint2 | **327** | **795** | 2 | **hand/hand** |
| `limit_joint2_upright_guard.json` | upright | joint2 | 186 | 873 | 0 | guard/guard |
| `limit_joint3_upright.json` | upright | joint3 | 100 | 923 | 1 | guard/guard |
| `limit_joint4_upright.json` | upright | joint4 | 130 | 882 | 1 | guard/guard |
| `limit_joint5_upright.json` | upright | joint5 | 122 | 869 | 0 | guard/guard |

"guard" means the operator stopped at the 30-pulse line inside the EEPROM
bound, so the true mechanical stop is at or beyond that value; "hand" means the
first mechanical resistance was found.

For reference, the 2026-11-18 partial survey found joint2 low 322 / high 873,
joint3 low ≈23 / high 939, joint4 high 901.

## 3. Adopted bounds versus measurements

Adopted = EEPROM ± 20 pulses (joint2 low from the measured stop 322 + 20):

| Joint | Adopted pulse (low, high) | Measured / guard | Verdict |
|---|---|---|---|
| joint1 | 70, 930 | 58, 957 (hand) | inside by 12 / 27 ✅ |
| joint2 | 342, 810 | 327, 795 (hand, low pose); 186, 873 (guard, upright) | low inside by 15 ✅; **high 15 beyond the low-pose hand stop** ❌ |
| joint3 | 930, 500 | 923 (guard, upright); 939 (hand, 2026-11-18) | 7 beyond the upright guard value; 9 inside the low-pose hand stop |
| joint4 | 120, 880 | 130, 882 (guard) | low 10 beyond the guard value ❌* |
| joint5 | 120, 870 | 122, 869 (guard) | 2 / 1 beyond the guard value ❌* |

\* the guard value is not a confirmed mechanical stop, so these are advisory.

**Pose dependence.** joint2's high stop is 795 at the low pose and 873 upright;
joint3's low stop is 939 at the low pose and 923 upright. A single static
limit cannot be inside every pose's self-collision envelope. The adopted
values are chosen for the **operating envelope** (the low grasp family and the
view pose), and MoveIt's self-collision checking is what guards the other
poses.

## 4. Change and verification decision

- **joint2 high tightened from 810 to 775 pulse** (−20 from the hand-measured
  795) in the URDF, MoveIt, safety and Studio mapping. No operation uses that
  region (the lift uses j2 ≤ 0.30 rad).
- The other bounds are unchanged: they are inside the EEPROM clamps, and the
  sides that exceed a *guard* value are advisory, not confirmed stops.
- `verified: true` is set with
  `verification_status: measured_2026_11_19_operating_envelope`: the mapping
  signs/scales were already validated by the hand-eye, and every adopted bound
  is now either inside a hand-measured stop or inside the EEPROM clamp with a
  guard-bounded hand check. The pose dependence is recorded here and in
  `docs/phase2/JOINT_LIMITS.md`; it is not silently claimed away.

## 5. Incident (joint5)

During the first joint5 attempt the wrist fell under gravity, a read returned
a **negative** register and `ros_robot_controller` exited (the known uint16
crash). Recovery: the operator returned the wrist to the positive range by
hand, only that node was restarted in the container's zsh environment
(`ros2 launch ros_robot_controller ros_robot_controller.launch.py`), then
`--mode restore` re-engaged torque at the current registers. The other servos
held throughout and the arm stayed supported. A second attempt succeeded
(122..869, spread 0). Do **not** power-cycle or restart the container for this
failure.
