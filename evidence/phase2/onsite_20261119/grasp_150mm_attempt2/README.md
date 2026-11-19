# Second grasp — operator-feedback pass (adopted-low start, horizontal wrist, one-step lift)

> Status: **PASS.** One attempt after the operator's review of the first run:
> the arm started directly at the **adopted-low forward pose**, the wrist never
> rolled (`joint5 = 0` throughout, so the gripper never crossed the lens), the
> grasp TCP went **1.0 mm deeper** than the first run, and the 150 mm lift was a
> **single** plan/execute (`+149.9 mm` TCP). The cube was put back within
> **6.6 mm** and re-verified in the same view frame (floor plane identical to
> 0.1 mm).

Operator feedback applied (2026-11-19):

1. *The initial pose is biased left and the gripper rotates into the lens.*
   → start at the adopted-low forward pose, `joint1 = 0`, `joint5 = 0`.
2. *The gripper can descend deeper.*
   → target the lowest TCP the adopted limits allow (see §4).
3. *The lift to the height should be one motion, not several.*
   → one planned/executed trajectory for the whole 150 mm.

## 1. Start / end pose

Target `joint1 0, joint2 −0.6618, joint3 −1.8012, joint4 −0.6704, joint5 0`
(the "adopted-low forward pose" defined in
[`../lowest_reach/README.md`](../lowest_reach/README.md)).

| Item | Value |
|---|---|
| MoveIt plan | 105 wp, 10.35 s, safety OK, `status=4` |
| Final joints | `−0.0057, −0.6592, −1.7953, −0.6750, −0.0064` |
| Measured registers | **499 / 343 / 929 / 339 / 499** (reference 496/339/925/334/500, ≤5 counts) |
| Camera at that pose | (0.228, 0.010, 0.033) m, optical axis **1.8° from vertical**, cube top **72 mm** away |
| End state | same pose after the place, gripper open at `−2.22`, battery 7814 mV |

The initial RGB (`low_pose.jpg`) shows the cube centred and unobstructed — the
fingers are at the bottom edge, not across the view. This is the pose the
operator asked for; the rest of the attempt keeps `joint5 = 0`.

## 2. Vision at the low pose — the depth camera is blind there

The Aurora 930 does not return usable depth at 72 mm: the segmentation found
7 clusters, none of them the cube (`seg_low.json`), all small floor/edge
fragments at `y ≈ +0.10` and negative height. The RGB is sharp, but a
single-camera back-projection from this pose disagreed with the model
(measured top-face rectangle 37.6 × 23.4 mm; the same camera's projected cube
model predicts 235 × 234 px, so the pose estimate, not the cube, dominates the
error — the same ~10 mm pose drift documented in the first record).

The grasp therefore used the **same camera's depth measurement taken two
minutes earlier** from the table-down view, with the cube untouched in
between:

| Quantity | Value |
|---|---|
| Cube centre | **(0.2305, 0.0085)** m |
| Cluster bounds | 39.0 × 41.1 mm |
| Height above the floor plane | 40.1 mm (cube 39.4 mm) |
| Yaw | 87.0° (tool frame) |

## 3. Grasp

The exact target is just outside the reachable set at this yaw: a joint-space
FK search (`reach_y.py`) shows the reachable edge at `y = 0.0085` is
**x ≈ 0.2278, z = −0.0406**, i.e. **2.7 mm short of the cube centre in x**
(along the finger length, not across the opening). MoveIt's exact IK to the
cube centre failed (22 `NO_IK_SOLUTION`, 5 `PLAN_FAIL`), so the reachable FK
pose was used directly.

| Item | Value |
|---|---|
| Executed joints | `0.0565, −0.6554, −1.7946, −0.4316, −0.0084` |
| TCP | **(0.2278, 0.0083, −0.0406)** — 2.7 mm behind the centre in x, **0.2 mm across the opening** |
| Plan | 46 wp, 4.45 s, safety OK, `status=4` |
| `joint5` | **0.0** (finger axis 90°, cube face normal ≈88° ⇒ 2°) |
| Grip | one goal, `r_joint −2.22 → −0.63` over 3 s, `status=4` |
| Depth vs the first run | **1.0 mm deeper** (first run TCP z −0.0396) |

### How much deeper is physically possible

- Table collision box in the planning scene: centre `(−0.001, 0.006, −0.1021)`,
  1 × 1 × 0.05 m ⇒ **top face z ≈ −0.0771**; the safety node also vetoes a TCP
  within 10 mm of the measured plane.
- Finger pads extend ≈32.6 mm below the TCP, so the **floor binds the TCP at
  z ≈ −0.0445** (pad bottom 2 mm above the box top).
- At this cube position the **adopted joint limits bind first at −0.0406**
  (joint2 −0.6618, joint3 −1.8012, both on their bounds). The mechanical stops
  (j2 322 / j3 939, measured in `../lowest_reach/`) would gain ~10 mm more
  travel, but the pads would then reach the floor; a deeper grasp needs the
  cube ~18 mm closer to the base, not a lower TCP.

## 4. One-step lift

Precomputed from the chosen grasp TCP and the physical floor
(`−0.0795 m`, the depth plane cross-checked by the 215 mm ruler):

```text
target object bottom  = floor + 0.150 = +0.0705 m
target TCP            = +0.1089 m
```

| Item | Value |
|---|---|
| Command | `joints 0.050, 0.205, −1.420, −1.260, 0.0` (single goal) |
| Plan / execute | 84 wp, **8.30 s**, safety OK, `status=4` |
| Final joints | `0.0585, 0.2035, −1.4200, −1.2548, 0.0050` |
| TCP | **(0.2287, 0.0090, 0.1093) m** |
| TCP travel from the grasp | **+149.9 mm** |
| Object bottom above the floor | **≈150 mm** (target 150) |
| Wrist roll during the lift | 0.005 − (−0.008) = **0.013 rad** (no twist) |

The first run used three motions (37 mm check, +107 mm, +6 mm); this run is a
single planned trajectory, and no intermediate height check was needed.

## 5. Place back and verification

Lower to the same grasp joints (one goal), open the gripper to −2.22 (one
goal), retreat to the adopted-low pose, then re-measure from the **same
table-down view pose** used before the grasp:

| Quantity | Before grasp | After place | Delta |
|---|---|---|---|
| Cube centre x | 0.2305 m | 0.2303 m | **−0.2 mm** |
| Cube centre y | 0.0085 m | 0.0151 m | **+6.6 mm** |
| Bounds | 39.0 × 41.1 mm | 40.0 × 41.2 mm | within noise |
| Height above floor | 40.1 mm | 40.1 mm | on the floor |
| Fitted floor plane (z at x=0.24) | −0.0763 m | −0.0762 m | **0.1 mm** |
| Yaw (tool frame) | 87.0° | 5.0° | ≈ **8°** rotation |

The floor plane matching to 0.1 mm confirms the two measurements share the
same pose frame, so the 6.6 mm residual is a real placement offset (in y,
i.e. across the opening axis) plus an ≈8° yaw change picked up during the
lift/release. This is the same order as the first run's residual.

## 6. What this does and does not show

Shows:

- the feedback sequence executes: adopted-low start, no wrist roll, no lens
  occlusion, one-step 150 mm lift, place back and release;
- the one-step lift is accurate to **0.4 mm** in TCP height (0.1093 vs 0.1089)
  and the whole motion took 8.3 s at velocity scaling 0.05;
- place-back accuracy of **6.6 mm / 8°** with a same-frame verification.

Does not show:

- a vision measurement from the low pose — the depth camera is blind at 72 mm,
  so the target still came from the table-down view;
- a deeper grasp than the first run beyond 1 mm — the adopted limits and the
  reachable set bind before the floor does;
- a repeatable placement.

## 7. Follow-ups

1. **Depth at the low pose is unusable.** If the low pose must be the only
   view, the measurement has to stay RGB-based and the pose drift (~10 mm)
   must be reduced first (a per-pose FK correction, or a fixed reference
   target in view).
2. The 6.6 mm y residual and 8° yaw come from the release; a short
   "open → pause → retreat" or a servo-register confirmation would bound it.
3. The `table_surface` box is correctly placed (top ≈ −0.0771) and is the
   guard for the fingers; the safety node only checks the TCP, so keep the box
   alive for any deeper attempt.

## 8. Files

| File | Content |
|---|---|
| `seg_att2.json` | pre-grasp table-down measurement (centre 0.2305, 0.0085) |
| `seg_low.json` | depth-at-the-low-pose failure record (7 non-cube clusters) |
| `seg_a2_after.json` | post-place measurement in the same view frame |
| `grasp2.json`, `grasp2b.json`, `grasp2c.json` | grasp searches: 19.4, 17.2 mm offsets and the reachable-target attempts |
| `low_pose.jpg` | initial view at the adopted-low pose (cube centred, fingers clear) |
| `a2_grip.jpg` | after closing at the grasp pose |
| `a2_lift150.jpg` | cube held at the top of the one-step lift |
| `a2_after.jpg` | released at the low pose |
| `a2_verify.jpg` | verification frame from the table-down view |
| `reach_y.py` | joint-space FK search that found the reachable edge (0.2278, 0.0085, −0.0406) |
| `minz_cube.py` | lowest-TCP search at the cube |
| `lift_target2.py` | one-step lift target derivation and joint search |
| `scene_probe.py`, `fk_states.py` | planning-scene / validity probes (table box top ≈ −0.0771) |
| `rgb_low_measure.py` | RGB-only top-face measurement and its failure at this pose |
