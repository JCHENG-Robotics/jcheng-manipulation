# Object grasp catalog (on-site, 2026-11-23)

> Status: **six object forms attempted on the real arm** through the Studio pose
> path; four were grasped, lifted, verified at the view and put back; two were
> rejected for a measured reason (a lying cylinder below the vertical reach and
> a puffy sachet that squeezes out). This page is the **reference for future
> grasps**: the gripper's measured envelope, the per-form parameters that
> worked, the register signatures, and the staged procedure. Evidence:
> [`evidence/onsite_20261123/object_forms/`](../../evidence/onsite_20261123/object_forms/).

Every number below was measured on the hardware that day (Aurora RGB/depth,
wrist camera, plus the auxiliary side camera for two diagnoses). Nothing here
is a planner output.

## The gate: the gripper's measured envelope

| Quantity | Measured value | Source |
|---|---|---|
| Jaw axis at `joint5 = 0` | base **y**; the pads close across the object's y width | FK + images |
| Jaw opening, open command | ≈ **100 mm** at the tip; the closed-empty register is **640–648** | images + bus reads |
| Pad contact plane | ≈ **13 mm in front of** the nominal `end_effector_link` TCP when the target is `measured + (13.7, 3.9)` mm | the Ø28.7 cylinder series |
| Lowest reachable pad point | ≈ **28–30 mm above the table** (URDF nominal 48–49 mm at the deep poses; the real pads are ≈ 16–20 mm longer) | the lying block (top 30 mm) grasped; the lying cylinder (top 29.5 mm) only grazed |
| Pad face along the approach | ≈ 20 mm long × ≈ 30 mm tall | the URDF finger meshes |
| `joint5` wrist roll | ±90° rotates the jaw axis y → **x**, pad z unchanged; the mapping limit is **1.5498 rad**, not π/2 (= `OUT_OF_LIMIT`) | FK + a rejected preview |
| Low-configuration correction | `(+13.7, +3.9, +8.9) mm` added to the measured top point | the 2026-11-19 calibration |

**Two derived rules.**

1. **Height gate** — the object's grip zone must reach at least ~30 mm above
   the table (a side grip needs the pads' bottom below the object's top). A
   lying Ø29 cylinder (top 29.5 mm, widest ring only 14.7 mm) fails; the same
   cylinder stood upright passes.
2. **Aim rule for small objects** — the pads land ~13 mm in front of the
   measured point when the target is the usual `measured + 13.7 mm`. Long
   objects (58 mm block) tolerate it, but an object whose half-extent along the
   approach is under ~15 mm (a Ø29 cylinder) needs the nominal TCP aimed at
   the object centre + ~27 mm.

## Depth-camera caveats

- The view-pose depth coverage starts at **x ≈ 0.225 m**; anything closer has
  no points. A near-range/glossy object can be invisible to the cloud while
  clearly visible in RGB.
- **Glossy/printed surfaces (the VLT carton) returned no depth in any
  orientation**; the pose had to be reconstructed from the RGB
  (back-projecting the bottom-front edge onto the table plane).
- The RANSAC plane's `z_at_0p24` extrapolation drifts **±9 mm** between
  captures (the plane's x-tilt sign flips). Compare **heights above the local
  table median**, not the absolute z or the extrapolated floor.
- The operator's hand in the view is segmented as a ~30 mm-high, 6 × 25 mm
  tilted patch and silently becomes the locate target. Clear the workspace
  before locating.

## The six forms

| # | Form | Size (mm) | Top above table | Verdict | Grasp pose | Close register |
|---|---|---|---|---|---|---|
| 1 | Blue block, lying (long axis fore-aft) | 29.4 × 58.3 × 30 | 30 | **grasped**, lifted, put back | `j1 0.018`, `j4 −0.50` | stall **482** |
| 2 | Blue block, upright (on its end) | 29 × 30 × 61 | 61 | **grasped**, lifted, put back (1.5 mm) | `j1 0.049`, `j4 −0.39` | stall **482** |
| 3 | Red cylinder, standing | Ø28.7 × 45 | 45 | **grasped**, lifted, put back | `j1 0.049`, `j4 −0.365` | stall **489** |
| 4 | Red cylinder, lying (axis fore-aft) | Ø28.6 × 45 | 29.5 | **rejected**: widest ring 14.7 mm, below the pads' reach; `j4 −0.74` only grazed (521) and slipped | — | 642 empty / 521 graze |
| 5 | Yellow "VLT" carton, standing | ~55 × ~35 × ~80 | ~80 | **grasped**, lifted, put back (no depth at all; RGB pose) | `j1 0.066`, `j2 −0.62`, `j3 −1.77`, `j4 −0.45` | stall **433**, held at 441 |
| 6 | Puffy snack sachet | 118 × 55 | 49 | **rejected**: contacted at 478 but squeezed out of the rigid pads (a second close reached 642) | — | 478 then 642 |

### Per-form notes

1. **Lying block** — the plan's clip-recovery assumed the 39.4 mm cube and
   moved the point ~12 mm along the long axis; the block is long enough to
   absorb it. A 30 mm-tall flat top is right at the pads' limit; the sharp top
   edge is what makes it hold.
2. **Upright block** — the same width on a 61 mm-tall body: a comfortable
   vertical target, the pads grip the top ~15 mm. `j4 −0.39` (shallower than the
   lying form because the top is 34 mm higher).
3. **Standing cylinder** — the pads meet the widest ring; the round side holds
   at the mid-height. The clip recovery's 39.4 mm cube assumption put the point
   ~10 mm off along the axis; the cylinder radius is only 14.3 mm, so this is
   the form that needed the aim rule above (`j4 −0.365`, TCP ≈ centre + 27 mm).
4. **Lying cylinder** — the fundamental vertical limit, not a targeting error.
   A `j5` ±90° roll does not help: the pads' min z is unchanged. Fix: a
   **riser of ≥ 15 mm** or stand it upright.
5. **Carton** — the only depth-free form. The approach, descend and close all
   passed the pose gate; the box is tall enough that the palm clears its top
   by ~25 mm.
6. **Sachet** — the only material failure: two rigid flat pads compress an
   air-filled bag and it escapes. The stiff crimped end seam (~5–10 mm) might
   work but lies at the register resolution floor (≈ 620 counts) and cannot be
   verified from the bus.

## Register reference

| Register | Means |
|---|---|
| 640–648 | empty close (the fingers meet; the mechanical stop) |
| 521 | a ~21 mm gap — a graze at a round top |
| 489 | Ø28.7 cylinder at its widest ring |
| 482 | the 29.4 mm block width |
| 478 | a compressed 55 mm sachet (not a hold — it escaped) |
| 433 | the ~55–60 mm carton width (a real hold) |
| 170–185 | open stop |

A register that **keeps moving** on a second close means the servo was still
travelling (no object); a register that **stays put** means a real mechanical
stop (either the object or the empty stop).

## The staged procedure that produced these results

1. **Clear the workspace.** The Studio UI's control lease must be released
   (`POST /api/control/release {lease_id, session_id}` — both ids are in the
   `/api/events` `LeaseHeartbeat` details) before an API client can acquire it.
2. **Locate at the view** (`POST /api/task/plan` is read-only) and apply the
   two gates: the height above the local table median and the footprint. Use
   the RGB when the depth has no return.
3. **Base approach only if needed**, from the plan's own G8.1 pulse or a manual
   `vx` pulse; the chassis is open-loop (±50 %), so re-measure after every
   pulse.
4. **Descend in three poses** — `approach_low` (`j4 0`) → `pre_grasp`
   (`j4 + 0.15`) → `grasp` — with `j1` from the target azimuth, and, while the
   j2/j3 stops are suspect, **`j2 −0.62`, `j3 −1.77`** instead of the adopted
   deep bounds.
5. **Close** `−1.6` preclose → `−0.05` full close, then read the bus register
   and classify it against the table above; a second close confirms.
6. **Lift in steps** to the view (≤ 0.4 rad per step, slow profile), probe the
   register again, and accept only when the view locate returns no candidate
   (or the operator confirms).
7. **Put back** at the same pose, open to `−2.2201`, return to the view, and
   re-locate to confirm the object is back.

All pose commands go through `POST /api/motion/preview` and
`POST /api/motion/execute`; **the execute must carry the same `profile` as the
preview** or it is re-validated against the 0.5 rad/s pose cap and rejected.

## Known system issues observed on 2026-11-23

- **The j2/j3 stops moved.** The adopted deep bounds (`j2 −0.6618` → raw 342,
  `j3 −1.80117` → raw 930) stall at raw 350–357 / 898–917; j3 alone exceeds the
  gateway's `verification_tolerance_pulses = 12`, so every multi-joint command
  ends `MOTION_OUTCOME_UNKNOWN` and the backend latches. The **motion itself
  always executes**; the latch is the verification. Workaround used: `j2 −0.62`,
  `j3 −1.77`, and a **backend restart** after any latch (the arm is already at
  the commanded pose — confirm with `/api/state`). Gripper commands complete by
  timing, so release/open always works. The stop regression (a loose horn or a
  shifted stop, ≈ 0.07 rad steady-state error on j3) is **unresolved** and
  deserves an operator check.
- The Studio backend has no autoreload and latches the motion fault in memory;
  a restart is the only clear.
- Cross-host absolute z frames drift ~9 mm between captures — compare relative
  heights only.

## Related

- [`docs/phase2/FAST_MOTION.md`](FAST_MOTION.md) — the S-curve profile and the
  task path.
- [`docs/phase2/JOINT_LIMITS.md`](JOINT_LIMITS.md) — the measured stops.
- [`docs/phase2/SAFETY_LAYER.md`](SAFETY_LAYER.md) — the pose gate.
- [`docs/phase2/MOTION_WRITE_PATHS.md`](MOTION_WRITE_PATHS.md) — every write
  path.
