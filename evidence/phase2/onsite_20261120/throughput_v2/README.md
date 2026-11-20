# Throughput v2 — faster arm envelope and duty-20 base, 2026-11-20

> Status: **PASS on hardware, two supervised real-cube runs.** The full
> grasp + put-back cycle fell from **11.67 s to 8.94 s of motion (-23 %)**
> (`4.729 + 4.207`; the second run 4.699 + 4.259) with the cube held at
> 4.0 rad/s / 20 rad/s^2 and the put-back descent at 1.4 rad/s. The base ran
> at **duty 20** for the first time: one 1.79 s pulse moved the cube
> **179 mm** (100.2 mm/s) and one 0.48 s strafe moved it **45 mm**, landing it
> at `(0.2316, -0.0025)` m. The aligned run grasped at `joint1 = 0.0065` and
> put the cube back **2.22 mm** from the measured start.

## Setup

| Item | Value |
|---|---|
| Battery | 8365 mV at start |
| View pose | `0, 0.2574, -1.18773, -1.70, -0.01676` |
| Correction | `(+13.7, +3.9, +8.9) mm` |
| Measurement | wrist RGB-D segmenter, **wide region** for the base phase (`--x-range 0.15,0.60 --y-range 0.40 --max-xy 0.12`) |
| Cube start (run 1) | `(0.3917, -0.1032, -0.0522)` m, radius 0.4051 m |
| Operator | present, workspace clear, no e-stop on this robot |

The wide region is new: the default region stops at x 0.32 / |y| 0.09, so a
cube that needs a base pulse is rejected before the base can approach it. The
base phase measures wide; the band and the grasp point are unchanged.

## Run 1 — `e2e_cube.json` (bearing pulse, yawed grasp)

One duty-20 pulse along the cube bearing (`vx 0.967, vy -0.255`, 1.786 s).

| Quantity | Value |
|---|---|
| Cube after | `(0.2200, -0.0530)` m, radius **0.2263 m** — inside the band, one pulse |
| Travel | 178.9 mm (vector), **100.2 mm/s** at duty 20 |
| Nominal | 196.3 mm at the 0.182 duty/mm-s scale (109.9 mm/s); the linear scale is ~9 % optimistic at duty 20 |
| Direction error | commanded `(0.967, -0.255)`, measured `(0.960, -0.281)` → **1.6 deg** |
| Residual lateral offset | **-53.0 mm**, so the fit used `joint1 = -0.2878` |
| Grasp / release motion | **4.729 s / 4.207 s** |
| Grip check | 0 clusters in the cube box; cube held through the 4.0/20 return and the 1.4 rad/s descent |
| Landing error | **6.72 mm** planar |

The yawed grasp held, but at `joint1 = -0.288` (~16.5 deg) the fingers close
across the cube diagonal (~49 mm) instead of the 39.4 mm face — the measured
failure edge from the 2026-11-19 yaw experiments. The planner was changed to
remove the lateral offset; run 2 validates that.

## Run 2 — `e2e_aligned.json` (lateral strafe, `joint1 ~ 0`)

The cube was left at `(0.2228, -0.0469)` by run 1. The new plan aims the cube
at `(0.230, 0)`: a `vx -0.152, vy -0.988` strafe for 0.484 s.

| Quantity | Value |
|---|---|
| Cube after | `(0.2316, -0.0025)` m, radius **0.2316 m**, lateral offset **-2.5 mm** |
| Travel | 45.3 mm, **93.5 mm/s** at duty 20 (short pulse: dead band and the 0.4 s coast are a larger share) |
| Direction error | commanded `(-0.152, -0.988)`, measured `(-0.194, -0.981)` → **2.5 deg** |
| Grasp joints | `0.0065, -0.6618, -1.8012, -0.2962, 0` — back in the 24/24 validated family |
| Grasp / release motion | **4.699 s / 4.259 s** |
| Grip check | 0 clusters in the cube box |
| Landing error | **2.22 mm** planar |

## Motion timing, before vs after

| Step | 2026-11-20 v1 | v2 run 1 | v2 run 2 | Planned |
|---|---:|---:|---:|---:|
| Grasp approach (4.0/20) | 1.021 | 0.798 | 0.799 | 0.733 |
| Grasp return (4.0/20) | 0.924 | 0.729 | 0.736 | 0.665 |
| Close wait (0.5 s) | 0.856 | 0.660 | 0.672 | — |
| Release descent (1.4) | 2.677 | 1.241 | 1.272 | 1.178 |
| Release final (0.8) | 0.476 | 0.283 | 0.289 | 0.219 |
| Release open (0.45 s) | 1.336 | 1.166 | 1.166 | — |
| **Grasp motion** | **5.447** | **4.729** | **4.699** | — |
| **Release motion** | **6.216** | **4.207** | **4.259** | — |

Every arm step settled (`settled: true`, 0.14-0.16 s), the post-stop register
tail stayed at the same 1-3 counts as at 3.0/12, and no run dropped the cube.

## Base startup cost

`base_pulse_vector.py` no longer sleeps 1.5 s for discovery and sends 4 zero
messages instead of 10. The pulse wall time in these runs (7.8-9.1 s) includes
the wide segmentation (1.3 s), the 0.4 s coast and three SSH round trips; the
pulse itself was 0.48 s / 1.79 s.

## Caveats

* Two runs, one operator, one floor, battery 8.365 V. The duty-20 scale is
  measured once per direction (100.2 and 93.5 mm/s); the planner keeps the
  conservative 4.9 mm/s per duty (98 mm/s at duty 20), so it undershoots
  rather than overshoots.
* The aligned run started from a cube that run 1 had placed; a fresh placement
  at a different bearing has not been run. The plan is geometric, and the
  closed loop re-measures between pulses, but the bearing range is unmeasured.
* The wide measurement region is new and was exercised on this floor only. It
  can select a background cluster if it is cube-sized and cube-shaped; the
  operator watched both runs.
* The 4.0/20 envelope is validated for this grasp/put-back path; the paired
  trial family was run at 3.0/12.
