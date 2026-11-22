# JCHENG Manipulation: A ROS 2-Based Robotic Manipulation System with Quantitative Reliability Evaluation

> Imported from `ROADMAP_PUBLIC.md` (USB, 2026-10-26) on 2026-11-15.
> The previous 8-week learning roadmap is archived at
> `docs/archive/ROADMAP_8WEEK.md`. The private master plan is intentionally
> kept outside this public repository.

> Technical roadmap: this document tracks the engineering and experimental development of a low-cost ROS2 manipulation platform based on ArmPi Ultra.

## Current status (2026-11-21)

Hardware results are as of 2026-11-21; this roadmap was synchronized with the
README, changelog and evidence on 2026-11-21 (random-placement paired trial,
Studio G7 pose takeover, Studio G8 task takeover and the G8.1 bounded chassis
approach, all real-motion verified with bus-servo completion). The Studio G8
**fast task path** (dense S-curves + the base approach inside one run) was
verified on the hardware the same day: 5.466 s commanded / 13.46 s wall,
`held: true`, landing 2.27 mm planar.

The 2026-11-22 offline hardening (`43647bc`) aligned the tool-side joint2 bound
to 775 pulse, wired `/safety_stop` into the Studio gateway, added the
safety/planner CI gate, moved the ticker off the event loop and locked the
hardware adapter state. No new hardware claims; the real-robot round trip and
low-speed joint2 check remain pending.

| Milestone | Status | Evidence / limit |
|---|---|---|
| Robot model and TF | **Implemented; live arm/camera TF verified; limits formally surveyed** | 17 links, 16 joints; live `base_link → rgb_camera_link` / `depth_camera_link`; measured optical frame `camera_optical_calibrated`; front `base_link → ext_cam_optical`; all five joints hand-surveyed 2026-11-19, `mapping_verified=true` for the operating envelope |
| MoveIt 2 constrained planning | **Implemented; mock + real execution validated** | Pilz `PTP` 6D-constrained goal (mock); real execution through Humble Docker MoveIt on 2026-11-18/19; D435i published as a MoveIt collision box |
| 5-DoF grasp candidate search | **Implemented; mock + real use** | Joint-space sampler: 80 candidates → 40 IK OK → 26 collision-free → 3 PTP plans (mock); the same search fed all five real grasp passes |
| Fiducial → grasp bridge | **Implemented; mock E2E validated (2026-11-17)** | `target_pose` → TF → grasp search → safety veto; synthetic marker, plan-only; the real grasps used the grasp planner directly, not this bridge |
| Fiducial target localization | **Implemented; synthetic + real-background validated; physical tag scale check** | AprilTag 36h11; max 1.82 mm / 1.49° synthetic; 15/15 semi-synthetic on real frames, max 1.69 mm / 0.53°; 0 false detections on 8 real frames; physical tag scale 100.67 mm vs 100.0 mm (0.67 %) |
| Front camera (D435i) | **Validated with limits** | Chassis-front USB 3.0; intrinsics fx = fy 903.5 px / RMS 0.371 px; extrinsic `[0.136, 0.027, 0.038] m`, 31.6° pitch; depth 640×480 Z16 ~62 % valid / 10.6 Hz depth-only; `armpi-rs-depth` + `armpi-ext-cam` boot cleanly; IMU unavailable; depth starts ~0.2 m |
| Safety supervisor | **Implemented; mock + hardware veto** | state watchdog, joint/velocity/acceleration/start-state/duration/workspace/table-clearance checks; veto service; adopted EEPROM-bounded limits; no e-stop or brake |
| Measurement protocols | **Implemented; error-budget sources measured on hardware** | 30-visit repeatability **0.449 mm RMS / 1.017 mm max**; backlash **1.117 mm**; 46-grasp tolerance grid **[-6, +12] mm** (±9 mm about +3 mm); 20-grasp execution **5.386 mm / 1.495° max**; 20-put-back placement **11.514 mm max / 3.080 mm mean**; 20-placement fixture **14.815 mm radial max**; rotation-tolerance and visual-displacement runners added, hardware runs pending |
| 5-DoF task-space policy | **Documented** | `docs/phase0/ARMPI_5DOF_CONSTRAINTS.md` |
| Hardware baseline | **Captured; all five joints formally surveyed; board probed; D435i added** | Pi/ROS/servo/TF/camera live baseline; joint1 58…957, joint2 186…873 upright / 327…795 low pose, joint3 100…923, joint4 130…882, joint5 122…869 pulse; D435i depth/colour services; IMU, encoders, e-stop, onboard gamepad/SBUS absent |
| Calibration and visual grasping | **Validated: 3/5 first series and 24/24 in the paired trial; dual-camera PASS; D435i calibrated** | 14-pose chessboard hand-eye 2.94 mm / 0.39°, held-out 3.29 mm / 0.43°; table-down 2.7 mm / 0.42°; front D435i 903.5 px / 0.371 px and 31.6° extrinsic; passes 1–2 and 5 succeeded; passes 3–4 exposed the low-configuration error; 24-pair trial 12/24 vs 24/24; D435i agreed to 2.58 mm x/y and verified one grip + put-back |
| Fast motion | **Validated on hardware (four real-cube runs at 4.0/20)** | Jerk-limited S-curve with measured bus-servo settle; grasp+put-back **11.67 s → 8.94 s (−23 %)**; 4.0/20 defaults; duty-20 base 100.2 mm/s; aligned auto-base landing 2.22 mm; later in-view/far runs grasped in 4.77/4.76 s, and `--return-home` reversed a 245.3 mm approach while holding the cube (11.06 mm landing vs the fitted target) |
| Error budget and tolerance envelope | **Framework implemented; budget complete, bound gates fail** | **measured tolerance 6 mm** (conservative worst side of the ±9 mm opening-axis envelope); all sources measured, including execution **5.386 mm / 1.495° max**, placement **11.514 mm max / 3.080 mm mean** and fixture **14.815 mm radial max**; translation **RSS 13.325 mm** (fail) / worst case **24.014 mm** (fail vs 6 mm); rotation **RSS 1.858°** (pass) / worst **3.322°** (fail vs the provisional 2° gate); dominant translation term is now `gripper_placement` (74.7 %), hand-eye is 6.1 %; empirical corrected placement mean **3.080 mm** (RMS 4.082 mm) is inside 6 mm; fixture-inclusive RSS 19.926 mm |
| Paired trial framework | **Two physical trials complete (2026-11-19 fixed, 2026-11-21 random)** | 2026-11-19: 24 pairs, baseline 12/24 vs intervention 24/24, McNemar p = 0.00049; among 12 dual-success pairs the intervention places 2.19 mm closer (95 % CI −3.96…−0.83, p = 0.0063). 2026-11-21 random-placement: free by-eye spots + base approach, baseline **9/24** vs intervention **24/24**, McNemar **p = 6.1×10⁻⁵**; home release error (+2.12 mm, anchor = corrected target) descriptive only; D1/D2 discards recorded; the fixed 24-mark fan was superseded before data, not run |
| Host/DDS benchmark | **Implemented, read-only hardware snapshot** | laptop + Pi host metrics, 20 Hz state, ~12 Hz camera cross-host, 0.24 ms ping; one snapshot, no load sweep |
| ArmPi Debug Studio | **Implemented through G8.1 + the G8 fast path; motion off by default** | read-only gateway + G2/G3/G4/G5; 144 backend + 81 gateway tests + 3 pose-scope Playwright; G6 supervised single-joint write path verified on hardware with no motion (measured delta 0.0 rad); **G7 pose takeover** real view→grasp→view + gripper sequence bus-verified (low pose within 4 pulses; gripper settled 184/645); **G8 task takeover**: live point cloud → locate → deep-family fit → 13-step task → grip verify → put-back, **verified on hardware** (target radius 0.2384 m, grasp actual 504/339/930/421/499, `close_grip` stalled at raw 416, put-back landing 3.6 mm), including a **min-range clip recovery** for the Aurora's ~0.25 m cutoff; **G8.1 bounded chassis approach**: `APPROACH_REQUIRED` plan + `POST /chassis/pulse` (duty ≤10, ≤2 s, open-loop) closed by the camera, a far cube at 0.357 m brought to 0.221 m in two pulses and grasped (landing 0.5 mm); **G8 fast task path**: 7 phases, dense S-curves at 4.0 rad/s / 20 rad/s² with per-segment gateway validation, base approach inside the run — **hardware-verified** (5.466 s commanded / 13.46 s wall, `close_grip` 414, landing 2.27 mm); **locate window x 0.15–0.55 m / y ±0.15 m** with a **30 mm minimum top extent** (a 34.6 mm visible top is accepted after the 2026-11-21 fix); the takeover launcher routes camera feeds through the read-only gateway; Chinese operator UI; `/servo_states` is a command cache, not measured feedback; `mapping_verified=true`; D435i aux feed on the Dashboard; no e-stop, no self-collision model |
| Clock sync | **Implemented and verified** | laptop-pushed systemd user timer every 60 s; restricted forced-command key; Pi held ~2 s ahead; forced 301 s offset restored; reboot test PASS |
| Mobile base | **Open-loop only; duty-20 auto-approach validated** | four motor duties, no encoders/IMU/odometry; 0.5 s watchdog; any base move invalidates the table calibration; one aligned auto-base run at (0.2316, −0.0025) m; the G8.1 bounded approach (duty ≤10, ≤2 s) is camera-closed but still open-loop |

By 2026-11-20 the live robot runs the vendor Humble core stack in
`ArmPiUltra`; the workstation runs Humble MoveIt/RViz in Docker. The Aurora 930
publishes RGB at about 12 Hz cross-host (`bgr8`) and depth at 8.2 Hz
in-container (`mono16`, mm); an in-container RGB subscriber undercounts to
~8 Hz, so the cross-host number is the sensor rate. The chessboard hand-eye is
**validated** with 14 poses and measured bus-servo joint angles: 2.94 mm /
0.39° board-pose scatter, a 3.29 mm / 0.43° held-out final base and
3.41 mm / 0.44° leave-one-out std; a table-down 15-pose re-fit gives 2.7 mm /
0.42°. The measured body→optical adapter (89.798°, `[5.4, 10.1, -27.4] mm`) is
published as `camera_optical_calibrated`. The 24-pose set selected offline from
14,176 evaluated poses (3,093 feasible) remains planned, not executed.

A chassis-front RealSense D435i was added on 2026-11-20: USB 3.0, colour UVC
1280×720, depth 640×480 Z16 through a patched librealsense (the kernel lacks
`CONFIG_HID_SENSOR_HUB`, so the IMU is unavailable). Its refined intrinsics are
fx = fy 903.5 px (RMS 0.371 px) and its chassis extrinsic is
`[0.136, 0.027, 0.038] m`, 31.6° pitch. `armpi-rs-depth` and `armpi-ext-cam`
boot with `NRestarts=0`, and the camera is published as a MoveIt collision box.
In the first dual-camera grasp it agreed with the wrist camera to **2.58 mm**
in x/y and verified the grip and put-back, while the validated target kept the
constant correction because `e` proved to be an execution offset, not a
measurement bias.

Real visual grasping ran on 2026-11-18/19. Five markerless pick–lift–place
attempts were made on a 39.4 mm cube; three succeeded. Passes 1–2 established
the pipeline (≈149 mm lift; return within 6.6 mm / 8°). Passes 3–4 failed and
measured a configuration-dependent FK/hand–eye error of −8.0/−15.3/−5.8 mm at
the low grasp pose; pass 5 corrected it with a single AprilTag observation and
succeeded (TCP +0.0993 m, object ≈148 mm). The gripper stroke was re-measured
to 2.22 rad after the original open value proved to be a binding stall. Joint
limits were formally surveyed on 2026-11-19 and adopted from the servo EEPROM
with a 20-pulse margin (joint2's high bound tightened to 775), and the live
stack was reloaded on 2026-11-20 so move_group, `armpi_safety` and the Studio
backend all use 775. The controller board probe found only battery, six bus
servos, four open-loop motor duties, buzzer and LEDs — no IMU, encoders,
e-stop or onboard gamepad.
The error budget is now complete: repeatability (0.449 mm RMS / 1.017 mm max),
backlash (1.117 mm), a 46-grasp tolerance envelope (±9 mm, conservative 6 mm)
and the 2026-11-20 terms (execution 5.386 mm / 1.495° max, placement
11.514 mm max / 3.080 mm mean, fixture 14.815 mm radial max) are all measured.
The bound aggregate fails the 6 mm gate (RSS 13.325 mm); the empirical
corrected placement mean (3.080 mm) is inside it. A pre-registered 24-pair
hardware trial measured the intervention: baseline 12/24 vs corrected 24/24
(McNemar p = 0.00049) and a 2.19 mm paired placement improvement
(p = 0.0063). Fast jerk-limited motion then cut the grasp+put-back cycle from
11.67 s to 8.94 s. Two later runs extended the same chain to a cube at
0.464 m, with two closed-loop base pulses to the grasp band and a reverse
pulse home while the cube was still held (landing 11.06 mm against the fitted
target; the return remains open-loop). A random-placement 24-pair follow-up
completed on 2026-11-21: the operator placed the cube at a fresh haphazard
point before every pair, and the paired effect repeated (baseline 9/24 vs
intervention 24/24, McNemar p = 6.1×10⁻⁵). The fixed 24-mark fan was
superseded before data (not run).
Detailed checklist:
[`docs/phase0/PHASE0_ACCEPTANCE.md`](docs/phase0/PHASE0_ACCEPTANCE.md).

## 1. Technical Goal

Develop a real robotic manipulation system with:

- corrected ArmPi robot model;
- 5-DoF constrained motion planning;
- self-implemented hand-eye / camera calibration workflow;
- real visual grasping;
- quantitative error characterization;
- reliability improvement based on measured failure modes;
- optional mobile-base integration.

## 2. Platform Constraint

ArmPi Ultra is treated as a 5-DoF manipulator.

The grasp planner therefore searches within a constrained task space rather than assuming arbitrary 6D end-effector targets.

Initial grasp variables:

```text
target position
+
approach pitch
+
wrist rotation
+
approach offset
```

Position-only IK may be used for reachability checks, but final grasp execution must still enforce a valid tool orientation.

## 3. Robot Model Audit

Known items to inspect or repair:

- `joint1` joint type;
- provisional joint limits;
- provisional effort limits;
- package/resource naming;
- gripper mimic behavior;
- missing camera optical frame;
- measured camera mounting transform.

Real mechanical limits were formally surveyed on hardware on 2026-11-19; see
[`docs/phase2/JOINT_LIMITS.md`](docs/phase2/JOINT_LIMITS.md).

## 4. Camera / Calibration Geometry

The camera is mounted relative to `link4`, before the final wrist rotation.

The calibration transform is therefore modeled as:

```text
link4 → camera
```

rather than assuming a direct gripper-to-camera transform.

Calibration pose collection must account for the limited orientation diversity of this mechanism.

The chassis-front D435i is a separate camera rigid to `base_link`, looking down
at the table. Its measured transform is
`base_link → ext_cam_optical = [0.136, 0.027, 0.038] m`, 31.6° pitch, −1.1°
yaw, with fx = fy 903.5 px (RMS 0.371 px). It is not in the vendor URDF; the
colour node publishes the static TF, and a planning-scene box represents the
body for MoveIt. Depth starts at ~0.2 m, so the grasp band is colour-only.

## 5. Milestone A — Hardware Baseline

- run vendor demos;
- verify ROS2 environment;
- measure real joint limits;
- inspect control / feedback interfaces;
- verify camera topics and TF;
- correct URDF and frame definitions.

Status (2026-11-20), evidence in
[`evidence/phase1/20261117/`](evidence/phase1/20261117/),
[`evidence/phase2/onsite_20261118/`](evidence/phase2/onsite_20261118/),
[`evidence/phase2/onsite_20261119/`](evidence/phase2/onsite_20261119/) and
[`evidence/phase2/onsite_20261120/`](evidence/phase2/onsite_20261120/):

- [x] Vendor core stack and `vision.launch.py` run; camera RGB/depth streams
  and intrinsics captured.
- [x] ROS 2 Humble, CycloneDDS, domain, container and one-action-server graph
  verified.
- [x] Measured joint limits and servo offsets — **formal survey complete
  2026-11-19** on all five arm joints with torque released (joint1 58…957,
  joint2 327…795 low pose / 186…873 upright, joint3 100…923, joint4 130…882,
  joint5 122…869 pulse). Adopted limits are EEPROM − 20 pulses, tightened on
  joint2's high side to 775; the stops are self-collision-limited and
  pose-dependent, so `mapping_verified=true` covers the operating envelope.
- [x] Control/feedback interfaces audited live; `joint_states` are cached
  commands, `get_state` reads the servo register.
- [x] Camera topics and TF verified; the live frames are `rgb_camera_link` /
  `depth_camera_link`; the measured optical frame
  `camera_optical_calibrated` is project-published.
- [x] Correct frame definitions for the real path — body/optical contract and
  `camera_optical_calibrated` are implemented; the model keeps its nominal
  `camera_link` / `camera_optical_frame` for mock only.
- [x] Gripper stroke re-measured to 2.22 rad; controller-board capabilities
  probed (no IMU, encoders, e-stop or onboard gamepad/SBUS).
- [x] Adopted joint2 bound reloaded into the live stack on 2026-11-20:
  symlink description rebuild, move_group/`armpi_safety`/Studio backend
  restarted; joint2 = 1.2 is now rejected.
- [x] Chassis-front D435i mounted on USB 3.0, calibrated
  (903.5 px / 0.371 px RMS; `[0.136, 0.027, 0.038] m`, 31.6°), published as a
  MoveIt collision box, and deployed as `armpi-rs-depth` + `armpi-ext-cam`;
  a 2026-11-20 reboot test passed with `NRestarts=0`.
- [x] Clock sync deployed: laptop systemd user timer every 60 s through a
  restricted forced-command key; Pi held ~2 s ahead; reboot test PASS.

## 6. Milestone B — Calibration

Two calibration pipelines will be evaluated:

```text
vendor calibration baseline
vs
self-implemented calibration workflow
```

Both will use the same independent validation procedure.

Metrics:

- translation error;
- orientation error;
- repeated-run variability;
- sensitivity to calibration samples.

## 7. Milestone C — Real Visual Grasping

Pipeline:

```text
fiducial detection
→ target pose
→ TF transform
→ constrained grasp generation
→ IK
→ MoveIt2
→ execution
→ grasp verification
```

**Status (2026-11-21): real execution validated on 3/5 first-series attempts,
then 24/24 with the constant correction.** Passes 1–2 and 5 completed
pick–lift–place on a 39.4 mm cube; passes 3–4 failed and measured a
configuration-dependent FK/hand–eye error of −8.0/−15.3/−5.8 mm at the low
grasp pose. Pass 5 corrected it with one AprilTag observation and succeeded
(TCP +0.0993 m, object ≈148 mm). The pre-registered 24-pair trial then
measured the corrected method against the raw target: 12/24 → 24/24
(McNemar p = 0.00049). The 2026-11-21 random-placement follow-up repeated the
effect on free by-eye spots with a base approach: **9/24 → 24/24** (McNemar
p = 6.1×10⁻⁵; the home release metric is descriptive). The same chain became
the Studio G8 takeover (live point cloud → locate → deep-family fit → 13-step
task → grip verify → put-back, hardware-verified), extended by G8.1's bounded
open-loop chassis approach and the 7-phase G8 fast path (13.46 s wall,
2.27 mm landing). The real trials used markerless RGB-D segmentation
rather than a fiducial marker; the fiducial bridge remains mock-validated.
A 2026-11-20 dual-camera run used the chassis D435i as an independent
`base_link` observer: it agreed with the wrist camera to **2.58 mm** in x/y,
vetoed nothing, verified the grip and put-back (front estimate 3.65 mm), and
showed that the constant correction is an execution offset rather than a
measurement bias. The same day replaced single-point moves with a jerk-limited
S-curve and measured settles: the grasp+put-back cycle fell **11.67 s →
8.94 s** and the aligned duty-20 base run landed **2.22 mm** from the start.
Two further runs then measured the v2 envelope on an in-view cube (grasp
4.77 s, release 4.44 s, landing **0.92 mm**) and on a cube at 0.464 m: two
closed-loop base pulses (294.7 + 32.4 mm) reached the grasp band, one 2.50 s
reverse pulse undid 245.3 mm with the cube held, and the release landed
**11.06 mm** from the fitted target. The return home is open-loop, so it is
centimetre-level (32.1 mm on 2026-11-19, 9.4 mm after one correction), and
the 11.06 mm figure is a base-frame placement error, not a home error.
Evidence:
[`evidence/phase2/onsite_20261119/`](evidence/phase2/onsite_20261119/),
[`evidence/phase2/onsite_20261120/`](evidence/phase2/onsite_20261120/) and
[`evidence/phase3/onsite_20261119/`](evidence/phase3/onsite_20261119/README.md).

## 8. Milestone D — Error Budget

Status (2026-11-20): the aggregation framework, **completed** project budget,
paired-trial recorder and host benchmark are implemented. Every required
source is now measured: hand-eye 3.29 mm, fiducial 1.69 mm, repeatability
1.017 mm, backlash 1.117 mm, **grasp execution 5.386 mm / 1.495° max**,
**gripper placement 11.514 mm max / 3.080 mm mean** and **object-fixture pose
14.815 mm radial max**; the task tolerance is the measured ±9 mm opening axis
(conservative 6 mm). The completed **bound-level** budget fails: translation
**RSS 13.325 mm** / worst case **24.014 mm** vs 6 mm; rotation **RSS 1.858°**
(pass) / worst **3.322°** (fail) vs the provisional 2° gate. The empirical
corrected placement mean is **3.080 mm** (RMS 4.082 mm), inside 6 mm; the tail
(11.514 mm) is what pushes the bound budget over. The fixture-inclusive
aggregate (19.926 mm RSS) is reported separately because every trial
re-measures the cube. The 2° rotation gate is still provisional. Two further
runners are implemented but not yet run: a wrist-yaw rotation-tolerance sweep
on `joint5`, and a ruler-fixture visual-displacement grid. See
[`docs/phase3/ERROR_BUDGET.md`](docs/phase3/ERROR_BUDGET.md),
[`docs/phase3/PAIRED_TRIALS.md`](docs/phase3/PAIRED_TRIALS.md) and
[`evidence/phase3/onsite_20261120/`](evidence/phase3/onsite_20261120/README.md).

Measure:

- arm repeatability;
- backlash / hysteresis;
- calibration error;
- visual relative-pose error;
- grasp execution error;
- task tolerance envelope.

### Visual Ground Truth

Use known relative target displacement from a physical fixture, such as:

- coordinate grid;
- linear guide;
- caliper-positioned jig.

Compare measured visual displacement against known physical displacement.

### Task Tolerance

Intentionally offset the grasp target to determine how much x/y/z/angular error the physical grasp can tolerate.

The purpose is to identify which measured error sources are large enough to exceed task tolerance.

## 9. Milestone E — Reliability Intervention

The intervention will be chosen from the measured dominant error source.

Possible examples:

- temporal / multi-view visual estimation;
- approach-direction control for backlash;
- empirical bias compensation;
- calibration sample-selection improvement;
- final visual correction.

**Status (2026-11-19): completed.** The dominant term is the FK/hand–eye error
at the low grasp pose. The planned joint-offset calibration was **attempted and
rejected**: joint offsets, a `link4` 3-DoF offset and a `link4` 6-DoF offset
all fit the tag survey to 1.4–2.3 mm but predict a correction 10.6–15.0 mm away
from the one four passing runs used, at their measured configurations. The
intervention is therefore the constant base-frame correction
(`(+13.7, +3.9, +8.9) mm`), and the paired hardware trial tested it: **24
pairs**, baseline 12/24 vs intervention 24/24 (McNemar p = 0.00049), and a
**2.19 mm** paired placement improvement among the 12 dual-success pairs
(95 % CI −3.96…−0.83 mm, p = 0.0063). The constant is supported inside the
measured envelope, which a fixed-tag yaw extension widened to `j1 ≈ 0.5 rad`
(at `j1 = 0.487 rad` the required correction is `(9.14, 7.18, 8.75) mm`, the
constant is 5.6 mm away and the fitted models 14–19 mm away with the `y` sign
flipped); tool-down and view `j4 −1.58…−1.66` remain the other bounds. Two
yawed *grasp* attempts failed for cube geometry, not for the correction: a
square cube at a 20–30° bearing presents its ~50 mm diagonal against a ~40 mm
opening, so the cube must be rotated to face the gripper before the yawed
grasp test. See
[`evidence/phase3/onsite_20261119/`](evidence/phase3/onsite_20261119/README.md)
and
[`evidence/calibration/20261119/low_config_error/model_selection.md`](evidence/calibration/20261119/low_config_error/model_selection.md).

## 10. Experimental Design

Primary outcome:

```text
continuous physical error
```

Examples:

- endpoint deviation in mm;
- relative visual displacement error;
- calibration validation error.

Secondary outcome:

```text
grasp success / failure
```

Where practical, use paired randomized trials:

```text
same object placement
├── baseline
└── intervention
```

Randomize execution order.

**Current evidence (2026-11-19):** the first physical paired trial (24 pairs)
is complete: the correction raises grasp success from 12/24 to 24/24 and
reduces the placement error by 2.19 mm among dual-success pairs. The earlier
five unpaired grasps (3/5) remain feasibility evidence. Because a baseline miss
has no release landing, the binary outcome is the primary test.

**Completed follow-up (2026-11-21):** the random-placement 24-pair trial
tested whether the same effect and the locate/approach chain survive unknown
placements; it did (baseline 9/24 = 37.5% vs intervention 24/24 = 100%,
McNemar p = 6.1×10⁻⁵; the home release metric is descriptive because its
anchor is the corrected target). Before every pair the operator puts the cube by eye at a new spot
in the forward workspace (no measuring; the radius 0.25-0.55 m / bearing
-20..+25 deg envelope is a guide). The robot locates it with the wrist wide
measurement, the chassis D435i or a `joint1` yaw sweep; if it is already in
the 0.226-0.234 m band it grasps directly, otherwise it drives the base
closed-loop into the band. After a held grip it **carries the cube back to the
start and releases it there**; the operator takes it from the start and
re-places it at the same spot for the second method. The design, reference
envelope, seed-17 plan and hashes were written before data
([`docs/phase3/RANDOM_SITE_DESIGN.md`](docs/phase3/RANDOM_SITE_DESIGN.md)).
The free near/far warm-up passed; one old-flow attempt (site put-back) was
discarded (D1) and the new-flow session then completed all 24 pairs. The
earlier fixed 24-mark fan was superseded before data and never run.

## 11. Safety

Automated experiments require:

- conservative velocity / acceleration limits;
- validated joint limits;
- execution timeout;
- communication watchdog;
- workspace checks;
- collision checking where available;
- accessible hardware stop / power cut.

**Current state (2026-11-20):** all five arm joints were formally surveyed on
2026-11-19 and the adopted position limits are servo EEPROM − 20 pulses
(joint2's high bound tightened to 775 from the low-pose hand stop), now
reloaded into the live move_group/`armpi_safety`/Studio stack. The stops are
pose-dependent self-collision limits, so `mapping_verified=true` covers the
operating envelope. The chassis D435i is represented by a 90×25×26 mm
planning-scene box; it protects MoveIt plans only and is bypassed by direct
joint moves and `armpi_safety`. Fast motion defaults to 4.0 rad/s / 20 rad/s²
with a 1.4 rad/s holding-cube descent and a measured bus-servo settle after
every move. The robot has no e-stop or brake and falls when power is cut; all
motion is supervised. A laptop-pushed timer now keeps the Pi clock ~2 s ahead
for MoveIt execution, and inbound access is restricted to the direct Ethernet
link.

## 12. Optional Mobile Extension

After the fixed-base pipeline is stable:

```text
mecanum base
→ teleoperation
→ odometry
→ TF integration
→ basic navigation
→ one mobile manipulation demo
```

This extension should not compromise the stability of the core experimental system.

## 13. Status Convention

Repository documentation should distinguish:

- **Implemented**
- **Validated**
- **In Progress**
- **Planned**

Vendor-provided functionality should be identified clearly and separated from project-specific implementation and experiments.
