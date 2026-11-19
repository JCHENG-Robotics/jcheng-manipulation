# JCHENG Manipulation: A ROS 2-Based Robotic Manipulation System with Quantitative Reliability Evaluation

> Imported from `ROADMAP_PUBLIC.md` (USB, 2026-10-26) on 2026-11-15.
> The previous 8-week learning roadmap is archived at
> `docs/archive/ROADMAP_8WEEK.md`. The private master plan is intentionally
> kept outside this public repository.

> Technical roadmap: this document tracks the engineering and experimental development of a low-cost ROS2 manipulation platform based on ArmPi Ultra.

## Current status (2026-11-19)

| Milestone | Status | Evidence / limit |
|---|---|---|
| Robot model and TF | **Implemented; live arm/camera TF verified** | 17 links, 16 joints; live `base_link → rgb_camera_link` / `depth_camera_link`; measured optical frame `camera_optical_calibrated`; adopted limits are EEPROM−20 pulses, `mapping_verified=false` |
| MoveIt 2 constrained planning | **Implemented; mock + real execution validated** | Pilz `PTP` 6D-constrained goal (mock); real execution through Humble Docker MoveIt on 2026-11-18/19 |
| 5-DoF grasp candidate search | **Implemented; mock + real use** | Joint-space sampler: 80 candidates → 40 IK OK → 26 collision-free → 3 PTP plans (mock); the same search fed all five real grasp passes |
| Fiducial → grasp bridge | **Implemented; mock E2E validated (2026-11-17)** | `target_pose` → TF → grasp search → safety veto; synthetic marker, plan-only; the real grasps used the grasp planner directly, not this bridge |
| Fiducial target localization | **Implemented; synthetic + real-background validated; physical tag scale check** | AprilTag 36h11; max 1.82 mm / 1.49° synthetic; 15/15 semi-synthetic on real frames, max 1.69 mm / 0.53°; 0 false detections on 8 real frames; physical tag scale 100.67 mm vs 100.0 mm (0.67 %) |
| Safety supervisor | **Implemented; mock + hardware veto** | state watchdog, joint/velocity/acceleration/start-state/duration/workspace/table-clearance checks; veto service; adopted EEPROM-bounded limits; no e-stop or brake |
| Measurement protocols | **Implemented; repeatability + backlash measured on hardware (2026-11-19)** | 30-visit repeatability **0.449 mm RMS / 1.017 mm max**; backlash **1.117 mm**; tag-based runner in `tools/phase3/measurement_run.py`; visual-displacement and gripper-envelope still simulated only |
| 5-DoF task-space policy | **Documented** | `docs/phase0/ARMPI_5DOF_CONSTRAINTS.md` |
| Hardware baseline | **Captured; limits partially measured; board probed** | Pi/ROS/servo/TF/camera live baseline; joint2 low 322, joint3 stop 939, joint4 high 901; IMU, encoders, e-stop, onboard gamepad/SBUS absent |
| Calibration and visual grasping | **Validated: 3/5 real pick–lift–place attempts; constant low-config correction supported, joint offsets rejected** | 14-pose chessboard hand-eye 2.94 mm / 0.39°, held-out 3.29 mm / 0.43°; table-down 15-pose re-fit 2.7 mm / 0.42°; passes 1–2 and 5 succeeded; passes 3–4 exposed the low-configuration error; a 10-pose fixed-tag survey measured it as a constant (+11.6, +4.0, +8.5) mm (spread 0.40–0.99 mm) and the +13.7/+3.9/+8.9 mm correction was grasp-validated (release 1.3 mm); four models were fitted, joint offsets and link4-frame models were rejected; 24-pose chessboard set still planned, not executed |
| Error budget and tolerance envelope | **Framework implemented; budget updated with measured repeatability and backlash** | translation **RSS 3.995 mm** (pass) / worst case **7.114 mm** (fail vs provisional 5 mm); rotation **RSS 1.103° / worst 1.827°** (both pass); dominant translation term is hand–eye (68 %); grasp execution, gripper placement and fixture pose still unmeasured |
| Paired trial framework | **Implemented; first physical trial complete (2026-11-19)** | 24 pairs / 48 trials: baseline 12/24 vs intervention 24/24, McNemar p = 0.00049; among 12 dual-success pairs the intervention places 2.19 mm closer (95 % CI −3.96…−0.83, p = 0.0063); four deviations documented |
| Host/DDS benchmark | **Implemented, read-only hardware snapshot** | laptop + Pi host metrics, 20 Hz state, ~12 Hz camera cross-host, 0.24 ms ping; one snapshot, no load sweep |
| ArmPi Debug Studio | **Implemented through G6; motion off by default** | read-only gateway + G2/G3/G4/G5; 92 backend + 38 gateway tests; G6 supervised single-joint write path verified on hardware with no motion (measured delta 0.0 rad); `mapping_verified=false`, no e-stop |
| Mobile base | **Open-loop only; planned extension** | four motor duties, no encoders/IMU/odometry; 0.5 s watchdog; any base move invalidates the table calibration |

By 2026-11-19 the live robot runs the vendor Humble core stack in
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

Real visual grasping ran on 2026-11-18/19. Five markerless pick–lift–place
attempts were made on a 39.4 mm cube; three succeeded. Passes 1–2 established
the pipeline (≈149 mm lift; return within 6.6 mm / 8°). Passes 3–4 failed and
measured a configuration-dependent FK/hand–eye error of −8.0/−15.3/−5.8 mm at
the low grasp pose; pass 5 corrected it with a single AprilTag observation and
succeeded (TCP +0.0993 m, object ≈148 mm). The gripper stroke was re-measured
to 2.22 rad after the original open value proved to be a binding stall. Joint
limits were adopted from the servo EEPROM with a 20-pulse margin, and the
controller board probe found only battery, six bus servos, four open-loop
motor duties, buzzer and LEDs — no IMU, encoders, e-stop or onboard gamepad.
The error budget remains partial and no paired hardware trial exists.
Detailed checklist: [`docs/phase0/PHASE0_ACCEPTANCE.md`](docs/phase0/PHASE0_ACCEPTANCE.md).

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

Real mechanical limits will be verified on hardware.

## 4. Camera / Calibration Geometry

The camera is mounted relative to `link4`, before the final wrist rotation.

The calibration transform is therefore modeled as:

```text
link4 → camera
```

rather than assuming a direct gripper-to-camera transform.

Calibration pose collection must account for the limited orientation diversity of this mechanism.

## 5. Milestone A — Hardware Baseline

- run vendor demos;
- verify ROS2 environment;
- measure real joint limits;
- inspect control / feedback interfaces;
- verify camera topics and TF;
- correct URDF and frame definitions.

Status (2026-11-19), evidence in
[`evidence/phase1/20261117/`](evidence/phase1/20261117/),
[`evidence/phase2/onsite_20261118/`](evidence/phase2/onsite_20261118/) and
[`evidence/phase2/onsite_20261119/`](evidence/phase2/onsite_20261119/):

- [x] Vendor core stack and `vision.launch.py` run; camera RGB/depth streams
  and intrinsics captured.
- [x] ROS 2 Humble, CycloneDDS, domain, container and one-action-server graph
  verified.
- [ ] Measured joint limits and servo offsets — **partial**: joint2 low 322,
  joint3 stop 939, joint4 high 901; joint1 low, joint3 positive and joint5
  remain unmeasured. Adopted limits are servo EEPROM − 20 pulses;
  `mapping_verified=false`.
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

**Status (2026-11-19): real execution validated on 3/5 attempts.** Passes 1–2
and 5 completed pick–lift–place on a 39.4 mm cube; passes 3–4 failed and
measured a configuration-dependent FK/hand–eye error of −8.0/−15.3/−5.8 mm at
the low grasp pose. Pass 5 corrected it with one AprilTag observation and
succeeded (TCP +0.0993 m, object ≈148 mm). The real trials used markerless
RGB-D segmentation rather than a fiducial marker; the fiducial bridge remains
mock-validated. Evidence:
[`evidence/phase2/onsite_20261119/`](evidence/phase2/onsite_20261119/).

## 8. Milestone D — Error Budget

Status (2026-11-19): the aggregation framework, partial project budget,
paired-trial recorder and host benchmark are implemented. Real unpaired grasps
now exist (3/5), but repeatability, backlash, grasp execution, gripper
placement and fixture pose remain unmeasured. The 5 mm / 2° tolerance is a
provisional gate, not a measured physical tolerance. See
[`docs/phase3/ERROR_BUDGET.md`](docs/phase3/ERROR_BUDGET.md) and
[`docs/phase3/PAIRED_TRIALS.md`](docs/phase3/PAIRED_TRIALS.md).

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
flipped); tool-down and view `j4 −1.58…−1.66` remain the other bounds. See
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

## 11. Safety

Automated experiments require:

- conservative velocity / acceleration limits;
- validated joint limits;
- execution timeout;
- communication watchdog;
- workspace checks;
- collision checking where available;
- accessible hardware stop / power cut.

**Current state (2026-11-19):** adopted position limits are servo EEPROM − 20
pulses; joint2's low stop is measured, the other mechanical stops are partial.
The robot has no e-stop or brake and falls when power is cut; all motion is
supervised. The robot clock must lead the workstation by 1–2 s for MoveIt
execution, and inbound access is restricted to the direct Ethernet link.

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
