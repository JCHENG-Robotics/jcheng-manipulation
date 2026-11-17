# JCHENG Manipulation: A ROS 2-Based Robotic Manipulation System with Quantitative Reliability Evaluation

> Imported from `ROADMAP_PUBLIC.md` (USB, 2026-10-26) on 2026-11-15.
> The previous 8-week learning roadmap is archived at
> `docs/archive/ROADMAP_8WEEK.md`. The private master plan is intentionally
> kept outside this public repository.

> Technical roadmap: this document tracks the engineering and experimental development of a low-cost ROS2 manipulation platform based on ArmPi Ultra.

## Current status (2026-11-17)

| Milestone | Status | Evidence / limit |
|---|---|---|
| Robot model and TF | **Implemented; live arm/camera TF verified** | 17 links, 16 joints; live `base_link → rgb_camera_link` / `depth_camera_link`; measured optical frame `camera_optical_calibrated`; model `camera_optical_frame` stays mock-only |
| MoveIt 2 constrained planning | **Implemented, mock-validated; real reset_servo verified** | Pilz `PTP` 6D-constrained goal (plan-only); one supervised real motion through Humble Docker MoveIt |
| 5-DoF grasp candidate search | **Implemented, mock-validated** | Joint-space sampler: 80 candidates → 40 IK OK → 26 collision-free → 3 PTP plans; no hardware trials |
| Fiducial target localization | **Implemented, synthetic + real-background validated** | AprilTag 36h11; max 1.82 mm / 1.49° synthetic; 15/15 semi-synthetic on real frames, max 1.69 mm / 0.53°; 0 false detections on 8 real frames |
| 5-DoF task-space policy | **Documented** | `docs/phase0/ARMPI_5DOF_CONSTRAINTS.md` |
| Hardware baseline | **In progress (2026-11-17)** | Pi/ROS/servo/TF/camera live baseline; joint limits and IMU open |
| Calibration and visual grasping | **In progress — hand-eye validated, diverse 24-pose set selected** | 14-pose chessboard hand-eye with measured bus-servo joints: 2.94 mm / 0.39° board scatter, held-out 3.29 mm / 0.43°; measured optical frame (89.798°) published; next capture uses 24 offline-selected poses (diversity/coplanarity checks pass, not executed); real grasp not started |
| Error budget and tolerance envelope | **Planned** | Requires measured repeatability, backlash and calibration |
| Mobile base | **Planned (bonus)** | Purchase decision deadline 2027-01-31 |

Phase 1 started on 2026-11-17. The live robot runs the vendor Humble core stack
in `ArmPiUltra`; the laptop runs Humble MoveIt/RViz in Docker and one supervised
`reset_servo` motion was verified. The Aurora 930 publishes RGB (8 Hz, `bgr8`)
and depth (8 Hz, `mono16`, mm). Joint limits are **not measured** and the IMU is
not publishing. The chessboard hand-eye is **validated** with 14 poses and
measured bus-servo joint angles: 2.94 mm / 0.39° board-pose scatter, a
3.29 mm / 0.43° held-out final base and 3.41 mm / 0.44° leave-one-out std.
The measured body→optical adapter
(89.798°, `[5.4, 10.1, -27.4] mm`) is published as
`camera_optical_calibrated` and is now used by the fiducial localizer. The
next hand-eye capture uses a 24-pose set selected offline from 14,176
evaluated poses (3,093 feasible) with 33.5° view-angle spread and
non-coplanar camera centres; the set is planned, not yet executed. The real-background
semi-synthetic check passes 15/15 poses (max 1.69 mm / 0.53°)
with zero false detections on 8 real frames. The shipped vendor
`hand2cam_tf_matrix` is **stale for the current mount** and is not valid
Baseline A. The grasp-candidate search remains mock-validated; the fiducial →
grasp bridge and a real visual grasp are not started.
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

Status (2026-11-17), evidence in
[`evidence/phase1/20261117/`](evidence/phase1/20261117/):

- [x] Vendor core stack and `vision.launch.py` run; camera RGB/depth streams
  and intrinsics captured.
- [x] ROS 2 Humble, CycloneDDS, domain, container and one-action-server graph
  verified.
- [ ] Measured joint limits and servo offsets — **open**; only two command
  snapshots exist.
- [x] Control/feedback interfaces audited live; `joint_states` are cached
  commands.
- [x] Camera topics and TF verified; the live frames are `rgb_camera_link` /
  `depth_camera_link`; the measured optical frame
  `camera_optical_calibrated` is project-published.
- [x] Correct frame definitions for the real path — body/optical contract and
  `camera_optical_calibrated` are implemented; the model keeps its nominal
  `camera_link` / `camera_optical_frame` for mock only.

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

## 8. Milestone D — Error Budget

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

## 11. Safety

Automated experiments require:

- conservative velocity / acceleration limits;
- validated joint limits;
- execution timeout;
- communication watchdog;
- workspace checks;
- collision checking where available;
- accessible hardware stop / power cut.

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
