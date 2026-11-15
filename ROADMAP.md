# ArmPi Ultra Manipulation Project — Technical Roadmap

> Imported from `ROADMAP_PUBLIC.md` (USB, 2026-10-26) on 2026-11-15.
> The previous 8-week learning roadmap is archived at
> `docs/archive/ROADMAP_8WEEK.md`. The private master plan is intentionally
> kept outside this public repository.

> This roadmap tracks the engineering and experimental development of a low-cost ROS2 manipulation platform based on ArmPi Ultra.

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
