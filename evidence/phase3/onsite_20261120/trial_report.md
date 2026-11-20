# Grasp execution and gripper placement (2026-11-20)

20 attempts, 20 held and put back.

## Grasp execution (commanded vs bus-servo registers at the grasp pose)

| Metric | Mean | SD | Max |
|---|---:|---:|---:|
| Translation | 4.220 mm | 0.628 mm | 5.386 mm |
| Rotation | 1.069 deg | 0.207 deg | 1.495 deg |

## Gripper placement (landing minus pre-grasp measurement)

| Metric | Value |
|---|---:|
| Successful placements | 20 |
| Mean radial displacement | 3.080 mm |
| SD | 2.748 mm |
| Max | 11.514 mm |
| Mean xy vector | (-0.987, +1.634) mm |

Execution is an arm-motion term: it is the joint-level tracking error at the grasp pose, not the object shift. Placement is the object displacement caused by the whole grasp-lift-release cycle.
