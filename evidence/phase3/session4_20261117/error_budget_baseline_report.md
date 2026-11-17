# Error Budget — fiducial_grasp_partial_20261117

- Generated: `2026-11-17T12:13:21.852726+00:00`
- Task: fiducial_grasp
- Status: **PARTIAL**
- Samples: 10000 (seed 7)
- Tolerance basis: Provisional protocol gates (5 mm / 2 deg); not a measured physical grasp tolerance.

## translation_mm

| Source | Category | Kind | Value (mm) | Basis | Contribution | Evidence |
|---|---|---|---:|---|---:|---|
| `hand_eye_translation` | calibration | bounded | 3.290 | held_out_max | 79.1% | evidence/calibration/20261117/ |
| `fiducial_translation` | perception | bounded | 1.690 | semi_synthetic_max | 20.9% | evidence/phase2/real_background_20261117/ |
| `arm_repeatability` | repeatability | bounded | unmeasured | - | - | - |
| `arm_backlash` | backlash | bounded | unmeasured | - | - | - |
| `grasp_execution` | execution | bounded | unmeasured | - | - | - |
| `gripper_placement` | gripper | bounded | unmeasured | - | - | - |
| `object_fixture_pose` | other | bounded | unmeasured | - | - | - |

- RSS: **3.699 mm**
- Worst case: **4.980 mm**
- Tolerance: 5.000 mm; RSS pass **True**; worst-case pass **True**; margin 1.301 mm
- 95% interval: not computed (a random/bounded source has no distribution).
- Dominant source: `hand_eye_translation`
- **Missing required sources:** `arm_repeatability`, `arm_backlash`, `grasp_execution`, `gripper_placement`, `object_fixture_pose`

## rotation_deg

| Source | Category | Kind | Value (deg) | Basis | Contribution | Evidence |
|---|---|---|---:|---|---:|---|
| `hand_eye_rotation` | calibration | bounded | 0.430 | held_out_max | 39.7% | evidence/calibration/20261117/ |
| `fiducial_rotation` | perception | bounded | 0.530 | semi_synthetic_max | 60.3% | evidence/phase2/real_background_20261117/ |
| `arm_repeatability_rotation` | repeatability | bounded | unmeasured | - | - | - |
| `grasp_execution_rotation` | execution | bounded | unmeasured | - | - | - |

- RSS: **0.682 deg**
- Worst case: **0.960 deg**
- Tolerance: 2.000 deg; RSS pass **True**; worst-case pass **True**; margin 1.318 deg
- 95% interval: not computed (a random/bounded source has no distribution).
- Dominant source: `fiducial_rotation`
- **Missing required sources:** `arm_repeatability_rotation`, `grasp_execution_rotation`

## Limitations

- Partial budget: repeatability, backlash, grasp execution, gripper placement and object-fixture pose are unmeasured.
- Hand-eye and fiducial values are maxima, not 1-sigma estimates; RSS is indicative and worst case is conservative.
- The 5 mm / 2 deg tolerance is a provisional protocol gate, not a measured physical grasp tolerance.
- The fiducial term comes from a composited marker on real frames, not a physically present marker.
- RSS assumes independent sources; correlated sources would be over-counted.
- Worst case is the linear sum of measured magnitudes and is conservative.
- Unmeasured sources are excluded from the totals; a partial budget is not a validated task error.
- A Monte Carlo interval is only reported when every random/bounded source has a distribution.
- The tolerance is a provisional gate unless a physical tolerance experiment has been recorded.

> **Partial budget.** Required sources are unmeasured; the totals are lower bounds for those metrics, not a complete task error estimate.
