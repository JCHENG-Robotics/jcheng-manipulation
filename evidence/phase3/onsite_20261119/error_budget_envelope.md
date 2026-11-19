# Error Budget — fiducial_grasp_onsite_20261119_envelope

- Generated: `2026-11-19T11:39:40.017436+00:00`
- Task: fiducial_grasp
- Status: **PARTIAL**
- Samples: 10000 (seed 7)
- Tolerance basis: Measured gripper envelope, 2026-11-19: the opening axis passes [-6, +12] mm (centre +3 mm, ±9 mm about the true cube centre), x and z pass at ±9 mm with no failure. 6 mm is the conservative worst-side value; the 2 deg rotation tolerance is still the provisional protocol gate.

## translation_mm

| Source | Category | Kind | Value (mm) | Basis | Contribution | Evidence |
|---|---|---|---:|---|---:|---|
| `hand_eye_translation` | calibration | bounded | 3.290 | held_out_max | 67.8% | evidence/calibration/20261117/ |
| `fiducial_translation` | perception | bounded | 1.690 | semi_synthetic_max | 17.9% | evidence/phase2/real_background_20261117/ |
| `arm_repeatability` | repeatability | bounded | 1.017 | hardware_max_30_visits | 6.5% | evidence/phase3/onsite_20261119/repeatability/ |
| `arm_backlash` | backlash | bounded | 1.117 | hardware_hysteresis | 7.8% | evidence/phase3/onsite_20261119/backlash/ |
| `grasp_execution` | execution | bounded | unmeasured | - | - | - |
| `gripper_placement` | gripper | bounded | unmeasured | - | - | - |
| `object_fixture_pose` | other | bounded | unmeasured | - | - | - |

- RSS: **3.995 mm**
- Worst case: **7.114 mm**
- Tolerance: 6.000 mm; RSS pass **True**; worst-case pass **False**; margin 2.005 mm
- 95% interval: not computed (a random/bounded source has no distribution).
- Dominant source: `hand_eye_translation`
- **Missing required sources:** `grasp_execution`, `gripper_placement`, `object_fixture_pose`

## rotation_deg

| Source | Category | Kind | Value (deg) | Basis | Contribution | Evidence |
|---|---|---|---:|---|---:|---|
| `hand_eye_rotation` | calibration | bounded | 0.430 | held_out_max | 15.2% | evidence/calibration/20261117/ |
| `fiducial_rotation` | perception | bounded | 0.530 | semi_synthetic_max | 23.1% | evidence/phase2/real_background_20261117/ |
| `arm_repeatability_rotation` | repeatability | bounded | 0.867 | hardware_max_30_visits | 61.7% | evidence/phase3/onsite_20261119/repeatability/ |
| `grasp_execution_rotation` | execution | bounded | unmeasured | - | - | - |

- RSS: **1.103 deg**
- Worst case: **1.827 deg**
- Tolerance: 2.000 deg; RSS pass **True**; worst-case pass **True**; margin 0.897 deg
- 95% interval: not computed (a random/bounded source has no distribution).
- Dominant source: `arm_repeatability_rotation`
- **Missing required sources:** `grasp_execution_rotation`

## Limitations

- Rotation tolerance (2 deg) is still a provisional gate; the envelope swept translation only.
- The envelope used two trials per offset point in one session; the boundary between 6 and 12 mm in y is bracketed, not resolved.
- Still partial: grasp execution, gripper placement and object-fixture pose are unmeasured as separate sources.
- Repeatability and backlash are hardware maxima over one session, treated as bounds like the calibration terms.
- The fiducial term comes from a composited marker on real frames, not a physically present marker.
- RSS assumes independent sources; correlated sources would be over-counted.
- Worst case is the linear sum of measured magnitudes and is conservative.
- Unmeasured sources are excluded from the totals; a partial budget is not a validated task error.
- A Monte Carlo interval is only reported when every random/bounded source has a distribution.
- The tolerance is a provisional gate unless a physical tolerance experiment has been recorded.

> **Partial budget.** Required sources are unmeasured; the totals are lower bounds for those metrics, not a complete task error estimate.
