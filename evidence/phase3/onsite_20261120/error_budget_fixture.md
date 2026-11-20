# Error Budget — fiducial_grasp_onsite_20261120

- Generated: `2026-11-20T02:45:35.340101+00:00`
- Task: fiducial_grasp
- Status: **COMPLETE**
- Samples: 10000 (seed 7)
- Tolerance basis: Measured 46-grasp tolerance envelope, conservative worst side 6 mm; the 2 deg rotation gate is still provisional.

## translation_mm

| Source | Category | Kind | Value (mm) | Basis | Contribution | Evidence |
|---|---|---|---:|---|---:|---|
| `hand_eye_translation` | calibration | bounded | 3.290 | held_out_max | 2.7% | evidence/calibration/20261117/ |
| `fiducial_translation` | perception | bounded | 1.690 | semi_synthetic_max | 0.7% | evidence/phase2/real_background_20261117/ |
| `arm_repeatability` | repeatability | bounded | 1.017 | hardware_max_30_visits | 0.3% | evidence/phase3/onsite_20261119/repeatability/ |
| `arm_backlash` | backlash | bounded | 1.117 | hardware_hysteresis | 0.3% | evidence/phase3/onsite_20261119/backlash/ |
| `grasp_execution` | execution | bounded | 5.386 | hardware_max_20_grasps | 7.3% | evidence/phase3/onsite_20261120/trial_report.json |
| `gripper_placement` | gripper | bounded | 11.514 | hardware_max_20_placements | 33.4% | evidence/phase3/onsite_20261120/trial_report.json |
| `object_fixture_pose` | other | bounded | 14.815 | hardware_max_20_placements | 55.3% | evidence/phase3/onsite_20261120/fixture_report.json |

- RSS: **19.926 mm**
- Worst case: **38.829 mm**
- Tolerance: 6.000 mm; RSS pass **False**; worst-case pass **False**; margin -13.926 mm
- 95% interval: not computed (a random/bounded source has no distribution).
- Dominant source: `object_fixture_pose`

## rotation_deg

| Source | Category | Kind | Value (deg) | Basis | Contribution | Evidence |
|---|---|---|---:|---|---:|---|
| `hand_eye_rotation` | calibration | bounded | 0.430 | held_out_max | 5.4% | evidence/calibration/20261117/ |
| `fiducial_rotation` | perception | bounded | 0.530 | semi_synthetic_max | 8.1% | evidence/phase2/real_background_20261117/ |
| `arm_repeatability_rotation` | repeatability | bounded | 0.867 | hardware_max_30_visits | 21.8% | evidence/phase3/onsite_20261119/repeatability/ |
| `grasp_execution_rotation` | execution | bounded | 1.495 | hardware_max_20_grasps | 64.7% | evidence/phase3/onsite_20261120/trial_report.json |

- RSS: **1.858 deg**
- Worst case: **3.322 deg**
- Tolerance: 2.000 deg; RSS pass **True**; worst-case pass **False**; margin 0.142 deg
- 95% interval: not computed (a random/bounded source has no distribution).
- Dominant source: `grasp_execution_rotation`

## Limitations

- The execution offset is systematic and the constant correction was fitted with it present; the component RSS and the corrected task outcome answer different questions.
- Gripper placement is a task-level landing displacement, not a pure release term; it overlaps execution and the residual correction.
- The object-fixture term is a nuisance in this pipeline because every trial re-measures the cube at the view pose; the fixture-inclusive report shows it separately.
- The measured values are maxima over one session, treated as bounds like the calibration terms.
- The 2 deg rotation tolerance is a provisional protocol gate, not a measured physical tolerance.
- RSS assumes independent sources; correlated sources would be over-counted.
- Worst case is the linear sum of measured magnitudes and is conservative.
- Unmeasured sources are excluded from the totals; a partial budget is not a validated task error.
- A Monte Carlo interval is only reported when every random/bounded source has a distribution.
- The tolerance is a provisional gate unless a physical tolerance experiment has been recorded.
