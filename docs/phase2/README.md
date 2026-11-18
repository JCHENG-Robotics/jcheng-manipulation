# Phase 2 — Calibration and Visual Grasping

> Phase 1 hardware baseline captured 2026-11-17; the real RGB/depth camera is
> available. Phase 2 hand-eye calibration is **validated** (2.94 mm / 0.39°
> board scatter, 3.29 mm / 0.43° held-out). The measured optical frame is
> wired into the localizer as `camera_optical_calibrated`. A 24-pose diverse
> set is selected offline for the next capture. The fiducial → grasp bridge and
> a mock visual-grasp E2E are now validated; a real visual grasp is still
> pending. A flat AprilTag independently checked the scale: **100.67 mm vs
> 100.0 mm lateral (0.67 %)**, and a table-down chessboard fit is
> relative-consistent (2.7 mm / 0.42°). A follow-up depth point-cloud plane
> (**147,626 inliers, 0.7 mm RMS**) plus a **215 mm ruler check** place the
> table at **z ≈ −0.078 m** in `base_link`. The board is **11 mm** thick and
> is **not visible to the depth camera** (high-contrast/glossy board → depth
> holes filled with the table); a same-frame matte-AprilTag test measured a
> **−4.6 mm same-surface RGB-vs-depth gap**, so the old 6.1 mm number is not
> the board thickness. A supervised object-free approach at
> `camera_forward` was planned and executed through the grasp planner
> (**1 mm** final TCP match); its main finding is that the camera_forward
> view volume is not a grasp volume (the arm reaches only ~0.10 m in front of
> the lens at that height).

| Document | Purpose |
|---|---|
| [GRASP_CANDIDATE_SEARCH.md](GRASP_CANDIDATE_SEARCH.md) | 5-DoF constrained grasp-candidate generation, MoveIt screening, mock evidence, limitations |
| [FIDUCIAL_TARGET_LOCALIZATION.md](FIDUCIAL_TARGET_LOCALIZATION.md) | AprilTag/ArUco detection, 6D pose, measured optical frame, real-background semi-synthetic validation |
| [FIDUCIAL_GRASP_BRIDGE.md](FIDUCIAL_GRASP_BRIDGE.md) | `target_pose` → TF → grasp search → safety veto; mock E2E result and limitations |
| [SAFETY_LAYER.md](SAFETY_LAYER.md) | state watchdog, joint/rate/workspace checks, validation service, mock evidence |
| [MEASUREMENT_PROTOCOL.md](MEASUREMENT_PROTOCOL.md) | repeatability, backlash, visual-displacement and gripper-envelope schedules and analyzers |
| [CALIBRATION.md](CALIBRATION.md) | Chessboard camera/hand-eye procedure, tools, validated result and open-loop limitations |
| [CALIBRATION_POSES.md](CALIBRATION_POSES.md) | Offline selection of 24 diverse calibration poses, diversity/coplanarity checks and on-site command |
| [../hardware/camera_frames.md](../hardware/camera_frames.md) | Body/optical frame contract and `camera_optical_calibrated` |

## State

| Item | State | Note |
|---|---|---|
| 5-DoF grasp candidate generation | **Implemented, mock-validated** | `armpi_grasp_planner`; Cartesian family + joint-space reachability sampler |
| IK → FK → collision → plan screening | **Implemented, mock-validated** | full-pose IK, FK verification, state validity, plan-only Pilz PTP |
| Fiducial target localization | **Implemented, synthetic + real-background semi-synthetic validated** | 15/15 semi-synthetic poses, 1.69 mm / 0.53° max; 0 false detections on 8 real frames; LM pose refinement |
| Camera optical frame | **Implemented, validated** | `camera_optical_calibrated` = measured body→optical adapter (89.798°, 5.4/10.1/−27.4 mm) |
| Fiducial → grasp planner hand-off | **Implemented, mock E2E validated (2026-11-17)** | `grasp_target_bridge`; age/TF/marker-tilt gate; 3/3 plans approved by safety; plan-only |
| Safety supervisor | **Implemented, mock-validated (2026-11-17); EEPROM-bounded limits adopted (2026-11-18)** | state watchdog, joint/velocity/acceleration/start-state/duration/workspace checks; veto service; the real MoveIt plan/execute path now requires approval |
| Measurement protocols | **Implemented, dry-run/simulate validated (2026-11-17)** | 30-trial repeatability/backlash, visual-displacement and gripper-envelope schedules; no hardware data |
| Hand-eye calibration | **Validated (2026-11-17)** | 14 poses with measured bus-servo joints; 2.94 mm / 0.39° board scatter; held-out 3.29 mm / 0.43°; leave-one-out 3.41 mm / 0.44° std |
| Independent tag displacement validation | **Lateral scale validated (2026-11-18)** | flat 36h11 ID 3, 24.9 mm: A→B **100.67 mm vs 100.0 mm (0.67 %)**; B→C 104.87 mm vs 100.0 mm (4.87 %); angle 92.3°; the tag absolute z is biased 6–13 mm low, so it is not an absolute reference |
| Table-down chessboard calibration | **Relative PASS (2026-11-18)** | 15 table poses; 2.7 mm / 0.42° board scatter, 2.2 mm / 0.29° held-out, 3.1 mm / 0.45° LOO; board top −0.0705 m. The board is 11 mm thick; the depth does not see it (holes filled) and the same-surface RGB-D gap is −4.6 mm, so the 6.1 mm board-vs-table difference is not the board thickness |
| Same-frame RGB-D same-surface check | **Done (2026-11-18)** | matte AprilTag on a flat sheet: RGB tag vs depth paper **−4.25 / −4.65 mm** (0.37° / 0.76°); depth paper vs table +1.66 mm vs ~0.1 mm paper; RGB absolute z biased −3 to −5 mm; use the depth table plane for z |
| Depth table plane and absolute z | **Validated (2026-11-18)** | 147,626-inlier RANSAC+SVD plane, 0.7 mm RMS, normal 3.66°; table z −0.0787 m (depth) vs −0.0784 m (215 mm ruler), 1.3 mm apart |
| Supervised object-free approach | **Executed (2026-11-18)** | `camera_forward` → planner plan-only (3/3 safety approved) → exact trajectory → final TCP 1 mm from plan; camera_forward view volume is not a grasp volume |
| Calibration pose selection | **Implemented, offline-validated (2026-11-17)** | 24 poses from 14,176 evaluated poses / 3,093 feasible; diversity, rotation-excitation and coplanarity checks pass; not executed |
| Real visual grasping | **Not started** | no real grasp; calibration and mock bridge validated, hardware integration pending |
| Grasp execution and verification | **Not started** | mock runs are plan-only by design |

Evidence:

- [`evidence/armpi_grasp/20261116/`](../../evidence/armpi_grasp/20261116/)
- [`evidence/fiducial/20261116/`](../../evidence/fiducial/20261116/)
- [`evidence/calibration/20261117/`](../../evidence/calibration/20261117/)
- [`evidence/calibration/20261117/pose_selection/`](../../evidence/calibration/20261117/pose_selection/)
- [`evidence/phase2/real_background_20261117/`](../../evidence/phase2/real_background_20261117/)
- [`evidence/phase2/frames_20261117/`](../../evidence/phase2/frames_20261117/)
- [`evidence/phase2/onsite_20261118/tag_displacement_validation/`](../../evidence/phase2/onsite_20261118/tag_displacement_validation/)
- [`evidence/calibration/20261118/table_down/`](../../evidence/calibration/20261118/table_down/)
- [`evidence/phase2/onsite_20261118/approach_20261118/`](../../evidence/phase2/onsite_20261118/approach_20261118/)
- [`evidence/phase2/onsite_20261118/depth_plane_20261118.md`](../../evidence/phase2/onsite_20261118/depth_plane_20261118.md)
- [`evidence/phase2/vendor_calibration_20261117/`](../../evidence/phase2/vendor_calibration_20261117/)
- [`evidence/phase2/session3_20261117/`](../../evidence/phase2/session3_20261117/)

> The hand-eye transform is validated against the 5 mm / 2° board gate, a
> held-out final base and 2-fold cross-validation. The measured optical frame
> is wired into the localizer. The 24-pose capture is planned offline and has
> not moved the robot. The mock E2E proves the perception → grasp → safety
> software chain, not a real grasp: the image is synthetic and the plan is not
> executed. The safety limits now use the adopted EEPROM-bounded joint ranges;
> a table-based grasp is still pending.
