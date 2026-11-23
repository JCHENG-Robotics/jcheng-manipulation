# Phase 2 — Calibration and Visual Grasping

> Phase 1 hardware baseline captured 2026-11-17; the real RGB/depth camera is
> available. Phase 2 hand-eye calibration is **validated** (2.94 mm / 0.39°
> board scatter, 3.29 mm / 0.43° held-out). The measured optical frame is
> wired into the localizer as `camera_optical_calibrated`. A 24-pose diverse
> set is selected offline for the next capture. The fiducial → grasp bridge and
> a mock visual-grasp E2E are now validated. **Five real markerless
> pick–lift–place attempts were run on 2026-11-19; three succeeded.** Passes
> 1–2 established the pipeline (pass 1: ≈149 mm lift; pass 2: 6.6 mm / 8°
> return), passes 3–4
> failed and measured a −8.0/−15.3/−5.8 mm low-configuration FK/hand–eye error,
> and pass 5 corrected it with one AprilTag observation (≈148 mm lift). A
> same-day low-configuration correction (`+13.7, +3.9, +8.9 mm`) was
> **grasp-validated** (release 1.3 mm off), **replayed without a new
> measurement** (grip check 0 floor clusters, identical registers), and used on
> a cube **0.20 m beyond reach** after driving the base in closed-loop
> (0.431 → 0.234 m, release 6.8 mm off). A later run started with the cube
> **outside the field of view**: a **+16° `joint1` yaw** found it at
> **0.469 m** (0.22 m beyond reach), the base was moved **once** (0.325 m in
> three closed-loop pulses), the cube was gripped and lifted 150 mm, the base
> was reversed home (final **9.4 mm** from a measured floor landmark) and the
> cube was released **1.1 mm** from its pre-grasp position. A pre-registered
> 24-pair trial then measured the constant correction on 2026-11-19:
> **baseline 12/24 vs intervention 24/24** (McNemar p = 0.00049), with a
> 2.19 mm paired placement improvement (p = 0.0063). See the state table
> and the evidence record. A flat
> AprilTag independently checked the scale: **100.67 mm vs
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
| [MOTION_WRITE_PATHS.md](MOTION_WRITE_PATHS.md) | every arm/chassis write path, its guard, limits and freeze rules |
| [MEASUREMENT_PROTOCOL.md](MEASUREMENT_PROTOCOL.md) | repeatability, backlash, visual-displacement and gripper-envelope schedules and analyzers |
| [FAST_MOTION.md](FAST_MOTION.md) | jerk-limited S-curve profiles, measured settles, gripper raw path and the measured speed envelope |
| [CALIBRATION.md](CALIBRATION.md) | Chessboard camera/hand-eye procedure, tools, validated result and open-loop limitations |
| [CALIBRATION_POSES.md](CALIBRATION_POSES.md) | Offline selection of 24 diverse calibration poses, diversity/coplanarity checks and on-site command |
| [../hardware/camera_frames.md](../hardware/camera_frames.md) | Body/optical frame contract and `camera_optical_calibrated` |

## State

| Item | State | Note |
|---|---|---|
| 5-DoF grasp candidate generation | **Implemented; mock-validated and used on hardware** | `armpi_grasp_planner`; Cartesian family + joint-space reachability sampler |
| IK → FK → collision → plan screening | **Implemented; mock-validated and used on hardware** | full-pose IK, FK verification, state validity, plan-only Pilz PTP |
| Fiducial target localization | **Implemented; synthetic + real-background validated; physical tag scale check** | 15/15 semi-synthetic poses, 1.69 mm / 0.53° max; 0 false detections on 8 real frames; physical tag scale 0.67 % lateral; LM pose refinement |
| Camera optical frame | **Implemented, validated** | `camera_optical_calibrated` = measured body→optical adapter (89.798°, 5.4/10.1/−27.4 mm) |
| Fiducial → grasp planner hand-off | **Implemented, mock E2E validated (2026-11-17)** | `grasp_target_bridge`; age/TF/marker-tilt gate; 3/3 plans approved by safety; plan-only |
| Safety supervisor | **Implemented, mock-validated (2026-11-17); EEPROM-bounded limits adopted (2026-11-18)** | state watchdog, joint/velocity/acceleration/start-state/duration/workspace checks; veto service; the real MoveIt plan/execute path now requires approval |
| Measurement protocols | **Implemented; repeatability, backlash and gripper envelope measured on hardware (2026-11-19/20)** | repeatability 30 visits **0.449 mm RMS / 1.017 mm max**; backlash **1.117 mm**; 46-grasp tolerance grid **[-6, +12] mm** (±9 mm opening axis, conservative 6 mm); visual-displacement and rotation-tolerance runners added 2026-11-20 but not yet run |
| Hand-eye calibration | **Validated (2026-11-17)** | 14 poses with measured bus-servo joints; 2.94 mm / 0.39° board scatter; held-out 3.29 mm / 0.43°; leave-one-out 3.41 mm / 0.44° std |
| Independent tag displacement validation | **Lateral scale validated (2026-11-18)** | flat 36h11 ID 3, 24.9 mm: A→B **100.67 mm vs 100.0 mm (0.67 %)**; B→C 104.87 mm vs 100.0 mm (4.87 %); angle 92.3°; the tag absolute z is biased 6–13 mm low, so it is not an absolute reference |
| Table-down chessboard calibration | **Relative PASS (2026-11-18)** | 15 table poses; 2.7 mm / 0.42° board scatter, 2.2 mm / 0.29° held-out, 3.1 mm / 0.45° LOO; board top −0.0705 m. The board is 11 mm thick; the depth does not see it (holes filled) and the same-surface RGB-D gap is −4.6 mm, so the 6.1 mm board-vs-table difference is not the board thickness |
| Same-frame RGB-D same-surface check | **Done (2026-11-18)** | matte AprilTag on a flat sheet: RGB tag vs depth paper **−4.25 / −4.65 mm** (0.37° / 0.76°); depth paper vs table +1.66 mm vs ~0.1 mm paper; RGB absolute z biased −3 to −5 mm; use the depth table plane for z |
| Depth table plane and absolute z | **Validated (2026-11-18)** | 147,626-inlier RANSAC+SVD plane, 0.7 mm RMS, normal 3.66°; table z −0.0787 m (depth) vs −0.0784 m (215 mm ruler), 1.3 mm apart |
| Supervised object-free approach | **Executed (2026-11-18)** | `camera_forward` → planner plan-only (3/3 safety approved) → exact trajectory → final TCP 1 mm from plan; camera_forward view volume is not a grasp volume |
| Low-configuration model selection | **Done (2026-11-19); joint offsets and `link4`-frame models rejected** | 4 models fitted to the 10-pose tag survey; all correction models fit to 1.4–2.3 mm RMS but predict the grasp correction 10.6–15.0 mm away from the constant four passing runs used; the constant is supported inside the envelope (tool-down, `j1 ±0.06 rad`, view `j4 −1.58…−1.66`). The fixed-tag yaw extension measured the correction at `j1 = 0.487 rad` (constant 5.6 mm away, fitted models 14–19 mm); the open test is a yawed grasp with the cube rotated to face the gripper, because the two attempts failed on the cube's ~50 mm diagonal vs the ~40 mm opening. See [`model_selection.md`](../../evidence/calibration/20261119/low_config_error/model_selection.md) |
| Calibration pose selection | **Implemented, offline-validated (2026-11-17)** | 24 poses from 14,176 evaluated poses / 3,093 feasible; diversity, rotation-excitation and coplanarity checks pass; not executed |
| Markerless RGB-D segmentation | **Implemented, real-cloud validated (2026-11-18)** | `tools/phase2/object_segment_rgbd.py`: floor RANSAC + 5 mm voxel clustering + top-slab fit; 39.2 × 39.8 mm cluster, 1.9/−4.1/−3.5 mm vs the RGB; `--min-top-extent` rejects a clipped top view (the first-grasp miss) |
| Real visual grasping | **First grasp (2026-11-18); five real attempts (2026-11-19), 3/5 passed; 24/24 in the paired trial** | 2026-11-18: markerless floor cube at the reach limit; the autonomous placement missed by a few mm, the operator-placed retry held and lifted the 39.4 mm cube, then jammed on release and needed a servo torque release. 2026-11-19 pass 1: one attempt, no operator — vision-measured 39.3 mm cube, grasp, **≈149 mm lift**, place back and release (7.5 mm residual). Pass 2 (operator feedback): start at the **adopted-low forward pose** with `joint5 = 0` (no wrist roll, no lens occlusion), **one-step** 150 mm lift (+149.9 mm TCP in a single 8.3 s trajectory), place back within **6.6 mm / 8°** verified in the same view frame. Pass 3: adopted home start/end, the object-centring gate was **measured to be incompatible** with the 0.176 m depth cutoff and the link4 camera mount (centred ⇒ ≤0.10 m ⇒ blind, and the FK/hand-eye is 15–50 mm off there); the reach planner's flat solution closed above the cube and the **deep family was executed but FAILED** (operator-confirmed: the cube was never held; the in-run "grip" evidence was a projection artefact). Pass 4 also failed and measured the cause: with `joint5 = 0` the TCP is a fixed image pixel (364.8, 344.2) and the cube sits 7–10 mm from it at the low configuration. Pass 5 **PASSED** after a one-point correction with a **60 mm AprilTag placed flat on the floor**: the tag's base position is (0.2373, 0.0202, **−0.0788**) at the validated view (on the floor) but (0.2293, 0.0049, −0.0846) at the grasp configuration, i.e. an error **e = (−8.0, −15.3, −5.8) mm**; grasping at `C_view + e` aligned the pads, the floor cube disappeared at the view-pose check, one-step lift reached TCP +0.0993 (object ≈148 mm), release, and return to adopted. A pre-registered 24-pair trial then measured the constant correction: **12/24 baseline vs 24/24 intervention** (McNemar p = 0.00049), paired placement improvement 2.19 mm (p = 0.0063). `joint5 = 0` at the view is required: 1.383 clips the top face to 35 mm and biases the centroid. See [`evidence/phase2/onsite_20261118/first_grasp/`](../../evidence/phase2/onsite_20261118/first_grasp/), [`evidence/phase2/onsite_20261119/grasp_150mm/`](../../evidence/phase2/onsite_20261119/grasp_150mm/) and [`.../grasp_150mm_attempt2/`](../../evidence/phase2/onsite_20261119/grasp_150mm_attempt2/), [`.../grasp_150mm_attempt5/`](../../evidence/phase2/onsite_20261119/grasp_150mm_attempt5/), [`evidence/phase3/onsite_20261119/`](../../evidence/phase3/onsite_20261119/README.md) |
| Grasp execution and verification | **Real execution validated (2026-11-18/19)** | measured bus-servo completion on every executed trajectory; lifts and releases verified in passes 1, 2 and 5; the view-pose floor check is the reliable grip test |
| Out-of-reach object (far cube) | **Validated (2026-11-19)** | the base was driven closed-loop with a direct `MotorsSpeedControl` publisher (no chassis node running): 0.431 m → 0.234 m in five pulses (net 0.197 m forward, 7.5 mm y drift; a 0.15 s pulse stays inside the dead band), re-segmenting from the park view between pulses; the validated measurement + correction pipeline then gripped, lifted ≈150 mm and released 6.8 mm off. The deep family reaches x ≈ 0.2447 at that z, and a cube at x 0.221 already clips the top face to 34.5 mm, so the base should land the cube at 0.226–0.234 m |
| Out-of-view search + base return | **Validated (2026-11-19)** | the cube was outside the frame at the tilted view; a **+16° `joint1` yaw at the raised view pose** found it at **0.469 m** (bearing +38.8°), 0.22 m beyond the 0.2447 m reach boundary; three closed-loop diagonal base pulses (0.325 m net, re-segmenting between pulses) landed it at (0.2284, −0.0010) m, inside the 0.226–0.234 m band; the validated deep family gripped it, the 150 mm lift held it (0 clusters), and the base was reversed in one 6.3 s pulse. A fixed floor landmark measured the return error (32.1 mm) and one 0.75 s correction pulse brought it to **9.4 mm**; the cube was released 1.1 mm from its pre-grasp position (robot frame) and at ≈(0.2115, 0.0278) m in the original base frame. The search executed only the positive yaw side; the return correction was applied after the release |
| Dual-camera supervised grasp | **PASS (2026-11-20)** | the chassis-mounted D435i measured the cube's top-face centre in `base_link` from its own depth plane and rectified colour at **2.58 mm** (x/y) from the wrist at the view pose, with 0.09–0.62 mm repeatability across an arm move. The validated target `C_view + e` was kept: the two cameras agree at the view pose while `e` is +13.7 mm in x, so `e` is a **low-config execution correction**, not a measurement bias, and a second camera cannot replace it. The front camera vetoed nothing (2.58 mm < 8 mm) and verified the grip (0 wrist clusters, no cube-sized front blob) and the put-back (front-camera planar displacement **3.65 mm**; wrist estimate 5.22 mm, the difference being the wrist's pose-dependent measurement). Its absolute height is 5.7–7.5 mm below the wrist's, consistent with the documented 116 vs 121 mm depth/board ambiguity. See [`evidence/phase2/onsite_20261120/dual_grasp/`](../../evidence/phase2/onsite_20261120/dual_grasp/README.md) |
| Fast motion | **PASS (2026-11-20, four real-cube runs at 4.0/20)** | jerk-limited joint-space S-curve with a measured bus-servo settle; the grasp+put-back cycle fell **11.67 s → 8.94 s (−23 %)**, the no-cube cycle from ~37 s to 5.43 s; defaults 4.0/20, release descent 1.4 rad/s; duty-20 base 100.2 mm/s; aligned auto-base landing 2.22 mm; later in-view/far runs grasped in 4.77/4.76 s, and `--return-home` reversed a 245.3 mm approach while holding the cube (11.06 mm landing vs the fitted target; return still open-loop). See [`docs/phase2/FAST_MOTION.md`](FAST_MOTION.md) and [`evidence/phase2/onsite_20261120/`](../../evidence/phase2/onsite_20261120/) |
| Object-form grasp catalog | **Six on-site forms attempted 2026-11-23; four grasped, lifted, verified and put back** | The measured gripper envelope (pads' bottom ≈ **28–30 mm above the table**, the contact plane ≈ **13 mm in front of the nominal TCP**, `j5 ±90°` rotates the jaw axis y → x with the pad height unchanged) and the per-form parameters: lying block `j4 −0.50` (stall 482), upright block `j4 −0.39` (482), standing Ø28.7 cylinder `j4 −0.365` (489), VLT carton `j4 −0.45` with `j2 −0.62 / j3 −1.77` (433). Rejected for a measured reason: the lying cylinder (widest ring 14.7 mm, below the reach) and the puffy sachet (squeezed out of the rigid pads). Also records the **j2/j3 stop regression** (raw 342→350–357, 930→898–917) that trips the gateway's 12-pulse verification into `MOTION_OUTCOME_UNKNOWN` and the backend-restart workaround. See [`docs/phase2/OBJECT_GRASP_CATALOG.md`](OBJECT_GRASP_CATALOG.md) and [`evidence/onsite_20261123/object_forms/`](../../evidence/onsite_20261123/object_forms/) |

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
- [`evidence/phase2/onsite_20261118/first_grasp/`](../../evidence/phase2/onsite_20261118/first_grasp/)
- [`evidence/phase2/onsite_20261118/depth_plane_20261118.md`](../../evidence/phase2/onsite_20261118/depth_plane_20261118.md)
- [`evidence/phase2/onsite_20261119/grasp_150mm/`](../../evidence/phase2/onsite_20261119/grasp_150mm/)
- [`evidence/phase2/onsite_20261119/grasp_150mm_attempt2/`](../../evidence/phase2/onsite_20261119/grasp_150mm_attempt2/)
- [`evidence/phase2/onsite_20261119/grasp_150mm_attempt3/`](../../evidence/phase2/onsite_20261119/grasp_150mm_attempt3/)
- [`evidence/phase2/onsite_20261119/grasp_150mm_attempt4/`](../../evidence/phase2/onsite_20261119/grasp_150mm_attempt4/)
- [`evidence/phase2/onsite_20261119/grasp_150mm_attempt5/`](../../evidence/phase2/onsite_20261119/grasp_150mm_attempt5/)
- [`evidence/phase2/onsite_20261119/low_config_grasp_validation/`](../../evidence/phase2/onsite_20261119/low_config_grasp_validation/)
- [`evidence/phase2/onsite_20261119/quick_replay_grasp/`](../../evidence/phase2/onsite_20261119/quick_replay_grasp/)
- [`evidence/phase2/onsite_20261119/far_grasp/`](../../evidence/phase2/onsite_20261119/far_grasp/)
- [`evidence/phase2/onsite_20261119/search_and_return/`](../../evidence/phase2/onsite_20261119/search_and_return/)
- [`evidence/phase2/onsite_20261119/lowest_reach/`](../../evidence/phase2/onsite_20261119/lowest_reach/)
- [`evidence/phase2/onsite_20261119/reach/`](../../evidence/phase2/onsite_20261119/reach/)
- [`evidence/phase2/onsite_20261119/board_capabilities/`](../../evidence/phase2/onsite_20261119/board_capabilities/)
- [`evidence/phase2/vendor_calibration_20261117/`](../../evidence/phase2/vendor_calibration_20261117/)
- [`evidence/phase2/session3_20261117/`](../../evidence/phase2/session3_20261117/)
- [`evidence/onsite_20261123/object_forms/`](../../evidence/onsite_20261123/object_forms/)

> The hand-eye transform is validated against the 5 mm / 2° board gate, a
> held-out final base and 2-fold cross-validation. The measured optical frame
> is wired into the localizer. The 24-pose capture is planned offline and has
> not moved the robot. The mock E2E proves the perception → grasp → safety
> software chain with a synthetic image and a plan-only trajectory; it is not
> the real grasp. The real floor-cube grasps are recorded in
> [`evidence/phase2/onsite_20261119/`](../../evidence/phase2/onsite_20261119/).
> The safety limits now use the adopted EEPROM-bounded joint ranges; a
> table-based grasp is still pending.
