# Object-form grasp trials, on-site 2026-11-23

> Six object forms were attempted on the real arm in one session through the
> Studio pose path (backend 8000, pose gateway 8766). Four were grasped,
> lifted, verified at the view and put back; two were rejected for a measured
> reason. The reference page is
> [`docs/phase2/OBJECT_GRASP_CATALOG.md`](../../../docs/phase2/OBJECT_GRASP_CATALOG.md);
> this folder holds the camera evidence.

## Results

| # | Form | Size (mm) | Top above table | Result | Close register |
|---|---|---|---|---|---|
| 1 | Blue block, lying | 29.4 × 58.3 × 30 | 30 mm | grasped, lifted, put back | 482 |
| 2 | Blue block, upright | 29 × 30 × 61 | 61 mm | grasped, lifted, put back (1.5 mm) | 482 |
| 3 | Red cylinder, standing | Ø28.7 × 45 | 45 mm | grasped, lifted, put back | 489 |
| 4 | Red cylinder, lying | Ø28.6 × 45 | 29.5 mm | **rejected** — widest ring below the pads' reach; grazed at 521, slipped | 642 / 521 |
| 5 | Yellow "VLT" carton, standing | ~55 × ~35 × ~80 | ~80 mm | grasped, lifted, put back (no depth; RGB pose) | 433 → 441 |
| 6 | Puffy snack sachet | 118 × 55 | 49 mm | **rejected** — squeezed out of the rigid pads | 478 → 642 |

Pose parameters, the gripper envelope, the register table and the decision
rules live in the catalog page above.

## Images

| File | Shows |
|---|---|
| `block_lying_grasp.jpg` | wrist view at `j4 −0.50`, the block between the finger tips |
| `block_lying_lift.jpg` | `lift_150`: the block held, the table empty |
| `block_lying_putback.jpg` | the block back on the table, upright |
| `block_upright_grasp.jpg` | wrist view at `j4 −0.39`, the standing block |
| `block_upright_lift.jpg` | the standing block held at `lift_150` |
| `cylinder_standing_grasp.jpg` | the red cylinder's top at `j4 −0.365` |
| `cylinder_standing_lift.jpg` | the cylinder held, between the fingers |
| `cylinder_standing_top_zoom.jpg` | the cylinder's round top, 3× zoom |
| `cylinder_lying_grasp.jpg` | the deepest pose `j4 −0.74` over the lying cylinder |
| `cylinder_lying_slip.jpg` | after the lift: the gripper empty, the cylinder slipped |
| `carton_grasp.jpg` | the VLT carton at the grasp pose (no depth return) |
| `carton_lift.jpg` | the carton held at the view |
| `carton_putback.jpg` | the carton back on the table |
| `sachet_contact.jpg` | the snack bag at the grasp pose, contact at 478 |
| `sachet_slip.jpg` | after the slip: the bag back on the table |
| `sachet_side_view.jpg` | the aux side camera: the bag squeezed against the closed fingers |

`session_log.json` is the staged-run driver's log (lease events, previews,
terminal stages, bus registers).

## Method

Everything was executed through `POST /api/motion/preview` →
`/api/motion/execute` (pose scope, bus verification) with explicit joint
targets from the FK. The depth point cloud came from the pose gateway
(`/pointcloud`, OpenCV optical layout labelled `depth_camera_link`, mapped with
`T_link4_camera` from the 2026-11-18 table-down calibration and the
`(+13.7, +3.9, +8.9) mm` correction). The carton's pose came from an RGB
back-projection (its surface returns no depth in any orientation).

## Caveats carried in the catalog

- The pads' vertical reach (≈ 28–30 mm above the table) is the binding limit
  for flat objects; a riser of ≥ 15 mm or an upright orientation fixes it.
- The j2/j3 mechanical stops moved on 2026-11-23 (raw 342→350–357,
  930→898–917), which trips the gateway's 12-pulse verification and latches
  the backend; the workaround (`j2 −0.62`, `j3 −1.77`, backend restart) was
  used throughout the last two forms.
