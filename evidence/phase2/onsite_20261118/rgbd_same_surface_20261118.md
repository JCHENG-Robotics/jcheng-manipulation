# Same-scene RGB-D / board-thickness check — 2026-11-18

Status: **done, no hardware motion.** This closes leftover issue 3 from the
2026-11-18 onsite session ("do a same-frame RGB-D + chessboard / known-thickness
comparison to remove the board-thickness ambiguity"). The arm stayed at
`table_down`; only the camera and static objects were used.

## Question

The table-down report measured the RGB chessboard top **6.1 mm above** the
depth table plane and interpreted it as the board thickness. A calipered
board is **11 mm** thick, so that interpretation could not be right. The
missing measurement was a comparison of **the same physical surface** in RGB
and depth, with the board thickness measured independently.

## Setups

1. **Chessboard on the table** (first snapshot, same RGB/cloud stamp
   `1794987576.183`).
2. **Matte AprilTag sheet on the chessboard** (tag 60 mm, ID 0, printed at
   100 % on matte paper; snapshot stamp `1794988465.395`).
3. **Matte AprilTag sheet directly on the table** (flat, no board support;
   snapshot stamp `1794988589.982`).

All snapshots are same-stamp RGB + registered cloud + `/joint_states`
(`tools/phase2/rgbd_snapshot.py`); the analyses use the table-view TSAI
hand-eye and the refined intrinsics.

The tag target is
[`armpi_apriltag_36h11_id0_60mm.pdf`](../../../armpi_ultra_ws_clean/docs/calibration/armpi_apriltag_36h11_id0_60mm.pdf)
(100 mm page, 60 mm tag, ID 0), printed at 100 % on matte paper. The chessboard
is 11×8 inner corners with 20 mm squares; its thickness is **11 mm** (caliper).

## Results

### 1. Chessboard: the depth does not see the board

| Metric | Value |
|---|---:|
| RGB board plane vs depth "board" plane | −1.95 mm |
| Depth "board" plane vs depth table plane | +0.01 mm |
| Depth "board" plane RMS | 0.79 mm |
| Raw depth, board area | filled / smooth; no 11 mm step |
| IR image | board pattern clearly visible |

The depth plane inside the board polygon is the **table plane**, not the board
top: the high-contrast, glossy chessboard returns no usable depth, and the
driver fills the hole. The RGB/depth "board" comparison is therefore a
comparison against the table, not the board surface.

### 2. Tag sheet on the chessboard (paper sag test)

| Metric | Value |
|---|---:|
| RGB tag plane vs depth paper plane | **−4.25 mm** (0.37°) |
| Depth paper plane vs depth table plane | +12.92 mm |
| RGB tag plane vs depth table plane | +8.67 mm |
| Caliper board thickness | 11.0 mm |
| Caliper (11 mm) − depth paper/table | −1.92 mm |

### 3. Tag sheet flat on the table (no board support)

| Metric | Value |
|---|---:|
| RGB tag plane vs depth paper plane | **−4.65 mm** (0.76°) |
| Depth paper plane vs depth table plane | +1.66 mm |
| RGB tag plane vs depth table plane | −3.00 mm |
| Assumed paper thickness | 0.1 mm |
| Caliper (0.1 mm) − depth paper/table | −1.56 mm |
| Depth paper plane RMS | 0.65 mm |
| Depth table plane RMS | 0.82 mm |
| Tag reprojection error | 0.24 px |

The same-surface gap is consistent on both supports (−4.25 mm on the board,
−4.65 mm flat), so it is **not paper sag**.

## Findings

1. **The board is 11 mm thick; 6.1 mm is not the board thickness.** The
   chessboard is invisible to the depth camera (depth holes filled with the
   table), so the old RGB board-top-vs-depth-table number compared the board
   top with the table, not with the board surface on the depth side.
2. **The same-surface RGB-D gap at the table view is ≈ −4.6 mm (0.4–0.8°).**
   It is stable across the two tag setups. This is the vendor RGB-D
   registration/depth measurement difference at this view, not the hand-eye:
   the hand-eye `T_base_cam` is common to both planes and cancels in the
   relative comparison. At ~0.31 m with a ~25 mm baseline and f ≈ 420 px,
   4.6 mm of depth is ≈ **0.5 px of disparity**, consistent with the vendor
   registration's half-pixel-level alignment rather than a hand-eye error.
3. **The RGB absolute-z bias at this view is ≈ −3 to −5 mm** (the tag plane is
   −3.0 mm vs the depth table; the old board top was ≈ −4.9 mm vs the physical
   board top). The depth paper sits ≈ +1.6 mm above the table even though the
   paper is ~0.1 mm thick.
4. **The earlier "RGB and depth agree within the board thickness" statement
   is superseded.** The 6.1 mm was the RGB board top minus the depth table
   with a ~−4.9 mm RGB bias; it is not the board thickness. The RGB and depth
   do **not** agree to 6.1 mm; their same-surface gap is ≈ 4.6 mm.
5. **The depth table plane remains the absolute-z reference.** It is
   RANSAC/SVD-stable (0.8 mm RMS) and agrees with the 215 mm ruler check to
   1.3 mm. For RGB grasp targets, use RGB **XY** and the depth/table **z**;
   do not use the RGB absolute z for the table height.
6. The tag's earlier 6–13 mm absolute-z deficit at the table view is
   consistent with this ~−3 mm RGB bias plus the tag-plane tilt, not a new
   hand-eye failure.

## Consequences for the project

- The table height `z ≈ −0.078 m` and the table-plane safety check stand.
- The current absolute RGB-D z accuracy at the table view is about
  **5 mm**, not the 1–2 mm implied by the old board-thickness argument. Any
  grasp plan that depends on RGB absolute z should either use the depth table
  plane or carry a ≥5 mm vertical safety margin.
- The hand-eye validation number (2.94 mm / 0.39°) is a hand-eye metric; it
  does not include the vendor RGB-D registration gap measured here.
- The chessboard must not be used as a depth target on this camera; use a
  uniform matte target (the AprilTag sheet) when a depth surface is needed.

## Reproduction

```bash
# In the Humble container, arm at table_down, target in view:
python3 /repo_tools/phase2/rgbd_snapshot.py \
  --output-dir /tmp/rgbd_tag_flat_20261118 --max-skew 0.20

# On the laptop (after docker cp of the snapshot and sourcing Jazzy):
python3 tools/phase2/rgbd_tag_check.py \
  --snapshot /tmp/rgbd_tag_flat_20261118 \
  --calibration \
  evidence/calibration/20261118/table_down/\
calibration_table_down_20261118.json \
  --output evidence/phase2/onsite_20261118/rgbd_same_surface_20261118/paper_flat_on_table.json \
  --board-thickness-mm 0.0 --paper-mm 0.1
```

The raw snapshots contain the private workspace and are **not committed**;
only the reports and cropped diagnostics are in this directory.

## Limitations

- One base pose/table view. The RGB-D gap is view-dependent; do not assume
  −4.6 mm at other poses.
- The paper thickness is assumed 0.1 mm; the caliper measurement is the
  user-provided **11 mm** board.
- The depth paper plane is a fit to the white margin around the 60 mm tag;
  its RMS is 0.65 mm, but the +1.6 mm offset vs the table includes a
  step/edge bias of the depth driver.
- No per-pixel RGB-D registration calibration was performed; the vendor's
  registration is treated as-is.
- No hardware motion, no grasp, no new hand-eye calibration.
