# Real-Background Fiducial Check — 2026-11-17

Detector and pose-accuracy check on real Aurora 930 frames, plus a
semi-synthetic overlay test. No robot motion.

## Capture

Raw frames and `capture.json` are **private** in
`~/jcheng/phase2/real_capture_20261117/`; they are not committed.

```bash
# inside the robot container; no servo commands
python3 /tmp/capture_still.py --output-dir /tmp/real_capture_20261117 \
  --count 8 --interval 0.4 --depth --tf link4:rgb_camera_link
```

| Item | Value |
|---|---|
| Frames | 8 RGB (640×400, `bgr8`, `rgb_camera_link`) + 8 depth |
| RGB rate | ~8 Hz (capture interval 0.4 s) |
| Scene | Chessboard on cardboard, room background |
| TF | full-precision `link4 → rgb_camera_link` recorded in `capture.json` |

## Offline check

```bash
python3 tools/phase2/real_background_fiducial_check.py \
  --capture ~/jcheng/phase2/real_capture_20261117/capture.json \
  --output evidence/phase2/real_background_20261117/check.json \
  --contact-sheet evidence/phase2/real_background_20261117/contact_sheet.png
```

The tool uses the driver intrinsics from `capture.json`, the
`DICT_APRILTAG_36h11` detector and the validated `solvePnPRefineLM` path.

## Results

| Check | Result |
|---|---|
| Negative control (unmodified real frames) | 8 frames, **0 detections** |
| Semi-synthetic poses | **15 / 15 detected** |
| Max translation error | **1.69 mm** |
| Mean translation error | 1.12 mm |
| Max rotation error | **0.53°** |
| Mean rotation error | 0.30° |
| Gate | 5 mm / 2° — **PASS** |

The 15 cases are 3 real backgrounds × 5 poses at 0.23–0.30 m, with in-plane
rotations up to ±30° and tilts up to 12°. The marker is composited with a
white quiet zone; the corners are projected with the full lens distortion.

Without `solvePnPRefineLM`, the fronto-parallel case had a 1.82° rotation
error from the IPPE ambiguity; refinement reduces it to 0.10°.

## Files

- `check.json` — camera info, negative control, per-case ground truth,
  detections, errors and summary.
- `contact_sheet.png` — 15 annotated overlays (ground truth green, detection
  red).

## Limitations

- Semi-synthetic: the marker is not physically present.
- The marker interior uses a planar homography; distortion is exact at the
  corners but not inside the marker.
- This validates the detector/pose math and the real image pipeline, not the
  camera extrinsic or a real grasp.
- Raw frames stay private because they show the workspace/room.
