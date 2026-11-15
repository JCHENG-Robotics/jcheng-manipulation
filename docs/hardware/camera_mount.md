# Camera Mount and Optical Frame — [U] extrinsic

> Geometry source: vendor `armpi_ultra.xacro`.
> Frames added 2026-11-15; the camera extrinsic is **nominal, not measured**.

## Frames

```text
link4
└── camera_connet_link     (vendor mount; MASTER_PLAN calls this camera_mount)
    └── camera_link        (project camera body frame; identity)
        └── camera_optical_frame   (REP-103: z forward, x right, y down)
```

## Vendor static transform (source-level)

`camera_connet_joint`: parent `link4`, child `camera_connet_link`
`xyz = (-0.040352, -0.019122, 0.048005) m`
`rpy = (0, -1.5614, 0) rad`

## Project nominal transform

`camera_link_joint`: identity (`xyz=0`, `rpy=0`)
`camera_optical_joint`: `rpy = (-π/2, 0, -π/2)` (REP-103)

## Still unknown

| Item | Status |
|---|---|
| Camera model / serial / firmware | `[U]` |
| Intrinsics (fx, fy, cx, cy, distortion) | `[U]` |
| Real `link4 → camera` translation/rotation | `[U]` |
| Depth scale / registration | `[U]` |
| Mount rigidity / repeatability | `[U]` |
| Calibration procedure (vendor vs custom) | planned |

## Calibration note

The camera is mounted on `link4`, before `joint5`. Orientation diversity during
hand-eye calibration is physically limited; the calibration plan must maximize
arm elevation, viewing direction, distance and image-position diversity and
state this limitation explicitly.
