# Third-Party Notices

This repository contains project code and vendor-derived model assets. The
project code is licensed under the root [`LICENSE`](LICENSE) (MIT). The
vendor-derived assets are **not** relicensed under MIT.

## Vendor-derived model assets

The following assets originate from the HiWonder ArmPi Ultra vendor workspace
and are included here for the robot model, TF tree and simulation:

- `ros2_ws/src/armpi_ultra_description/meshes/*.STL` and
  `ros2_ws/src/armpi_ultra_description/meshes/gripper/*`
- the vendor-derived geometry and joint layout reflected in
  `ros2_ws/src/armpi_ultra_description/urdf/armpi_ultra.xacro`

**Permission:** the project owner confirmed on 2026-11-21 that the vendor has
agreed to their use in this repository. This permission covers the assets
listed above; it does not relicense them under MIT. No vendor source code,
firmware, driver or SDK is copied into this repository.

## MoveIt configuration

`ros2_ws/src/armpi_ultra_moveit_config` declares `BSD-3-Clause` in its
`package.xml`. Its launch and configuration files were generated with the
MoveIt Setup Assistant and follow the upstream MoveIt 2 licensing. The
ArmPi-specific values in those files are project measurements.

## Runtime dependencies

The repository references, but does not vendor, third-party software
including ROS 2, MoveIt 2, Pilz Industrial Motion Planner, KDL, OpenCV,
librealsense, FastAPI, React and their transitive dependencies. Each remains
under its own license.

`tools/phase2/patches/librealsense-2.55.1-d400-hid-gate.patch` is a project
patch against librealsense 2.55.1. The patch file itself is project code; the
patched library remains under the librealsense license (Apache-2.0).

## Reporting

If you believe an asset is attributed incorrectly, open an issue with the
path and the applicable license so the notice can be corrected.
