# Reproducible ROS build — 2026-11-23

Scope: the `v1.0.0` blocker **reproducible ROS build**. This is the first
`colcon build` + `colcon test` run of the public packages on the target
architecture, plus the CI and pinning changes that make the run repeatable.

## Environment

The test ran inside the live `ArmPiUltra` container (Humble, aarch64, Python
3.10.12, colcon core 0.15.2). It used a new temporary workspace
`/tmp/ros_ci_ws` and a read-only copy of `ros2_ws/src` + `packages/jcheng_core`
in `/tmp/ros_ci_src`; it did not touch `/home/ubuntu/ros2_ws`, start or stop a
node, or move the arm.

## Packages

Selected packages:

`arm_interfaces`, `armpi_analysis`, `armpi_fiducial_localizer`,
`armpi_grasp_planner`, `armpi_safety`, `armpi_ultra_description`,
`armpi_ultra_moveit_config`.

`arm_description` is the archived 2-DOF learning profile and is not part of
the public production build; `arm_basics` and `arm_moveit_demo` already carry
`COLCON_IGNORE`.

## Command

```bash
source /opt/ros/humble/setup.bash
cd /tmp/ros_ci_ws
PKGS="arm_interfaces armpi_analysis armpi_fiducial_localizer \
  armpi_grasp_planner armpi_safety armpi_ultra_description \
  armpi_ultra_moveit_config"
colcon build --base-paths /tmp/ros_ci_src/ros2_ws/src \
  --packages-select $PKGS
source install/setup.bash
PYTHONPATH=/tmp/urdf_py_site:/tmp/ros_ci_src/packages/jcheng_core:$PYTHONPATH \
  colcon test --base-paths /tmp/ros_ci_src/ros2_ws/src \
  --packages-select $PKGS --ctest-args -E xmllint
colcon test-result --verbose
```

## Result

- **7 packages built.**
- **118 tests, 0 errors, 0 failures, 1 skipped.**
- Per-package pytest: analysis 36, fiducial 18, planner 34, safety 25,
  description 3 (1 skipped).
- The only local skip was `arm_interfaces` `xmllint`, because the vendor
  container has no route to `download.ros.org` for the package-format schema.
  The new CI job runs it with network access.

## Changes that came out of this run

- `armpi_analysis/paired_trials.py` had 10 blank lines before
  `_count_failures`; its own `test_flake8` caught `E303`. Fixed.
- `.github/workflows/ros-ci.yml` builds and tests the seven packages in a
  `ros:humble-ros-base` container with an explicit dependency list.
- `requirements/ci.txt` pins the pure-Python test tooling; the backend
  `pyproject.toml` now pins its runtime and dev dependencies and caps Python
  at `<3.13`.
- `docs/hardware/vendor_baseline_manifest.json` + 
  `tools/vendor_baseline_manifest.py` pin the private vendor source tree and
  its archive by SHA-256 without publishing vendor source.
- `tools/humble_moveit/Dockerfile` defaults to the same digest and still
  accepts `--build-arg ROS_BASE_IMAGE=...`.

## CI confirmation

The GitHub `ros-colcon` run for commit `95e4517`
([run 45098657645](https://github.com/JerichoCheng/jcheng-manipulation/actions/runs/45098657645))
completed successfully: `Initialize containers` pulled
`ros:humble-ros-base@sha256:dfc94c85d8e01f230951b2a85ca5576e08473081c3da5fbbe8f3ab844703b9ba`,
then built and tested all seven packages. The `studio` workflow passed on the
preceding commit (`a29ad90`). There is no remaining reproducibility gap in the
build path.

## Evidence files

- `colcon_test_result.txt` — final `colcon test-result --verbose` summary.
- `armpi_*/pytest.xml`, `arm_interfaces/test_results/**` — raw test reports.
- `environment.txt` — tool and package versions.
- `acceptance.json` — compact machine-readable record.
