# Pose/motion gateway systemd unit — 2026-11-23

Scope: close the P1 item "the 8766 pose/motion gateway is a manual process" by
installing a systemd unit. The unit is deliberately **not enabled**: motion
requires an explicit operator start and the validated `takeover_limits.json`.

## Change

- New `armpi-gateway-pose.service` + `armpi-gateway-pose-run.sh` +
  `pose.env.example` + `install-pose-gateway.sh`.
- The host wrapper runs as `pi:docker`; the in-container gateway runs as
  `ubuntu` and sources the vendor zsh environment (`~/.zshrc`,
  `/opt/ros/humble/setup.zsh`, `~/ros2_ws/install/setup.zsh`) so CycloneDDS
  and the bus service are available.
- Camera is disabled in this instance (`--no-aux-camera
  --camera-topic /armpi_pose_camera_off`); 8765 owns the cameras.
- The installer checks the shared token and `takeover_limits.json`, copies the
  package, installs `root:docker 0640` `pose.env`, and runs only
  `daemon-reload`. It does not enable or start the unit.

## Deployment

- Installed to `/etc/systemd/system/armpi-gateway-pose.service`.
- `is-enabled` → `disabled`.
- Started explicitly with `sudo systemctl start armpi-gateway-pose`.
- `User=pi`, `Group=docker`, `ActiveState=active`, `SubState=running`.
- `/etc/armpi-gateway/pose.env` is `root:docker 0640`.

## Result

- 8766 `/health`: `status=ok`, `ros_ready=true`, `motion_enabled=true`,
  `motion_state=idle`, `pointcloud_configured=true`, `safety_stop=false`.
- `/pointcloud/meta`: `available=true`, `count=217382`, `sample_age_ms=0`,
  frame `depth_camera_link`, layout `opencv_optical_xyz`.
- `/motion/status`: `mode=pose`, `max_velocity_rad_s=0.5`, `safety_stop=false`.
- 8765 `/health`: `status=ok`, `camera_ready=true`, `joints_ready=true`.
- Backend `/api/health`: `status=ok`, `mode=hardware`.
- An out-of-limit probe (`r_joint` pulse 2000) was rejected with
  `MOTION_OUT_OF_LIMIT` before publication. No arm or chassis command was
  sent.

The first restart logged a zsh `setup.bash` prefix error; the run script now
sources `setup.zsh`, and the restart log is clean.

## Residual

- The unit is intentionally not enabled at boot. An operator must start it
  after checking the workspace and the motion limits.
- The limits JSON remains a generated artifact; regenerate it from
  `configs/joint_mapping.yaml`/`configs/takeover.yaml` after any limit change.
- The gateway still has no e-stop/brake; the physical safety layer is
  unchanged.

## Files

- `runtime.txt` — systemd identity/enabled state, health, pointcloud and
  motion/status.
