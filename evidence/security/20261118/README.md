# Inbound lockdown (Plan A) — 2026-11-18

> Verdict: **applied and verified.** The robot now accepts inbound traffic only
> on `eth0` (direct cable, 10.42.0.218 ↔ 10.42.0.1). Office WiFi is
> outbound-only. SSH, DDS and Studio were re-verified over the cable; the same
> probes from the laptop's WiFi interface fail.

## Raw evidence

| File | Content |
|---|---|
| `before_snapshot.txt` | pre-change services, listeners, sshd, firewall, gateway bind |
| `after_robot.txt` | post-change listeners, units, sshd, nftables table, docker NAT intact, gateway env, robot DDS XML |
| `after_laptop.txt` | links/routes, blocked WiFi probes, wired gateway 200, Studio health/peripherals |
| `dds_wired.txt` | wired-only CycloneDDS discovery and `/controller_manager/joint_states` rate |

## Result summary

**Closed**

- `nftables` input policy `drop`; `wlan0` accepts only DHCP replies.
- SSH password authentication off (`passwordauthentication no`); key-only.
- `wayvnc` disabled (5900 gone).
- vendor `remote.service` disabled (9026 gone; it was an unauthenticated root
  service that rewrote `/etc/wifi/wifi_conf.py`).
- gateway moved from `10.51.128.226:8765` to `10.42.0.218:8765`.

**Still working over the cable**

- SSH to `10.42.0.218` (used for every check in this session).
- gateway `/health` → HTTP 200, `ros_ready=true`.
- CycloneDDS discovery with the wired-only config: 44 topics, including
  `/controller_manager/joint_states`, `/controller_manager/servo_states`,
  `/ros_robot_controller/battery` and `/cmd_vel`.
- `/controller_manager/joint_states` measured at **~48.0 Hz** (min 0.019 s,
  max 0.023 s), matching the ~47.7 Hz baseline.
- Studio, hardware read-only: `mode=hardware`, `source_label=LIVE`,
  `battery truth=measured`, `camera truth=measured`.

**Blocked from WiFi (same laptop, `wlp4s0` route)**

- TCP 22, 8765 and 9026 all fail; configured before the probes, so this is the
  firewall, not a missing listener.

## Notes and caveats

- **Battery is low: 6.720 V, 16 %, `status=low`.** Charge before any motion
  work. This session only started the read-only backend; nothing moved.
- **No reboot test.** `nftables.service` is enabled and `systemctl restart
  nftables` was verified, but a full reboot is deferred because the vendor
  startup check moves the arm. Apply the same caution used for the earlier
  persistent-gateway work.
- **Docker NAT survives.** The installed drop-in overrides the stock
  `ExecStop=nft flush ruleset`; `table ip nat` is still present after the
  restart (see `after_robot.txt`).
- **The running `armpi-humble` container is a stale start.** It has no
  `/config` mount, so its `CYCLONEDDS_URI` points to a missing file and
  CycloneDDS falls back to all interfaces. It still reaches the robot over
  `eth0`, but restart it with `tools/humble_moveit/run_humble.sh` to get the
  explicit wired config at `/config/cyclonedds-laptop.xml`.
- **A stale Studio backend was replaced.** The script killed pid 2391666
  (started against the old WiFi gateway URL) and left pid 2449959 running with
  `ARMPI_GATEWAY_URL=http://10.42.0.218:8765`.
- **Phone provisioning is off.** `remote.service` must be re-enabled manually
  (and firewalled) if the vendor phone app is needed again.
- **Tailscale was already absent** (no binary, no `tailscale0`), so nothing was
  disabled for it.

## Rollback

```bash
sudo nft delete table inet lockdown          # firewall only
sudo systemctl disable --now nftables        # also stop boot load
# gateway back to WiFi would also require editing gateway.env and accepting
# that wlan0 is still firewalled; do not do this casually.
```

Installer with a timed self-rollback: `tools/net/install-lockdown.sh`.
Procedure and DDS rules: `docs/hardware/network_access.md`.
