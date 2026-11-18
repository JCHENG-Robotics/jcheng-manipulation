# Network access — ArmPi Ultra (Plan A, 2026-11-18)

> Status: **applied and verified on the live robot (2026-11-18 20:20 HKT)**.
> Scope: inbound access only. The robot keeps its office-WiFi internet path.

## Topology

```text
laptop                                    robot (Pi 5)
enx00e04ca373c7  10.42.0.1  ── cable ──  eth0  10.42.0.218   SSH, DDS, gateway :8765
wlp4s0           10.51.128.246  ~~~ WiFi ~~~  wlan0 10.51.128.226   outbound only
```

The direct Ethernet link is the **only** way in. Office WiFi is still used by
the robot for internet access (apt, time sync); its inbound traffic is dropped.

## Why

ROS 2/DDS has no authentication. Before the lockdown the robot had **no host
firewall** and these services were reachable from the whole office LAN:

| Port | Service | Problem |
|---|---|---|
| 22/tcp | sshd | `PasswordAuthentication yes`; the `pi` password is vendored |
| 5900/tcp | wayvnc | PAM auth with the same password |
| 9026/tcp | vendor `remote.py` (root) | no auth; rewrites `/etc/wifi/wifi_conf.py` |
| 8765/tcp | `armpi-gateway` | shared token; bound to wlan0 |
| 7400/udp+ | CycloneDDS | unauthenticated; any ROS 2 node on domain 0 could publish `/cmd_vel` |

## What changed

| Item | Before | After |
|---|---|---|
| Host firewall | none (docker NAT only) | `inet lockdown`, input policy `drop` |
| SSH password auth | yes | no (key only) |
| `wayvnc` | enabled, all interfaces | disabled |
| vendor `remote.service` (9026) | enabled, all interfaces | disabled |
| gateway bind | `10.51.128.226:8765` | `10.42.0.218:8765` |
| Tailscale | not installed (no binary, no `tailscale0`) | still none |
| DDS interfaces | robot `eth0`, laptop `enx00e04ca373c7` | unchanged |

## Firewall policy

Files:

| File | Purpose |
|---|---|
| `tools/net/armpi-lockdown.nft` | the ruleset; installed as `/etc/nftables.conf` |
| `tools/net/install-lockdown.sh` | installs it, enables boot load, arms rollback |

Rules (`inet`, input policy `drop`):

1. `lo` and `ct state established,related` accepted;
2. DHCP replies (`udp 67 -> 68`) accepted on both links;
3. **everything arriving on `eth0` is accepted** — eth0 must stay a direct
   cable to the laptop. If it is ever plugged into a shared switch, change the
   rule to `iifname "eth0" ip saddr 10.42.0.1 accept` (commented in the file);
4. everything else (all of `wlan0` inbound) is dropped and counted by the
   `inbound-denied` counter.

`nftables.service` is enabled. A drop-in
`/etc/systemd/system/nftables.service.d/execstop.conf` overrides the stock
`ExecStop=nft flush ruleset`, because that would also delete the docker
daemon's `table ip nat`. `systemctl stop nftables` now removes only
`table inet lockdown`.

## Apply, verify, roll back

```bash
# on the robot, from a directory holding the two files
sudo ROLLBACK_SECONDS=300 bash install-lockdown.sh ./armpi-lockdown.nft

# the installer arms a timer BEFORE applying; if SSH dies, the table is
# deleted automatically when it fires
systemctl list-timers nft-lockdown-rollback.timer

# after verifying, keep the rules
sudo systemctl stop nft-lockdown-rollback.timer

# immediate manual rollback
sudo nft delete table inet lockdown
```

Verification (laptop unless noted):

```bash
# 1. wired path still works: SSH, gateway, DDS
ssh -i ~/.ssh/id_ed25519_armpi pi@10.42.0.218 true
curl -s -H "X-Armpi-Token: $TOKEN" http://10.42.0.218:8765/health
# inside the Humble container:
ros2 topic list | grep -E 'joint_states|servo_states|battery|cmd_vel'
ros2 topic hz /controller_manager/joint_states      # ~48 Hz

# 2. WiFi inbound is blocked (route to 10.51.128.226 goes via wlp4s0)
timeout 5 bash -c 'cat < /dev/null > /dev/tcp/10.51.128.226/22'   # must fail
timeout 5 bash -c 'cat < /dev/null > /dev/tcp/10.51.128.226/8765' # must fail

# 3. on the robot: the counter should grow while probing wlan0
sudo nft list table inet lockdown
```

Expected: `nftables=active enabled=enabled`; gateway `200`; the wired DDS graph
shows the robot topics; both WiFi probes fail.

## DDS rules (do not regress these)

1. **Robot** `/home/ubuntu/cyclonedds.xml` (inside `ArmPiUltra`) lists **only
   `eth0`**. Keep the `.wifi.bak` untouched; restoring it makes the robot bind
   an interface the firewall now drops.
2. **Laptop** `tools/humble_moveit/cyclonedds-laptop.xml` lists **only
   `enx00e04ca373c7`** (10.42.0.1). `run_humble.sh` mounts this file at
   `/config/cyclonedds-laptop.xml`; edit the host file **in place** or restart
   the container, because a single-file bind mount keeps the old inode.
3. **Only one interface per XML.** A second interface without an IPv4 address
   makes participant creation fail (`does not match an available interface` /
   `rcl node's rmw handle is invalid`).
4. Link parameters from `tools/humble_moveit/README.md` still apply: static
   `10.42.0.1/24`, `ipv4.method manual` (never `shared`), `ethtool.eee-enabled
   off`, no gateway/DNS on the laptop profile.
5. `ROS_DOMAIN_ID=0`, `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`,
   `ROS2CLI_NO_DAEMON=1` on both sides.
6. If the robot is powered on with the cable unplugged, NetworkManager keeps
   the static address but there is no peer; DDS starts when the cable is
   connected. Do not boot with the cable plugged into a switch and expect the
   IP allowlist to save you — that is what rule 3 above is for.

## Studio and gateway

`tools/studio/start_studio.sh` now defaults to `ROBOT_HOST=10.42.0.218` and
reads the token over SSH on that link. The persistent gateway is bound to
`10.42.0.218` in `/etc/armpi-gateway/gateway.env`. The supervised motion
instance (port 8766) must also be started with `--host 10.42.0.218`.

## If wireless access is ever needed again

Do **not** revert the whole firewall. Add one narrow rule and reload:

```nft
# /etc/nftables.conf, before the inbound-denied counter
iifname "wlan0" ip saddr 10.51.128.246 tcp dport 8765 accept
```

That exposes only the read-only, token-protected gateway. SSH, VNC, the vendor
9026 service and DDS stay unreachable over WiFi.

## If a tunnel is ever reintroduced

The tunnel's own interface is not covered by the eth0 rule. Trust it
explicitly (and document who can join it), for example:

```nft
iifname "tailscale0" accept
```

Never add `wlan0` to the trusted set to "make it work"; that reopens every
service.

## Recovery

- **SSH lost after applying:** the installer's timer deletes the table after
  `ROLLBACK_SECONDS` (default 180/300). Wait for it, then reconnect. If the
  rules were installed manually without a timer, use the robot's console
  (keyboard/monitor) and run `sudo nft delete table inet lockdown`.
- **Gateway unreachable:** check `systemctl status armpi-gateway`; a bind
  failure means `/etc/armpi-gateway/gateway.env` still names wlan0 or an
  address that does not exist. Expect `Cannot assign requested address`.
- **DDS sees nothing:** first `ping 10.42.0.218`, then check the interface
  names in both XML files and `wc -c /config/cyclonedds-laptop.xml` inside the
  container. A discovery-only graph is a link problem, not a ROS bug.
- **Phone app needs to provision WiFi:** re-enable `remote.service` only
  temporarily, and only with the wlan0 rule above narrowed to the phone/host
  that needs it; 9026 has no authentication.
