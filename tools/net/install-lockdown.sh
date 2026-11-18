#!/usr/bin/env bash
# Install the ArmPi inbound lockdown on the robot and keep it across reboots.
#
# Run on the robot as root (or via sudo). It always arms a 3-minute rollback:
# if the rules cut your SSH session, `nft delete table inet lockdown` restores
# the previous open state automatically. Cancel the timer after verifying.
#
#   sudo bash install-lockdown.sh ./armpi-lockdown.nft
#   sudo systemctl stop nft-lockdown-rollback.timer   # keep the rules
#
# The drop-in overrides the stock nftables ExecStop (`nft flush ruleset`),
# which would also delete the docker daemon's `table ip nat`.
set -euo pipefail

RULES="${1:-$(dirname "${BASH_SOURCE[0]}")/armpi-lockdown.nft}"
ROLLBACK_SECONDS="${ROLLBACK_SECONDS:-180}"

[[ $EUID -eq 0 ]] || { echo "run as root" >&2; exit 1; }
[[ -r "$RULES" ]] || { echo "ruleset not found: $RULES" >&2; exit 1; }

install -m 0644 "$RULES" /etc/nftables.conf

install -d /etc/systemd/system/nftables.service.d
cat > /etc/systemd/system/nftables.service.d/execstop.conf <<'EOF'
# Keep `systemctl stop nftables` from flushing the docker daemon's tables.
[Service]
ExecStop=
ExecStop=/usr/sbin/nft delete table inet lockdown
EOF
systemctl daemon-reload
systemctl enable nftables >/dev/null

# Arm the rollback before applying, so a bad rule can never strand the robot.
# Clear a previous transient timer first: systemd-run refuses a unit name that
# already exists.
systemctl stop nft-lockdown-rollback.timer 2>/dev/null || true
systemd-run --on-active="$ROLLBACK_SECONDS" --unit=nft-lockdown-rollback \
    /usr/sbin/nft delete table inet lockdown >/dev/null

nft -f /etc/nftables.conf
echo "ruleset loaded; rollback armed for ${ROLLBACK_SECONDS}s"
echo "verify SSH, DDS, and Studio, then: systemctl stop nft-lockdown-rollback.timer"
