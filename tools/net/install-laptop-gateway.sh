#!/usr/bin/env bash
# Install the laptop-side robot gateway and keep it across reboots.
#
# Run on the laptop as root:
#   sudo bash install-laptop-gateway.sh
#
# Then, on the remote host, add the one static route it needs:
#   sudo ip route add 10.42.0.0/24 via 192.168.5.22
set -euo pipefail

SRC="$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")"

[[ $EUID -eq 0 ]] || { echo "run as root" >&2; exit 1; }

install -m 0755 "$SRC/laptop-robot-gateway.sh" /usr/local/sbin/laptop-robot-gateway
install -m 0644 "$SRC/laptop-gateway.service" /etc/systemd/system/laptop-gateway.service
install -m 0644 "$SRC/99-laptop-gateway.conf" /etc/sysctl.d/99-laptop-gateway.conf

sysctl -q --system
systemctl daemon-reload
systemctl enable --now laptop-gateway
/usr/local/sbin/laptop-robot-gateway status
