#!/usr/bin/env bash
# Publish the robot's direct USB link to the 192.168.5.0/24 LAN.
#
#   remote host 192.168.5.x --wlp4s0--> laptop 192.168.5.22
#                                     --enx00e04ca373c7--> robot 10.42.0.218
#
# The laptop masquerades the forwarded traffic, so the robot keeps seeing
# 10.42.0.1 and needs no route of its own: its inbound lockdown trusts every
# packet arriving on eth0 (see armpi-lockdown.nft). The remote host only needs
#
#   sudo ip route add 10.42.0.0/24 via 192.168.5.22
#
# Rules are inserted explicitly instead of adding an nftables base chain: the
# docker daemon already owns the `ip filter`/`ip nat` tables, and its FORWARD
# policy is DROP. An ACCEPT at the top of the iptables FORWARD chain composes
# with that; a separate nft base chain with `policy accept` would silently
# bypass docker's filtering.
#
# Usage: laptop-robot-gateway.sh up|down|status   (run as root)
set -euo pipefail

LAN_IF=wlp4s0              # office/remote LAN, 192.168.5.22
ROBOT_IF=enx00e04ca373c7   # USB Ethernet to the robot (NetworkManager: armpi-direct)
LAN_NET=192.168.5.0/24
ROBOT_NET=10.42.0.0/24

fwd_in=(-i "$LAN_IF" -o "$ROBOT_IF" -s "$LAN_NET" -d "$ROBOT_NET" -j ACCEPT)
fwd_out=(-i "$ROBOT_IF" -o "$LAN_IF" -s "$ROBOT_NET" -d "$LAN_NET" -j ACCEPT)
nat_out=(-o "$ROBOT_IF" -s "$LAN_NET" -d "$ROBOT_NET" -j MASQUERADE)

up() {
    sysctl -q -w net.ipv4.ip_forward=1

    iptables -C FORWARD "${fwd_in[@]}" 2>/dev/null ||
        iptables -I FORWARD 1 "${fwd_in[@]}"
    iptables -C FORWARD "${fwd_out[@]}" 2>/dev/null ||
        iptables -I FORWARD 1 "${fwd_out[@]}"
    iptables -t nat -C POSTROUTING "${nat_out[@]}" 2>/dev/null ||
        iptables -t nat -A POSTROUTING "${nat_out[@]}"

    echo "gateway up: $LAN_NET -> $ROBOT_NET via $ROBOT_IF (MASQUERADE)"
}

down() {
    iptables -C FORWARD "${fwd_in[@]}" 2>/dev/null &&
        iptables -D FORWARD "${fwd_in[@]}" || true
    iptables -C FORWARD "${fwd_out[@]}" 2>/dev/null &&
        iptables -D FORWARD "${fwd_out[@]}" || true
    iptables -t nat -C POSTROUTING "${nat_out[@]}" 2>/dev/null &&
        iptables -t nat -D POSTROUTING "${nat_out[@]}" || true

    echo "gateway down"
}

status() {
    # `iptables -C` fails with EPERM for non-root, which would look like a
    # missing rule; refuse early instead of reporting a false negative.
    [[ $EUID -eq 0 ]] || { echo "status needs root: sudo $0 status" >&2; return 2; }
    local rc=0
    iptables -C FORWARD "${fwd_in[@]}" 2>/dev/null || { echo "missing: FORWARD $LAN_IF->$ROBOT_IF"; rc=1; }
    iptables -C FORWARD "${fwd_out[@]}" 2>/dev/null || { echo "missing: FORWARD $ROBOT_IF->$LAN_IF"; rc=1; }
    iptables -t nat -C POSTROUTING "${nat_out[@]}" 2>/dev/null || { echo "missing: MASQUERADE $LAN_NET->$ROBOT_NET"; rc=1; }
    [[ $(sysctl -n net.ipv4.ip_forward) == 1 ]] || { echo "missing: net.ipv4.ip_forward=1"; rc=1; }
    [[ $rc -eq 0 ]] && echo "gateway ok"
    return $rc
}

case "${1:-status}" in
    up) up ;;
    down) down ;;
    status) status ;;
    *) echo "usage: $0 up|down|status" >&2; exit 2 ;;
esac
