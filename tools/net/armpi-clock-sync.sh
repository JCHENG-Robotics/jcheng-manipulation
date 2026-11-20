#!/usr/bin/env bash
# Keep the ArmPi host clock ~2 s ahead of this laptop.
#
# The robot has no battery-backed RTC, no internet and no NTP, and MoveIt
# rejects a plan when /joint_states stamps are older than the request. This
# script pushes the laptop clock over SSH with the dedicated, forced-command
# key created by install-clock-sync.sh. It is idempotent and silent when the
# robot is unreachable or already inside the tolerance window.
#
# Run from armpi-clock-sync.timer (every 60 s) or by hand.
set -euo pipefail

HOST="${ARMPI_CLOCK_HOST:-pi@10.42.0.218}"
KEY="${ARMPI_CLOCK_KEY:-$HOME/.ssh/id_ed25519_armpi_clock}"
AHEAD="${ARMPI_CLOCK_AHEAD_S:-2}"
TOLERANCE="${ARMPI_CLOCK_TOLERANCE_S:-1}"

[[ -r "$KEY" ]] || exit 0

ssh_opts=(
    -o BatchMode=yes
    -o ConnectTimeout=3
    -o StrictHostKeyChecking=accept-new
    -i "$KEY"
)

pi_epoch=$(ssh "${ssh_opts[@]}" "$HOST" get 2>/dev/null) || exit 0
[[ "$pi_epoch" =~ ^[0-9]+$ ]] || exit 0

laptop_epoch=$(date +%s)
offset=$((pi_epoch - laptop_epoch))
low=$((AHEAD - TOLERANCE))
high=$((AHEAD + TOLERANCE))
if (( offset >= low && offset <= high )); then
    exit 0
fi

target=$((laptop_epoch + AHEAD))
ssh "${ssh_opts[@]}" "$HOST" "set $target" >/dev/null 2>&1 || exit 0
printf 'armpi clock: offset %+d s -> set to laptop+%d s\n' "$offset" "$AHEAD"
