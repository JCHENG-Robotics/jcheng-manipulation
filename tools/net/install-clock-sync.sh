#!/usr/bin/env bash
# Install the laptop -> robot clock sync.
#
# Run on the laptop (the machine that runs MoveIt), from any directory:
#
#   tools/net/install-clock-sync.sh
#
# The robot has no battery-backed RTC, no internet and no NTP. This creates a
# dedicated keypair with no passphrase, restricts it on the robot to a
# forced command that can only read/set the clock, installs that receiver, and
# enables a systemd user timer that pushes the laptop clock every 60 s. The
# robot is kept ~2 s ahead of the laptop because MoveIt rejects /joint_states
# stamps older than the request.
#
# The admin key (default ~/.ssh/id_ed25519_armpi) must be usable non-
# interactively, e.g. loaded in the ssh-agent.
set -euo pipefail

REPO="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
HOST="${ARMPI_HOST:-pi@10.42.0.218}"
ADMIN_KEY="${ARMPI_ADMIN_KEY:-$HOME/.ssh/id_ed25519_armpi}"
CLOCK_KEY="${ARMPI_CLOCK_KEY:-$HOME/.ssh/id_ed25519_armpi_clock}"

ssh_admin=(-o BatchMode=yes -o ConnectTimeout=5 -i "$ADMIN_KEY")
ssh_clock=(-o BatchMode=yes -o ConnectTimeout=5 -i "$CLOCK_KEY")

if ! ssh "${ssh_admin[@]}" "$HOST" true; then
    echo "ERROR: cannot reach $HOST with $ADMIN_KEY (is the key in ssh-agent?)" >&2
    exit 1
fi

if [[ ! -f "$CLOCK_KEY" ]]; then
    ssh-keygen -q -t ed25519 -N '' -C 'armpi-clock-sync' -f "$CLOCK_KEY"
fi
chmod 600 "$CLOCK_KEY"

# Receiver: a root-owned forced command on the robot.
scp "${ssh_admin[@]}" "$REPO/tools/net/armpi-clock-set" "$HOST:/tmp/armpi-clock-set"
ssh "${ssh_admin[@]}" "$HOST" \
    'sudo -n install -m 0755 -o root -g root /tmp/armpi-clock-set /usr/local/sbin/armpi-clock-set && rm -f /tmp/armpi-clock-set'

# Restricted authorized_keys entry, idempotent. Keep a dated backup.
line="command=\"/usr/local/sbin/armpi-clock-set\",no-port-forwarding,no-agent-forwarding,no-pty,no-X11-forwarding $(cat "$CLOCK_KEY.pub")"
work="$(mktemp)"
printf '%s\n' "$line" > "$work"
scp "${ssh_admin[@]}" "$work" "$HOST:/tmp/armpi-clock-authorized-line"
rm -f "$work"
ssh "${ssh_admin[@]}" "$HOST" \
    'umask 077; mkdir -p ~/.ssh; touch ~/.ssh/authorized_keys; cp -a ~/.ssh/authorized_keys ~/.ssh/authorized_keys.bak.$(date +%Y%m%d) 2>/dev/null || true; grep -qxF "$(cat /tmp/armpi-clock-authorized-line)" ~/.ssh/authorized_keys || cat /tmp/armpi-clock-authorized-line >> ~/.ssh/authorized_keys; rm -f /tmp/armpi-clock-authorized-line'

# User units (the ExecStart path is substituted with this checkout).
mkdir -p "$HOME/.config/systemd/user"
sed "s|@REPO@|$REPO|g" \
    "$REPO/tools/net/systemd-user/armpi-clock-sync.service" \
    > "$HOME/.config/systemd/user/armpi-clock-sync.service"
install -m 0644 "$REPO/tools/net/systemd-user/armpi-clock-sync.timer" \
    "$HOME/.config/systemd/user/armpi-clock-sync.timer"
systemctl --user daemon-reload
systemctl --user enable --now armpi-clock-sync.timer
systemctl --user start armpi-clock-sync.service || true

# The dedicated key must not open a shell; `get` must work.
if ssh "${ssh_clock[@]}" "$HOST" id >/dev/null 2>&1; then
    echo "ERROR: the clock key opened a shell; check authorized_keys" >&2
    exit 1
fi
ssh "${ssh_clock[@]}" "$HOST" get >/dev/null

echo "installed: $(systemctl --user is-enabled armpi-clock-sync.timer) (timer), key $CLOCK_KEY"
systemctl --user --no-pager status armpi-clock-sync.timer | head -4
