#!/usr/bin/env bash
# Reduce recurring Pi network dropouts: disable wlan0 power save, install mDNS,
# and report the undervoltage/throttle history that explains radio loss.
#
# Usage: ./scripts/harden_pi_network.sh [host]
set -euo pipefail

HOST="${1:-192.168.0.241}"
KEY="${HOUSE_BOT_SSH_KEY:-$HOME/.ssh/housebot_ed25519}"

ssh -i "$KEY" -o BatchMode=yes "james@${HOST}" 'bash -s' <<'REMOTE'
set -euo pipefail

echo "=== throttle history (0x0 is clean) ==="
# bit 0 under-voltage now; bit 16 under-voltage has occurred since boot
vcgencmd get_throttled 2>/dev/null || echo "vcgencmd unavailable"

echo "=== disabling wlan0 power save ==="
sudo iw wlan0 set power_save off 2>/dev/null || echo "iw failed; check interface name"
iw wlan0 get power_save 2>/dev/null || true

# Persist across reboots; NetworkManager re-enables it otherwise.
sudo tee /etc/NetworkManager/conf.d/wifi-powersave-off.conf >/dev/null <<'CONF'
[connection]
wifi.powersave = 2
CONF

echo "=== installing avahi for housebot.local ==="
if ! command -v avahi-daemon >/dev/null 2>&1; then
  sudo apt-get update -qq && sudo apt-get install -y -qq avahi-daemon
fi
sudo systemctl enable --now avahi-daemon

echo "=== result ==="
hostname
hostname -I
REMOTE

echo
echo "Done. Verify name resolution from this machine with:"
echo "  ping -c1 housebot.local"
