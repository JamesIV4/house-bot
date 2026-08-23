#!/usr/bin/env bash
# Install the Wi-Fi repair watchdog on the House Bot Pi.
#
# The watchdog runs as root from a systemd timer and can rebuild the
# NetworkManager profile from scratch, so it needs the Wi-Fi credentials. They
# are prompted for here and written only to /etc/house-bot/wifi.env (mode 0600,
# root-owned) on the Pi. They are never stored in this repository, never passed
# as command-line arguments, and never echoed.
set -euo pipefail

if [[ $# -gt 1 ]]; then
    echo "Usage: $0 [pi-host-or-ip]" >&2
    exit 2
fi

repo_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
pi_host="${1:-192.168.0.241}"
ssh_key="${HOUSEBOT_SSH_KEY:-$HOME/.ssh/housebot_ed25519}"
remote_stage="/home/james/house-bot-deploy"

if [[ ! -f "$ssh_key" ]]; then
    echo "House Bot SSH key is missing: $ssh_key" >&2
    exit 1
fi

ssh_args=(-i "$ssh_key" -o BatchMode=yes)

# Default the SSID to whatever the Pi is presently associated with.
current_ssid="$(ssh "${ssh_args[@]}" "james@$pi_host" \
    "nmcli -t -f ACTIVE,SSID device wifi list 2>/dev/null | awk -F: '\$1==\"yes\"{print \$2; exit}'" || true)"

read -r -p "Wi-Fi SSID${current_ssid:+ [$current_ssid]}: " wifi_ssid
wifi_ssid="${wifi_ssid:-$current_ssid}"
if [[ -z "$wifi_ssid" ]]; then
    echo "An SSID is required." >&2
    exit 1
fi
read -r -s -p "Wi-Fi password for '$wifi_ssid': " wifi_password
echo
if [[ -z "$wifi_password" ]]; then
    echo "A password is required." >&2
    exit 1
fi

ssh "${ssh_args[@]}" "james@$pi_host" "mkdir -p '$remote_stage' && chmod 700 '$remote_stage'"
scp "${ssh_args[@]}" \
    "$repo_root/scripts/pi_wifi_watchdog.sh" \
    "$repo_root/deploy/house-bot-wifi-watchdog.service" \
    "$repo_root/deploy/house-bot-wifi-watchdog.timer" \
    "james@$pi_host:$remote_stage/"

# Stage credentials as a 0600 file rather than piping them, so that sudo keeps a
# TTY for its own password prompt. The staged copy is removed by the installer.
ssh "${ssh_args[@]}" "james@$pi_host" \
    "umask 077 && cat > '$remote_stage/wifi.env'" <<ENVFILE
WIFI_SSID="$wifi_ssid"
WIFI_PASSWORD="$wifi_password"
ENVFILE

# -t gives sudo a terminal; you will be prompted for the Pi's sudo password.
ssh -t "${ssh_args[@]}" "james@$pi_host" "sudo bash -s '$remote_stage'" <<'REMOTE'
set -euo pipefail
stage="$1"

install -m 0755 "$stage/pi_wifi_watchdog.sh" /usr/local/sbin/house-bot-wifi-watchdog
install -m 0644 "$stage/house-bot-wifi-watchdog.service" /etc/systemd/system/
install -m 0644 "$stage/house-bot-wifi-watchdog.timer" /etc/systemd/system/

install -d -m 0750 /etc/house-bot
install -m 0600 -o root -g root "$stage/wifi.env" /etc/house-bot/wifi.env
shred -u "$stage/wifi.env" 2>/dev/null || rm -f "$stage/wifi.env"

systemctl daemon-reload
systemctl enable --now house-bot-wifi-watchdog.timer

echo "--- installed ---"
ls -l /usr/local/sbin/house-bot-wifi-watchdog /etc/house-bot/wifi.env
systemctl is-active house-bot-wifi-watchdog.timer
REMOTE

echo
echo "Watchdog installed. Inspect its activity on the Pi with:"
echo "  journalctl -t house-bot-wifi -n 50"
