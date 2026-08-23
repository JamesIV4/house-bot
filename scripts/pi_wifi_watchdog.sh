#!/usr/bin/env bash
# Repair the House Bot Pi's Wi-Fi when it loses its address, its NetworkManager
# profile, or its radio. Runs on the Pi from a systemd timer; logs to journal.
#
# Credentials live in /etc/house-bot/wifi.env (root-only, never in git):
#   WIFI_SSID="..."
#   WIFI_PASSWORD="..."
set -uo pipefail

IFACE="${WIFI_IFACE:-wlan0}"
ENV_FILE=/etc/house-bot/wifi.env
STATE_DIR=/run/house-bot
FAIL_COUNT="${STATE_DIR}/wifi-fail-count"

log() { logger -t house-bot-wifi -- "$*"; }

has_address() { ip -4 addr show "$IFACE" 2>/dev/null | grep -q 'inet '; }

# Healthy: clear the failure counter and leave immediately.
if has_address; then
  [ -f "$FAIL_COUNT" ] && rm -f "$FAIL_COUNT"
  exit 0
fi

mkdir -p "$STATE_DIR" 2>/dev/null || true
fails=$(cat "$FAIL_COUNT" 2>/dev/null || echo 0)
fails=$((fails + 1))
echo "$fails" > "$FAIL_COUNT" 2>/dev/null || true
log "no IPv4 on $IFACE (attempt $fails); starting repair"

if [ ! -r "$ENV_FILE" ]; then
  log "ERROR: $ENV_FILE missing or unreadable; cannot rebuild profile"
  exit 1
fi
# shellcheck disable=SC1090
. "$ENV_FILE"
if [ -z "${WIFI_SSID:-}" ] || [ -z "${WIFI_PASSWORD:-}" ]; then
  log "ERROR: WIFI_SSID/WIFI_PASSWORD not set in $ENV_FILE"
  exit 1
fi

# 1. The service that owns the interface must be running.
if ! systemctl is-active --quiet NetworkManager; then
  log "NetworkManager inactive; starting it"
  systemctl start NetworkManager
  sleep 5
fi

# 2. Clear any rfkill block and make sure the radio is enabled.
rfkill unblock wifi 2>/dev/null
nmcli radio wifi on 2>/dev/null

# 3. Recreate the profile if it vanished. This is the failure mode that
#    autoconnect cannot recover from on its own.
if ! nmcli -t -f NAME connection show 2>/dev/null | grep -qxF "$WIFI_SSID"; then
  log "profile '$WIFI_SSID' missing; recreating it"
  nmcli connection add type wifi \
    con-name "$WIFI_SSID" ifname "$IFACE" ssid "$WIFI_SSID" \
    wifi-sec.key-mgmt wpa-psk wifi-sec.psk "$WIFI_PASSWORD" \
    connection.autoconnect yes >/dev/null 2>&1 \
    && log "profile recreated" \
    || log "ERROR: failed to recreate profile"
fi

# 4. Rescan and bring the connection up.
nmcli device wifi rescan >/dev/null 2>&1
sleep 3
nmcli connection up "$WIFI_SSID" >/dev/null 2>&1

sleep 5
if has_address; then
  log "recovered: $(ip -4 -brief addr show "$IFACE")"
  rm -f "$FAIL_COUNT"
  exit 0
fi

# 5. Escalate only after repeated failures: reload the Broadcom driver, which
#    re-initialises a radio that came up wedged. Bounded so we never loop on it.
if [ "$fails" -ge 3 ] && [ $((fails % 3)) -eq 0 ]; then
  log "still down after $fails attempts; reloading brcmfmac"
  systemctl stop NetworkManager
  pkill wpa_supplicant
  sleep 2
  modprobe -r brcmfmac_wcc 2>/dev/null
  modprobe -r brcmfmac 2>/dev/null
  sleep 2
  modprobe brcmfmac
  sleep 5
  systemctl start NetworkManager
  sleep 10
  if has_address; then
    log "recovered after driver reload"
    rm -f "$FAIL_COUNT"
    exit 0
  fi
fi

log "repair did not restore connectivity (attempt $fails)"
exit 1
