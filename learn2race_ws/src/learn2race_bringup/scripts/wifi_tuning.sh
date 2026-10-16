#!/usr/bin/env bash

# References (full entries: docs/references.md at the repository root):
#   [R65] Raspberry Pi 4/5 documentation: Raspberry Pi Wi-Fi
# More details: README.md of this package

# WiFi tuning for a Learn2Race robot viewed from another computer.
#
# The usual cause of a stuttering preview is not bandwidth. A 320-pixel JPEG at
# 10 Hz is roughly 150 kB/s, which any wireless link carries easily. The cause is
# the Raspberry Pi WiFi power management, which lets the radio sleep between
# packets and adds latency spikes of hundreds of milliseconds. The picture then
# arrives in bursts and freezes in between.
#
#   sudo ./wifi_tuning.sh           # apply now and make it persistent
#   ./wifi_tuning.sh --status       # report only

set -uo pipefail
IFACE="${WIFI_IFACE:-wlan0}"

status () {
  echo "--- $IFACE ---"
  iw dev "$IFACE" get power_save 2>/dev/null || echo "  iw unavailable"
  iw dev "$IFACE" link 2>/dev/null | grep -E 'SSID|signal|tx bitrate' | sed 's/^/  /'
  echo
  echo "  Signal above -60 dBm is good; below -70 dBm the preview will stutter"
  echo "  whatever the settings. Move the robot or the access point."
}

if [[ "${1:-}" == "--status" ]]; then
  status
  exit 0
fi

if [[ $EUID -ne 0 ]]; then
  echo "Run with sudo to change settings, or pass --status to report only."
  exit 1
fi

echo "Disabling WiFi power saving on $IFACE."
iw dev "$IFACE" set power_save off

# Persist across reboots. NetworkManager value 2 means disabled.
if command -v nmcli >/dev/null 2>&1; then
  mkdir -p /etc/NetworkManager/conf.d
  cat > /etc/NetworkManager/conf.d/99-learn2race-wifi-powersave.conf <<'CONF'
[connection]
wifi.powersave = 2
CONF
  echo "Persisted through NetworkManager."
fi

echo
status
