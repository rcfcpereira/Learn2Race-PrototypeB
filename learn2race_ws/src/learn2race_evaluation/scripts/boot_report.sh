#!/usr/bin/env bash

# References (full entries: docs/references.md at the repository root):
#   [R3] Learn2Race dissertation, requirements FR/NFR: NFR2 boot-to-ready
# More details: README.md of this package; startup_probe.py

# Boot and startup report for a Learn2Race robot.
#
# Separates the three contributions to the time between pressing the power
# switch and having a commandable robot:
#
#   1. kernel and firmware boot
#   2. systemd unit startup, including the provisioning service
#   3. ROS graph startup and the micro-ROS agent handshake
#
# Knowing which of the three dominates is what makes the number actionable: a
# slow kernel boot is fixed differently from a slow agent handshake.
#
#   ./boot_report.sh            # report on the current boot
#   ./boot_report.sh --reboot   # reboot, then report on the next boot

set -euo pipefail

OUT="boot_report_$(date +%Y%m%d_%H%M%S).txt"

if [[ "${1:-}" == "--reboot" ]]; then
  echo "Scheduling this report to run after the next boot, then rebooting."
  echo "Re-run without --reboot once the system is back up."
  sudo systemctl reboot
  exit 0
fi

{
  echo "=== Learn2Race boot report ==="
  echo "host:  $(hostname)"
  echo "date:  $(date -Iseconds)"
  echo "model: $(tr -d '\0' < /proc/device-tree/model 2>/dev/null || echo unknown)"
  echo

  echo "--- 1. Overall boot time ---"
  systemd-analyze 2>/dev/null || echo "systemd-analyze unavailable"
  echo

  echo "--- 2. Slowest units ---"
  systemd-analyze blame 2>/dev/null | head -15 || true
  echo

  echo "--- 3. Critical chain ---"
  systemd-analyze critical-chain 2>/dev/null | head -20 || true
  echo

  echo "--- 4. Learn2Race units ---"
  systemctl list-units 'learn2race*' --all --no-pager 2>/dev/null || true
  echo
  for unit in $(systemctl list-unit-files 'learn2race*' --no-legend 2>/dev/null | awk '{print $1}'); do
    echo "  $unit:"
    systemctl show "$unit" -p ActiveEnterTimestampMonotonic --value 2>/dev/null \
      | awk '{printf "    active after %.2f s\n", $1/1e6}'
  done
  echo

  echo "--- 5. Undervoltage and thermal events ---"
  # A Raspberry Pi that throttles during boot will also throttle during a run.
  if command -v vcgencmd >/dev/null 2>&1; then
    echo "  throttled flags: $(vcgencmd get_throttled)"
    echo "  core temp:       $(vcgencmd measure_temp)"
    echo "  (0x50000 or 0x50005 indicates undervoltage has occurred)"
  else
    echo "  vcgencmd unavailable"
  fi
  echo

  echo "--- 6. Serial devices present ---"
  ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null || echo "  none"
  echo

  echo "--- 7. ROS graph readiness ---"
  echo "  Run separately, immediately after a reboot:"
  echo "    ros2 run learn2race_evaluation startup_probe"
} | tee "$OUT"

echo
echo "Written to $OUT"
