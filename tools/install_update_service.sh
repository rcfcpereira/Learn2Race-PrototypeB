#!/bin/bash
# Install the learn2race-update service on this robot (NFR2). Run once, as the
# robot user, from the repository checkout:
#
#   ~/Learn2Race-Prototype?/tools/install_update_service.sh [REF]
#
# REF is the branch or tag every robot follows (default main). For a fixed
# release use a tag, e.g. v1.0. Remove with:
#   sudo systemctl disable --now learn2race-update && sudo rm /etc/systemd/system/learn2race-update.service

# References (full entries: docs/references.md at the repository root):
#   [R3] Learn2Race dissertation, requirements FR/NFR: NFR2 automated deployment
# More details: INSTALL.md, section "Automatic updates"

set -e
REPO=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
REF=${1:-main}
USER_NAME=$(id -un)
[ "$USER_NAME" = root ] && { echo "Run as the robot user (e.g. learn2race), not root."; exit 1; }

sed -e "s#@USER@#$USER_NAME#" -e "s#@REPO@#$REPO#" "$REPO/tools/systemd/learn2race-update.service" \
  | sudo tee /etc/systemd/system/learn2race-update.service >/dev/null
printf 'L2R_REF=%s\nL2R_REMOTE=origin\n' "$REF" | sudo tee /etc/default/learn2race-update >/dev/null
chmod +x "$REPO/tools/update_robot.sh" "$REPO/tools/check_robot_config.py"
sudo systemctl daemon-reload
sudo systemctl enable learn2race-update.service
echo "Installed: updates to '$REF' at every boot. Run one now with: ~/robot.sh update"
echo "Settings: /etc/default/learn2race-update    Logs: ~/update_logs/latest.log"
