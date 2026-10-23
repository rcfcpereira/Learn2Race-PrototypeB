#!/bin/bash
# Pull-and-rebuild update for one Learn2Race robot (NFR2).
#
# Brings the robot to the pinned revision of this repository and rebuilds the
# workspace, so a fleet is updated by one push followed by a power cycle,
# without SD-card cloning or an SSH session per robot. Run at boot by the
# learn2race-update systemd service (tools/install_update_service.sh), or by
# hand with `~/robot.sh update`.
#
#   1. skip if the robot is running, or if the network is down
#   2. refuse if the checkout has uncommitted changes (commit calibration first)
#   3. fetch and move to the pinned revision (L2R_REF: a branch, tag or commit)
#   4. check robot.yaml against the firmware and ROS config; roll back on failure
#   5. rosdep install (only if sudo works without a password), colcon build;
#      roll back and rebuild the previous revision if the build fails
#   6. refresh ~/robot.sh and record the version in ~/.learn2race_version
#
# The robot is NOT started afterwards: a robot that moves on its own after a
# reboot is a hazard, and the competitions require a start signal. Start it as
# usual with ~/robot.sh. The ESP32 firmware is flashed from a PC (INSTALL.md):
# when platformio.ini or firmware/ changed, the log says so.
#
# Settings, all optional, in /etc/default/learn2race-update:
#   L2R_REF=main          branch, tag (e.g. v1.0) or commit to follow
#   L2R_REMOTE=origin
#   L2R_ROS_SETUP=/opt/ros/jazzy/setup.bash
# A revision that failed (check or build) is not retried at every boot; push a
# fix, or force a retry with L2R_FORCE=1 ~/robot.sh update.

# References (full entries: docs/references.md at the repository root):
#   [R3]  Learn2Race dissertation, requirements FR/NFR: NFR2 automated deployment
#   [R10] ROS 2 Jazzy documentation: rosdep, colcon
# More details: INSTALL.md, section "Automatic updates"

set -u
REPO=$(cd "$(dirname "$(readlink -f "$0")")/.." && pwd)
WS=$REPO/learn2race_ws
[ -r /etc/default/learn2race-update ] && . /etc/default/learn2race-update
REF=${L2R_REF:-main}
ROS_SETUP=${L2R_ROS_SETUP:-/opt/ros/jazzy/setup.bash}
REMOTE=${L2R_REMOTE:-origin}
LOGDIR=$HOME/update_logs
mkdir -p "$LOGDIR"
LOG=$LOGDIR/$(date +%Y%m%d-%H%M%S).log
ln -sfn "$LOG" "$LOGDIR/latest.log"
exec > >(tee -a "$LOG") 2>&1

say() { echo "[$(date +%T)] $*"; }
git_() { git -C "$REPO" "$@"; }

build() {   # the same build as `~/robot.sh build` on Prototype A: skip the micro-ROS agent
  bash -c "source '$ROS_SETUP' && cd '$WS' && \
           colcon build --packages-skip micro_ros_agent"
}

say "Learn2Race update: $REPO, following $REMOTE/$REF"

if pgrep -f "[r]os2 launch" >/dev/null; then
  say "The robot is running: no update. Stop it (~/robot.sh stop) and run ~/robot.sh update."
  exit 0
fi

if ! timeout 20 git -C "$REPO" ls-remote --exit-code "$REMOTE" HEAD >/dev/null 2>&1; then
  say "Remote not reachable: keeping the current build ($(git_ rev-parse --short HEAD))."
  exit 0
fi

if [ -n "$(git_ status --porcelain --untracked-files=no)" ]; then
  say "Uncommitted changes in the checkout; not updating. Commit them (calibration"
  say "belongs in git) or discard them, then run ~/robot.sh update:"
  git_ status --short --untracked-files=no
  exit 1
fi

git_ fetch --quiet --tags "$REMOTE" || { say "git fetch failed"; exit 1; }
if git_ rev-parse -q --verify "refs/remotes/$REMOTE/$REF^{commit}" >/dev/null; then
  TARGET=$(git_ rev-parse "refs/remotes/$REMOTE/$REF^{commit}"); KIND=branch
elif git_ rev-parse -q --verify "$REF^{commit}" >/dev/null; then
  TARGET=$(git_ rev-parse "$REF^{commit}"); KIND=fixed
else
  say "Unknown revision '$REF' on $REMOTE"; exit 1
fi
OLD=$(git_ rev-parse HEAD)

if [ -z "${L2R_FORCE:-}" ] && grep -qs "^failed=$TARGET\$" "$HOME/.learn2race_version"; then
  say "Revision $(git_ rev-parse --short "$TARGET") failed to update before; staying on"
  say "$(git_ rev-parse --short "$OLD"). Push a fix, or retry with: L2R_FORCE=1 ~/robot.sh update"
  exit 1
fi

if [ "$OLD" = "$TARGET" ] && [ -f "$WS/install/setup.bash" ] && \
   grep -qs "^commit=$OLD\$" "$HOME/.learn2race_version" && \
   grep -qs "^build=ok\$" "$HOME/.learn2race_version"; then
  say "Up to date at $(git_ rev-parse --short HEAD) ($REF); nothing to do."
  exit 0
fi

if [ "$OLD" = "$TARGET" ]; then
  say "At $(git_ rev-parse --short "$TARGET") but no successful build recorded: building."
else
  say "Updating $(git_ rev-parse --short "$OLD") -> $(git_ rev-parse --short "$TARGET")"
fi
if [ "$KIND" = branch ]; then
  git_ checkout --quiet "$REF" 2>/dev/null || git_ checkout --quiet -b "$REF" "$REMOTE/$REF"
  git_ merge --quiet --ff-only "$REMOTE/$REF" || {
    say "Local commits on $REF that are not on $REMOTE: not updating."; exit 1; }
else
  git_ checkout --quiet --detach "$TARGET"
fi

rollback() {
  say "Rolling back to $(git_ rev-parse --short "$OLD")"
  git_ checkout --quiet --detach "$OLD"
  build && say "Previous revision rebuilt." || say "Rebuild of the previous revision failed too."
  printf 'commit=%s\nref=%s\nbuild=%s\nfailed=%s\ndate=%s\n' "$OLD" "$REF" "rolled-back" \
    "$TARGET" "$(date -Iseconds)" > "$HOME/.learn2race_version"
  exit 1
}

if ! python3 "$REPO/tools/check_robot_config.py" --root "$REPO"; then
  say "robot.yaml disagrees with the firmware or ROS configuration in this revision."
  rollback
fi

if git_ diff --quiet "$OLD" "$TARGET" -- firmware; then :; else
  say "NOTE: firmware/ changed in this update. Flash the ESP32 from a PC (INSTALL.md, section 7)."
fi

if sudo -n true 2>/dev/null; then
  bash -c "source '$ROS_SETUP' && rosdep install --from-paths '$WS/src' --ignore-src -r -y" \
    || say "rosdep install reported problems (see above); building anyway."
else
  bash -c "source '$ROS_SETUP' && rosdep check --from-paths '$WS/src' --ignore-src" \
    || say "Missing system dependencies (above) and no passwordless sudo: install them by hand."
fi

say "Building the workspace (this takes a few minutes)"
if ! build; then
  say "Build failed at $(git_ rev-parse --short HEAD)."
  rollback
fi

if [ -f "$REPO/tools/robot.sh" ] && ! cmp -s "$REPO/tools/robot.sh" "$HOME/robot.sh"; then
  cp "$REPO/tools/robot.sh" "$HOME/robot.sh" && chmod +x "$HOME/robot.sh" && say "~/robot.sh refreshed."
fi

printf 'commit=%s\nref=%s\nbuild=%s\ndate=%s\n' "$(git_ rev-parse HEAD)" "$REF" ok "$(date -Iseconds)" \
  > "$HOME/.learn2race_version"
say "Updated to $(git_ rev-parse --short HEAD) ($REF). Start the robot with ~/robot.sh as usual."
