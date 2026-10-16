#!/usr/bin/env bash

# References (full entries: docs/references.md at the repository root):
#   [R14] micro-ROS / micro_ros_arduino: micro-ROS benchmark
# More details: README.md of this package; microros_benchmark.py

# Learn2Race benchmark campaign.
#
# Records a consistent set of bags for one prototype, then generates the
# Chapter 5 results table from them.
#
#   ./run_benchmark.sh proto_a 10
#
# Every recording is tagged with the workspace and firmware revisions, because a
# result that cannot be traced to the software that produced it cannot be
# reproduced or defended.

set -euo pipefail

PROTOTYPE="${1:-proto_a}"
REPETITIONS="${2:-10}"
CAMPAIGN="runs/${PROTOTYPE}_$(date +%Y%m%d_%H%M%S)"
TOPICS="/cmd_vel /odom /imu/data /competition/state /safety/fault /scan"

mkdir -p "$CAMPAIGN"

{
  echo "prototype:  $PROTOTYPE"
  echo "date:       $(date -Iseconds)"
  echo "workspace:  $(git -C "$(dirname "$0")/../../.." rev-parse --short HEAD 2>/dev/null || echo unknown)"
  echo "firmware:   ${FIRMWARE_REV:-set FIRMWARE_REV before running}"
  echo "ros_distro: ${ROS_DISTRO:-unknown}"
} > "$CAMPAIGN/metadata.txt"

cat "$CAMPAIGN/metadata.txt"
echo

record () {  # record <name> <seconds>
  echo "--- $1 (${2}s)"
  timeout "$2" ros2 bag record -o "$CAMPAIGN/$1" $TOPICS >/dev/null 2>&1 || true
}

echo ">>> Straight-line runs for odometry scale."
echo ">>> Mark a 1 m line. Drive the robot along it for each repetition."
for i in $(seq 1 "$REPETITIONS"); do
  read -rp "    Position on the start line, then press Enter for straight_$i "
  record "straight_$i" 15
  read -rp "    Measure the ACTUAL distance and note it, then press Enter "
done

echo
echo ">>> Rotation runs for heading drift."
for i in 1 2 3; do
  read -rp "    Press Enter for spin_$i (10 complete turns in place) "
  record "spin_$i" 40
done

echo
echo ">>> Free runs for velocity tracking, timing and stop behaviour."
for i in $(seq 1 3); do
  read -rp "    Press Enter for run_$i "
  record "run_$i" 60
done

echo
echo ">>> Generating the results table."
ros2 run learn2race_evaluation make_report \
  --campaign "$CAMPAIGN" \
  --out "$CAMPAIGN/results.tex" \
  --caption "Measured results for ${PROTOTYPE}." \
  --label "tab:results_${PROTOTYPE}"

echo
echo "Campaign complete: $CAMPAIGN"
echo "Include $CAMPAIGN/results.tex from Chapter 5."
