#!/usr/bin/env bash

# References (full entries: docs/references.md at the repository root):
#   [R21] Fast DDS / Cyclone DDS: middleware comparison
# More details: README.md of this package; rmw_report.py

# Middleware comparison for the Learn2Race platform.
#
# Runs the same micro-ROS benchmark under each installed RMW implementation on
# the same hardware, sampling CPU throughout, then generates a comparison table.
#
#   ./run_rmw_comparison.sh prototype_a
#
# Fair comparison requires that nothing else changes between runs: same battery
# state, same launch file, same duration, same room. The script enforces what it
# can and prompts for the rest.

set -euo pipefail

PROTOTYPE="${1:-prototype_a}"
DURATION="${2:-120}"
OUTDIR="runs/rmw_$(date +%Y%m%d_%H%M%S)"
CPU_CSV="$OUTDIR/cpu.csv"

RMWS=(
  rmw_fastrtps_cpp
  rmw_cyclonedds_cpp
  rmw_zenoh_cpp
)

mkdir -p "$OUTDIR"
echo "label,sample,cpu_percent,mem_percent" > "$CPU_CSV"

installed () {
  ros2 doctor --report 2>/dev/null | grep -q "$1" || \
    [ -d "/opt/ros/${ROS_DISTRO}/lib/${1}" ] || \
    ldconfig -p 2>/dev/null | grep -q "$1"
}

sample_cpu () {   # sample_cpu <label> <seconds>
  local label="$1" seconds="$2" i=0
  while [ "$i" -lt "$seconds" ]; do
    # Aggregate across every ROS process: the question is what the middleware
    # costs the robot, not what one node costs.
    read -r cpu mem < <(ps -eo %cpu,%mem,comm --no-headers \
      | awk '$3 ~ /ros|micro_ros|component_con/ {c+=$1; m+=$2} END {print c+0, m+0}')
    echo "$label,$i,$cpu,$mem" >> "$CPU_CSV"
    i=$((i + 1))
    sleep 1
  done
}

for RMW in "${RMWS[@]}"; do
  echo
  echo "=============================================================="
  echo " $RMW"
  echo "=============================================================="

  if ! installed "$RMW"; then
    echo "  not installed, skipping (apt install ros-${ROS_DISTRO}-${RMW//_/-})"
    continue
  fi

  export RMW_IMPLEMENTATION="$RMW"

  if [ "$RMW" = "rmw_zenoh_cpp" ]; then
    echo "  starting the Zenoh router"
    ros2 run rmw_zenoh_cpp rmw_zenohd > "$OUTDIR/zenohd_$RMW.log" 2>&1 &
    ZENOHD=$!
    sleep 3
  fi

  read -rp "  Battery charged and robot in position. Press Enter to start. "

  ros2 launch learn2race_bringup "${PROTOTYPE}.launch.py" \
    > "$OUTDIR/launch_$RMW.log" 2>&1 &
  LAUNCH=$!
  sleep 10                      # allow the graph and the agent handshake

  sample_cpu "$RMW" "$DURATION" &
  SAMPLER=$!

  ros2 run learn2race_evaluation microros_benchmark --ros-args \
    -p mode:=latency \
    -p duration:="$DURATION" \
    -p result_file:="$OUTDIR/$RMW.json" \
    -p label:="$RMW" \
    > "$OUTDIR/bench_$RMW.log" 2>&1 || true

  wait "$SAMPLER" 2>/dev/null || true
  kill "$LAUNCH" 2>/dev/null || true
  [ -n "${ZENOHD:-}" ] && kill "$ZENOHD" 2>/dev/null && unset ZENOHD
  sleep 5

  echo "  done"
done

echo
echo "Generating the comparison table."
ros2 run learn2race_evaluation rmw_report \
  --results "$OUTDIR"/*.json \
  --cpu "$CPU_CSV" \
  --out "$OUTDIR/results_rmw.tex"

echo
echo "Campaign complete: $OUTDIR"
echo "Include $OUTDIR/results_rmw.tex from Chapter 5."
