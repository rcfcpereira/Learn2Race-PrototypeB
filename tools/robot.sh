#!/bin/bash

# References (full entries: docs/references.md at the repository root):
#   [R91] tmux: tmux sessions
#   [R10] ROS 2 Jazzy documentation: ros2 launch
# More details: learn2race_ws/README.md

# Prototype B run helper. Everything runs inside tmux on the robot.
#
# Why: launched straight from an SSH terminal, every node prints through that
# SSH connection. When the link stalls (phone hotspot), the pipe fills and any
# node that logs blocks in write() - the LiDAR driver was caught in pipe_write
# - its data goes stale, safety latches a fault and the robot "freezes".
# tmux keeps reading the output whatever SSH does.
#
#   ~/robot.sh base                 motors, LiDAR, IMU (drive from the PC)
#   ~/robot.sh slam [ekf:=true]     base + slam_toolbox mapping (ekf:=true: gyro heading via the EKF)
#   ~/robot.sh explore [SECONDS]    base with EKF + SLAM + autonomous explorer (default 180 s)
#   ~/robot.sh newmap               clear the SLAM map (restart slam_toolbox only)
#   ~/robot.sh save NAME            save the current map to ~/maps/NAME
#   ~/robot.sh reset                clear a latched safety fault
#   ~/robot.sh attach               watch the logs (leave: Ctrl+b then d)
#   ~/robot.sh stop                 stop everything, including orphans
#   ~/robot.sh status               what runs, safety state, load
#
# After an SSH drop, just reconnect: the run is still going.

SESSION=robot
ROS="source /opt/ros/jazzy/setup.bash; source ~/learn2race_ws/install/setup.bash"

running() { pgrep -f "[r]os2 launch" >/dev/null; }

stop_all() {
  tmux kill-session -t $SESSION 2>/dev/null
  pkill -INT -f "[r]os2 launch"; sleep 3
  pkill -KILL -f "[r]os2 launch"
  pkill -KILL -f "[l]earn2race_ws/install"
  pkill -KILL -x micro_ros_agent
  pkill -KILL -f "[s]lam_toolbox"
  pkill -f "[r]os2cli.daemon"
  sleep 1
  echo "stopped ($(pgrep -af '[l]earn2race_ws/install|[m]icro_ros_agent|[s]lam_toolbox' | wc -l) processes left)"
}

base() {
  if running; then echo "Already running: ~/robot.sh stop first"; exit 1; fi
  tmux new-session -d -s $SESSION -n base \
    "$ROS; ros2 launch learn2race_bringup prototype_b.launch.py teleop:=false $*; bash"
  echo "Base starting in tmux session '$SESSION' (~15 s for the sensors)."
}

command -v tmux >/dev/null || { echo "tmux missing: sudo apt install tmux"; exit 1; }

case "$1" in
  base) shift; base "$@" ;;
  slam)
    shift; base "$@"
    sleep 15    # sensors and the ESP32 link first, or SLAM starts without odometry
    tmux new-window -t $SESSION -n slam \
      "$ROS; ros2 launch learn2race_bringup slam_b.launch.py; bash"
    echo "SLAM started. Save before stopping: ~/robot.sh save NAME" ;;
  save)
    [ -n "$2" ] || { echo "usage: ~/robot.sh save NAME"; exit 1; }
    mkdir -p ~/maps
    # the default 2 s wait for /map failed under load on the Pi (15 Oct 2026)
    bash -c "$ROS; ros2 run nav2_map_server map_saver_cli -f ~/maps/$2 --ros-args -p save_map_timeout:=15.0" 2>&1 | grep -E "saved|Failed|ERROR" ;;
  explore)
    secs=${2:-180}
    running || { base ekf:=true; sleep 15; tmux new-window -t $SESSION -n slam "$ROS; ros2 launch learn2race_bringup slam_b.launch.py; bash"; sleep 12; }
    tmux kill-window -t $SESSION:explore 2>/dev/null
    tmux new-window -t $SESSION -n explore "$ROS; ros2 launch learn2race_bringup explore_b.launch.py max_run_s:=$secs; bash"
    echo "Exploring for $secs s (gamepad overrides). Save after: ~/robot.sh save NAME" ;;
  newmap)
    tmux kill-window -t $SESSION:slam 2>/dev/null; sleep 3
    pkill -KILL -f "[a]sync_slam_toolbox"; pkill -f "[s]can_fixed_node"; sleep 1
    tmux new-window -t $SESSION -n slam "$ROS; ros2 launch learn2race_bringup slam_b.launch.py; bash"
    echo "New map started." ;;
  reset)
    bash -c "$ROS; ros2 topic pub --times 3 /safety/reset std_msgs/msg/Bool '{data: true}'" >/dev/null 2>&1
    echo "reset sent" ;;
  attach) tmux attach -t $SESSION ;;
  stop) stop_all ;;
  status)
    running && echo "RUNNING: $(tmux list-windows -t $SESSION -F '#W' 2>/dev/null | tr '\n' ' ')" || echo "stopped"
    uptime
    running && bash -c "$ROS; timeout 5 ros2 topic echo --once /safety/fault_reason 2>/dev/null | head -1; timeout 5 ros2 topic echo --once /safety/fault 2>/dev/null | head -1" ;;
  *) sed -n 2,25p "$0" ;;
esac
