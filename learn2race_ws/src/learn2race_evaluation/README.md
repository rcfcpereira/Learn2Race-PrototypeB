# learn2race_evaluation

Tools that produce the measurements Chapter 5 requires, and emit them as LaTeX
rows so the numbers in the dissertation are the numbers the tools produced.

## Live test drivers

| Command | Verifies | What it does |
|---|---|---|
| `odom_calibration` | FR2 | Drives a measured straight line, then prints the corrected wheel radius and gear ratio |
| `stop_test` | FR10, NFR1 | Cuts the command stream and times the watchdog stop, repeatedly |
| `endurance_logger` | FR11 | Logs pack voltage to CSV across a full round, unattended |
| `microros_benchmark` | FR8, NFR9 | Transport latency, rate holding under load, drop rate |
| `startup_probe` | NFR2, NFR5 | Time from power-on to a commandable robot |

`stop_test` deliberately stops publishing rather than sending a zero twist. A
zero twist tests commanded braking; silence tests the firmware watchdog, which
is what the requirement constrains.

## Offline bag analysis

| Command | Verifies | Reports |
|---|---|---|
| `analyze_timing` | FR8, NFR9 | Achieved rate, jitter SD, p95 interval |
| `analyze_odometry` | FR2 | Scale factor, heading drift |
| `analyze_tracking` | FR1, FR10 | Steady-state velocity error, stop time |
| `make_report` | — | Runs all of the above over a campaign, writes `results.tex` |

Analysis reads bags rather than live topics, so a recording can be re-analysed
after the analysis changes — which is not true of a number written down once.

## A full campaign

    ./scripts/run_benchmark.sh proto_a 10

Prompts through straight-line, rotation and free runs, tags every bag with the
workspace and firmware revision, and writes `results.tex` for inclusion.

## micro-ROS performance

    ros2 run learn2race_evaluation microros_benchmark --ros-args -p mode:=latency
    ros2 run learn2race_evaluation microros_benchmark --ros-args -p mode:=load_ramp

Because both firmwares synchronise the session clock, `now - header.stamp` on a
telemetry message gives one-way transport delay directly. A latency that grows
across a run points at clock drift between re-synchronisations rather than at a
slow link.

The load ramp raises the command rate in steps and reports the telemetry rate
achieved at each. The step where telemetry starts to degrade is the usable
ceiling of the serial link — publishing commands faster past that point makes
the robot less responsive, not more.

## Boot and startup

    ./scripts/boot_report.sh
    ros2 run learn2race_evaluation startup_probe

`boot_report.sh` separates kernel boot, systemd units and the ROS graph, so a
slow startup can be attributed rather than merely observed. It also reports the
Raspberry Pi throttling flags: a board that browns out during boot will brown
out during a run, and `0x50000` in `get_throttled` is the evidence.

`startup_probe` reads `/proc/uptime` rather than its own start time, so the
figure it reports is power-on to commandable, including the micro-ROS agent
handshake, and it names the subsystem that came up last.

## Notes

* Nothing here has been run; no ROS 2 installation was available. Expect to fix
  message field names on first use.
* `analyze_odometry` in scale mode is the tool that exposes a wrong reduction
  ratio: a scale factor near 0.58 is 29/50, not a worn wheel.
* `endurance_logger` consumes `sensor_msgs/BatteryState`. Prototype A publishes
  it from the INA219 fitted on the driver board, so no extra hardware is needed
  there. Prototype B has no pack monitor: the DDSM210 reports motor current,
  which is not the same quantity, so an external INA226 on the battery lead is
  required before FR11 can be measured on that build.

## References and more details

- [R3] dissertation requirements (FR/NFR); [R16] rosbag2; [R42] Siegwart et al.; [R43] Borenstein & Feng, UMBmark; [R49] NXP AN4246; [R34] Zhang; [R35] Hartley & Zisserman; [R14] micro-ROS; [R21] DDS middleware.
- Full entries: [`docs/references.md`](../../../docs/references.md).
