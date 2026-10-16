# learn2race_safety

Drive-command arbitration and telemetry supervision. Nothing drives the motors except
through `cmd_vel_mux`.

## Nodes

| Executable | Module | Role |
|---|---|---|
| `cmd_vel_mux` | `cmd_vel_mux_node.py` | Selects one drive source and publishes `cmd_vel` at 25 Hz. Priority: competition stop, then teleoperation (only outside competition mode), then armed / enabled autonomy (`autonomy/cmd_vel`). Publishes zero during start-up, faults and stale supervision |
| `safety_monitor` | `safety_monitor_node.py` | Watches telemetry freshness (odometry 0.5 s, IMU 1 s, lidar scan 1 s when `require_scan`), after a 15 s start-up grace, and publishes the `safety/fault` and `safety/ready` heartbeats the mux requires; `safety/reset` clears a latched fault and restarts a 3 s grace (`reset_grace_seconds`) |

`safety_policy.py` holds the arbitration and watchdog rules without ROS, tested in
`test/test_safety_policy.py`.

The ESP32 firmware adds an independent layer: it stops the motors 500 ms after the last
command (FR10), and short-brakes on an explicit zero.

## References and more details

- [R3] Dissertation requirements: FR10 command timeout and safe stop, NFR1.
- [R10] ROS 2 Jazzy documentation.
- Full entries: [`docs/references.md`](../../../docs/references.md)
