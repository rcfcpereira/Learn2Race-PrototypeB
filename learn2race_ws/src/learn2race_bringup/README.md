# learn2race_bringup (Prototype B)

Launch files and parameter sets for Prototype B (Raspberry Pi 5, ESP32 on the DDSM
Driver HAT over the Pi's header UART, LD lidar, stereo cameras). Normally started
through `tools/robot.sh` (`base`, `slam`, `explore`, `newmap`, `save NAME`,
`reset`, `attach`, `stop`, `status`).

## Main launch file: `launch/prototype_b.launch.py`

| Argument | Default | Meaning |
|---|---|---|
| `competition` | `false` | Competition mode for the drive mux |
| `teleop` | `true` | Gamepad driving |
| `serial_port` / `agent_baud` | `/dev/ttyAMA0` / `921600` | ESP32 link on the Pi header UART |
| `lidar_port` | `/dev/lidar` | Lidar serial port |
| `battery` | `false` | Start the battery monitor |
| `wheel_odom_tf` | `true` | Publish odom → base_link from wheel odometry |
| `ekf` | `false` | Wheel + gyro EKF (`ekf_b_imu.yaml`) publishes odom → base_link instead |

`launch/hardware_check.launch.py` runs the first-line hardware check (`~/robot.sh check`, see USAGE.md).

Mapping: `slam_b.launch.py` (SLAM Toolbox fed by `scan_fixed_node`, which
resamples every lidar scan to 453 beams), `explore_b.launch.py` (autonomous
exploration, `room_mapping_explorer_node`, settings in
`config/room_mapping_explorer_b.yaml`) and `mapping_room.launch.py` (SLAM Toolbox with
`config/slam_toolbox_mapping.yaml`), `rtabmap_stereo_b.launch.py` (RTAB-Map with the
stereo pair), `stereo_cameras` / `stereo_capture` (camera pair). Sensor frames:
`config/sensor_frames_b.yaml`. Other launch files as in Prototype A's bringup.

## References and more details

- [R15] ROS 2 launch; [R14] micro-ROS agent; [R47] SLAM Toolbox; [R48] RTAB-Map;
  [R79] lidar driver; [R17] camera_ros; [R19] joystick teleoperation; [R13] frames;
  [R64] DDSM Driver HAT.
- Full entries: [`docs/references.md`](../../../docs/references.md)
