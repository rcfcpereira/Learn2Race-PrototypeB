# Using Prototype B

The robot is operated through `~/robot.sh`. It runs everything inside a `tmux` session, so a dropped SSH connection or a slow Wi-Fi link does not stall the nodes. Install the robot first ([INSTALL.md](INSTALL.md)).

## Run helper: `~/robot.sh`

| Command | What it does |
|---|---|
| `~/robot.sh base [ekf:=true]` | Motors, lidar, IMU, ToF sensors, safety and drive mux. With `ekf:=true`, heading comes from the wheel and gyro EKF |
| `~/robot.sh slam [ekf:=true]` | Base plus SLAM Toolbox mapping |
| `~/robot.sh explore [SECONDS]` | Base with the EKF, plus SLAM and the autonomous explorer (default 180 s) |
| `~/robot.sh newmap` | Clear the map (restarts SLAM only) |
| `~/robot.sh save NAME` | Save the current map to `~/maps/NAME.pgm/.yaml` |
| `~/robot.sh reset` | Clear a latched safety fault |
| `~/robot.sh attach` | Show the logs (leave with `Ctrl+b`, then `d`) |
| `~/robot.sh status` | What runs, the safety state and the load |
| `~/robot.sh stop` | Stop everything, including orphan processes |

## Driving with a gamepad

Prototype B is normally driven from a PC, which needs no ROS installation:

1. On the robot, run `~/robot.sh base` (or `slam`), then `python3 ~/Learn2Race-PrototypeB/tools/udp_teleop.py`.
2. On the PC, run `python gamepad_udp.py <robot address>` from Learn2Race-PC-Control.
3. Hold **LB** to drive (dead-man switch). Hold **RB** as well to go faster.
   - left stick: forward and back;
   - right stick: turn.
4. Release LB, or close the program, and the robot stops within 0.5 s (the mux timeout).

`udp_teleop.py` publishes `teleop/cmd_vel`. The mux gives priority to the explorer whenever it is armed, and the gamepad takes over as soon as the explorer stops.

A gamepad plugged into the robot also works: `ros2 launch learn2race_bringup xbox_drive.launch.py`.

## Mapping a room

```bash
~/robot.sh slam              # or: ~/robot.sh slam ekf:=true
# drive slowly round the room (turn slowly: scan matching needs overlap)
~/robot.sh save room_1
~/robot.sh stop
```

To watch the map live, use `rviz/rviz_b.sh` from Learn2Race-PC-Control.

Tips:
- Start each map from the same place and drive the same route. Maps made this way agree within about 2 cm.
- The map is accurate to about 5 cm, which is one to two cells.
- Save before stopping. The map only lives in the SLAM node.

## Autonomous exploration

```bash
~/robot.sh explore 180       # 3 minutes, then the explorer stops by itself
~/robot.sh save room_auto_1
```

The explorer (`room_mapping_explorer_node`, settings in `config/room_mapping_explorer_b.yaml`) works as follows:
- It sends `autonomy/cmd_vel` and arms the mux while it runs. Moving a stick on the gamepad overrides it.
- In `wall_follow` mode (the default), it keeps a wall on the right at 0.40 m, turns in arcs and spins on the spot only when cornered.
- The three VL6180X sensors guard against low objects the lidar cannot see. If the front sensor sees something within 10 cm, the robot sidesteps it using the gyro heading from the EKF:
  1. back off and turn 90°;
  2. move 0.40 m sideways;
  3. turn back to the original heading and drive 0.8 m past.
- The other mode is `bounce`: `ros2 launch learn2race_bringup explore_b.launch.py mode:=bounce`.

## Stereo cameras

```bash
ros2 launch learn2race_bringup stereo_cameras.launch.py            # the camera pair
ros2 launch learn2race_bringup calibrate_stereo_cameras.launch.py  # checkerboard calibration
ros2 launch learn2race_bringup rtabmap_stereo_b.launch.py          # visual SLAM (experimental)
```

The calibration target is in `firmware/calibration/`: 8 × 6 inner corners, printed squares measured before use.
- Recalibrate after any disturbance to the cameras or their ribbons: a change of a few tenths of a degree in relative pointing shifts depth by 5–20 %.
- After calibration, depth is within 5 % up to 2 m.

## Packages (`learn2race_ws/src`)

| Package | Role | Main entry points |
|---|---|---|
| `learn2race_bringup` | Launch files, parameters, sensor frames, explorer, scan resampler | `prototype_b.launch.py`, `slam_b.launch.py`, `explore_b.launch.py`, `stereo_*.launch.py`, `rtabmap_stereo_b.launch.py`; `config/prototype_b.yaml`, `config/sensor_frames_b.yaml`, `config/slam_toolbox_mapping.yaml` |
| `learn2race_hardware` | Pi-side drivers: LSM6DS3 IMU, VL6180X behind the PCA9548A, chainable LED, OLED, battery monitor | `lsm6ds3_imu_node`, `vl6180_tof_node`, `chainable_led_node` |
| `learn2race_safety` | Drive-command mux (competition > autonomy > teleoperation) and telemetry supervision | `cmd_vel_mux`, `safety_monitor` |
| `learn2race_odometry` | `odom` → `base_link` transform from wheel odometry | |
| `learn2race_localization` | EKF of wheel odometry and gyro (`robot_localization`) | `config/ekf_b_imu.yaml` |
| `learn2race_evaluation` | Calibration and measurement tools (odometry, IMU, sensors, micro-ROS link) | |
| `learn2race_imu_calibration` | Stationary IMU bias calibration | |
| `learn2race_control` | Optional body-velocity PID | |
| `learn2race_interfaces` | Custom action definitions | |

### Main topics

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | from `cmd_vel_mux` only, to the ESP32 |
| `/teleop/cmd_vel`, `/autonomy/cmd_vel` | `Twist` | sources into the mux |
| `/odom` | `nav_msgs/Odometry` | ESP32, 25 Hz |
| `/odometry/filtered` | `nav_msgs/Odometry` | EKF (with `ekf:=true`) |
| `/scan`, `/scan_fixed` | `sensor_msgs/LaserScan` | lidar; resampled to 453 beams for SLAM |
| `/map` | `nav_msgs/OccupancyGrid` | SLAM Toolbox |
| `/imu/data_raw`, `/imu/data` | `sensor_msgs/Imu` | IMU and Madgwick filter |
| `/tof/front`, `/tof/left`, `/tof/right` | `sensor_msgs/Range` | VL6180X |
| `/safety/fault`, `/safety/reset` | `std_msgs/Bool` | latched fault and its reset |

## Calibration

| What | How |
|---|---|
| Wheel radius | Straight runs against a tape; set `WHEEL_RADIUS_M` in `firmware/platformio.ini` (currently 36.74 mm effective) and `wheel_radius` in `config/prototype_b.yaml` |
| Track width | Rotation measured with the lidar, and the 1 m square test (UMBmark); currently 0.1816 m effective |
| IMU bias | `ros2 run learn2race_evaluation calibrate_imu` with the robot still |
| ToF sensors | Readings are within about ±10 mm at 3–15 cm and usable to about 19 cm; offsets (`range_offsets_m`) need a fixed jig to be meaningful |

## Working rules

- Do not run another robot's ROS stack on the same network at the same time: both robots use ROS domain 0.
- Avoid `ros2 topic echo` on large topics over Wi-Fi during a run. Use RViz on the PC instead.
