# Learn2Race DDSM micro-ROS PlatformIO Firmware

ESP32 PlatformIO firmware for the Waveshare DDSM Driver HAT (A), using DDSM210
motors and the `micro_ros_arduino` Jazzy precompiled library.

At run time the ESP32 talks to the Raspberry Pi over the Pi header UART
(`/dev/ttyAMA0`, 921 600 baud). The HAT's USB port is used only for flashing,
from a PC, with the Pi disconnected (see [../INSTALL.md](../INSTALL.md)).

## PlatformIO Setup

`platformio.ini` installs:

```ini
https://github.com/micro-ROS/micro_ros_arduino.git#v2.0.8-jazzy
```

Build and upload:

```bash
pio run
pio run --target upload
```

If `pio` is not on PATH on Windows, use:

```powershell
& "$env:USERPROFILE\.platformio\penv\Scripts\pio.exe" run
```

## Local Libraries

- `lib/RobotConfig`: build-time pins, IDs, geometry, timing, and baud settings.
- `lib/MicroRosSerialTransport`: custom micro-ROS serial transport at `921600` baud.
- `lib/DdsmMotor`: DDSM packet protocol, speed commands, CRC, and feedback parsing.
- `lib/DifferentialDrive`: wheel speed conversion and odometry integration.
- `lib/RobotApp`: ROS node, `/cmd_vel`, `/odom`, watchdog, and motor update loop.

## Minimal Teleop-Only Test Firmware

`src/teleop_wheels_test_main.cpp` is a parallel, stripped-down firmware for isolating
wheel control from the rest of the stack: it subscribes to `/cmd_vel` and
drives the DDSM wheels directly, with no odometry publishing, no agent clock
sync, no reconnect state machine, and no startup wheel test. It reuses
`DdsmMotor`, `DifferentialDrive`, and `RobotConfig` unchanged - `src/main.cpp`
and `RobotApp` are untouched by it. Useful for checking that teleop actually
reaches the wheels before debugging the full firmware's extra moving parts.

```bash
pio run -e teleop_wheels_test -t upload
```

Plain `pio run` / `pio run -t upload` (no `-e` flag) still build/upload the
normal `esp32dev` firmware by default.

Note: this test firmware never publishes `/odom`, so the Pi-side
`micro_ros_link_led_node` (which watches `/odom` freshness on the RGB stick)
will not show "connected" while it's running - check the agent's own log for
`session established`, or `ros2 node list` / `ros2 topic list`, instead.

## Serial Ports

- `Serial`: micro-ROS agent, `921600` baud. The HAT wires it both to its USB
  bridge and to the Pi header (GPIO14/15); the robot uses the header.
- `Serial1`: DDSM RS485 motor bus, GPIO RX `18`, TX `19`, `115200` baud.

The DDSM motor baud is set separately from the micro-ROS serial baud.

## ROS Interfaces

- Subscribes: `/cmd_vel` (`geometry_msgs/msg/Twist`)
- Publishes: `/odom` (`nav_msgs/msg/Odometry`)
- Publishes: `/ddsm/measured_twist` (`geometry_msgs/msg/Twist`)

## Robot Defaults

- Motor type: DDSM210
- Motor IDs: left front `1`, right front `2`, left rear `3`, right rear `4`
- Right-side sign is inverted by default for a mirrored differential-drive layout.
- Prototype B has two driven wheels (`-D MOTOR_COUNT=2` in `platformio.ini`).
- DDSM210 speed commands and feedback use a `10x` protocol scale internally;
  the firmware still exposes RPM, m/s, and rad/s to the rest of the code.

Calibrated values in `platformio.ini` (re-measure after changing wheels):

- `WHEEL_RADIUS_M` (0.03674 m effective)
- `TRACK_WIDTH_M` (0.1816 m effective)
- `LEFT_MOTOR_SIGN`
- `RIGHT_MOTOR_SIGN`
- `MAX_WHEEL_RPM`

## ROS 2 Jazzy Agent

Normally started by `prototype_b.launch.py`. By hand:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyAMA0 -b 921600 -v6
```

Drive test:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10
```

The motors are commanded to zero if `/cmd_vel` is not refreshed within `CMD_TIMEOUT_MS`.

## References and more details

- [R14] micro-ROS / micro_ros_arduino; [R64] Waveshare DDSM Driver HAT and DDSM210; [R63] ESP32, PlatformIO; [R42] Siegwart et al. (odometry); [R34] Zhang (the stereo checkerboard in `calibration/`).
- Full entries: [`docs/references.md`](../docs/references.md)
