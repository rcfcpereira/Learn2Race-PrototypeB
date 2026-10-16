# Learn2Race Control Package

This package contains optional control helpers for tuning the robot.

## Twist PID Controller

The firmware already commands DDSM210 motors in speed mode. The PID node adds an
optional outer loop around measured robot body velocity:

```text
/cmd_vel_setpoint + /ddsm/measured_twist -> twist_pid_controller_node -> /cmd_vel
```

The equation follows the standard PID form:

```text
error = target - measured
correction = kp * error + ki * integral(error) + kd * d(error)/dt
command = target + correction
```

Integral and output limits prevent runaway commands. If measured feedback times
out, the default behavior is pass-through to keep the robot controllable while
the firmware watchdog remains the final stop layer.

## Run PID Only

```bash
ros2 launch learn2race_control twist_pid.launch.py
```

## Run Xbox Through PID

```bash
ros2 launch learn2race_control pid_xbox_drive.launch.py \
  serial_dev:=/dev/serial/by-id/YOUR_ESP32 joy_dev:=/dev/input/js0
```

Useful checks:

```bash
ros2 topic echo /cmd_vel_setpoint
ros2 topic echo /ddsm/measured_twist
ros2 topic echo /cmd_vel
```

## Tuning Order

1. Keep `linear_ki`, `angular_ki`, `linear_kd`, and `angular_kd` at `0`.
2. Raise `linear_kp` until forward speed tracks without oscillation.
3. Raise `angular_kp` until turning tracks without oscillation.
4. Add a tiny `*_ki` only if steady-state error remains.
5. Add `*_kd` only if the robot overshoots quickly.

## Required Packages

- `geometry_msgs`
- `rclpy`
- `launch`
- `launch_ros`
- `learn2race_bringup`

## Sources

- ROS 2 Jazzy `geometry_msgs/Twist`: https://docs.ros.org/en/ros2_packages/jazzy/api/geometry_msgs/msg/Twist.html
- ROS 2 Jazzy `control_toolbox` PID equation reference: https://control.ros.org/jazzy/doc/control_toolbox/doc/index.html

## References and more details

- [R40] Åström & Murray (PID, feed-forward, anti-windup); [R64] DDSM210; [R19] joystick teleoperation.
- Full entries: [`docs/references.md`](../../../docs/references.md).
