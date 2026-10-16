# Learn2Race IMU Calibration Package

This package calibrates the Grove 6-Axis Accelerometer & Gyroscope v1.0
LSM6DS3 raw IMU stream.

## Algorithm

The robot must stay completely still during calibration.

1. Subscribe to `/imu/data_raw` as `sensor_msgs/msg/Imu`.
2. Collect `sample_count` raw accelerometer and gyroscope samples.
3. Average angular velocity to estimate gyroscope bias in rad/s.
4. Average linear acceleration to record the gravity vector seen by the mounted IMU.
5. Compare acceleration norm against standard gravity, `9.80665 m/s^2`.
6. Write `~/.ros/learn2race/imu_calibration.yaml`.

The output is meant for later filtering/localization work, such as subtracting
gyro bias or configuring an IMU filter.

## Run

```bash
ros2 launch learn2race_imu_calibration imu_calibration.launch.py sample_count:=500
```

Check the output:

```bash
cat ~/.ros/learn2race/imu_calibration.yaml
```

## Required Packages

- `learn2race_bringup`
- `rclpy`
- `sensor_msgs`
- `launch`
- `launch_ros`

## Sources

- ROS 2 Jazzy `sensor_msgs` message package: https://docs.ros.org/en/ros2_packages/jazzy/api/sensor_msgs/
- ROS 2 Jazzy `sensor_msgs` message definitions, including `Imu`: https://docs.ros.org/en/ros2_packages/jazzy/api/sensor_msgs/__message_definitions.html

## References and more details

- [R12] REP 103 units and axes; [R42] Siegwart et al. (inertial sensor errors); [R46] Madgwick filter (consumer of the calibrated IMU).
- Full entries: [`docs/references.md`](../../../docs/references.md).
