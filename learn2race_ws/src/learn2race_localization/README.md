# Learn2Race Localization Package

This package configures `robot_localization` EKF for the Learn2Race robot.

## Algorithm

The EKF fuses:

- `/odom` from the ESP32 DDSM firmware: planar `x`, `y`, `yaw`, forward velocity,
  and yaw rate.
- `/imu/data_raw` from the Grove LSM6DS3: yaw-rate gyroscope only.

The output is:

```text
/odometry/filtered
odom -> base_link
```

Because `ekf_node` publishes `odom -> base_link`, do not run
`learn2race_odometry/odom_tf_broadcaster_node` at the same time.

## Run

Start the robot sensors/firmware, then:

```bash
ros2 launch learn2race_localization ekf_odom_imu.launch.py
```

Useful checks:

```bash
ros2 topic echo /odometry/filtered --once
ros2 run tf2_ros tf2_echo odom base_link
```

## Tuning Notes

Tune the covariance matrices using real `rosbag2` recordings. If the filtered
pose jitters, raise the IMU or odometry measurement covariances. If the filter
is slow to respond, lower the process noise for the relevant velocity state.

## Required Packages

- `robot_localization`
- `launch`
- `launch_ros`

## Sources

- `robot_localization` state estimation nodes: https://docs.ros.org/en/jade/api/robot_localization/html/state_estimation_nodes.html
- ROS 2 Jazzy `nav_msgs`: https://docs.ros.org/en/jazzy/p/nav_msgs/index.html

## References and more details

- [R45] Moore & Stouch, robot_localization; [R44] Thrun et al.; [R47] SLAM Toolbox; [R13] REP 105 frames.
- Full entries: [`docs/references.md`](../../../docs/references.md).
