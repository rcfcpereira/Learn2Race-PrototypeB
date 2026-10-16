# Learn2Race Odometry Package

This package owns odometry-specific ROS helpers.

## Algorithm

The ESP32 firmware publishes `/odom` as `nav_msgs/msg/Odometry`. SLAM and Nav2
also need TF, so `odom_tf_broadcaster_node` converts each odometry pose into:

```text
odom -> base_link
```

The node uses the Raspberry Pi ROS clock by default because the LD06 LiDAR and
camera nodes are Pi-side sensors. Set `use_msg_stamp:=true` only after micro-ROS
time synchronization is known to be correct.

## Run

```bash
ros2 launch learn2race_odometry odometry.launch.py
```

Useful checks:

```bash
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

## Required Packages

- `rclpy`
- `nav_msgs`
- `geometry_msgs`
- `tf2_ros`
- `launch`
- `launch_ros`

## References and more details

- [R42] Siegwart et al. (differential-drive odometry); [R13] REP 105 frames; [R43] Borenstein & Feng.
- Full entries: [`docs/references.md`](../../../docs/references.md).
