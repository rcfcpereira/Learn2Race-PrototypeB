# Prototype B – ROS 2 Jazzy workspace

The packages the Raspberry Pi 5 runs. Installation: [../INSTALL.md](../INSTALL.md). Use: [../USAGE.md](../USAGE.md).

```bash
cd ~/learn2race_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch learn2race_bringup prototype_b.launch.py
```

In normal use the robot is started with `~/robot.sh` (`base`, `slam`, `explore`).

The micro-ROS agent (`micro-ROS-Agent`, `micro_ros_msgs`) and the lidar driver (`ldlidar_stl_ros2`) are cloned into `src/` during installation; they are not part of this repository. The `competition:=true` option of `prototype_b.launch.py` (a classroom demonstration of the race state machine) also needs the `learn2race_competition` package from the Prototype A repository.

Each package has its own `README.md`. References cited as `[Rnn]` in the file headers are listed in [`docs/references.md`](../docs/references.md).
