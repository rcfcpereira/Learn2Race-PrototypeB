# Installation guide – Prototype B

This guide takes a blank Raspberry Pi 5 and a new DDSM Driver HAT to a robot that runs `~/robot.sh slam`.

## 1. What you need

- Raspberry Pi 5 (4 GB or more) with a 32 GB microSD card.
- Prototype B assembled:
  - the Waveshare DDSM Driver HAT (ESP32) on the Pi header, with two DDSM210 motors on its RS-485 bus;
  - the LD06 lidar on USB;
  - two IMX219 cameras on the two CSI ports;
  - on I2C bus 1: the LSM6DS3 IMU, and the PCA9548A multiplexer with the three VL6180X sensors on channels 0–2;
  - the P9813 LED on GPIO 5 (clock) and GPIO 6 (data).
- A PC with [PlatformIO](https://platformio.org/) to build and flash the ESP32.
- A network shared by the robot and the PC (the project uses a TP-Link TL-WR802N mini router).

## 2. Operating system

1. Write **Ubuntu Server 24.04 LTS (64-bit) for Raspberry Pi** with Raspberry Pi Imager. Create the user `learn2race` and enable SSH.
2. Update and install the basics:

   ```bash
   sudo apt update && sudo apt full-upgrade -y
   sudo apt install -y git tmux sysstat i2c-tools python3-pip python3-smbus2 python3-spidev python3-gpiozero
   ```

3. Edit `/boot/firmware/config.txt` and add, under `[all]`:

   ```ini
   dtparam=i2c_arm=on
   dtparam=spi=on
   # The ESP32 talks to the Pi on the header UART (GPIO14/15) -> /dev/ttyAMA0
   enable_uart=1
   dtparam=uart0=on
   camera_auto_detect=0
   dtoverlay=imx219,cam0
   dtoverlay=imx219,cam1
   ```

4. Free the header UART from the login console: remove `console=serial0,115200` (or `console=ttyAMA0,…`) from `/boot/firmware/cmdline.txt`.
5. Add the user to the hardware groups and reboot:

   ```bash
   sudo usermod -aG dialout,i2c,spi,gpio,video learn2race
   sudo reboot
   ```

6. Check the buses:

   ```bash
   ls -l /dev/ttyAMA0        # the ESP32 link
   i2cdetect -y 1            # 0x70 multiplexer, 0x6a/0x6b IMU
   ```

7. Optional, recommended:
   - disable Wi-Fi if the robot is on Ethernet (`dtoverlay=disable-wifi`, and remove `wlan0` from netplan);
   - turn off unattended upgrades.

## 3. ROS 2 Jazzy

Install `ros-jazzy-ros-base` and `ros-dev-tools` following the official [ROS 2 Jazzy guide for Ubuntu](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html). Then:

```bash
sudo apt install -y \
  ros-jazzy-slam-toolbox ros-jazzy-nav2-map-server \
  ros-jazzy-robot-localization ros-jazzy-imu-filter-madgwick \
  ros-jazzy-joy ros-jazzy-teleop-twist-joy ros-jazzy-tf2-ros \
  ros-jazzy-camera-ros ros-jazzy-camera-calibration ros-jazzy-stereo-image-proc \
  ros-jazzy-rtabmap-ros ros-jazzy-rmw-cyclonedds-cpp \
  python3-opencv python3-numpy
sudo rosdep init 2>/dev/null; rosdep update
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
```

RTAB-Map and the camera packages are needed only for stereo work.

## 4. Cameras (only for stereo work)

As on Prototype A, the ROS build of `libcamera` does not deliver frames from the Raspberry Pi cameras on Ubuntu. Build the Raspberry Pi fork of libcamera and `rpicam-apps` into `/usr/local`, following the Raspberry Pi documentation. The stereo launch files load it from `/usr/local/lib/aarch64-linux-gnu` when it is present. Check with `rpicam-hello --list-cameras`: both cameras must be listed.

## 5. Workspace

```bash
cd ~
git clone https://github.com/rcfcpereira/Learn2Race-PrototypeB.git
ln -s ~/Learn2Race-PrototypeB/learn2race_ws ~/learn2race_ws
cp ~/Learn2Race-PrototypeB/tools/robot.sh ~/robot.sh && chmod +x ~/robot.sh

cd ~/learn2race_ws/src
git clone -b jazzy https://github.com/micro-ROS/micro-ROS-Agent.git
git clone -b jazzy https://github.com/micro-ROS/micro_ros_msgs.git
git clone https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2.git

cd ~/learn2race_ws
rosdep install --from-paths src --ignore-src -r -y
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
```

The third-party folders are ignored by git, so `git pull` leaves them alone.

## 6. Lidar device name

Give the lidar a fixed name, `/dev/lidar`. The script asks you to unplug the device once; skip the ESP32 step, since B's ESP32 uses `/dev/ttyAMA0`:

```bash
sudo ~/learn2race_ws/src/learn2race_bringup/scripts/setup_udev.sh
```

## 7. ESP32 firmware (from the PC)

The HAT's USB port cannot put the ESP32 into boot mode while the HAT sits on the Pi, so flashing from the Pi fails ("Wrong boot mode"). Instead:

1. Power the robot off and lift the HAT off the Pi, or disconnect the Pi.
2. Connect the HAT's USB port to the PC and flash:

   ```bash
   cd firmware
   pio run -t upload
   ```

3. Refit the HAT. At run time the micro-ROS agent uses the header UART, `/dev/ttyAMA0` at 921 600 baud.

Use the HAT's USB port **only for flashing**. With `enable_uart=1`, the Pi's TX line also drives the ESP32's RX, so an agent on the USB port never forms a session.

## 8. First run

Lift the wheels off the floor, then:

```bash
~/robot.sh base
~/robot.sh status          # expect RUNNING and no safety fault
```

On the PC, start the gamepad bridge (see Learn2Race-PC-Control). On the robot, start `python3 ~/Learn2Race-PrototypeB/tools/udp_teleop.py`. Hold LB and move the sticks: the wheels should turn.

Then stop the base (`~/robot.sh stop`) and run the hardware check with the robot on a stand: `~/robot.sh check wheels_lifted:=true` (see [USAGE.md](USAGE.md#hardware-check)).

## 9. Automatic updates

Each robot updates itself from this repository, so a change reaches every unit with one `git push` and a power cycle instead of an SD-card image or an SSH session per robot. Install the service once:

```bash
~/Learn2Race-PrototypeB/tools/install_update_service.sh          # follow main
~/Learn2Race-PrototypeB/tools/install_update_service.sh v1.0     # or pin a release tag
```

At every boot, once the network is up, the `learn2race-update` service runs `tools/update_robot.sh`, which:

1. does nothing if the robot is running or the network is down;
2. refuses to update a checkout with uncommitted changes (commit calibration changes first);
3. fetches and moves to the pinned branch or tag (`L2R_REF` in `/etc/default/learn2race-update`);
4. runs `tools/check_robot_config.py`, which checks that `robot.yaml` agrees with `firmware/platformio.ini` and the ROS configuration, and rolls back if it does not;
5. installs missing dependencies with `rosdep` (when `sudo` needs no password) and rebuilds the workspace; if the build fails, it rolls back to the previous revision and rebuilds that;
6. refreshes `~/robot.sh` and records the version in `~/.learn2race_version`.

A revision that failed is not retried at every boot; push a fix, or force a retry with `L2R_FORCE=1 ~/robot.sh update`. The robot is never started by the update: start it with `~/robot.sh` as usual, which refuses to start while an update is building. The ESP32 is not flashed automatically: when `firmware/` changed, the log says so, and the firmware is flashed from a PC (section 7).

```bash
~/robot.sh update               # update now (the robot must be stopped)
~/robot.sh status               # shows the installed version
cat ~/update_logs/latest.log    # what the last update did
```

`robot.yaml`, at the root of this repository, records the robot's identity and calibration constants. Two robots built from the same release differ only in that file. When a constant changes, change it in `robot.yaml` and in the file it is compiled from, together; `python3 tools/check_robot_config.py` confirms they agree.

## Troubleshooting

| Symptom | Cause and fix |
|---|---|
| Agent never sees the ESP32 | Agent started on the USB port instead of `/dev/ttyAMA0`; serial console still on the UART; baud differs from `ROS_SERIAL_BAUD` (921 600) |
| Safety fault "odom stale" right after start | The ESP32 takes up to about 12 s to reconnect after an agent restart; wait, then `~/robot.sh reset` |
| SLAM map with doubled walls | Lidar transform wrong. The LD06's 0° points to the robot's right, so `base_laser` yaw must be −90° (`config/sensor_frames_b.yaml`) |
| SLAM drops many scans | Use `slam_b.launch.py`, which resamples each scan to a fixed beam count (`scan_fixed_node`) |
| `map_saver` times out | Already handled by `~/robot.sh save` (15 s timeout) |
| Robot moves on its own or stops unexpectedly | Another robot's ROS stack is running on the same network and domain. Run one at a time, or set different `ROS_DOMAIN_ID` values |
