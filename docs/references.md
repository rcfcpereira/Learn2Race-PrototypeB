# References

Every code file in this repository starts with a short header listing the references
below that it relies on, as `[Rnn]`, and where to read more. This page holds the full
entries. Datasheets are cited by manufacturer and part number: get the current revision
from the manufacturer's website.

Related documents in this repository:
- [`docs/README.md`](README.md): documentation index and system overview
-: what was built,
  why, and the measured results (the main source for the report)

---

## Competition and project

- **[R1]** Instituto Superior de Engenharia do Porto (ISEP). *TrackBotGP – Competition
  rules (2026 Robotics@ISEP Open version)*, February 2026. In this repository:
  `Competition_Rules_TrackBotGP_v2026_ISEP-ENG (1).pdf`. Homepage:
  https://www.isep.ipp.pt/Page/ViewPage/openrobotica_v
- **[R2]** ISEP. *Dragster – Competition rules (Robotics@ISEP Open)*. Not in this
  repository; the dragster code (start light, finish band, closed-park endurance)
  follows it.
- **[R3]** R. Pereira. *Learn2Race* dissertation (ISEP). Defines the functional and
  non-functional requirements quoted in the code as **FRn** / **NFRn** (e.g. FR10:
  command timeout and safe stop) and the SBC–MCU interface contract. Not in this
  repository.
- **[R4]** ``: development record and measured results,
  29 September – 3 October 2026.

## ROS 2 and middleware

- **[R10]** Open Robotics. *ROS 2 Jazzy Jalisco documentation*.
  https://docs.ros.org/en/jazzy/
- **[R11]** S. Macenski, T. Foote, B. Gerkey, C. Lalancette, W. Woodall. "Robot
  Operating System 2: Design, architecture, and uses in the wild." *Science Robotics*
  7(66), 2022.
- **[R12]** T. Foote, M. Purvis. *REP 103 – Standard Units of Measure and Coordinate
  Conventions*. https://www.ros.org/reps/rep-0103.html
- **[R13]** W. Meeussen. *REP 105 – Coordinate Frames for Mobile Platforms*.
  https://www.ros.org/reps/rep-0105.html
- **[R14]** micro-ROS project. *micro-ROS documentation*. https://micro.ros.org/ and
  the Arduino library used by both firmwares, `micro_ros_arduino` v2.0.8-jazzy:
  https://github.com/micro-ROS/micro_ros_arduino
- **[R15]** ROS 2 *launch* and *launch_ros* (launch files, substitutions):
  https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Launch/Launch-Main.html
- **[R16]** ROS 2 *rosbag2* (recording and reading bags):
  https://github.com/ros2/rosbag2
- **[R17]** C. Rauch. *camera_ros*: ROS 2 node for libcamera cameras.
  https://github.com/christianrauch/camera_ros
- **[R18]** Raspberry Pi Ltd. *Camera software documentation* (libcamera, rpicam
  apps, IMX219 sensor modes).
  https://www.raspberrypi.com/documentation/computers/camera_software.html
- **[R19]** ROS 2 *joy* and *teleop_twist_joy* (gamepad teleoperation).
  https://github.com/ros-drivers/joystick_drivers and
  https://github.com/ros2/teleop_twist_joy
- **[R20]** ROS 2 *cv_bridge* (ROS image ⇄ OpenCV):
  https://github.com/ros-perception/vision_opencv
- **[R21]** ROS middleware implementations: eProsima Fast DDS
  (https://fast-dds.docs.eprosima.com/) and Eclipse Cyclone DDS
  (https://cyclonedds.io/docs/).

## Computer vision

- **[R30]** G. Bradski. "The OpenCV Library." *Dr. Dobb's Journal of Software Tools*,
  2000; and *OpenCV 4.x documentation*, https://docs.opencv.org/4.x/ (thresholding,
  morphology, connected components, contours, line fitting, JPEG encoding, stereo).
- **[R31]** N. Otsu. "A threshold selection method from gray-level histograms." *IEEE
  Transactions on Systems, Man, and Cybernetics* 9(1):62–66, 1979.
- **[R32]** R. C. Gonzalez, R. E. Woods. *Digital Image Processing*, 4th ed. Pearson,
  2018 (local thresholding, morphological opening/closing, connected components).
- **[R33]** S. Suzuki, K. Abe. "Topological structural analysis of digitized binary
  images by border following." *Computer Vision, Graphics, and Image Processing*
  30(1):32–46, 1985 (the contour algorithm behind `cv2.findContours`).
- **[R34]** Z. Zhang. "A flexible new technique for camera calibration." *IEEE
  Transactions on Pattern Analysis and Machine Intelligence* 22(11):1330–1334, 2000.
- **[R35]** R. Hartley, A. Zisserman. *Multiple View Geometry in Computer Vision*, 2nd
  ed. Cambridge University Press, 2004 (pinhole model, stereo, depth Z = f·B/d).

## Control, kinematics and estimation

- **[R40]** K. J. Åström, R. M. Murray. *Feedback Systems: An Introduction for
  Scientists and Engineers*, 2nd ed. Princeton University Press, 2021 (PID control,
  feed-forward, integrator anti-windup, setpoint rate limiting).
- **[R41]** R. C. Coulter. *Implementation of the Pure Pursuit Path Tracking
  Algorithm*. Technical report CMU-RI-TR-92-01, Carnegie Mellon University, 1992.
- **[R42]** R. Siegwart, I. R. Nourbakhsh, D. Scaramuzza. *Introduction to Autonomous
  Mobile Robots*, 2nd ed. MIT Press, 2011 (differential-drive kinematics, odometry and
  its error sources).
- **[R43]** J. Borenstein, L. Feng. "Measurement and correction of systematic odometry
  errors in mobile robots." *IEEE Transactions on Robotics and Automation*
  12(6):869–880, 1996 (the UMBmark test).
- **[R44]** S. Thrun, W. Burgard, D. Fox. *Probabilistic Robotics*. MIT Press, 2005
  (Kalman filtering, SLAM).
- **[R45]** T. Moore, D. Stouch. "A Generalized Extended Kalman Filter Implementation
  for the Robot Operating System." *Intelligent Autonomous Systems 13*, Springer,
  2016; package *robot_localization*: https://github.com/cra-ros-pkg/robot_localization
- **[R46]** S. O. H. Madgwick. *An efficient orientation filter for inertial and
  inertial/magnetic sensor arrays*. Report, University of Bristol, 2010; ROS package
  *imu_filter_madgwick* (imu_tools): https://github.com/CCNYRoboticsLab/imu_tools
- **[R47]** S. Macenski, I. Jambrecic. "SLAM Toolbox: SLAM for the dynamic world."
  *Journal of Open Source Software* 6(61):2783, 2021.
  https://github.com/SteveMacenski/slam_toolbox
- **[R48]** M. Labbé, F. Michaud. "RTAB-Map as an open-source lidar and visual
  simultaneous localization and mapping library for large-scale and long-term online
  operation." *Journal of Field Robotics* 36(2):416–446, 2019.
- **[R49]** T. Ozyagcilar. *Calibrating an eCompass in the Presence of Hard- and
  Soft-Iron Interference*. Application note AN4246, Freescale/NXP Semiconductors
  (magnetometer hard-iron offset).
- **[R50]** D. Harel. "Statecharts: A visual formalism for complex systems." *Science
  of Computer Programming* 8(3):231–274, 1987 (finite state machines for the
  competition sequence).

## Hardware: boards, actuators, sensors

Datasheets and manufacturer documentation, by part:

- **[R60]** Waveshare. *General Driver for Robots* (ESP32 board with TB6612FNG
  H-bridge, QMI8658C IMU, AK09918 magnetometer, INA219 monitor) – wiki and schematic:
  https://www.waveshare.com/wiki/General_Driver_for_Robots
- **[R61]** Toshiba. *TB6612FNG* dual DC motor driver – datasheet (short brake,
  peak current).
- **[R62]** *CHP-20GP-180ABHL* geared DC motor with Hall encoder (i = 29.16, 11 PPR) –
  manufacturer datasheet.
- **[R63]** Espressif Systems. *ESP32 Technical Reference Manual* (LEDC PWM, PCNT,
  UART) and *Arduino core for the ESP32*: https://github.com/espressif/arduino-esp32 ;
  PlatformIO: https://docs.platformio.org/
- **[R64]** Waveshare. *DDSM Driver HAT (A)* and *DDSM210* direct-drive servo motor –
  wiki pages (serial command protocol), https://www.waveshare.com/wiki/
- **[R65]** Raspberry Pi Ltd. *Raspberry Pi 4 Model B* and *Raspberry Pi 5*
  documentation (GPIO, SPI, UART, `config.txt`):
  https://www.raspberrypi.com/documentation/
- **[R66]** Sony. *IMX219* image sensor (Raspberry Pi Camera Module v2) – see [R18].
- **[R67]** Lite-On. *LTR-303ALS-01* ambient light sensor – datasheet.
- **[R68]** QST. *QMI8658C* 6-axis IMU – datasheet. AKM. *AK09918* magnetometer –
  datasheet.
- **[R69]** Texas Instruments. *INA219* and *INA226* current/power monitors –
  datasheets; Arduino library *INA219_WE*: https://github.com/wollewald/INA219_WE
- **[R70]** Texas Instruments. *ADS1115* 16-bit ADC and *ADS7830* 8-bit ADC –
  datasheets. NXP. *PCF8591* 8-bit ADC/DAC – datasheet.
- **[R71]** STMicroelectronics. *LSM6DS3* 6-axis IMU – datasheet.
- **[R72]** STMicroelectronics. *VL6180X* time-of-flight proximity sensor – datasheet;
  *TOF050C* module (VL6180X-based, TOF0xxC family) – vendor documentation.
- **[R73]** Sharp. *GP2Y0A21YK0F* infrared distance sensor – datasheet.
- **[R74]** Seeed Studio. *Grove* modules: Ultrasonic Ranger, Light Sensor v1.2,
  6-Axis Accelerometer & Gyroscope (LSM6DS3), RGB LED Stick (10 × WS2813 Mini),
  Chainable RGB LED (P9813) – https://wiki.seeedstudio.com/
- **[R75]** Worldsemi. *WS2813* intelligent control LED – datasheet (bit timing,
  reset/latch time). Driven over SPI as in the Linux *spidev* interface:
  https://www.kernel.org/doc/html/latest/spi/spidev.html ; alternative driver
  *rpi_ws281x*: https://github.com/jgarff/rpi_ws281x
- **[R76]** P9813 chainable RGB LED driver – datasheet; Arduino library
  *ChainableLED* by P. Marques (MIT licence), bundled unchanged in
  Prototype A firmware, `firmware/lib/ChainableLED`: https://github.com/pjpmarques/ChainableLED
- **[R77]** NXP *PCA9548A* / Texas Instruments *TCA9548A* 8-channel I2C multiplexer –
  datasheets.
- **[R78]** Solomon Systech. *SSD1306* OLED controller – datasheet.
- **[R79]** LDROBOT. *STL-06P / LD06* 2D lidar and ROS 2 driver *ldlidar_stl_ros2*:
  https://github.com/ldrobotSensorTeam/ldlidar_stl_ros2
- **[R80]** *QYF-750* reflective line-sensor array module – vendor documentation.

## Linux and tools on the robots

- **[R90]** Canonical. *Netplan* documentation (`ignore-carrier`, `critical`):
  https://netplan.readthedocs.io/
- **[R91]** *tmux* terminal multiplexer: https://github.com/tmux/tmux/wiki
- **[R92]** Linux *udev* rules (stable device names): `man 7 udev`
