// References (full entries: docs/references.md at the repository root):
//   [R14] micro-ROS / micro_ros_arduino: micro-ROS client
//   [R63] ESP32 TRM / Arduino-ESP32 / PlatformIO: ESP32 Arduino
// More details: firmware/README.md

#include <Arduino.h>
#include <RobotApp.h>

// The Arduino sketch stays intentionally tiny. All robot behavior lives in
// RobotApp so the setup/loop lifecycle is easy to test and reason about.
static l2r::RobotApp robot;

void setup() {
  // Start the firmware with micro-ROS enabled so the ESP32 can receive
  // commands from a ROS 2 agent over the USB serial transport.
  robot.begin(true);
}

void loop() {
  // Run the control loop and service the micro-ROS executor when the agent is
  // available.
  robot.spinOnce();
}
