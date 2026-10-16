// References (full entries: docs/references.md at the repository root):
//   [R14] micro-ROS / micro_ros_arduino: custom serial transport
//   [R63] ESP32 TRM / Arduino-ESP32 / PlatformIO: ESP32 UART
// More details: firmware/README.md

#include "MicroRosSerialTransport.h"

#include <Arduino.h>
#include <micro_ros_arduino.h>

#include <RobotConfig.h>

extern "C" bool arduino_transport_open(struct uxrCustomTransport* transport) {
  (void)transport;
  // A larger RX buffer helps the ESP32 keep up with 921600 baud XRCE-DDS traffic.
  Serial.setRxBufferSize(1024);
  Serial.begin(l2r::config::RosSerialBaud);
  return true;
}

extern "C" bool arduino_transport_close(struct uxrCustomTransport* transport) {
  (void)transport;
  Serial.flush();
  Serial.end();
  return true;
}

extern "C" size_t arduino_transport_write(
    struct uxrCustomTransport* transport,
    const uint8_t* buffer,
    size_t length,
    uint8_t* error) {
  (void)transport;
  if (error != nullptr) {
    *error = 0;
  }
  // micro-ROS calls this whenever it needs to send XRCE-DDS bytes to the agent.
  return Serial.write(buffer, length);
}

extern "C" size_t arduino_transport_read(
    struct uxrCustomTransport* transport,
    uint8_t* buffer,
    size_t length,
    int timeout,
    uint8_t* error) {
  (void)transport;
  if (error != nullptr) {
    *error = 0;
  }
  // Respect the XRCE read timeout so agent pings and executor spins do not hang.
  Serial.setTimeout(timeout);
  return Serial.readBytes(reinterpret_cast<char*>(buffer), length);
}

namespace l2r {

void beginMicroRosSerialTransport() {
  // set_microros_transports() binds the Arduino Serial transport callbacks.
  Serial.setRxBufferSize(1024);
  Serial.begin(config::RosSerialBaud);
  set_microros_transports();
}

}  // namespace l2r
