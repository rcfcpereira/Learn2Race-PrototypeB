// References (full entries: docs/references.md at the repository root):
//   [R64] Waveshare DDSM Driver HAT / DDSM210: DDSM serial protocol
// More details: DdsmMotor.cpp

#ifndef L2R_DDSM_MOTOR_H
#define L2R_DDSM_MOTOR_H

#include <Arduino.h>
#include <stdint.h>

#include <RobotConfig.h>

namespace l2r {

// Parsed response frame from a DDSM motor. Some fields are only meaningful for
// specific DDSM models, but keeping one struct simplifies the drive code.
struct DdsmFeedback {
  uint8_t id = 0;
  uint8_t mode = 0;
  float speed_rpm = 0.0f;
  int16_t torque_or_current = 0;
  uint16_t position_raw = 0;
  int32_t mileage = 0;
  uint8_t temperature_c = 0;
  uint8_t error = 0;
  bool valid = false;
};

// Thin serial protocol wrapper for the Waveshare DDSM Driver HAT. It writes one
// 10-byte command frame and optionally parses one 10-byte feedback frame.
class DdsmBus {
 public:
  explicit DdsmBus(HardwareSerial& serial);

  // Configure the ESP32 UART pins and put motors into speed-control mode.
  void begin(int rx_pin, int tx_pin);
  void clear();
  void setSpeedMode(uint8_t id);
  // Valid only for this motor's speed reply with good CRC and no motor error.
  bool commandSpeed(uint8_t id, float rpm, uint8_t accel_time, DdsmFeedback& feedback);
  void stop(uint8_t id);

 private:
  HardwareSerial& serial_;
  int tx_enable_pin_ = -1;

  void writeFrame(const uint8_t* data, size_t length);

  // DDSM frames use Dallas/Maxim CRC-8 over the first nine bytes.
  static uint8_t crc8Update(uint8_t crc, uint8_t data);
  static uint8_t crc8(const uint8_t* data, size_t length);
  static int16_t signed16(uint8_t high, uint8_t low);
  static int16_t rpmToCommand(float rpm);
  bool readFrame(uint8_t* data, uint8_t expected_id, uint32_t timeout_ms);
  static DdsmFeedback parseFeedback(const uint8_t* data);
};

}  // namespace l2r

#endif  // L2R_DDSM_MOTOR_H
