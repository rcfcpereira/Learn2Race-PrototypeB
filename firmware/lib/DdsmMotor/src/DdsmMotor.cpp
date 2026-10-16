// References (full entries: docs/references.md at the repository root):
//   [R64] Waveshare DDSM Driver HAT / DDSM210: DDSM serial protocol
// More details: firmware/README.md

#include "DdsmMotor.h"

#include <math.h>
#include <string.h>

namespace l2r {

DdsmBus::DdsmBus(HardwareSerial& serial) : serial_(serial) {}

void DdsmBus::begin(int rx_pin, int tx_pin) {
  // Serial1 is connected to the motor driver HAT, not to the micro-ROS USB link.
  serial_.begin(config::DdsmBaud, SERIAL_8N1, rx_pin, tx_pin);
  tx_enable_pin_ = config::DdsmTxEnablePin;
  if (tx_enable_pin_ >= 0) {
    pinMode(tx_enable_pin_, OUTPUT);
    digitalWrite(tx_enable_pin_, LOW);
  }
}

void DdsmBus::clear() {
  // Drop stale bytes before each command so the next 10 bytes are the fresh reply.
  while (serial_.available() > 0) {
    serial_.read();
  }
}

void DdsmBus::setSpeedMode(uint8_t id) {
  // Model 210 uses a CRC-protected speed-mode packet; model 115 uses a fixed
  // byte in the last position according to the DDSM serial protocol.
  uint8_t packet[10] = {id, 0xA0, 0, 0, 0, 0, 0, 0, 0, 0};

#if DDSM_MOTOR_TYPE == 210
  packet[2] = 0x02;
  packet[9] = crc8(packet, 9);
#else
  packet[9] = 0x02;
#endif

  writeFrame(packet, sizeof(packet));
}

bool DdsmBus::commandSpeed(uint8_t id, float rpm, uint8_t accel_time, DdsmFeedback& feedback) {
  const int16_t command = rpmToCommand(rpm);
  // Command frame: id, opcode, signed speed high/low, reserved bytes,
  // acceleration time, reserved bytes, CRC.
  uint8_t packet[10] = {
    id,
    0x64,
    static_cast<uint8_t>((command >> 8) & 0xFF),
    static_cast<uint8_t>(command & 0xFF),
    0,
    0,
    accel_time,
    0,
    0,
    0
  };
#if DDSM_MOTOR_TYPE == 210
  // Select speed and bus-current feedback explicitly (DDSM210 protocol).
  packet[4] = 0x01;
  packet[5] = 0x02;
#endif
  packet[9] = crc8(packet, 9);

  writeFrame(packet, sizeof(packet));

  uint8_t reply[10];
  if (!readFrame(reply, id, config::DdsmResponseTimeoutMs)) {
    feedback.valid = false;
    return false;
  }

  feedback = parseFeedback(reply);
  feedback.valid = feedback.valid && feedback.error == 0;
  return feedback.valid;
}

void DdsmBus::stop(uint8_t id) {
  DdsmFeedback unused;
  commandSpeed(id, 0.0f, 0, unused);
}

void DdsmBus::writeFrame(const uint8_t* data, size_t length) {
  clear();
  if (tx_enable_pin_ >= 0) {
    digitalWrite(tx_enable_pin_, HIGH);
    delayMicroseconds(100);
  }

  serial_.write(data, length);
  serial_.flush();

  if (tx_enable_pin_ >= 0) {
    delay(3);
    digitalWrite(tx_enable_pin_, LOW);
  }
}

uint8_t DdsmBus::crc8Update(uint8_t crc, uint8_t data) {
  // Bitwise CRC keeps the firmware dependency-free and is fast enough at 50 Hz.
  crc ^= data;
  for (uint8_t i = 0; i < 8; ++i) {
    crc = (crc & 0x01) ? ((crc >> 1) ^ 0x8C) : (crc >> 1);
  }
  return crc;
}

uint8_t DdsmBus::crc8(const uint8_t* data, size_t length) {
  uint8_t crc = 0;
  for (size_t i = 0; i < length; ++i) {
    crc = crc8Update(crc, data[i]);
  }
  return crc;
}

int16_t DdsmBus::signed16(uint8_t high, uint8_t low) {
  return static_cast<int16_t>((static_cast<uint16_t>(high) << 8) | low);
}

int16_t DdsmBus::rpmToCommand(float rpm) {
  if (!isfinite(rpm)) {
    return 0;
  }
  // Clamp before scaling to protect the motors from accidental high commands.
  if (rpm > config::MaxWheelRpm) {
    rpm = config::MaxWheelRpm;
  } else if (rpm < -config::MaxWheelRpm) {
    rpm = -config::MaxWheelRpm;
  }

  const float scaled = rpm * config::DdsmSpeedCommandScale;
  if (scaled >= 0.0f) {
    return static_cast<int16_t>(scaled + 0.5f);
  }
  return static_cast<int16_t>(scaled - 0.5f);
}

bool DdsmBus::readFrame(uint8_t* data, uint8_t expected_id, uint32_t timeout_ms) {
  const uint32_t start = millis();
  size_t count = 0;

  // Scan within one fixed deadline. Ignore late replies from another motor,
  // non-speed replies and noise instead of assigning them to this wheel.
  while ((millis() - start) < timeout_ms) {
    if (serial_.available() > 0) {
      data[count++] = static_cast<uint8_t>(serial_.read());
      if (count == 10) {
#if DDSM_MOTOR_TYPE == 210
        const bool speed_reply = data[1] == 0x64;
#else
        const bool speed_reply = data[1] == 0x02;  // DDSM115 speed-loop mode
#endif
        if (data[0] == expected_id && speed_reply && crc8(data, 9) == data[9]) {
          return true;
        }
        memmove(data, data + 1, 9);
        count = 9;
      }
    } else {
      delay(0);
    }
  }

  return false;
}

DdsmFeedback DdsmBus::parseFeedback(const uint8_t* data) {
  DdsmFeedback feedback;
  feedback.id = data[0];
  feedback.mode = data[1];
  feedback.error = data[8];
  feedback.valid = true;

#if DDSM_MOTOR_TYPE == 210
  // DDSM210 reports speed/current and odometer/position in different modes.
  if (data[1] == 0x64) {
    feedback.speed_rpm = static_cast<float>(signed16(data[2], data[3])) / config::DdsmSpeedCommandScale;
    feedback.torque_or_current = signed16(data[4], data[5]);
    feedback.temperature_c = data[7];
  } else if (data[1] == 0x74) {
    feedback.mileage = static_cast<int32_t>(
      (static_cast<uint32_t>(data[2]) << 24) |
      (static_cast<uint32_t>(data[3]) << 16) |
      (static_cast<uint32_t>(data[4]) << 8) |
      static_cast<uint32_t>(data[5]));
    feedback.position_raw = (static_cast<uint16_t>(data[6]) << 8) | data[7];
  }
#else
  // DDSM115 speed feedback is already in RPM scale.
  feedback.torque_or_current = signed16(data[2], data[3]);
  feedback.speed_rpm = static_cast<float>(signed16(data[4], data[5]));
  feedback.position_raw = (static_cast<uint16_t>(data[6]) << 8) | data[7];
#endif

  return feedback;
}

}  // namespace l2r
