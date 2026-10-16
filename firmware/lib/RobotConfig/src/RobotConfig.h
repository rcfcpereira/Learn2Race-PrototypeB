// References (full entries: docs/references.md at the repository root):
//   [R64] Waveshare DDSM Driver HAT / DDSM210: DDSM210
//   [R42] Siegwart et al., Autonomous Mobile Robots: geometry
// More details: firmware/README.md

#ifndef L2R_ROBOT_CONFIG_H
#define L2R_ROBOT_CONFIG_H

#include <stddef.h>
#include <stdint.h>

// Build flags can override every default below from platformio.ini. Keeping
// defaults in this header makes the firmware buildable even without overrides.
#ifndef ROS_SERIAL_BAUD
#define ROS_SERIAL_BAUD 921600
#endif

// ESP32 UART pins connected to the Waveshare DDSM Driver HAT serial input.
#ifndef DDSM_RX_PIN
#define DDSM_RX_PIN 18
#endif

#ifndef DDSM_TX_PIN
#define DDSM_TX_PIN 19
#endif

// Half-duplex RS485-style driver-enable pin for the DDSM bus transceiver.
// -1 means "no enable pin" (bus transceiver is always in receive mode /
// full-duplex hardware), matching how DdsmBus treats a negative pin number.
#ifndef DDSM_TX_ENABLE_PIN
#define DDSM_TX_ENABLE_PIN -1
#endif

#ifndef DDSM_BAUD
#define DDSM_BAUD 115200
#endif

// DDSM115 and DDSM210 report/accept speed with different scaling.
#ifndef DDSM_MOTOR_TYPE
#define DDSM_MOTOR_TYPE 210
#endif

// Differential drive can run as 2 motors or 4 motors with left/right pairs.
#ifndef MOTOR_COUNT
#define MOTOR_COUNT 2
#endif

#if MOTOR_COUNT != 2 && MOTOR_COUNT != 4
#error "MOTOR_COUNT must be 2 or 4"
#endif

#ifndef LEFT_FRONT_ID
#define LEFT_FRONT_ID 1
#endif

#ifndef RIGHT_FRONT_ID
#define RIGHT_FRONT_ID 2
#endif

#ifndef LEFT_REAR_ID
#define LEFT_REAR_ID 3
#endif

#ifndef RIGHT_REAR_ID
#define RIGHT_REAR_ID 4
#endif

#ifndef LEFT_MOTOR_SIGN
#define LEFT_MOTOR_SIGN 1
#endif

#ifndef RIGHT_MOTOR_SIGN
#define RIGHT_MOTOR_SIGN -1
#endif

// Wheel radius and track width are the two calibration constants that most
// affect odometry distance and rotation accuracy.
#ifndef WHEEL_RADIUS_M
#define WHEEL_RADIUS_M 0.03663f   // DDSM210: 74.5 mm wheel, measured effective
#endif

#ifndef TRACK_WIDTH_M
#define TRACK_WIDTH_M 0.1816f   // effective, LiDAR-calibrated in-place rotation
#endif

#ifndef MAX_WHEEL_RPM
#if DDSM_MOTOR_TYPE == 210
#define MAX_WHEEL_RPM 210.0f
#else
#define MAX_WHEEL_RPM 200.0f
#endif
#endif

#ifndef CONTROL_PERIOD_MS
// Budget 20 ms per motor for transmit/turnaround, reply and inter-frame gap.
#define CONTROL_PERIOD_MS (MOTOR_COUNT * 20)
#endif

#ifndef ODOM_PUBLISH_PERIOD_MS
#define ODOM_PUBLISH_PERIOD_MS 50
#endif

#ifndef CMD_TIMEOUT_MS
#define CMD_TIMEOUT_MS 500
#endif

// Motor serial reads are kept short so one slow motor cannot stall the control loop.
#ifndef DDSM_RESPONSE_TIMEOUT_MS
#define DDSM_RESPONSE_TIMEOUT_MS 10
#endif

// Explicit bench-only opt-in. Normal firmware never moves at power-on.
#ifndef STARTUP_WHEEL_TEST
#define STARTUP_WHEEL_TEST 0
#endif

#ifndef DDSM_ACCEL_TIME
#define DDSM_ACCEL_TIME 4
#endif

namespace l2r {
namespace config {

// Constants are copied from macros into typed values for the C++ modules.
static constexpr uint32_t RosSerialBaud = ROS_SERIAL_BAUD;
static constexpr int DdsmRxPin = DDSM_RX_PIN;
static constexpr int DdsmTxPin = DDSM_TX_PIN;
static constexpr int DdsmTxEnablePin = DDSM_TX_ENABLE_PIN;
static constexpr uint32_t DdsmBaud = DDSM_BAUD;
static constexpr int DdsmMotorType = DDSM_MOTOR_TYPE;
static constexpr size_t MotorCount = MOTOR_COUNT;
static constexpr float WheelRadiusM = WHEEL_RADIUS_M;
static constexpr float TrackWidthM = TRACK_WIDTH_M;
static constexpr float MaxWheelRpm = MAX_WHEEL_RPM;
static constexpr uint32_t ControlPeriodMs = CONTROL_PERIOD_MS;
static constexpr uint32_t OdomPublishPeriodMs = ODOM_PUBLISH_PERIOD_MS;
static constexpr uint32_t CmdTimeoutMs = CMD_TIMEOUT_MS;
static constexpr uint32_t DdsmResponseTimeoutMs = DDSM_RESPONSE_TIMEOUT_MS;
static constexpr bool StartupWheelTest = STARTUP_WHEEL_TEST != 0;
static constexpr uint8_t DdsmAccelTime = DDSM_ACCEL_TIME;
static constexpr float TwoPi = 6.28318530718f;

static_assert(ControlPeriodMs >= MotorCount * (DdsmResponseTimeoutMs + 6),
              "Control period must budget each motor reply plus bus overhead");

#if DDSM_MOTOR_TYPE == 210
static constexpr float DdsmSpeedCommandScale = 10.0f;
#else
static constexpr float DdsmSpeedCommandScale = 1.0f;
#endif

}  // namespace config
}  // namespace l2r

#endif  // L2R_ROBOT_CONFIG_H
