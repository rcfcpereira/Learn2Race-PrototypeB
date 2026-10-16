// References (full entries: docs/references.md at the repository root):
//   [R64] Waveshare DDSM Driver HAT / DDSM210: DDSM210 commands
// More details: firmware/README.md

// =============================================================================
// src/teleop_wheels_test_main.cpp - Minimal wheel-only teleop firmware.
// =============================================================================
// A parallel, stripped-down alternative to the full RobotApp (src/main.cpp):
// subscribes to /cmd_vel and drives the DDSM wheels directly. No odometry
// publishing, no agent clock sync, no reconnect state machine, no startup
// wheel test. Use this to check the teleop -> ESP32 -> DDSM path in isolation
// before dealing with the full RobotApp's extra moving parts.
//
// Reuses the same project-local libraries as RobotApp, untouched:
//   DdsmMotor, DifferentialDrive, MicroRosSerialTransport, RobotConfig.
//
// Build/upload this instead of the main firmware:
//   pio run -e teleop_wheels_test -t upload
//
// Caveat: this firmware never publishes /odom, so the Pi-side
// micro_ros_link_led_node (which watches /odom freshness on the RGB stick)
// will not show "connected" here - use the agent's own log ("session
// established") or `ros2 node list` / `ros2 topic list` to confirm the link.
// =============================================================================

#include <Arduino.h>
#include <math.h>
#include <geometry_msgs/msg/twist.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include <micro_ros_arduino.h>
#include <rmw_microros/rmw_microros.h>

#include <DdsmMotor.h>
#include <DifferentialDrive.h>
#include <MicroRosSerialTransport.h>
#include <RobotConfig.h>

using namespace l2r;

namespace {

DdsmBus ddsm(Serial1);
DifferentialDrive drive(config::WheelRadiusM, config::TrackWidthM, config::MaxWheelRpm);

struct MotorEntry {
  uint8_t id;
  MotorSide side;
  int8_t sign;
};

MotorEntry motors[config::MotorCount] = {
  {LEFT_FRONT_ID, MotorSide::Left, LEFT_MOTOR_SIGN},
  {RIGHT_FRONT_ID, MotorSide::Right, RIGHT_MOTOR_SIGN},
#if MOTOR_COUNT == 4
  {LEFT_REAR_ID, MotorSide::Left, LEFT_MOTOR_SIGN},
  {RIGHT_REAR_ID, MotorSide::Right, RIGHT_MOTOR_SIGN},
#endif
};

rcl_allocator_t allocator;
rclc_support_t support;
rcl_node_t node;
rclc_executor_t executor;
rcl_subscription_t cmd_vel_sub;
geometry_msgs__msg__Twist cmd_vel_msg;

float target_linear_mps = 0.0f;
float target_angular_rps = 0.0f;
uint32_t last_cmd_ms = 0;
bool motor_feedback_ready = false;

enum class State { Waiting, Connected };
State state = State::Waiting;

// rclc subscription callback for /cmd_vel. Same watchdog pattern as
// RobotApp: just record the latest command and when it arrived.
void cmdVelCallback(const void* msg_in) {
  const auto* msg = static_cast<const geometry_msgs__msg__Twist*>(msg_in);
  const float linear = static_cast<float>(msg->linear.x);
  const float angular = static_cast<float>(msg->angular.z);
  target_linear_mps = isfinite(linear) && isfinite(angular) ? linear : 0.0f;
  target_angular_rps = isfinite(linear) && isfinite(angular) ? angular : 0.0f;
  last_cmd_ms = millis();
}

// Body twist -> per-side wheel RPM -> one DDSM command per motor. If no
// /cmd_vel has arrived within CmdTimeoutMs, command zero (same safety
// watchdog as the full firmware).
void commandWheels() {
  const bool stale = (millis() - last_cmd_ms) > config::CmdTimeoutMs || !motor_feedback_ready;
  const float linear = stale ? 0.0f : target_linear_mps;
  const float angular = stale ? 0.0f : target_angular_rps;

  const WheelVelocityCommand command = drive.toWheelRpm(linear, angular);
  for (size_t i = 0; i < config::MotorCount; ++i) {
    const float side_rpm = motors[i].side == MotorSide::Left ? command.left_rpm : command.right_rpm;
    const float motor_rpm = side_rpm * static_cast<float>(motors[i].sign);
    DdsmFeedback feedback;
    if (!ddsm.commandSpeed(motors[i].id, motor_rpm, config::DdsmAccelTime, feedback)) {
      target_linear_mps = 0.0f;
      target_angular_rps = 0.0f;
      motor_feedback_ready = false;
      for (size_t j = 0; j < config::MotorCount; ++j) {
        ddsm.stop(motors[j].id);
        delay(2);
      }
      return;
    }
    delay(1);  // inter-frame gap on the half-duplex DDSM bus
  }
  motor_feedback_ready = true;
}

// Build the minimal node/subscription/executor. Torn down never - if the
// agent goes away, the command watchdog above just zeroes the wheels; there
// is no reconnect/rebuild cycle in this deliberately-minimal test firmware.
bool connectToAgent() {
  allocator = rcl_get_default_allocator();
  if (rclc_support_init(&support, 0, nullptr, &allocator) != RCL_RET_OK) {
    return false;
  }
  if (rclc_node_init_default(&node, "teleop_wheels_test", "", &support) != RCL_RET_OK) {
    return false;
  }
  if (rclc_subscription_init_default(
        &cmd_vel_sub,
        &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel") != RCL_RET_OK) {
    return false;
  }
  if (rclc_executor_init(&executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    return false;
  }
  if (rclc_executor_add_subscription(
        &executor, &cmd_vel_sub, &cmd_vel_msg, cmdVelCallback, ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }
  return true;
}

}  // namespace

void setup() {
  // Bind Serial to the micro-ROS transport BEFORE anything else touches it -
  // same reasoning as RobotApp::begin(): no plain-text prints on this wire.
  beginMicroRosSerialTransport();

  ddsm.begin(config::DdsmRxPin, config::DdsmTxPin);
  delay(100);  // let the DDSM HAT finish its own power-on init
  for (size_t i = 0; i < config::MotorCount; ++i) {
    ddsm.setSpeedMode(motors[i].id);
    delay(4);
    ddsm.stop(motors[i].id);
    delay(4);
  }

  last_cmd_ms = millis();
}

void loop() {
  switch (state) {
    case State::Waiting:
      if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
        state = connectToAgent() ? State::Connected : State::Waiting;
      } else {
        delay(100);
      }
      break;

    case State::Connected:
      (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));
      {
        static uint32_t last_control_ms = 0;
        const uint32_t now_ms = millis();
        if (now_ms - last_control_ms >= config::ControlPeriodMs) {
          last_control_ms = now_ms;
          commandWheels();
        }
      }
      break;
  }
}
