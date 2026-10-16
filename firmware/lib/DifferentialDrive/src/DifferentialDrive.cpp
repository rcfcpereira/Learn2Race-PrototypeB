// References (full entries: docs/references.md at the repository root):
//   [R42] Siegwart et al., Autonomous Mobile Robots: kinematics and odometry
// More details: firmware/README.md

#include "DifferentialDrive.h"

#include <Arduino.h>
#include <math.h>

#include <RobotConfig.h>

namespace l2r {

DifferentialDrive::DifferentialDrive(float wheel_radius_m, float track_width_m, float max_wheel_rpm)
    : wheel_radius_m_(wheel_radius_m),
      track_width_m_(track_width_m),
      max_wheel_rpm_(max_wheel_rpm) {}

WheelVelocityCommand DifferentialDrive::toWheelRpm(float linear_mps, float angular_rps) const {
  // Differential-drive inverse kinematics:
  // left = v - omega * track/2, right = v + omega * track/2.
  const float left_mps = linear_mps - angular_rps * (track_width_m_ * 0.5f);
  const float right_mps = linear_mps + angular_rps * (track_width_m_ * 0.5f);

  WheelVelocityCommand command;
  command.left_rpm = clampFloat(metersPerSecondToRpm(left_mps), -max_wheel_rpm_, max_wheel_rpm_);
  command.right_rpm = clampFloat(metersPerSecondToRpm(right_mps), -max_wheel_rpm_, max_wheel_rpm_);
  return command;
}

BodyVelocity DifferentialDrive::measuredVelocity(
    const MotorConfig* motors,
    const MotorState* states,
    size_t count) const {
  float left_rpm_sum = 0.0f;
  float right_rpm_sum = 0.0f;
  size_t left_count = 0;
  size_t right_count = 0;

  // Multiple motors per side are averaged so a 4WD chassis behaves like one
  // virtual left wheel and one virtual right wheel.
  for (size_t i = 0; i < count; ++i) {
    const DdsmFeedback& feedback = states[i].feedback;
    if (!feedback.valid) {
      continue;
    }

    const float robot_rpm = feedback.speed_rpm * static_cast<float>(motors[i].sign);
    if (motors[i].side == MotorSide::Left) {
      left_rpm_sum += robot_rpm;
      ++left_count;
    } else {
      right_rpm_sum += robot_rpm;
      ++right_count;
    }
  }

  // A missing side is unknown, not a stationary wheel. RobotApp suppresses
  // odometry on incomplete feedback; this also prevents invented yaw here.
  if (left_count == 0 || right_count == 0) {
    return BodyVelocity{};
  }
  const float left_rpm = left_rpm_sum / static_cast<float>(left_count);
  const float right_rpm = right_rpm_sum / static_cast<float>(right_count);
  const float left_mps = rpmToMetersPerSecond(left_rpm);
  const float right_mps = rpmToMetersPerSecond(right_rpm);

  // Differential-drive forward kinematics:
  // v = (left + right)/2, omega = (right - left)/track.
  BodyVelocity velocity;
  velocity.linear_mps = (left_mps + right_mps) * 0.5f;
  velocity.angular_rps = (right_mps - left_mps) / track_width_m_;
  return velocity;
}

void DifferentialDrive::resetIntegratorTime(uint32_t now_ms) {
  last_integrator_ms_ = now_ms;
}

void DifferentialDrive::integrate(const BodyVelocity& velocity, uint32_t now_ms, OdometryState& odom) {
  const float dt = static_cast<float>(now_ms - last_integrator_ms_) / 1000.0f;
  last_integrator_ms_ = now_ms;

  // Ignore impossible time jumps, often caused by reconnects or timer reset.
  if (dt <= 0.0f || dt > 1.0f) {
    return;
  }

  const float delta_yaw = velocity.angular_rps * dt;
  const float midpoint_yaw = odom.yaw_rad + 0.5f * delta_yaw;
  odom.x_m += velocity.linear_mps * cosf(midpoint_yaw) * dt;
  odom.y_m += velocity.linear_mps * sinf(midpoint_yaw) * dt;
  odom.yaw_rad = normalizeAngle(odom.yaw_rad + delta_yaw);
}

float DifferentialDrive::metersPerSecondToRpm(float meters_per_second) const {
  // wheel circumference = 2*pi*r, then convert rev/s to rev/min.
  return (meters_per_second / (config::TwoPi * wheel_radius_m_)) * 60.0f;
}

float DifferentialDrive::rpmToMetersPerSecond(float rpm) const {
  return (rpm / 60.0f) * config::TwoPi * wheel_radius_m_;
}

float DifferentialDrive::clampFloat(float value, float min_value, float max_value) {
  if (value < min_value) {
    return min_value;
  }
  if (value > max_value) {
    return max_value;
  }
  return value;
}

float DifferentialDrive::normalizeAngle(float angle) {
  while (angle > PI) {
    angle -= config::TwoPi;
  }
  while (angle < -PI) {
    angle += config::TwoPi;
  }
  return angle;
}

}  // namespace l2r
