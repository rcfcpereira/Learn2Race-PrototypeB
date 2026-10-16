// References (full entries: docs/references.md at the repository root):
//   [R42] Siegwart et al., Autonomous Mobile Robots: kinematics
// More details: DifferentialDrive.cpp

#ifndef L2R_DIFFERENTIAL_DRIVE_H
#define L2R_DIFFERENTIAL_DRIVE_H

#include <stddef.h>
#include <stdint.h>

#include <DdsmMotor.h>

namespace l2r {

// Side lets four physical motors collapse into two differential-drive sides.
enum class MotorSide : uint8_t {
  Left,
  Right
};

// sign converts motor-positive RPM into robot-positive wheel RPM.
struct MotorConfig {
  uint8_t id;
  MotorSide side;
  int8_t sign;
};

// Feedback plus timestamp for each physical motor.
struct MotorState {
  DdsmFeedback feedback;
  uint32_t last_ok_ms = 0;
  uint8_t missed_replies = 0;  // consecutive control steps without a reply
};

// Left/right side command in motor RPM after differential kinematics.
struct WheelVelocityCommand {
  float left_rpm = 0.0f;
  float right_rpm = 0.0f;
};

// Robot-frame body velocity, matching geometry_msgs/Twist units.
struct BodyVelocity {
  float linear_mps = 0.0f;
  float angular_rps = 0.0f;
};

// Dead-reckoned planar pose in the odom frame.
struct OdometryState {
  float x_m = 0.0f;
  float y_m = 0.0f;
  float yaw_rad = 0.0f;
};

class DifferentialDrive {
 public:
  DifferentialDrive(float wheel_radius_m, float track_width_m, float max_wheel_rpm);

  // Convert commanded body velocity into left/right wheel RPM.
  WheelVelocityCommand toWheelRpm(float linear_mps, float angular_rps) const;
  // Average valid motor feedback into measured robot body velocity.
  BodyVelocity measuredVelocity(const MotorConfig* motors, const MotorState* states, size_t count) const;
  void resetIntegratorTime(uint32_t now_ms);
  // Integrate measured velocity using the heading at the interval midpoint.
  void integrate(const BodyVelocity& velocity, uint32_t now_ms, OdometryState& odom);

 private:
  float wheel_radius_m_;
  float track_width_m_;
  float max_wheel_rpm_;
  uint32_t last_integrator_ms_ = 0;

  float metersPerSecondToRpm(float meters_per_second) const;
  float rpmToMetersPerSecond(float rpm) const;
  static float clampFloat(float value, float min_value, float max_value);
  static float normalizeAngle(float angle);
};

}  // namespace l2r

#endif  // L2R_DIFFERENTIAL_DRIVE_H
