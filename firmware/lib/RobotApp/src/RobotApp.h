// References (full entries: docs/references.md at the repository root):
//   [R14] micro-ROS / micro_ros_arduino: micro-ROS
// More details: RobotApp.cpp; firmware/README.md

#ifndef L2R_ROBOT_APP_H
#define L2R_ROBOT_APP_H

// =============================================================================
// RobotApp.h - Top-level ESP32 firmware application for the Learn2Race base.
// =============================================================================
// Responsibilities:
//   * micro-ROS node lifecycle over USB serial (connect / reconnect / teardown)
//   * agent <-> ESP32 clock synchronization (FIX #1, see below)
//   * DDSM motor bus control (Waveshare DDSM Driver HAT, DDSM210 motors)
//   * /cmd_vel subscription with a safety watchdog
//   * differential-drive kinematics and /odom + measured twist publishing
//   * optional startup wheel test, disabled in the normal build
//
// External libraries used (citations):
//   [1] micro_ros_arduino (Jazzy release) - micro-ROS client for Arduino:
//       https://github.com/micro-ROS/micro_ros_arduino
//       Provides rcl/rclc headers, message structs, rmw_microros utilities
//       (ping, time sync), and the prebuilt libmicroros.a (platformio.ini).
//   [2] rclc executor/support API (upstream project used through [1]):
//       https://github.com/ros2/rclc
//   [3] Arduino-ESP32 core (Serial, millis, delay):
//       https://github.com/espressif/arduino-esp32
//   [4] DDSM serial protocol reference (Waveshare DDSM Driver HAT wiki):
//       https://www.waveshare.com/wiki/DDSM_Driver_HAT
//   [5] micro-ROS time-sync API documentation (rmw_uros_sync_session /
//       rmw_uros_epoch_nanos): https://micro.ros.org/docs/api/rmw/
//
// Project-local libraries (this repository, not external):
//   * DdsmMotor          - DDSM bus framing/CRC and per-motor commands [4]
//   * DifferentialDrive  - Twist <-> wheel RPM kinematics + odom integration
//   * MicroRosSerialTransport - binds Arduino Serial to micro-ROS transport [1]
//
// FIX HISTORY:
//   FIX #1 (time sync): the firmware previously stamped /odom with
//     rmw_uros_epoch_nanos() but NEVER called rmw_uros_sync_session(), so
//     epoch_nanos() always returned 0 and every /odom stamp fell back to
//     ESP32 uptime. The Raspberry Pi side (IMU, LiDAR) stamps with wall
//     clock, so robot_localization fused two incompatible time domains.
//     Now: sync once per (re)connect and re-sync periodically to bound
//     crystal drift. See TimeSync* constants and last_time_sync_ms_.
//   FIX #2 (bus flooding): controlStep() calls made directly from loop()
//     (startup test / micro-ROS disabled) are rate-limited to
//     CONTROL_PERIOD_MS via last_manual_control_ms_, and are skipped while
//     Connected so the rclc control timer never double-steps the motors.
// =============================================================================
//
// Connection diagnostics: once micro-ROS is enabled, the USB serial line is
// entirely owned by the XRCE transport [1], so there is no firmware-side way
// to print human-readable status. Rather than dedicating a pin/LED on the
// ESP32 itself, connection health is surfaced Pi-side instead: the
// learn2race_bringup micro_ros_link_led_node watches /odom freshness (only
// published while Connected, see publishOdometryIfDue) and reflects it on
// the robot's existing Grove RGB Stick.
// =============================================================================

#include <Arduino.h>                    // [3] Arduino-ESP32 core API
#include <geometry_msgs/msg/twist.h>    // [1] Twist message C struct
#include <nav_msgs/msg/odometry.h>      // [1] Odometry message C struct
#include <std_msgs/msg/bool.h>          // [1] Bool message C struct (safety/fault)
#include <rcl/rcl.h>                    // [1][2] ROS client library C API
#include <rclc/executor.h>              // [1][2] rclc executor (callbacks)
#include <rclc/rclc.h>                  // [1][2] rclc convenience init API

#include <DdsmMotor.h>                  // project: DDSM bus driver (protocol [4])
#include <DifferentialDrive.h>          // project: diff-drive kinematics

namespace l2r {

// Top-level firmware application. One instance is created in main.cpp and
// driven by the Arduino setup()/loop() lifecycle.
class RobotApp {
 public:
  RobotApp();

  // Initialize hardware and (optionally) the micro-ROS transport.
  // enable_micro_ros=false gives a standalone bench mode with no ROS agent.
  void begin(bool enable_micro_ros = false);

  // Called from loop(). Runs the agent state machine, services the rclc
  // executor, keeps the session clock synchronized, and drives the control
  // loop when the executor timer is not active yet.
  void spinOnce();

 private:
  // ---------------------------------------------------------------------------
  // FIX #1 constants - agent clock synchronization [1][5].
  // ---------------------------------------------------------------------------
  // How long one rmw_uros_sync_session() exchange may block. The sync is a
  // small request/reply on the serial link; 200 ms is generous at 921600 baud.
  static constexpr int kTimeSyncTimeoutMs = 200;
  // Re-sync period. The ESP32 crystal drifts on the order of tens of ppm
  // (~ms per minute); a 60 s re-sync keeps /odom stamps well inside the
  // tolerance of robot_localization and TF consumers.
  static constexpr uint32_t kTimeSyncPeriodMs = 60000;

  // How often the Connected state re-checks agent reachability. Pinging on
  // every spinOnce() would add a blocking round-trip on the same serial link
  // as XRCE-DDS traffic on every loop iteration; 200 ms still detects a lost
  // agent well within CmdTimeoutMs.
  static constexpr uint32_t kAgentPingPeriodMs = 200;

  // Consecutive control steps a motor may miss its reply before the base is
  // stopped. One lost frame on the DDSM bus is noise, not a dead motor;
  // stopping on it made the robot stutter. 2 steps = 80 ms at 25 Hz, still
  // well inside CmdTimeoutMs. A reply carrying a motor error always stops.
  static constexpr uint8_t kMaxMissedReplies = 2;

  // ---------------------------------------------------------------------------
  // micro-ROS reconnect state machine.
  // Pattern follows the micro-ROS "reconnection example" [1]:
  // examples/micro-ros_reconnection_example
  // ---------------------------------------------------------------------------
  enum class AgentState {
    Waiting,       // no agent reachable yet; ping periodically
    Available,     // ping succeeded; create ROS entities next
    Connected,     // entities exist; executor is spinning
    Disconnected   // ping lost; stop motors and tear entities down
  };

  // Startup self-test phases: spin both wheels forward, then reverse, then
  // stop. Confirms wiring, motor IDs, and direction signs before ROS control.
  enum class WheelTestState {
    Forward,
    Stop,
    Reverse
  };

  // rclc callbacks are plain C function pointers, so they cannot capture
  // `this`; they dispatch through this static instance pointer instead.
  static RobotApp* instance_;

  // --- Hardware and kinematics objects (project-local libraries) -----------
  DdsmBus ddsm_;                                  // DDSM serial bus on Serial1
  DifferentialDrive drive_;                       // kinematics + odom math
  MotorConfig motors_[config::MotorCount];        // id / side / sign per motor
  MotorState motor_states_[config::MotorCount];   // last feedback per motor
  AgentState agent_state_ = AgentState::Waiting;  // reconnect state machine

  // --- micro-ROS C handles ([1][2]) -----------------------------------------
  // These must stay alive for the entire Connected state; created together in
  // createRosEntities() and destroyed together in destroyRosEntities().
  rcl_allocator_t allocator_;
  rclc_support_t support_;
  rcl_node_t node_;
  rclc_executor_t executor_;
  rcl_subscription_t cmd_vel_sub_;      // /cmd_vel        (geometry_msgs/Twist)
  rcl_subscription_t safety_fault_sub_; // /safety/fault   (std_msgs/Bool)
  rcl_publisher_t odom_pub_;            // /odom           (nav_msgs/Odometry)
  rcl_publisher_t measured_twist_pub_;  // /ddsm/measured_twist (Twist)
  rcl_timer_t control_timer_;           // fires controlStep() @ ControlPeriodMs

  // Message storage. micro-ROS deserializes directly into these structs, so
  // they must outlive the subscription/publishers (hence class members).
  geometry_msgs__msg__Twist cmd_vel_msg_;
  std_msgs__msg__Bool safety_fault_msg_;
  geometry_msgs__msg__Twist measured_twist_msg_;
  nav_msgs__msg__Odometry odom_msg_;

  // --- Runtime state (SI units to match ROS message conventions) -----------
  bool ros_entities_ready_ = false;     // guards destroy/publish paths
  // Track successful init stages so failed connections can be cleaned up.
  enum RosInitStage : uint8_t {
    SupportReady = 1, NodeReady = 2, SubscriptionReady = 4,
    OdomPublisherReady = 8, TwistPublisherReady = 16,
    TimerReady = 32, ExecutorReady = 64, FaultSubscriptionReady = 128
  };
  uint8_t ros_init_stages_ = 0;
  bool micro_ros_enabled_ = false;      // set once in begin()
  bool time_synced_ = false;            // FIX #1: agent clock sync succeeded?
  bool startup_test_complete_ = true;   // bench test is disabled by default
  bool motor_feedback_ready_ = false;  // require all wheels before motion
  // Last /safety/fault from the Pi's safety monitor. While true the base is
  // held stopped and /cmd_vel is ignored, even if the Pi-side multiplexer has
  // died; clearing it does not replay an old command.
  bool safety_fault_ = false;
  WheelTestState wheel_test_state_ = WheelTestState::Forward;
  uint32_t wheel_test_state_started_ms_ = 0;  // phase start (millis)
  uint32_t last_cmd_ms_ = 0;                  // last /cmd_vel time (watchdog)
  uint32_t last_odom_publish_ms_ = 0;         // odometry publish throttle

  // FIX #1: timestamp of the last successful rmw_uros_sync_session() so the
  // Connected state can re-sync every kTimeSyncPeriodMs.
  uint32_t last_time_sync_ms_ = 0;

  // Timestamp of the last agentIsReachable() check made from the Connected
  // state, throttled to kAgentPingPeriodMs.
  uint32_t last_agent_ping_ms_ = 0;

  // FIX #2: rate limiter for controlStep() calls made directly from loop()
  // (startup test and micro-ROS-disabled mode). Without it, loop() called
  // controlStep() every iteration, flooding the DDSM bus far above the
  // intended CONTROL_PERIOD_MS rate. The rclc timer path has its own period.
  uint32_t last_manual_control_ms_ = 0;

  float target_linear_mps_ = 0.0f;      // commanded forward velocity  (m/s)
  float target_angular_rps_ = 0.0f;     // commanded yaw rate          (rad/s)
  BodyVelocity measured_velocity_;      // wheel-feedback-derived body twist
  OdometryState odom_state_;            // integrated x, y, yaw

  // --- rclc callbacks (static, dispatch via instance_) ---------------------
  static void cmdVelCallback(const void* msg_in);
  static void safetyFaultCallback(const void* msg_in);
  static void controlTimerCallback(rcl_timer_t* timer, int64_t last_call_time);

  // --- Internal helpers ------------------------------------------------------
  bool agentIsReachable();              // rmw_uros_ping_agent wrapper [1]
  bool createRosEntities();             // node/pubs/sub/timer/executor init
  void destroyRosEntities();            // symmetric teardown (best effort)
  void syncAgentTime();                 // FIX #1: rmw_uros_sync_session [1][5]
  void configureOdomMessage();          // frame ids + covariance constants
  void setOdomPoseAndTwist();           // fill odom/twist msgs from state
  void stopAllMotors();                 // explicit zero-speed to every motor
  void setupMotors();                   // speed mode + initial stop
  bool runWheelTestSequence();          // advance startup test state machine
  void controlStep();                   // one control period (test or ROS)
  void publishOdometryIfDue(uint32_t now_ms);
  builtin_interfaces__msg__Time rosNow();  // agent-synced time or uptime [1][5]
};

}  // namespace l2r

#endif  // L2R_ROBOT_APP_H
