// References (full entries: docs/references.md at the repository root):
//   [R14] micro-ROS / micro_ros_arduino: micro-ROS
//   [R64] Waveshare DDSM Driver HAT / DDSM210: DDSM HAT
//   [R12] REP 103, units and axes: units
//   [R3]  Learn2Race dissertation, requirements FR/NFR: FR10
// More details: firmware/README.md

// =============================================================================
// RobotApp.cpp - Implementation of the Learn2Race ESP32 firmware application.
// =============================================================================
// See RobotApp.h for the full architecture description and citation list.
//
// External libraries used (citations):
//   [1] micro_ros_arduino (Jazzy release):
//       https://github.com/micro-ROS/micro_ros_arduino
//       - rcl/rclc API, rmw_microros (ping / time sync), message structs.
//       - Agent reconnect pattern adapted from its official example:
//         examples/micro-ros_reconnection_example/micro-ros_reconnection_example.ino
//   [2] rclc (upstream executor/support API used through [1]):
//       https://github.com/ros2/rclc
//   [3] Arduino-ESP32 core (Serial, millis, delay):
//       https://github.com/espressif/arduino-esp32
//   [4] DDSM motor serial protocol (Waveshare DDSM Driver HAT wiki):
//       https://www.waveshare.com/wiki/DDSM_Driver_HAT
//   [5] micro-ROS time-sync API (rmw_uros_sync_session, rmw_uros_epoch_nanos,
//       rmw_uros_epoch_synchronized): https://micro.ros.org/docs/api/rmw/
//   [6] REP 103 (SI units, x forward, z up, yaw counter-clockwise):
//       https://www.ros.org/reps/rep-0103.html
//   [7] REP 105 (map/odom/base_link frame semantics for /odom):
//       https://www.ros.org/reps/rep-0105.html
//
// FIX HISTORY:
//   FIX #1 (time sync): createRosEntities() now calls syncAgentTime()
//     (rmw_uros_sync_session [5]) after a successful connect, and the
//     Connected state re-syncs every kTimeSyncPeriodMs. Before this fix
//     rmw_uros_epoch_nanos() always returned 0 (sync was never performed),
//     so /odom was stamped with ESP32 uptime while Pi-side sensors used
//     wall clock - breaking robot_localization fusion and any consumer of
//     use_msg_stamp:=true.
//   FIX #2 (bus flooding / double-step): loop()-driven controlStep() calls
//     are rate-limited to CONTROL_PERIOD_MS and suppressed while Connected
//     (the rclc timer is then the single owner of the control loop).
// =============================================================================
//
// Connection diagnostics live on the Pi side, not here: once micro-ROS is
// enabled, USB serial is entirely owned by the XRCE transport, so there is
// no way to print firmware-side status. learn2race_bringup's
// micro_ros_link_led_node watches /odom freshness (only published below,
// from the Connected state) and reflects it on the robot's Grove RGB Stick.
// =============================================================================

#include "RobotApp.h"

#include <math.h>                        // sinf/cosf for the yaw quaternion
#include <micro_ros_arduino.h>           // [1] transport + rmw_microros glue
#include <rmw_microros/rmw_microros.h>   // [1][5] ping, sync_session, epoch time

#include <MicroRosSerialTransport.h>     // project: Serial <-> XRCE transport
#include <RobotConfig.h>                 // project: all tunables (see
                                         // platformio.ini build_flags)

namespace l2r {

// Static instance pointer used by the C-style rclc callbacks (see header).
RobotApp* RobotApp::instance_ = nullptr;

// Cleanup/publish results are intentionally ignored: on a microcontroller
// there is no meaningful recovery from a failed fini/publish, and the
// reconnect state machine already handles the "agent gone" case.
static void ignoreRclResult(rcl_ret_t result) {
  (void)result;
}

RobotApp::RobotApp()
    // Serial1 is the ESP32 hardware UART wired to the DDSM Driver HAT [4];
    // pins are configured later in begin() via ddsm_.begin(RX, TX).
    : ddsm_(Serial1),
      // Kinematics constants come from RobotConfig.h (overridable from
      // platformio.ini): wheel radius, track width, and the motor RPM limit.
      drive_(config::WheelRadiusM, config::TrackWidthM, config::MaxWheelRpm),
      // Motor table: bus ID, which side of the chassis, and a sign that
      // absorbs the mirrored mounting of left vs right motors.
      motors_{
        {LEFT_FRONT_ID, MotorSide::Left, LEFT_MOTOR_SIGN},
        {RIGHT_FRONT_ID, MotorSide::Right, RIGHT_MOTOR_SIGN},
#if MOTOR_COUNT == 4
        // 4WD variant: rear pair shares the same side signs as the front.
        {LEFT_REAR_ID, MotorSide::Left, LEFT_MOTOR_SIGN},
        {RIGHT_REAR_ID, MotorSide::Right, RIGHT_MOTOR_SIGN},
#endif
      },
      // Zero-initialize every micro-ROS handle and message struct. The rcl C
      // API requires zeroed handles before *_init and after *_fini [1][2].
      allocator_{},
      support_{},
      node_{},
      executor_{},
      cmd_vel_sub_{},
      safety_fault_sub_{},
      odom_pub_{},
      measured_twist_pub_{},
      control_timer_{},
      cmd_vel_msg_{},
      safety_fault_msg_{},
      measured_twist_msg_{},
      odom_msg_{} {}

void RobotApp::begin(bool enable_micro_ros) {
  instance_ = this;
  micro_ros_enabled_ = enable_micro_ros;

  if (micro_ros_enabled_) {
    // Bind Arduino Serial to the micro-ROS custom transport [1] BEFORE any
    // other Serial.begin() call (see MicroRosSerialTransport.cpp). From this
    // point USB serial carries binary XRCE-DDS frames to the agent, so any
    // plain-text Serial.print() here would corrupt that byte stream and
    // prevent the agent from ever completing a session - debug prints are
    // skipped entirely in this mode for that reason.
    beginMicroRosSerialTransport();
  } else {
    // Debug console for bench mode only (micro-ROS disabled).
    Serial.begin(115200);
    delay(200);  // give the host time to enumerate USB CDC before printing
    Serial.println("[RobotApp] begin");
    Serial.println("[RobotApp] micro-ROS disabled");
    Serial.print("[RobotApp] configured motor ids: ");
    for (size_t i = 0; i < config::MotorCount; ++i) {
      Serial.print(motors_[i].id);
      if (i + 1 < config::MotorCount) {
        Serial.print(", ");
      }
    }
    Serial.println();
  }

  // Serial1 (pins from RobotConfig.h) is dedicated to the DDSM motor bus [4].
  ddsm_.begin(config::DdsmRxPin, config::DdsmTxPin);
  setupMotors();

  // Reset all time bases together so the watchdog and odometry integrator
  // start from a consistent "now".
  const uint32_t now_ms = millis();
  last_cmd_ms_ = now_ms;
  drive_.resetIntegratorTime(now_ms);

  // The autonomous bench test is opt-in at build time. A normal boot leaves
  // the motors stopped until a fresh ROS command arrives.
  wheel_test_state_ = WheelTestState::Forward;
  wheel_test_state_started_ms_ = now_ms;
  startup_test_complete_ = !config::StartupWheelTest;
  last_cmd_ms_ = wheel_test_state_started_ms_;
  if (!micro_ros_enabled_ && config::StartupWheelTest) {
    Serial.println("[RobotApp] starting startup wheel test");
    Serial.println("[RobotApp] phase: forward");
  }
}

void RobotApp::spinOnce() {
  // ---------------------------------------------------------------------------
  // FIX #2: rate-limit controlStep() calls that originate from loop().
  //
  // Before this fix, loop() invoked controlStep() on EVERY iteration during
  // the startup test and in micro-ROS-disabled mode. Each controlStep()
  // performs one serial transaction per motor on the DDSM bus [4], so the bus
  // was hammered far above the intended CONTROL_PERIOD_MS rate.
  // Additionally, once the agent connected mid-test, the rclc control timer
  // ALSO called controlStep(), sending duplicate motor commands.
  //
  // Now: loop()-driven control runs at most once per CONTROL_PERIOD_MS, and
  // is skipped entirely while Connected because the rclc timer (created in
  // createRosEntities with the same period) is then the single owner of the
  // control loop.
  // ---------------------------------------------------------------------------
  const uint32_t now_ms = millis();
  const bool control_due = (now_ms - last_manual_control_ms_) >= config::ControlPeriodMs;

  if (!micro_ros_enabled_) {
    // Bench mode: no agent, no executor - loop() owns the control period.
    if (control_due) {
      last_manual_control_ms_ = now_ms;
      controlStep();
    }
    return;
  }

  // Drive the startup wheel test from loop() until the executor timer takes
  // over (Connected state).
  if (!startup_test_complete_ && agent_state_ != AgentState::Connected && control_due) {
    last_manual_control_ms_ = now_ms;
    controlStep();
  }

  // ---------------------------------------------------------------------------
  // Agent reconnect state machine, adapted from the micro-ROS reconnection
  // example [1]. The firmware tolerates cable pulls and agent restarts
  // without rebooting: ping -> create entities -> spin executor -> teardown.
  // ---------------------------------------------------------------------------
  switch (agent_state_) {
    case AgentState::Waiting:
      // Poll for the agent. The 100 ms delay keeps the ping rate (and the
      // motor-test cadence above) sane while no agent is present.
      agent_state_ = agentIsReachable() ? AgentState::Available : AgentState::Waiting;
      if (agent_state_ == AgentState::Waiting) {
        delay(100);
      }
      break;

    case AgentState::Available:
      // Ping succeeded: build node, pubs, sub, timer, executor. On partial
      // failure, tear everything down so the retry starts from clean state.
      agent_state_ = createRosEntities() ? AgentState::Connected : AgentState::Waiting;
      if (agent_state_ == AgentState::Waiting) {
        destroyRosEntities();
        delay(100);
      }
      break;

    case AgentState::Connected: {
      // Throttled reachability check: only actually ping the agent every
      // kAgentPingPeriodMs instead of every spinOnce(), since each ping is a
      // blocking round-trip on the same serial link as XRCE-DDS traffic.
      bool reachable = true;
      if ((now_ms - last_agent_ping_ms_) >= kAgentPingPeriodMs) {
        last_agent_ping_ms_ = now_ms;
        reachable = agentIsReachable();
      }
      if (!reachable) {
        stopAllMotors();
        agent_state_ = AgentState::Disconnected;
      } else {
        // FIX #1: periodic clock re-sync. The ESP32 crystal drifts relative
        // to the Pi's NTP-disciplined clock; re-syncing every
        // kTimeSyncPeriodMs keeps /odom stamps aligned with Pi-side sensor
        // stamps for robot_localization and TF consumers [5][7].
        if ((now_ms - last_time_sync_ms_) >= kTimeSyncPeriodMs) {
          syncAgentTime();
        }
        // Service subscriptions and the control timer. 5 ms budget keeps
        // loop() responsive for the reachability check above [2].
        ignoreRclResult(rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(5)));
      }
      break;
    }

    case AgentState::Disconnected:
      // Safety first: explicitly stop the wheels before tearing down ROS,
      // otherwise the last commanded speed would persist on the motors.
      stopAllMotors();
      destroyRosEntities();
      agent_state_ = AgentState::Waiting;
      break;
  }
}

// rclc subscription callback for /cmd_vel (geometry_msgs/Twist) [1][2].
// Runs inside rclc_executor_spin_some, i.e. on the main loop task.
void RobotApp::cmdVelCallback(const void* msg_in) {
  if (instance_ == nullptr) {
    return;
  }

  if (instance_->safety_fault_) {
    return;  // no motion while faulted; do not feed the watchdog either
  }
  const auto* msg = static_cast<const geometry_msgs__msg__Twist*>(msg_in);
  // A differential-drive base can only realize forward velocity (linear.x)
  // and yaw rate (angular.z); the other Twist components are ignored [6].
  const float linear = static_cast<float>(msg->linear.x);
  const float angular = static_cast<float>(msg->angular.z);
  instance_->target_linear_mps_ = isfinite(linear) && isfinite(angular) ? linear : 0.0f;
  instance_->target_angular_rps_ = isfinite(linear) && isfinite(angular) ? angular : 0.0f;
  // Feed the command watchdog: controlStep() zeroes the target if this
  // timestamp becomes older than CMD_TIMEOUT_MS.
  instance_->last_cmd_ms_ = millis();
}

// rclc subscription callback for /safety/fault (std_msgs/Bool). The Pi's
// safety monitor publishes it continuously; a fault stops the base here even
// if the Pi-side multiplexer is no longer publishing zero commands.
void RobotApp::safetyFaultCallback(const void* msg_in) {
  if (instance_ == nullptr) {
    return;
  }
  const bool fault = static_cast<const std_msgs__msg__Bool*>(msg_in)->data;
  if (fault && !instance_->safety_fault_) {
    // Also drops the target, so a later clear cannot replay the old command.
    instance_->stopAllMotors();
  }
  instance_->safety_fault_ = fault;
}

// rclc timer callback: the ROS-driven control period (CONTROL_PERIOD_MS).
// Once Connected, this is the ONLY caller of controlStep() (see FIX #2).
void RobotApp::controlTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == nullptr || instance_ == nullptr) {
    return;
  }
  instance_->controlStep();
}

bool RobotApp::agentIsReachable() {
  // rmw_uros_ping_agent(timeout_ms, attempts) [1]. A short 100 ms single
  // attempt keeps reconnect checks responsive without starving the control
  // loop when the agent is down.
  return rmw_uros_ping_agent(100, 1) == RMW_RET_OK;
}

// -----------------------------------------------------------------------------
// FIX #1: synchronize the micro-ROS session clock with the agent [1][5].
//
// rmw_uros_sync_session() runs an NTP-like request/reply with the agent and
// stores the offset between ESP32 uptime and the agent's (Pi's) wall clock.
// Only after a successful sync does rmw_uros_epoch_nanos() return real epoch
// time; before that it returns 0, which is exactly the bug this fixes.
// Failure is tolerated (best effort): rosNow() falls back to uptime stamps,
// and the periodic re-sync in spinOnce() will retry.
// -----------------------------------------------------------------------------
void RobotApp::syncAgentTime() {
  last_time_sync_ms_ = millis();  // schedule the next attempt regardless
  if (rmw_uros_sync_session(kTimeSyncTimeoutMs) == RMW_RET_OK) {
    time_synced_ = true;
  }
  // rmw_uros_epoch_synchronized() [5] could be checked here for diagnostics;
  // time_synced_ is kept as the firmware's own record of a successful sync.
}

bool RobotApp::createRosEntities() {
  // All entities are (re)built together after every (re)connect so a stale
  // handle can never outlive an agent session. Init sequence follows the
  // canonical rclc pattern [2]: support -> node -> endpoints -> executor.
  allocator_ = rcl_get_default_allocator();

  if (rclc_support_init(&support_, 0, nullptr, &allocator_) != RCL_RET_OK) {
    return false;
  }
  ros_init_stages_ |= SupportReady;
  if (rclc_node_init_default(&node_, "ddsm_hat_controller", "", &support_) != RCL_RET_OK) {
    return false;
  }
  ros_init_stages_ |= NodeReady;
  // Subscriber: /cmd_vel - velocity commands from twist_mux on the Pi
  // (teleop / explorer / PID are arbitrated Pi-side; the firmware only ever
  // sees the winning command stream).
  if (rclc_subscription_init_default(
        &cmd_vel_sub_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/cmd_vel") != RCL_RET_OK) {
    return false;
  }
  ros_init_stages_ |= SubscriptionReady;
  if (rclc_subscription_init_default(
        &safety_fault_sub_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Bool),
        "/safety/fault") != RCL_RET_OK) {
    return false;
  }
  ros_init_stages_ |= FaultSubscriptionReady;
  // Publisher: /odom - integrated wheel odometry (REP 105 odom frame [7]).
  if (rclc_publisher_init_default(
        &odom_pub_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
        "/odom") != RCL_RET_OK) {
    return false;
  }
  ros_init_stages_ |= OdomPublisherReady;
  // Publisher: /ddsm/measured_twist - raw wheel-feedback body twist, used by
  // the Pi-side twist PID controller as its measurement input.
  if (rclc_publisher_init_default(
        &measured_twist_pub_,
        &node_,
        ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
        "/ddsm/measured_twist") != RCL_RET_OK) {
    return false;
  }
  ros_init_stages_ |= TwistPublisherReady;
  // Control timer: fires controlStep() every CONTROL_PERIOD_MS. The last
  // argument (autostart=true) matches rclc_timer_init_default2 [2].
  if (rclc_timer_init_default2(
        &control_timer_,
        &support_,
        RCL_MS_TO_NS(config::ControlPeriodMs),
        controlTimerCallback,
        true) != RCL_RET_OK) {
    return false;
  }
  ros_init_stages_ |= TimerReady;
  // Executor with 3 handles: /cmd_vel, /safety/fault and the control timer.
  if (rclc_executor_init(&executor_, &support_.context, 3, &allocator_) != RCL_RET_OK) {
    return false;
  }
  ros_init_stages_ |= ExecutorReady;
  if (rclc_executor_add_subscription(
        &executor_,
        &cmd_vel_sub_,
        &cmd_vel_msg_,
        cmdVelCallback,
        ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }
  if (rclc_executor_add_subscription(
        &executor_,
        &safety_fault_sub_,
        &safety_fault_msg_,
        safetyFaultCallback,
        ON_NEW_DATA) != RCL_RET_OK) {
    return false;
  }
  if (rclc_executor_add_timer(&executor_, &control_timer_) != RCL_RET_OK) {
    return false;
  }

  // One-time message setup (frame ids, covariance) - constant per session.
  configureOdomMessage();

  // FIX #1: synchronize the session clock with the agent immediately so the
  // very first /odom messages already carry wall-clock stamps [5].
  syncAgentTime();

  // Restart all time bases for the new session so the watchdog does not
  // immediately trip on a stale pre-connection timestamp.
  const uint32_t now_ms = millis();
  target_linear_mps_ = 0.0f;
  target_angular_rps_ = 0.0f;
  motor_feedback_ready_ = false;
  last_cmd_ms_ = now_ms;
  last_odom_publish_ms_ = 0;
  last_agent_ping_ms_ = now_ms;
  drive_.resetIntegratorTime(now_ms);
  ros_entities_ready_ = true;
  return true;
}

void RobotApp::destroyRosEntities() {
  if (ros_init_stages_ == 0) {
    return;
  }

  // Teardown mirrors creation order (children before node before support).
  // Results are ignored: after an agent loss some fini calls legitimately
  // fail, and the next createRosEntities() starts from zeroed handles anyway.
  if (ros_init_stages_ & ExecutorReady) ignoreRclResult(rclc_executor_fini(&executor_));
  if (ros_init_stages_ & TimerReady) ignoreRclResult(rcl_timer_fini(&control_timer_));
  if (ros_init_stages_ & SubscriptionReady) ignoreRclResult(rcl_subscription_fini(&cmd_vel_sub_, &node_));
  if (ros_init_stages_ & FaultSubscriptionReady) ignoreRclResult(rcl_subscription_fini(&safety_fault_sub_, &node_));
  if (ros_init_stages_ & OdomPublisherReady) ignoreRclResult(rcl_publisher_fini(&odom_pub_, &node_));
  if (ros_init_stages_ & TwistPublisherReady) ignoreRclResult(rcl_publisher_fini(&measured_twist_pub_, &node_));
  if (ros_init_stages_ & NodeReady) ignoreRclResult(rcl_node_fini(&node_));
  if (ros_init_stages_ & SupportReady) ignoreRclResult(rclc_support_fini(&support_));
  support_ = {};
  node_ = {};
  executor_ = {};
  control_timer_ = {};
  cmd_vel_sub_ = {};
  safety_fault_sub_ = {};
  odom_pub_ = {};
  measured_twist_pub_ = {};
  ros_init_stages_ = 0;
  ros_entities_ready_ = false;
  time_synced_ = false;  // FIX #1: a new session needs a fresh sync
}

void RobotApp::configureOdomMessage() {
  // micro-ROS string fields are C structs (data/size/capacity) that need
  // stable backing storage for the message's lifetime [1]. Function-local
  // static char arrays satisfy that without heap allocation.
  static char odom_frame[] = "odom";        // REP 105 odometry frame [7]
  static char base_frame[] = "base_link";   // REP 105 robot body frame [7]

  odom_msg_.header.frame_id.data = odom_frame;
  odom_msg_.header.frame_id.size = 4;               // strlen("odom")
  odom_msg_.header.frame_id.capacity = sizeof(odom_frame);
  odom_msg_.child_frame_id.data = base_frame;
  odom_msg_.child_frame_id.size = 9;                // strlen("base_link")
  odom_msg_.child_frame_id.capacity = sizeof(base_frame);

  // Zero the full 6x6 covariance matrices first...
  for (size_t i = 0; i < 36; ++i) {
    odom_msg_.pose.covariance[i] = 0.0;
    odom_msg_.twist.covariance[i] = 0.0;
  }

  // ...then set diagonal hints for the states this base can actually
  // observe: x (index 0), y (7), yaw (35) for pose; x and yaw for twist.
  // Values are conservative constants for downstream EKF fusion
  // (robot_localization) - tune against real data if needed.
  odom_msg_.pose.covariance[0] = 0.02;
  odom_msg_.pose.covariance[7] = 0.02;
  odom_msg_.pose.covariance[35] = 0.05;
  odom_msg_.twist.covariance[0] = 0.03;
  odom_msg_.twist.covariance[35] = 0.06;
}

void RobotApp::setOdomPoseAndTwist() {
  // Planar robot: yaw about +z only. The quaternion for a pure yaw rotation
  // is (0, 0, sin(yaw/2), cos(yaw/2)) - REP 103 conventions [6].
  const float half_yaw = odom_state_.yaw_rad * 0.5f;

  odom_msg_.header.stamp = rosNow();
  odom_msg_.pose.pose.position.x = odom_state_.x_m;
  odom_msg_.pose.pose.position.y = odom_state_.y_m;
  odom_msg_.pose.pose.position.z = 0.0;
  odom_msg_.pose.pose.orientation.x = 0.0;
  odom_msg_.pose.pose.orientation.y = 0.0;
  odom_msg_.pose.pose.orientation.z = sinf(half_yaw);
  odom_msg_.pose.pose.orientation.w = cosf(half_yaw);
  // Twist is expressed in the child frame (base_link) per nav_msgs/Odometry.
  odom_msg_.twist.twist.linear.x = measured_velocity_.linear_mps;
  odom_msg_.twist.twist.linear.y = 0.0;
  odom_msg_.twist.twist.linear.z = 0.0;
  odom_msg_.twist.twist.angular.x = 0.0;
  odom_msg_.twist.twist.angular.y = 0.0;
  odom_msg_.twist.twist.angular.z = measured_velocity_.angular_rps;

  // Mirror the measured body twist on its own topic for controller tuning.
  measured_twist_msg_.linear.x = measured_velocity_.linear_mps;
  measured_twist_msg_.linear.y = 0.0;
  measured_twist_msg_.linear.z = 0.0;
  measured_twist_msg_.angular.x = 0.0;
  measured_twist_msg_.angular.y = 0.0;
  measured_twist_msg_.angular.z = measured_velocity_.angular_rps;
}

bool RobotApp::runWheelTestSequence() {
  // Advance the startup test state machine once per second. Returns true
  // while the test is still running.
  const uint32_t now_ms = millis();
  const uint32_t elapsed_ms = now_ms - wheel_test_state_started_ms_;
  if (elapsed_ms < 1000) {
    return true;  // stay in the current phase for a full second
  }

  // Phase order: Forward -> Reverse -> Stop -> complete.
  switch (wheel_test_state_) {
    case WheelTestState::Forward:
      wheel_test_state_ = WheelTestState::Reverse;
      if (!micro_ros_enabled_) {
        Serial.println("[RobotApp] phase: reverse (both wheels reverse)");
      }
      break;
    case WheelTestState::Reverse:
      wheel_test_state_ = WheelTestState::Stop;
      if (!micro_ros_enabled_) {
        Serial.println("[RobotApp] phase: stop");
      }
      break;
    case WheelTestState::Stop:
      startup_test_complete_ = true;
      if (!micro_ros_enabled_) {
        Serial.println("[RobotApp] startup wheel test complete");
      }
      return false;
  }

  wheel_test_state_started_ms_ = now_ms;
  last_cmd_ms_ = now_ms;  // keep the watchdog quiet during the test
  return !startup_test_complete_;
}

void RobotApp::stopAllMotors() {
  // Never retain a motion target across disconnects or bus failures.
  target_linear_mps_ = 0.0f;
  target_angular_rps_ = 0.0f;
  motor_feedback_ready_ = false;
  // Send an individual zero-speed frame to every motor so each one receives
  // an explicit stop even if a previous frame was lost on the bus [4]. The
  // 2 ms gap gives the half-duplex DDSM bus time between frames.
  for (size_t i = 0; i < config::MotorCount; ++i) {
    ddsm_.stop(motors_[i].id);
    motor_states_[i].feedback.valid = false;
    motor_states_[i].missed_replies = 0;
    delay(2);
  }
}

void RobotApp::setupMotors() {
  delay(100);  // let the DDSM HAT finish its own power-on init [4]
  // Put every motor in closed-loop speed mode and clear any speed command
  // that may have survived a warm reboot of the ESP32.
  for (size_t i = 0; i < config::MotorCount; ++i) {
    ddsm_.setSpeedMode(motors_[i].id);
    delay(4);
    ddsm_.stop(motors_[i].id);
    delay(4);
  }
}

void RobotApp::controlStep() {
  const uint32_t now_ms = millis();

  // ---------------------------------------------------------------------------
  // Phase 1: startup wheel test. Direct RPM commands (bypassing kinematics)
  // exercise each motor's wiring, ID, and direction sign.
  // ---------------------------------------------------------------------------
  if (!startup_test_complete_) {
    const bool test_running = runWheelTestSequence();

    // Fixed test speed per phase. 140 RPM is ~2/3 of the DDSM210 maximum,
    // fast enough to be obvious on the bench but well inside limits [4].
    float test_rpm = 0.0f;
    switch (wheel_test_state_) {
      case WheelTestState::Forward:
        test_rpm = 140.0f;
        break;
      case WheelTestState::Stop:
        test_rpm = 0.0f;
        break;
      case WheelTestState::Reverse:
        test_rpm = -140.0f;
        break;
    }

    for (size_t i = 0; i < config::MotorCount; ++i) {
      // Apply the per-motor mounting sign so "forward" spins both sides the
      // same chassis direction despite mirrored motor orientation.
      const float motor_rpm = test_rpm * static_cast<float>(motors_[i].sign);
      DdsmFeedback feedback;
      if (ddsm_.commandSpeed(motors_[i].id, motor_rpm, config::DdsmAccelTime, feedback)) {
        motor_states_[i].feedback = feedback;
        motor_states_[i].last_ok_ms = now_ms;
      } else {
        motor_states_[i].feedback.valid = false;
        if (!micro_ros_enabled_) {
          Serial.print("[RobotApp] motor ");
          Serial.print(motors_[i].id);
          Serial.println(" command failed");
        }
      }

      delay(1);  // inter-frame gap on the half-duplex DDSM bus [4]
    }

    if (!test_running) {
      startup_test_complete_ = true;
    }
    return;
  }

  // ---------------------------------------------------------------------------
  // Phase 2 (bench mode only): with micro-ROS disabled there is no /cmd_vel,
  // so after the test the motors simply hold the final Stop state.
  // ---------------------------------------------------------------------------
  if (!micro_ros_enabled_) {
    for (size_t i = 0; i < config::MotorCount; ++i) {
      DdsmFeedback feedback;
      if (ddsm_.commandSpeed(motors_[i].id, 0.0f, config::DdsmAccelTime, feedback)) {
        motor_states_[i].feedback = feedback;
        motor_states_[i].last_ok_ms = now_ms;
      } else {
        motor_states_[i].feedback.valid = false;
        Serial.print("[RobotApp] motor ");
        Serial.print(motors_[i].id);
        Serial.println(" command failed");
      }
      delay(1);
    }
    return;
  }

  // ---------------------------------------------------------------------------
  // Phase 3: normal ROS control (called by the rclc timer while Connected).
  //
  // Command watchdog: if no /cmd_vel arrived within CMD_TIMEOUT_MS, command
  // zero velocity. This is the final firmware-side safety layer - it covers
  // teleop crashes, Wi-Fi/SSH loss on the Pi, agent hangs, and (together
  // with the Pi-side twist_mux lock topic) the software e-stop path.
  // ---------------------------------------------------------------------------
  const bool stale_command = (now_ms - last_cmd_ms_) > config::CmdTimeoutMs ||
                             !motor_feedback_ready_ || safety_fault_;
  const float linear_mps = stale_command ? 0.0f : target_linear_mps_;
  const float angular_rps = stale_command ? 0.0f : target_angular_rps_;

  // Inverse kinematics: body twist -> per-side wheel RPM (DifferentialDrive,
  // project code; standard diff-drive equations, REP 103 conventions [6]).
  const WheelVelocityCommand command = drive_.toWheelRpm(linear_mps, angular_rps);

  for (size_t i = 0; i < config::MotorCount; ++i) {
    // Pick the side's RPM, then apply the mounting sign for this motor.
    const float robot_rpm = motors_[i].side == MotorSide::Left ? command.left_rpm : command.right_rpm;
    const float motor_rpm = robot_rpm * static_cast<float>(motors_[i].sign);
    DdsmFeedback feedback;

    // commandSpeed both commands the motor and returns its feedback frame
    // (measured RPM etc.) in a single bus transaction [4].
    if (ddsm_.commandSpeed(motors_[i].id, motor_rpm, config::DdsmAccelTime, feedback)) {
      motor_states_[i].feedback = feedback;
      motor_states_[i].last_ok_ms = now_ms;
      motor_states_[i].missed_replies = 0;
    } else if (feedback.error == 0 && motor_feedback_ready_ &&
               ++motor_states_[i].missed_replies <= kMaxMissedReplies) {
      // No reply this step, but the motor answered recently and reported no
      // error: keep driving and let odometry reuse its last measured speed.
    } else {
      motor_states_[i].feedback.valid = false;
      // A motor error, or an unknown wheel speed for too long, is not zero
      // speed. Stop the whole base and withhold /odom so ROS freshness checks
      // can detect the bus fault.
      stopAllMotors();
      measured_velocity_ = BodyVelocity{};
      drive_.resetIntegratorTime(millis());
      return;
    }

    delay(1);  // inter-frame gap on the half-duplex DDSM bus [4]
  }

  // Forward kinematics from measured wheel speeds -> body twist, then
  // integrate to x/y/yaw for /odom (dead reckoning, REP 105 [7]).
  const uint32_t measured_ms = millis();
  if (!motor_feedback_ready_) {
    // Do not extrapolate a recovered sample through the missing interval.
    drive_.resetIntegratorTime(measured_ms);
  }
  motor_feedback_ready_ = true;
  measured_velocity_ = drive_.measuredVelocity(motors_, motor_states_, config::MotorCount);
  drive_.integrate(measured_velocity_, measured_ms, odom_state_);
  publishOdometryIfDue(measured_ms);
}

void RobotApp::publishOdometryIfDue(uint32_t now_ms) {
  // Publish at ODOM_PUBLISH_PERIOD_MS (slower than the control loop) to keep
  // serial bandwidth for /cmd_vel; only when the ROS entities exist.
  // Half a control step of slack: odometry is computed once per step, and a
  // step that fires a millisecond early must not be skipped, or the stream
  // alternates between one and two periods.
  if (!ros_entities_ready_ ||
      (now_ms - last_odom_publish_ms_) + config::ControlPeriodMs / 2 < config::OdomPublishPeriodMs) {
    return;
  }

  last_odom_publish_ms_ = now_ms;
  setOdomPoseAndTwist();
  ignoreRclResult(rcl_publish(&odom_pub_, &odom_msg_, nullptr));
  ignoreRclResult(rcl_publish(&measured_twist_pub_, &measured_twist_msg_, nullptr));
}

builtin_interfaces__msg__Time RobotApp::rosNow() {
  builtin_interfaces__msg__Time stamp;
  // FIX #1: after syncAgentTime() succeeded, rmw_uros_epoch_nanos() returns
  // the agent's wall clock (Pi time) [5], so /odom stamps live in the same
  // time domain as the Pi-side IMU / LiDAR / camera stamps.
  int64_t time_ns = rmw_uros_epoch_nanos();

  // Fallback: ESP32 uptime, so message stamps stay monotonic during the
  // early connection window before the first sync completes.
  if (time_ns <= 0) {
    time_ns = static_cast<int64_t>(millis()) * 1000000LL;
  }

  stamp.sec = static_cast<int32_t>(time_ns / 1000000000LL);
  stamp.nanosec = static_cast<uint32_t>(time_ns % 1000000000LL);
  return stamp;
}

}  // namespace l2r
