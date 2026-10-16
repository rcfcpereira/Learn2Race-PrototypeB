"""Optional closed-loop body-velocity PID controller.

The firmware already sends speed commands to DDSM210 motors. This node is an
optional outer loop for tuning robot body velocity:

`/cmd_vel_setpoint` + `/ddsm/measured_twist` -> PID correction -> `/cmd_vel`

It uses the standard PID equation:

command = feedforward + kp * error + ki * integral(error) + kd * d(error)/dt
"""

# References (full entries: docs/references.md at the repository root):
#   [R40] Astrom & Murray, Feedback Systems (PID): PID body-velocity loop
#   [R64] Waveshare DDSM Driver HAT / DDSM210: DDSM210 measured speed
# More details: README.md of this package

from dataclasses import dataclass

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node


@dataclass
class PidAxis:
    """One scalar PID axis with integral clamping and output limiting."""

    kp: float
    ki: float
    kd: float
    integral_limit: float
    output_limit: float
    integral: float = 0.0
    last_error: float = 0.0
    has_last_error: bool = False

    def reset(self) -> None:
        self.integral = 0.0
        self.last_error = 0.0
        self.has_last_error = False

    def update(self, target: float, measured: float, dt: float) -> float:
        if dt <= 0.0:
            return 0.0

        error = target - measured
        self.integral += error * dt
        self.integral = self._clamp(self.integral, self.integral_limit)

        derivative = 0.0
        if self.has_last_error:
            derivative = (error - self.last_error) / dt
        self.last_error = error
        self.has_last_error = True

        correction = self.kp * error + self.ki * self.integral + self.kd * derivative
        return self._clamp(correction, self.output_limit)

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        if limit <= 0.0:
            return value
        return max(-limit, min(limit, value))


class TwistPidControllerNode(Node):
    """Correct commanded linear.x and angular.z using measured body velocity."""

    def __init__(self) -> None:
        super().__init__("twist_pid_controller_node")

        self.declare_parameter("setpoint_topic", "/cmd_vel_setpoint")
        self.declare_parameter("measured_topic", "/ddsm/measured_twist")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("control_rate_hz", 30.0)
        self.declare_parameter("command_timeout_s", 0.5)
        self.declare_parameter("measured_timeout_s", 0.5)
        self.declare_parameter("pass_through_on_measured_timeout", True)
        self.declare_parameter("enabled", True)

        # Conservative defaults: start with P only, no integral or derivative.
        self.declare_parameter("linear_kp", 0.20)
        self.declare_parameter("linear_ki", 0.00)
        self.declare_parameter("linear_kd", 0.00)
        self.declare_parameter("linear_integral_limit", 0.20)
        self.declare_parameter("linear_correction_limit", 0.08)
        self.declare_parameter("linear_output_limit", 0.35)

        self.declare_parameter("angular_kp", 0.20)
        self.declare_parameter("angular_ki", 0.00)
        self.declare_parameter("angular_kd", 0.00)
        self.declare_parameter("angular_integral_limit", 0.40)
        self.declare_parameter("angular_correction_limit", 0.20)
        self.declare_parameter("angular_output_limit", 1.50)

        self.enabled = bool(self.get_parameter("enabled").value)
        self.command_timeout_s = float(self.get_parameter("command_timeout_s").value)
        self.measured_timeout_s = float(self.get_parameter("measured_timeout_s").value)
        self.pass_through_on_measured_timeout = bool(
            self.get_parameter("pass_through_on_measured_timeout").value
        )
        self.linear_output_limit = float(self.get_parameter("linear_output_limit").value)
        self.angular_output_limit = float(self.get_parameter("angular_output_limit").value)

        self.linear_pid = PidAxis(
            kp=float(self.get_parameter("linear_kp").value),
            ki=float(self.get_parameter("linear_ki").value),
            kd=float(self.get_parameter("linear_kd").value),
            integral_limit=float(self.get_parameter("linear_integral_limit").value),
            output_limit=float(self.get_parameter("linear_correction_limit").value),
        )
        self.angular_pid = PidAxis(
            kp=float(self.get_parameter("angular_kp").value),
            ki=float(self.get_parameter("angular_ki").value),
            kd=float(self.get_parameter("angular_kd").value),
            integral_limit=float(self.get_parameter("angular_integral_limit").value),
            output_limit=float(self.get_parameter("angular_correction_limit").value),
        )

        self.target = Twist()
        self.measured = Twist()
        self.last_target_time = self.get_clock().now()
        self.last_measured_time = self.get_clock().now()
        self.last_update_time = self.get_clock().now()

        self.output_pub = self.create_publisher(
            Twist, str(self.get_parameter("output_topic").value), 10
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("setpoint_topic").value),
            self._target_callback,
            10,
        )
        self.create_subscription(
            Twist,
            str(self.get_parameter("measured_topic").value),
            self._measured_callback,
            10,
        )

        rate_hz = max(1.0, float(self.get_parameter("control_rate_hz").value))
        self.create_timer(1.0 / rate_hz, self._update)

    def _target_callback(self, msg: Twist) -> None:
        self.target = msg
        self.last_target_time = self.get_clock().now()

    def _measured_callback(self, msg: Twist) -> None:
        self.measured = msg
        self.last_measured_time = self.get_clock().now()

    def _update(self) -> None:
        now = self.get_clock().now()
        dt = (now - self.last_update_time).nanoseconds / 1e9
        self.last_update_time = now

        target_age = (now - self.last_target_time).nanoseconds / 1e9
        measured_age = (now - self.last_measured_time).nanoseconds / 1e9

        if target_age > self.command_timeout_s:
            self.linear_pid.reset()
            self.angular_pid.reset()
            self.output_pub.publish(Twist())
            return

        measured_stale = measured_age > self.measured_timeout_s
        if not self.enabled or (measured_stale and self.pass_through_on_measured_timeout):
            self.linear_pid.reset()
            self.angular_pid.reset()
            self.output_pub.publish(self._limited_target())
            return

        if measured_stale:
            self.linear_pid.reset()
            self.angular_pid.reset()
            self.output_pub.publish(Twist())
            return

        cmd = Twist()
        linear_correction = self.linear_pid.update(
            self.target.linear.x, self.measured.linear.x, dt
        )
        angular_correction = self.angular_pid.update(
            self.target.angular.z, self.measured.angular.z, dt
        )
        cmd.linear.x = self._clamp(
            self.target.linear.x + linear_correction, self.linear_output_limit
        )
        cmd.angular.z = self._clamp(
            self.target.angular.z + angular_correction, self.angular_output_limit
        )
        self.output_pub.publish(cmd)

    def _limited_target(self) -> Twist:
        cmd = Twist()
        cmd.linear.x = self._clamp(self.target.linear.x, self.linear_output_limit)
        cmd.angular.z = self._clamp(self.target.angular.z, self.angular_output_limit)
        return cmd

    @staticmethod
    def _clamp(value: float, limit: float) -> float:
        if limit <= 0.0:
            return value
        return max(-limit, min(limit, value))

    def destroy_node(self) -> bool:
        self.output_pub.publish(Twist())
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = TwistPidControllerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
